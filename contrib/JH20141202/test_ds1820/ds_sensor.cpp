/*
The OpenTRV project licenses this file to you
under the Apache Licence, Version 2.0 (the "Licence");
you may not use this file except in compliance
with the Licence. You may obtain a copy of the Licence at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing,
software distributed under the Licence is distributed on an
"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
KIND, either express or implied. See the Licence for the
specific language governing permissions and limitations
under the Licence.

Author(s) / Copyright (s): John Harvey 2014
*/

#include <stdint.h>
#include "Serial_IO.h"
#include "Power_Management.h"
#include "ds_sensor.h"
#include <OneWire.h>

#define DS1820_PRECISION_MASK 0x60
#define DS1820_PRECISION_9 0x00
#define DS1820_PRECISION_10 0x20
#define DS1820_PRECISION_11 0x40
#define DS1820_PRECISION_12 0x60

#define DS1820_PRECISION DS1820_PRECISION_10
#define DS1820_ONEWIRE_PIN 10 // on pin 10 (a 4.7K resistor is necessary)
#define DS1820_ONEWIRE_PIN PIN_OW_DQ_DATA
static OneWire  ds(DS1820_ONEWIRE_PIN);  

static struct DS1820_INFO{
  int m_numDevices;
  int m_convertDelay;
  byte m_addr[8];
  volatile bool m_initialised;
  bool m_oldType;
} DS1820_info;
// Temperature read uses/selects one of the implementations/sensors.
static void DS1820_init()
{
    byte addr[8];
    bool found=false;
    //Look for the first temp device
    while(ds.search(addr))
    {
	int i;
	DS1820_info.m_numDevices++;
	DEBUG_SERIAL_PRINTLN();
	DEBUG_SERIAL_PRINTLN();
	DEBUG_SERIAL_PRINT_FLASHSTRING("ROM =");
	for( i = 0; i < 8; i++) {
	    DEBUG_SERIAL_PRINT(' ');
	    DEBUG_SERIAL_PRINTFMT(addr[i], HEX);
	}

	if (OneWire::crc8(addr, 7) != addr[7])
	{
	    DEBUG_SERIAL_PRINTLN_FLASHSTRING("CRC is not valid!");
	    continue;
	}
	DEBUG_SERIAL_PRINTLN(); 
	if (found)
	  continue;
	// the first ROM byte indicates which chip
	switch (addr[0]) {
	case 0x10:
	  DEBUG_SERIAL_PRINTLN_FLASHSTRING("  Chip = DS18S20");  // or old DS1820
	  DS1820_info.m_oldType = true;
	  found = true;
	  break;
	case 0x28:
	  DEBUG_SERIAL_PRINTLN_FLASHSTRING("  Chip = DS18B20");
	  DS1820_info.m_oldType = false;
	  found = true;
	  break;
	case 0x22:
	  DEBUG_SERIAL_PRINTLN_FLASHSTRING("  Chip = DS1822");
	  DS1820_info.m_oldType = false;
	  found = true;
	  break;
	} 
	if (found)
	  {
	    for (i=0;i<8;i++)
	      {
		DS1820_info.m_addr[i] = addr[i];
	      }
	  }
	ds.write(0xBE);         // Read Scratchpad
	byte data[9];
	for ( i = 0; i < 9; i++) {           // we need 9 bytes
	  data[i] = ds.read();
	  DEBUG_SERIAL_PRINTFMT(data[i], HEX);
	  DEBUG_SERIAL_PRINT_FLASHSTRING(" ");
	}
	DEBUG_SERIAL_PRINTLN();
	data[2] &= !DS1820_PRECISION_MASK;
	data[2] |= DS1820_PRECISION;

	ds.reset();
	ds.select(DS1820_info.m_addr);
	ds.write(0x4E);         // Write Scratchpad
	for ( i = 0; i < 3; i++) {           // we need 9 bytes
	  ds.write(data[i]);
	}
	byte cfg = data[2] & DS1820_PRECISION_MASK;
	if (cfg == DS1820_PRECISION_9) DS1820_info.m_convertDelay=130;
	else if (cfg == DS1820_PRECISION_10) DS1820_info.m_convertDelay=400;
	else if (cfg == DS1820_PRECISION_11) DS1820_info.m_convertDelay=550;
	else DS1820_info.m_convertDelay=850;

    }

    if (!found)
      DS1820_info.m_numDevices = -1;
    DS1820_info.m_initialised = true;    
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("Done Init");
}

static int badRead;
static int Sensor_DS1820_readTemperatureC16()
{
  if (!DS1820_info.m_initialised) DS1820_init();
  if (DS1820_info.m_numDevices >0)
  {
      ds.reset();
      if (DS1820_info.m_numDevices == 1)
	{ // With only one device dont need to send its address
	  ds.skip();
      } else
      { 
	  ds.select(DS1820_info.m_addr);
      }
      ds.write(0x44, 1);        // start conversion, with parasite power on at the end and wait for the conversion
      delay(DS1820_info.m_convertDelay); // This will need to be changed for something better and this can be dar too long as well
      ds.reset();
      ds.skip();
      ds.write(0xBE);         // Read Scratchpad
      byte data[9];
      int i;
      DEBUG_SERIAL_PRINT_FLASHSTRING("SCRATCHPAD =");
      for ( i = 0; i < 9; i++) {           // we need 9 bytes
  	  data[i] = ds.read();
	  DEBUG_SERIAL_PRINTFMT(data[i], HEX);
	  DEBUG_SERIAL_PRINT_FLASHSTRING(" ");
      }
      DEBUG_SERIAL_PRINTLN();
      if (OneWire::crc8(data,8) != data[8])
      {
	  DEBUG_SERIAL_PRINT_FLASHSTRING(" BAD CRC=");
	  DEBUG_SERIAL_PRINTFMT(OneWire::crc8(data, 8), HEX);
	  DEBUG_SERIAL_PRINTLN();
      }

      // Convert the data to actual temperature
      // because the result is a 16 bit signed integer, it should
      // be stored to an "int16_t" type, which is always 16 bits
      // even when compiled on a 32 bit processor.
      int16_t raw = (data[1] << 8) | data[0];
      if (raw == 0x0550) // After a reset it sends out this value so resetting it helps
      {
	  badRead++;
	  DS1820_init();
      }

      if (DS1820_info.m_oldType) 
      {
	  raw = raw << 3; // 9 bit resolution default
	  if (data[7] == 0x10) {
	    // "count remain" gives full 12 bit resolution
	    raw = (raw & 0xFFF0) + 12 - data[6];
	  }
      } else 
      {
	  byte cfg = (data[4] & DS1820_PRECISION_MASK);
	  // at lower res, the low bits are undefined, so let's zero them
	  if (cfg == DS1820_PRECISION_9) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
	  else if (cfg == DS1820_PRECISION_10) raw = raw & ~3; // 10 bit res, 187.5 ms
	  else if (cfg == DS1820_PRECISION_11) raw = raw & ~1; // 11 bit res, 375 ms
	  //// default is 12 bit resolution, 750 ms conversion time
      }
      DEBUG_SERIAL_PRINT_FLASHSTRING("Bad count  ");
      DEBUG_SERIAL_PRINT(badRead);
      DEBUG_SERIAL_PRINTLN();
      return raw;
    }
  return 0;

}

int DS1820::read()
  {
  const int result = Sensor_DS1820_readTemperatureC16();
  value = result;
  return(value);
  }
// Singleton implementation/instance.
DS1820 TemperatureDS1820;

