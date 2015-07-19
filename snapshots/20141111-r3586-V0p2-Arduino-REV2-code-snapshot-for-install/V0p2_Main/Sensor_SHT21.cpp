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

Author(s) / Copyright (s): Damon Hart-Davis 2014
*/

/*
 SHT21 (SHT2x) temperature and humidity I2C sensor.
 */

#include "Sensor_SHT21.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

// Functionality and code only enabled if SENSOR_SHT21_ENABLE is defined.
#ifdef SENSOR_SHT21_ENABLE

#include <stdint.h>
#include "Control.h"
#include "Security.h"
#include "Serial_IO.h"
#include "Power_Management.h"
#include "UI_Minimal.h"

#include <Wire.h> // Arduino I2C library.

#define SHT21_I2C_ADDR 0x40
#define SHT21_I2C_CMD_TEMP_HOLD	0xe3
#define SHT21_I2C_CMD_TEMP_NOHOLD 0xf3
#define SHT21_I2C_CMD_RH_HOLD	0xe5
#define SHT21_I2C_CMD_RH_NOHOLD 0xf5
#define SHT21_I2C_CMD_USERREG 0xe7 // User register...

// If defined, sample 8-bit RH (for for 1%) and 12-bit temp (for 1/16C).
// This should save time and energy.
#define SHT21_USE_REDUCED_PRECISION 1

// Set try once SHT21 is initialised.
static volatile bool initialised;

// Initialise/configure SHT21, once only generally.
// TWI must already be powered up.
static void SHT21_init()
  {
#if defined(SHT21_USE_REDUCED_PRECISION)
  // Soft reset in order to sample at reduced precision.
  Wire.beginTransmission(SHT21_I2C_ADDR);
  Wire.write((byte) SHT21_I2C_CMD_USERREG); // Select control register.
  Wire.endTransmission();
  Wire.requestFrom(SHT21_I2C_ADDR, 1);
  while(Wire.available() < 1)
    {
    // Wait for data, but avoid rolling over the end of a minor cycle...
    if(getSubCycleTime() >= GSCT_MAX-2)
      {
      return; // Failed, and not initialised.
      }
    }
  const uint8_t curUR = Wire.read();
//  DEBUG_SERIAL_PRINT_FLASHSTRING("UR: ");
//  DEBUG_SERIAL_PRINTFMT(curUR, HEX);
//  DEBUG_SERIAL_PRINTLN();

  // Preserve reserved bits (3, 4, 5) and sample 8-bit RH (for for 1%) and 12-bit temp (for 1/16C). 
  const uint8_t newUR = (curUR & 0x38) | 3;
  Wire.beginTransmission(SHT21_I2C_ADDR);
  Wire.write((byte) SHT21_I2C_CMD_USERREG); // Select control register.
  Wire.write((byte) newUR);
  Wire.endTransmission();

#endif
  initialised = true;
  }

// Measure and return the current ambient temperature in units of 1/16th C.
// This may contain up to 4 bits of information to the right of the fixed binary point.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
int Sensor_SHT21_readTemperatureC16()
  {
  const bool neededPowerUp = powerUpTWIIfDisabled();

  // Initialise/config if necessary.
  if(!initialised) { SHT21_init(); }
    
  // Max RH measurement time:
  //   * 14-bit: 85ms
  //   * 12-bit: 22ms
  //   * 11-bit: 11ms
  // Use blocking data fetch for now.
  Wire.beginTransmission(SHT21_I2C_ADDR);
  Wire.write((byte) SHT21_I2C_CMD_TEMP_HOLD); // Select control register.
#if defined(SHT21_USE_REDUCED_PRECISION)
  nap(WDTO_30MS); // Should cover 12-bit conversion (22ms).
#else
  sleepLowPowerMs(90); // Should be plenty for slowest (14-bit) conversion (85ms).
#endif
  //delay(100);
  Wire.endTransmission();
  Wire.requestFrom(SHT21_I2C_ADDR, 3);
  while(Wire.available() < 3)
    {
    // Wait for data, but avoid rolling over the end of a minor cycle...
    if(getSubCycleTime() >= GSCT_MAX-2)
      {
      return(0); // Failure value: may be able to to better.
      }
    }
  uint16_t rawTemp = (Wire.read() << 8);
  rawTemp |= (Wire.read() & 0xfc); // Clear status ls bits.
  // Nominal formula: C = -46.85 + ((175.72*raw) / (1L << 16));
  const int c16 = -750 + ((5623L * rawTemp) >> 17); // FIXME: find a faster approximation...

  if(neededPowerUp) { powerDownTWI(); }

  return(c16);
  }

// Measure and return the current relative humidity in %; range [0,100] and 255 for error.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
// Returns 255 (~0) in case of error.
uint8_t Sensor_SHT21_readRHpc()
  {
  const bool neededPowerUp = powerUpTWIIfDisabled();

  // Initialise/config if necessary.
  if(!initialised) { SHT21_init(); }

  // Get RH%...
  // Max RH measurement time:
  //   * 12-bit: 29ms
  //   *  8-bit:  4ms
  // Use blocking data fetch for now.
  Wire.beginTransmission(SHT21_I2C_ADDR);
  Wire.write((byte) SHT21_I2C_CMD_RH_HOLD); // Select control register.
#if defined(SHT21_USE_REDUCED_PRECISION)
  sleepLowPowerMs(5); // Should cover 8-bit conversion (4ms).
#else
  nap(WDTO_30MS); // Should cover even 12-bit conversion (29ms).
#endif
  Wire.endTransmission();
  Wire.requestFrom(SHT21_I2C_ADDR, 3);
  while(Wire.available() < 3)
    {
    // Wait for data, but avoid rolling over the end of a minor cycle...
    if(getSubCycleTime() >= GSCT_MAX)
      {
//      DEBUG_SERIAL_PRINTLN_FLASHSTRING("giving up");
      return(~0);
      }
    }  
  uint16_t rawRH = (Wire.read() << 8);
  rawRH |= (Wire.read() & 0xfc); // Clear status ls bits.
  const uint8_t rh = -6 + ((125L * rawRH) >> 16);

  if(neededPowerUp) { powerDownTWI(); }

  return(rh);
  }


#endif

