
#include <SPI.h>

#include <Wire.h>

#include <util/crc16.h>
#include <avr/eeprom.h>
#include <OneWire.h>
#include "ds_sensor.h"
#include "Serial_IO.h"
// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library


// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic()
  {
}
void setup(void) {
  Serial.begin(9600);
}

void loop(void) {
     int val = TemperatureDS1820.read();
     float celsius = (float)val/16.0;
     Serial.print("Temp in celsius ");
     Serial.print(celsius);
     Serial.println();
}
