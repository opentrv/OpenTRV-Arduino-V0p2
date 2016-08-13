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

Author(s) / Copyright (s): Damon Hart-Davis 2013
*/

/*
 Temperature sensor module.
 
 Default implementation is TMP102/TMP102 with ADD0 tied to Gnd.
 */

#include "Temperature_Sensor.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "Serial_Debug.h"
#include "Power_Management.h"

#include <Wire.h> // Arduino I2C library.

// TMP102 and TMP112 should be interchangeable: latter has better guaranteed accuracy.
#define TMP102_I2C_ADDR 72
#define TMP102_REG_TEMP 0 // Temperature register.
#define TMP102_REG_CTRL 1 // Control register.
#define TMP102_CTRL_B1 0x31 // Byte 1 for control register: 12-bit resolution and shutdown mode (SD).
#define TMP102_CTRL_B1_OS 0x80 // Control register: one-shot flag in byte 1.
#define TMP102_CTRL_B2 0x0 // Byte 2 for control register: 0.25Hz conversion rate and not extended mode (EM).

// Last temperature read with readTemperatureC16(); initially 0 and set to 0 on error.
static int temp16;


// Measure/store/return the current room ambient temperature in units of 1/16th C.
// This may contain up to 4 bits of information to the right of the fixed binary point.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
// This will simulate a zero temperature in case of detected error talking to the sensor as fail-safe for this use.
// Check for errors at certain critical places, not everywhere.
int readTemperatureC16()
  {
  const bool neededPowerUp = powerUpTWIIfDisabled();

  // Force start of new one-shot temperature measurement/conversion to complete.
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write((byte) TMP102_REG_CTRL); // Select control register.
  Wire.write((byte) TMP102_CTRL_B1); // Clear OS bit.
  //Wire.write((byte) TMP102_CTRL_B2);
  Wire.endTransmission();
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write((byte) TMP102_REG_CTRL); // Select control register.
  Wire.write((byte) TMP102_CTRL_B1 | TMP102_CTRL_B1_OS); // Start one-shot conversion.
  //Wire.write((byte) TMP102_CTRL_B2);
  if(Wire.endTransmission()) { temp16 = 0; return(0); } // Exit if error.

  // Wait for temperature measurement/conversion to complete, in low-power sleep mode for the bulk of the time.
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write((byte) TMP102_REG_CTRL); // Select control register.
  if(Wire.endTransmission()) { temp16 = 0; return(0); } // Exit if error.
  for(int i = 8; --i; ) // 2 orbits should generally be plenty.
    {
    if(i <= 0) { temp16 = 0; return(0); } // Exit if error.
    if(Wire.requestFrom(TMP102_I2C_ADDR, 1) != 1) { temp16 = 0; return(0); } // Exit if error.
    const byte b1 = Wire.read();
    if(b1 & TMP102_CTRL_B1_OS) { break; } // Conversion completed.
    nap(WDTO_15MS); // One or two of these naps should allow typical ~26ms conversion to complete...
    }

  // Fetch temperature.
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write((byte) TMP102_REG_TEMP); // Select temperature register (set ptr to 0).
  if(Wire.endTransmission()) { temp16 = 0; return(0); } // Exit if error.
  if(Wire.requestFrom(TMP102_I2C_ADDR, 2) != 2)  { temp16 = 0; return(0); }
  if(Wire.endTransmission()) { temp16 = 0; return(0); } // Exit if error.

  const byte b1 = Wire.read(); // MSByte, should be signed whole degrees C.
  const uint8_t b2 = Wire.read(); // Avoid sign extension...

  // Builds 12-bit value (assumes not in extended mode) and sign-extends if necessary for sub-zero temps.
  const int t16 = (b1 << 4) | (b2 >> 4) | ((b1 & 0x80) ? 0xf000 : 0);

  // Store the result for access at any time.
  temp16 = t16;

#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Temp: ");
  DEBUG_SERIAL_PRINT(b1);
  DEBUG_SERIAL_PRINT_FLASHSTRING("C / ");
  DEBUG_SERIAL_PRINT(temp16);
  DEBUG_SERIAL_PRINTLN();
#endif

  if(neededPowerUp) { powerDownTWI(); }

  return(t16);
  }

// Return previously-read (with readTemperatureC16()) temperature; very fast.
int getTemperatureC16() { return(temp16); }

