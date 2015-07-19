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
 Humidity sensor module / front-end.
 
 Default implementation is STH21, if present.
 */

#include "Humidity_Sensor.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "Serial_IO.h"
#include "Power_Management.h"


#if defined(HUMIDITY_SENSOR_SUPPORT)
// Saved relative humidity % [0,100] or 255 (~0) if not valid/read.
static uint8_t rhpc = 255;
// Measure and return the current relative humidity in %; range [0,100] and 255 for error/unread.
// Always fast.
uint8_t getRHpc() { return(rhpc); }
#endif


#if defined(SENSOR_SHT21_ENABLE)
// Measure and return the current relative humidity in %; range [0,100] and 255 for error.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
uint8_t readRHpc()
  {
  const uint8_t v = Sensor_SHT21_readRHpc();
  rhpc = v;
  return(v);
  }
#endif

//// Return previously-read (with readTemperatureC16()) temperature; very fast.
//int getTemperatureC16() { return(temp16); }
