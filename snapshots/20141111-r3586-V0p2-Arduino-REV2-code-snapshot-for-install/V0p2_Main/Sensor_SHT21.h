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

 Functionality and code only enabled if SENSOR_SHT21_ENABLE is defined. 
 */

#ifndef SENSOR_SHT21_H
#define SENSOR_SHT21_H

#include "V0p2_Main.h"

// Functionality and code only enabled if SENSOR_SHT21_ENABLE is defined.
#ifdef SENSOR_SHT21_ENABLE

#include <stdint.h>

// Measure and return the current ambient temperature in units of 1/16th C.
// This may contain up to 4 bits of information to the right of the fixed binary point.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
int Sensor_SHT21_readTemperatureC16();

// Measure and return the current relative humidity in %; range [0,100] and 255 for error.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
// Returns 255 (~0) in case of error.
uint8_t Sensor_SHT21_readRHpc();

#endif

#endif
