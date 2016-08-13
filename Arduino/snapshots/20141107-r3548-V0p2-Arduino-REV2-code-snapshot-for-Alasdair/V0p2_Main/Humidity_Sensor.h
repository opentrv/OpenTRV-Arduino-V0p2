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
 */

#ifndef HUMIDITY_SENSOR_H
#define HUMIDITY_SENSOR_H

#include "V0p2_Main.h"


// High and low bounds on relative humidity for comfort and (eg) mite/mould growth.
#define HUMIDTY_HIGH_RHPC 70
#define HUMIDTY_LOW_RHPC 30
// Epsilon bounds (absolute % +/- around thresholds) for accuracy and hysteresis.
#define HUMIDITY_EPSILON_RHPC 5

#if ((HUMIDTY_HIGH_RHPC + HUMIDITY_EPSILON_RHPC) >= 100)
#error bad RH constants!
#endif
#if ((HUMIDTY_LOW_RHPC - HUMIDITY_EPSILON_RHPC) <= 0)
#error bad RH constants!
#endif


// HUMIDITY_SENSOR_SUPPORT is defined if at least one humdity sensor has support compiled in.
// Simple implementations can assume that the sensor will be present if defined;
// more sophisticated implementations may wish to make run-time checks.

// IF SHT21 support is enabled at compile-time then its humidity sensor may be used at run-time.
// There may be other alternatived
#if defined(SENSOR_SHT21_ENABLE)
#define HUMIDITY_SENSOR_SUPPORT // Humidity sensing available.
#include "Sensor_SHT21.h"
#endif


// If humidity is supported, provide definitions.
#if defined(HUMIDITY_SENSOR_SUPPORT)
// Measure and return the current relative humidity in %; range [0,100] and 255 for error.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
// Returns 255 (~0) in case of error.
uint8_t readRHpc();
// Return the last relative humidity in %; range [0,100] and 255 for error/unread as returned by readRHpc().
// Always fast.
uint8_t getRHpc();
// Returns true if humidity sensing available.
#define isRHAvailable() (true) // No run-time detection.
#define isRHHigh() (isRHAvailable() && (getRHpc() >= (HUMIDTY_HIGH_RHPC+HUMIDITY_EPSILON_RHPC)))
#else
#define isRHAvailable() (false) // Not available.
#define isRHHigh() (false)
#endif




#endif


