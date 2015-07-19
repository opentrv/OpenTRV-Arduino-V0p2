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
 Ambient light sensor module.
 */

#ifndef AMBIENT_LIGHT_SENSOR_H
#define AMBIENT_LIGHT_SENSOR_H

#include <stdint.h>

#include "V0p2_Main.h"


// Measure/store/return the current room ambient light levels.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
int readAmbientLight();

// Returns true if room/environs well enough lit for normal activity.
// Based on results of last call to readAmbientLight().
bool isRoomLit();



#endif
