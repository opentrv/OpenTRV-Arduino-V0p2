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
 Voice sensor.
 */

#include "Sensor_Voice.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

// Functionality and code only enabled if SENSOR_SHT21_ENABLE is defined.
#ifdef ENABLE_VOICE_SENSOR

#include <stdint.h>
#include "Control.h"
#include "Serial_IO.h"
#include "Power_Management.h"



#endif

