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
 Ambient light sensor module.
 */

#ifndef TEMP_POT_H
#define TEMP_POT_H

#include <stdint.h>

#include "V0p2_Main.h"

#if V0p2_REV >= 2 // Only supported in REV onwards.

#define TEMP_POT_AVAILABLE

// Read the user 'temperature pot' setting in range [0,1023].
// This may consume significant power and time.
int readTempPot();

// Return previously-read (with readTempPot()) temperature pot user control level in range [0,1023]; very fast.
int getTempPot();

// Get reduced-noise temperature pot user control value (previously read with readTempPot()) in range [0,255]; very fast.
// In particular this value should not jitter between readings even if the pot is sitting at the boundary.
uint8_t getTempPotReducedNoise();

#endif



#endif


