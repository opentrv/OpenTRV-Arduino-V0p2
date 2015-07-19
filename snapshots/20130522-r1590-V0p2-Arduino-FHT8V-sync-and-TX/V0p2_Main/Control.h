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
 Control/model for TRV and boiler.
 */

#ifndef CONTROL_H
#define CONTROL_H

#include "V0p2_Main.h"

// Minimum and maximum bounds target temperatures; degrees C/Celsius/centigrade, strictly positive.
// Minimum is some way above 0C to avoid freezing pipework even with small measurement errors and non-uniform temperatures.
// Maximum is set a little below boiling/100C for DHW applications for safety.
// Setbacks and uplifts cannot move temperature targets outside this range for safety.
#define MIN_TARGET_C 5 // Minimum temperature setting allowed (to avoid freezing, allowing for offsets at temperature sensor, etc). 
#define MAX_TARGET_C 95 // Maximum temperature setting allowed (eg for DHW).

// Default frost (minimum) temperature in degrees C, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
#define FROST 5
#ifndef DHW_TEMPERATURES
// Default warm/comfort room (air) temperature in degrees C; strictly greater than FROST, in range [MIN_TARGET_C,MAX_TARGET_C].
// Control loop effectively targets upper end of this 1C window as of 20130518.
#define WARM 17 // 17 or 18 good for energy saving at maybe 1C below typical UK room temperatures (~19C in 2012).
#else // Default settings for DHW control.
#define WARM 60 // 60C+ for DHW Legionella control.
#endif

// Raise target by this many degrees in 'BAKE' mode (strictly positive).
#define BAKE_UPLIFT 5
// Maximum 'BAKE' minutes, ie time to crank heating up to BAKE setting (minutes, strictly positive, <255).
#define BAKE_MAX_M 30

// Initial setback degrees C (non-negative).  Note that 1C setback may result in ~8% saving in UK.
#define SETBACK 1
// Full setback degrees C (non-negative).  Should result in significant automatic energy savings if engaged.
#define SETBACK_FULL 3
// Prolonged inactivity time deemed to indicate room really unoccupied to trigger full setback (seconds, strictly positive).
#define SETBACK_FULL_S 3600 // 1 hour

// Get dynamically-set thresholds/parameters.
#ifdef SETTABLE_TARGET_TEMPERATURES
// Get 'FROST' protection target in C; no higher than getWARMTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
uint8_t getFROSTTargetC();
// Get 'WARM' target in C; no lower than getFROSTTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
uint8_t getWARMTargetC();
// Set (non-volatile) 'FROST' protection target in C; no higher than getWARMTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Returns false if not set, eg because outside range [MIN_TARGET_C,MAX_TARGET_C], else returns true.
bool setFROSTTargetC(uint8_t tempC);
// Set 'WARM' target in C; no lower than getFROSTTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Returns false if not set, eg because below FROST setting or outside range [MIN_TARGET_C,MAX_TARGET_C], else returns true.
bool setWARMTargetC(uint8_t tempC);
#endif


// Percentage open for local TRV being controlled in range [0,100]; 0 is closed/off and the initial state.
uint8_t getTRVPercentOpen();

// Target temperature in Centigrade.
uint8_t getTargetTempC();

// Compute target temperature and set heat demand for TRV and boiler.
// CALL APPROXIMATELY ONCE PER MINUTE TO ALLOW SIMPLE TIME-BASED CONTROLS.
// Inputs are inWarmMode(), isRoomLit().
// The inputs must be valid (and recent).
// Values set are targetTempC, TRVPercentOpen.
// This may also prepare data such as TX command sequences for the TRV, boiler, etc.
// This routine may take significant CPU time; no I/O is done, only internal state is updated.
// Returns true if valve target changed and thus messages may need to be recomputed/sent/etc.
bool computeTargetAndDemand();


#endif
