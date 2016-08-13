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

#include <stdint.h>

#include "V0p2_Main.h"

// Minimum and maximum bounds target temperatures; degrees C/Celsius/centigrade, strictly positive.
// Minimum is some way above 0C to avoid freezing pipework even with small measurement errors and non-uniform temperatures.
// Maximum is set a little below boiling/100C for DHW applications for safety.
// Setbacks and uplifts cannot move temperature targets outside this range for safety.
#define MIN_TARGET_C 5 // Minimum temperature setting allowed (to avoid freezing, allowing for offsets at temperature sensor, etc). 
#define MAX_TARGET_C 95 // Maximum temperature setting allowed (eg for DHW).


// Default frost (minimum) temperature in degrees C, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
#define BIASECO_FROST MIN_TARGET_C // Target FROST temperature for ECO bias; must be in range [MIN_TARGET_C,MAX_TARGET_C].
#define BIASCOM_FROST (5+MIN_TARGET_C) // Target FROST temperature for Comfort bias; must be in range [MIN_TARGET_C,MAX_TARGET_C].
#define FROST BIASECO_FROST
#ifndef DHW_TEMPERATURES
// Default warm/comfort room (air) temperature in degrees C; strictly greater than FROST, in range [MIN_TARGET_C,MAX_TARGET_C].
// Control loop effectively targets upper end of this 1C window as of 20130518.
#define BIASECO_WARM 17 // Target WARM temperature for ECO bias; must be in range [MODECOM_WARM,MAX_TARGET_C].
#define BIASCOM_WARM 20 // Target WARM temperature for Comfort bias; must be in range ]MIN_TARGET_C,MODEECO_WARM].
#define WARM BIASECO_WARM // 17 or 18 good for energy saving at maybe 1C below typical UK room temperatures (~19C in 2012).
#else // Default settings for DHW control.
#define BIASECO_WARM 55 // Target WARM temperature for ECO bias; must be in range [MODECOM_WARM,MAX_TARGET_C].
#define BIASCOM_WARM 65 // Target WARM temperature for Comfort bias; must be in range ]MIN_TARGET_C,MODEECO_WARM].
#define WARM BIASECO_WARM // 55C+ with boost to 60C+ for DHW Legionella control.
#endif

// Raise target by this many degrees in 'BAKE' mode (strictly positive).
#define BAKE_UPLIFT 5
// Maximum 'BAKE' minutes, ie time to crank heating up to BAKE setting (minutes, strictly positive, <255).
#define BAKE_MAX_M 30

// Initial minor setback degrees C (strictly positive).  Note that 1C heating setback may result in ~8% saving in UK.
#define SETBACK 1
// Full setback degrees C (strictly positive and significantly, ie several degrees, greater than SETBACK, less than MIN_TARGET_C).
// This must be less than MIN_TARGET_C to avoid problems with unsigned arithmetic.
#define SETBACK_FULL 3
// Prolonged inactivity time deemed to indicate room(s) really unoccupied to trigger full setback (minutes, strictly positive).
#define SETBACK_FULL_M 45

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
// Not persistent (computed at run-time).
uint8_t getTRVPercentOpen();

// Target temperature in Centigrade for this OpenTRV unit.
// Not persistent (computed at run-time).
uint8_t getTargetTempC();


#ifdef ENABLE_BOILER_HUB
// Get minimum on (and off) time for pointer (minutes); zero if not in hub mode.
uint8_t getMinBoilerOnMinutes();
// Set minimum on (and off) time for pointer (minutes); zero to disable hub mode.
// Suggested minimum of 4 minutes for gas combi; much longer for heat pumps for example.
void setMinBoilerOnMinutes(uint8_t mins);
#else
#define getMinBoilerOnMinutes() (0) // Always disabled.
#define setMinBoilerOnMinutes(mins) {} // Do nothing.
#endif
// True if in central hub/listen mode (possibly with local radiator also).
#define inHubMode() (0 != getMinBoilerOnMinutes())



// Compute target temperature and set heat demand for TRV and boiler.
// CALL APPROXIMATELY ONCE PER MINUTE TO ALLOW SIMPLE TIME-BASED CONTROLS.
// Inputs are inWarmMode(), isRoomLit().
// The inputs must be valid (and recent).
// Values set are targetTempC, TRVPercentOpen.
// This may also prepare data such as TX command sequences for the TRV, boiler, etc.
// This routine may take significant CPU time; no I/O is done, only internal state is updated.
// Returns true if valve target changed and thus messages may need to be recomputed/sent/etc.
bool computeTargetAndDemand();


// Sample statistics once per hour as background to simple monitoring and adaptive behaviour.
// Call this once per hour with fullSample==true, as near the end of the hour as possible;
// this will update the non-volatile stats record for the current hour.
// Optionally call this at a small (2--10) even number of evenly-spaced number of other times thoughout the hour
// with fullSample=false to sub-sample (and these may receive lower weighting or be ignored).
// (EEPROM wear should not be an issue at this update rate in normal use.)
void sampleStats(bool fullSample);

// Clear all collected statistics, eg when moving device to a new room or at a major time change.
// Requires 1.8ms per byte for each byte that actually needs erasing.
//   * maxBytesToErase limit the number of bytes erased to this; strictly positive, else 0 to allow 65536
// Returns true if finished with all bytes erased.
bool zapStats(uint16_t maxBytesToErase = 0);

// Get raw stats value for hour HH [0,23] from stats set N from non-volatile (EEPROM) store.
// A value of 0xff (255) means unset (or out of range); other values depend on which stats set is being used.
uint8_t getByHourStat(uint8_t hh, uint8_t statsSet);

// 'Unset'/invalid values for byte (eg raw EEPROM byte) and int (eg after decompression).
#define STATS_UNSET_BYTE 0xff
#define STATS_UNSET_INT 0x7fff

// Returns true iff room likely to be occupied and need warming at the specified hour's sample point based on collected stats.
// Used for predictively warming a room in smart mode and for choosing setback depths.
// Returns false if no good evidence to warm the room at the given time based on past history over about one week.
//   * hh hour to check for predictive warming [0,23]
bool shouldBeWarmedAtHour(const uint_least8_t hh);



#ifdef UNIT_TESTS
// Compute new linearly-smoothed value given old smoothed value and new value.
// Guaranteed not to produce a value higher than the max of the old smoothed value and the new value.
// Uses stochastic rounding to nearest to allow nominally sub-lsb values to have an effect over time.
// Usually only made public for unit testing.
uint8_t smoothStatsValue(uint8_t oldSmoothed, uint8_t newValue);
#endif

// Range-compress an signed int 16ths-Celsius temperature to a unsigned single-byte value < 0xff.
// This preserves at least the first bit after the binary point for all values,
// and three bits after binary point for values in the most interesting mid range around normal room temperatures,
// with transitions at whole degrees Celsius.
// Input values below 0C are treated as 0C, and above 100C as 100C, thus allowing air and DHW temperature values.
#define COMPRESSION_C16_FLOOR_VAL 0 // Floor input value to compression.
#define COMPRESSION_C16_LOW_THRESHOLD (16<<4) // Values in range [COMPRESSION_LOW_THRESHOLD_C16,COMPRESSION_HIGH_THRESHOLD_C16[ have maximum precision.
#define COMPRESSION_C16_LOW_THR_AFTER (COMPRESSION_C16_LOW_THRESHOLD>>3) // Low threshold after compression.
#define COMPRESSION_C16_HIGH_THRESHOLD (24<<4)
#define COMPRESSION_C16_HIGH_THR_AFTER (COMPRESSION_C16_LOW_THR_AFTER + ((COMPRESSION_C16_HIGH_THRESHOLD-COMPRESSION_C16_LOW_THRESHOLD)>>1)) // High threshold after compression.
#define COMPRESSION_C16_CEIL_VAL (100<<4) // Ceiling input value to compression.
#define COMPRESSION_C16_CEIL_VAL_AFTER (COMPRESSION_C16_HIGH_THR_AFTER + ((COMPRESSION_C16_CEIL_VAL-COMPRESSION_C16_HIGH_THRESHOLD) >> 3)) // Ceiling input value after compression.
uint8_t compressTempC16(int tempC16);
// Reverses range compression done by compressTempC16(); results in range [0,100], with varying precision based on original value.
// 0xff (or other invalid) input results in STATS_UNSET_INT. 
int expandTempC16(uint8_t cTemp);

// Maximum valid encoded/compressed stats values.
#define MAX_STATS_TEMP COMPRESSION_C16_CEIL_VAL_AFTER // Maximum valid compressed temperature value in stats.
#define MAX_STATS_AMBLIGHT 254 // Maximum valid ambient light value in stats (very top of range is compressed).


#endif

