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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2014
*/

/*
 Control/model for TRV and boiler.
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#include "V0p2_Main.h"
#include "Temp_Pot.h"

// Minimum and maximum bounds target temperatures; degrees C/Celsius/centigrade, strictly positive.
// Minimum is some way above 0C to avoid freezing pipework even with small measurement errors and non-uniform temperatures.
// Maximum is set a little below boiling/100C for DHW applications for safety.
// Setbacks and uplifts cannot move temperature targets outside this range for safety.
#define MIN_TARGET_C 5 // Minimum temperature setting allowed (to avoid freezing, allowing for offsets at temperature sensor, etc). 
#define MAX_TARGET_C 95 // Maximum temperature setting allowed (eg for DHW).


// Default frost (minimum) temperature in degrees C, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
#define BIASECO_FROST MIN_TARGET_C // Target FROST temperature for ECO bias; must be in range [MIN_TARGET_C,BIASCOM_FROST[.
#define BIASCOM_FROST (5+MIN_TARGET_C) // Target FROST temperature for Comfort bias; must be in range ]BIASECO_FROST,MAX_TARGET_C].
#define FROST BIASECO_FROST
#ifndef DHW_TEMPERATURES
// Default warm/comfort room (air) temperature in degrees C; strictly greater than FROST, in range [MIN_TARGET_C,MAX_TARGET_C].
// Control loop effectively targets upper end of this 1C window as of 20130518.
#define BIASECO_WARM 17 // Target WARM temperature for ECO bias; must be in range ]BIASCOM_FROST+1,BIASCOM_WARM[.
#define BIASCOM_WARM 20 // Target WARM temperature for Comfort bias; must be in range ]BIASECO_WARM,MAX_TARGET_C-BAKE_UPLIFT-1].
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

#ifdef LEARN_BUTTON_AVAILABLE
// Period in minutes for simple learned on-time; strictly positive (and less than 256).
#define LEARNED_ON_PERIOD_M 60
// Period in minutes for simple learned on-time with comfort bias; strictly positive (and less than 256).
#define LEARNED_ON_PERIOD_COMFORT_M 120
#endif



// If true (the default) then the system has an 'Eco' energy-saving bias, else it has a 'comfort' bias.
// Several system parameters are adjusted depending on the bias,
// with 'eco' slanted toward saving energy, eg with lower target temperatures and shorter on-times.
// This is determined from user-settable temperature values.
bool hasEcoBias();


// Get dynamically-set thresholds/parameters.
#if defined(SETTABLE_TARGET_TEMPERATURES) || defined(TEMP_POT_AVAILABLE)
// Get 'FROST' protection target in C; no higher than getWARMTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Depends dynamically on current (last-read) temp-pot setting.
uint8_t getFROSTTargetC();
// Get 'WARM' target in C; no lower than getFROSTTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Depends dynamically on current (last-read) temp-pot setting.
uint8_t getWARMTargetC();
#endif

#if defined(SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
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


// Typical minimum valve percentage open to be considered actually/significantly open; [1,100].
// Setting this above 0 delays calling for heat from a central boiler until water is likely able to flow.
// (It may however be possible to scavenge some heat if a particular valve opens below this and the circulation pump is already running, for example.)
// DHD20130522: FHT8V + valve heads I have been using are not typically upen until around 6%.
// Allowing valve to linger at just below this level without calling for heat when shutting
// may allow comfortable bolier pump overrun in older systems with no/poor bypass to avoid overheating.
#define DEFAULT_MIN_VALVE_PC_REALLY_OPEN 10

// Return minimum valve percentage open to be considered actually/significantly open; [1,100].
// At the boiler hub this is also the threshold precentage-open on eavesdropped requests that will call for heat.
// If no override is set then DEFAULT_MIN_VALVE_PC_REALLY_OPEN is used.
uint8_t getMinValvePcReallyOpen();

// Set percent open to be considered really open.
// Applies to local valve and, at hub, to calls for remote calls for heat.
// Any out-of-range value (eg >100) clears the override and DEFAULT_MIN_VALVE_PC_REALLY_OPEN will be used.
void setMinValvePcReallyOpen(uint8_t percent);

// Default maximum time to allow the boiler to run on to allow for lost call-for-heat transmissions etc.
// Should be (much) greater than the gap between transmissions (eg ~2m for FHT8V/FS20).
// Should be greater than the run-on time at the OpenTRV boiler unit and any further pump run-on time.
// Valves may have to linger open at minimum of this plus maybe an extra minute or so for timing skew
// for systems with poor/absent bypass to avoid overheating.
// Having too high a linger time value will cause excessive temperature overshoot.
#define DEFAULT_MAX_RUN_ON_TIME_M 6

// If defined then turn off valve very slowly after stopping call for heat (ie when shutting) which
// may allow comfortable bolier pump overrun in older systems with no/poor bypass to avoid overheating.
// In any case this should help reduce strain on circulation pumps, etc.
#define VALVE_TURN_OFF_LINGER


// True iff the valve(s) (if any) controlled by this unit are really open.
// This waits until, for example, an ACK where appropriate, or at least the command has been sent.
// This also implies open to DEFAULT_MIN_VALVE_PC_REALLY_OPEN or equivalent.
// Must be exactly one definition/implementation supplied at link time.
// If more than one valve is being controlled by this unit,
// then this should return true if anyof the valves are (significantly) open.
bool isControlledValveOpen();

// Compute target temperature.
// Can be called as often as require though may be slow/expensive.
// Will be called by computeCallForHeat().
void computeTargetTemperature();

// Compute target temperature and set heat demand for TRV and boiler.
// CALL APPROXIMATELY ONCE PER MINUTE TO ALLOW SIMPLE TIME-BASED CONTROLS.
// Inputs are inWarmMode(), isRoomLit().
// The inputs must be valid (and recent).
// Values set are targetTempC, TRVPercentOpen.
// This may also prepare data such as TX command sequences for the TRV, boiler, etc.
// This routine may take significant CPU time; no I/O is done, only internal state is updated.
// Returns true if valve target changed and thus messages may need to be recomputed/sent/etc.
bool computeCallForHeat();


// Returns true if system is in 'learn'/smart mode.
// If in 'smart' mode can anticipate user demand to pre-warm rooms, maintain customary temperatures, etc.
bool inSmartMode();


// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#ifdef OCCUPANCY_SUPPORT
// Returns true if the room appears to be likely occupied (with active users) recently.
// This uses the same timer as isLikelyOccupied() (restarted by markAsOccupied())
// but returns to false somewhat sooner for example to allow ramping up more costly occupancy detection methods
// and to allow some simple graduated occupancy responses.
// Do not call from an ISR.
bool isLikelyRecentlyOccupied();

// Returns true if the estimated likelyhood of occupancy is deminishing
// and expending effort above a basic level to check for continuing occupancy is worthwhile.
#define increaseCheckForOccupancy() (!isLikelyRecentlyOccupied() && isLikelyOccupied())

// Returns true if the room appears to be likely occupied (with active users) now or recently.
// Operates on a timeout; calling markAsOccupied() restarts the timer.
// Defaults to false (and API still exists) when OCCUPANCY_SUPPORT not defined.
// Do not call from an ISR.
bool isLikelyOccupied();

// False if room likely currently unoccupied (no active users).
// Defaults to false (and API still exists) when OCCUPANCY_SUPPORT not defined.
// This may require a substantial timeout (many hours) of inactivity to become true.
// This and isLikelyOccupied() cannot be true together; it is possible for neither to be true.
// Do not call from an ISR.
#define isLikelyUnoccupied() (!isLikelyOccupied())

// Call when some strong evidence of room occupation and human activity has occurred.
// Such evidence may include operation of buttons (etc) on the unit or PIR.
// Do not call from (for example) 'on' schedule change.
// Do not call from an ISR.
void markAsOccupied();
// Call when some/weak evidence of room occupation, such as light going on.
// Also use to simulate demand on behalf of user, eg for some part of schedule.
// Do not call from an ISR.
void markAsPossiblyOccupied();

#else
#define markAsOccupied() {} // Defined as NO-OP for convenience when no general occupancy support.
#define markAsPossiblyOccupied() {} // Defined as NO-OP for convenience when no general occupancy support.
#define isLikelyOccupied() (false) // Always false without OCCUPANCY_SUPPORT
#define isLikelyUnoccupied() (false) // Always false without OCCUPANCY_SUPPORT
#endif



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

