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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2016
*/

/*
 Control/model for TRV and boiler.
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

#include <OTV0p2Base.h>

#include "V0p2_Main.h"
#include "Messaging.h"


// Special setup for OpenTRV beyond generic hardware setup.
void setupOpenTRV();

// Main loop for OpenTRV radiator control.
void loopOpenTRV();


// Select basic parameter set to use (or could define new set here).
#ifndef DHW_TEMPERATURES
// Settings for room TRV.
typedef OTRadValve::DEFAULT_ValveControlParameters PARAMS;
#else
// Default settings for DHW control.
typedef OTRadValve::DEFAULT_DHW_ValveControlParameters PARAMS;
#endif


// Period in minutes for simple learned on-time; strictly positive (and less than 256).
#ifndef LEARNED_ON_PERIOD_M
#define LEARNED_ON_PERIOD_M 60
#endif
// Period in minutes for simple learned on-time with comfort bias; strictly positive (and less than 256).
// Defaults to twice LEARNED_ON_PERIOD_M.
// Should be no shorter/less than LEARNED_ON_PERIOD_M to avoid confusion.
#ifndef LEARNED_ON_PERIOD_COMFORT_M
#define LEARNED_ON_PERIOD_COMFORT_M (min(2*(LEARNED_ON_PERIOD_M),255))
#endif


// Radiator valve mode (FROST, WARM, BAKE).
extern OTRadValve::ValveMode valveMode;


// Choose which subtype to use depending on enabled settings and board type.
#if defined(TEMP_POT_AVAILABLE) // Eg REV2/REV7.
  #if defined(HUMIDITY_SENSOR_SUPPORT)
  typedef OTRadValve::TempControlTempPot<&TempPot, PARAMS, decltype(RelHumidity), &RelHumidity> TempControl_t;
  #else
  typedef OTRadValve::TempControlTempPot<&TempPot, PARAMS> TempControl_t;
  #endif
#elif defined(ENABLE_SETTABLE_TARGET_TEMPERATURES) // Eg REV1.
typedef OTRadValve::TempControlSimpleEEPROMBacked<PARAMS> TempControl_t;
#else
// Dummy temperature control.
typedef OTRadValve::TempControlBase TempControl_t;
#endif
extern TempControl_t tempControl;


// Default minimum on/off time in minutes for the boiler relay.
// Set to 5 as the default valve Tx cycle is 4 mins and 5 mins is a good amount for most boilers.
// This constant is necessary as if V0P2BASE_EE_START_MIN_BOILER_ON_MINS_INV is not set, the boiler relay will never be turned on.
static const constexpr uint8_t DEFAULT_MIN_BOILER_ON_MINS = 5;
#if defined(ENABLE_DEFAULT_ALWAYS_RX)
#define getMinBoilerOnMinutes() (DEFAULT_MIN_BOILER_ON_MINS)
#elif defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)
// Get minimum on (and off) time for pointer (minutes); zero if not in hub mode.
uint8_t getMinBoilerOnMinutes();
// Set minimum on (and off) time for pointer (minutes); zero to disable hub mode.
// Suggested minimum of 4 minutes for gas combi; much longer for heat pumps for example.
void setMinBoilerOnMinutes(uint8_t mins);
#else
#define getMinBoilerOnMinutes() (0) // Always disabled.
#define setMinBoilerOnMinutes(mins) {} // Do nothing.
#endif

#if defined(ENABLE_DEFAULT_ALWAYS_RX)
// True: always in central hub/listen mode.
#define inHubMode() (true)
// True: always in stats hub/listen mode.
#define inStatsHubMode() (true)
#elif !defined(ENABLE_RADIO_RX)
// No RX/listening allowed, so never in hub mode.
// False: never in central hub/listen mode.
#define inHubMode() (false)
// False: never in stats hub/listen mode.
#define inStatsHubMode() (false)
#else
// True if in central hub/listen mode (possibly with local radiator also).
#define inHubMode() (0 != getMinBoilerOnMinutes())
// True if in stats hub/listen mode (minimum timeout).
#define inStatsHubMode() (1 == getMinBoilerOnMinutes())
#endif // defined(ENABLE_DEFAULT_ALWAYS_RX)

// FIXME Moved up from line 464 to fix compilation errors (OccupancyTracker needed on line 261)
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#ifdef ENABLE_OCCUPANCY_SUPPORT
typedef OTV0P2BASE::PseudoSensorOccupancyTracker OccupancyTracker;
#else
// Placeholder class with dummy static status methods to reduce code complexity.
typedef OTV0P2BASE::DummySensorOccupancyTracker OccupancyTracker;
#endif
// Singleton implementation for entire node.
extern OccupancyTracker Occupancy;


#if defined(ENABLE_SINGLETON_SCHEDULE)
#define SCHEDULER_AVAILABLE
// Singleton scheduler instance.
typedef OTV0P2BASE::SimpleValveSchedule<
    LEARNED_ON_PERIOD_M, LEARNED_ON_PERIOD_COMFORT_M,
    decltype(tempControl), &tempControl,
#if defined(ENABLE_OCCUPANCY_SUPPORT)
    decltype(Occupancy), &Occupancy
#endif
    > Scheduler_t;
#else
// Dummy scheduler to simplify coding.
typedef OTV0P2BASE::NULLValveSchedule Scheduler_t;
#endif // defined(ENABLE_SINGLETON_SCHEDULE)
extern Scheduler_t Scheduler;


#if defined(ENABLE_LOCAL_TRV)
#define ENABLE_MODELLED_RAD_VALVE
#define ENABLE_NOMINAL_RAD_VALVE
// Singleton implementation for entire node.
extern OTRadValve::ModelledRadValve NominalRadValve;
#elif defined(ENABLE_SLAVE_TRV)
#define ENABLE_NOMINAL_RAD_VALVE
// Simply alias directly to FHT8V for REV9 slave for example.
#define NominalRadValve FHT8V
#endif

// Sample statistics once per hour as background to simple monitoring and adaptive behaviour.
// Call this once per hour with fullSample==true, as near the end of the hour as possible;
// this will update the non-volatile stats record for the current hour.
// Optionally call this at a small (2--10) even number of evenly-spaced number of other times throughout the hour
// with fullSample=false to sub-sample (and these may receive lower weighting or be ignored).
// (EEPROM wear should not be an issue at this update rate in normal use.)
void sampleStats(bool fullSample);


#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Clear and populate core stats structure with information from this node.
// Exactly what gets filled in will depend on sensors on the node,
// and may depend on stats TX security level (if collecting some sensitive items is also expensive).
void populateCoreStats(OTV0P2BASE::FullStatsMessageCore_t *content);
#endif // ENABLE_FS20_ENCODING_SUPPORT

#ifdef ENABLE_SETBACK_LOCKOUT_COUNTDOWN // TODO Move this into OTRadioLink for mainline version.
    /**
     * @brief   Retrieve the current setback lockout value from the EEPROM.
     * @retval  The number of days left of the setback lockout. Setback lockout is disabled when this reaches 0.
     * @note    The value is stored inverted in EEPROM.
     * @note    This is stored as G 0 for TRV1.5 devices, but may change in future.
     */
    static inline uint8_t getSetbackLockout() { return ~(eeprom_read_byte((uint8_t *)OTV0P2BASE::V0P2BASE_EE_START_SETBACK_LOCKOUT_COUNTDOWN_D_INV)); }
#endif // ENABLE_SETBACK_LOCKOUT_COUNTDOWN

// Do bare stats transmission.
// Output should be filtered for items appropriate
// to current channel security and sensitivity level.
// This may be binary or JSON format.
//   * allowDoubleTX  allow double TX to increase chance of successful reception
//   * doBinary  send binary form if supported, else JSON form if supported
// Sends stats on primary radio channel 0 with possible duplicate to secondary channel.
// If sending encrypted then ID/counter fields (eg @ and + for JSON) are omitted
// as assumed supplied by security layer to remote recipent.
void bareStatsTX(bool allowDoubleTX, bool doBinary);

#ifdef ENABLE_BOILER_HUB
// Raw notification of received call for heat from remote (eg FHT8V) unit.
// This form has a 16-bit ID (eg FHT8V housecode) and percent-open value [0,100].
// Note that this may include 0 percent values for a remote unit explicitly confirming
// that is is not, or has stopped, calling for heat (eg instead of replying on a timeout).
// This is not filtered, and can be delivered at any time from RX data, from a non-ISR thread.
// Does not have to be thread-/ISR- safe.
void remoteCallForHeatRX(uint16_t id, uint8_t percentOpen);
#endif


#endif
