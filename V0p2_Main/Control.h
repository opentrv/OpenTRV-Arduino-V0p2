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

#include "V0p2_Main.h"
#include "Messaging.h"
#include <OTV0p2Base.h>

// Special setup for OpenTRV beyond generic hardware setup.
void setupOpenTRV();

// Main loop for OpenTRV radiator control.
void loopOpenTRV();


// Minimum and maximum bounds target temperatures; degrees C/Celsius/centigrade, strictly positive.
// Minimum is some way above 0C to avoid freezing pipework even with small measurement errors and non-uniform temperatures.
// Maximum is set a little below boiling/100C for DHW applications for safety.
// Setbacks and uplifts cannot move temperature targets outside this range for safety.
#define MIN_TARGET_C 5 // Minimum temperature setting allowed (to avoid freezing, allowing for offsets at temperature sensor, etc). 
#define MAX_TARGET_C 95 // Maximum temperature setting allowed (eg for DHW).


// Default frost (minimum) temperature in degrees C, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Setting frost temperatures at a level likely to protect (eg) fridge/freezers as well as water pipes.
// Note that 5C or below carries a risk of hypothermia: http://ipc.brookes.ac.uk/publications/pdf/Identifying_the_health_gain_from_retirement_housing.pdf
// Other parts of the room may be somewhat colder than where the sensor is, so aim a little over 5C.
// 14C avoids risk of raised blood pressure and is a generally safe and comfortable sleeping temperature.
// Note: BS EN 215:2004 S5.3.5 says maximum setting must be <= 32C, minimum in range [5C,12C].
// 15C+ may help mould/mold risk from condensation, see: http://www.nea.org.uk/Resources/NEA/Publications/2013/Resource%20-%20Dealing%20with%20damp%20and%20condensation%20%28lo%20res%29.pdf
#define BIASECO_FROST (max(6,MIN_TARGET_C)) // Target FROST temperature for ECO bias; must be in range [MIN_TARGET_C,BIASCOM_FROST[.
#define BIASCOM_FROST (max(14,MIN_TARGET_C)) // Target FROST temperature for Comfort bias; must be in range ]BIASECO_FROST,MAX_TARGET_C].
#define FROST BIASECO_FROST
// 18C is a safe room temperature even for the slightly infirm according to NHS England 2014:
//    http://www.nhs.uk/Livewell/winterhealth/Pages/KeepWarmKeepWell.aspx
// so could possibly be marked explicitly on the control.
// 21C is recommended living temperature in retirement housing:
//     http://ipc.brookes.ac.uk/publications/pdf/Identifying_the_health_gain_from_retirement_housing.pdf
#define SAFE_ROOM_TEMPERATURE 18 // Safe for most purposes.
// Default warm/comfort room (air) temperature in degrees C; strictly greater than FROST, in range [MIN_TARGET_C,MAX_TARGET_C].
// Control loop effectively targets upper end of this 1C window as of 20130518, middle as of 20141209.

#ifndef DHW_TEMPERATURES // Settings for room TRV.
// Set so that mid-point is at ~19C (BRE and others regard this as minimum comfort temperature)
// and half the scale will be below 19C and thus save ('eco') compared to typical UK room temperatures.
// (17/18 good for energy saving at ~1C below typical UK room temperatures of ~19C in 2012).
// Note: BS EN 215:2004 S5.3.5 says maximum setting must be <= 32C, minimum in range [5C,12C].
#define BIASECO_WARM 17 // Target WARM temperature for ECO bias; must be in range ]BIASCOM_FROST+1,BIASCOM_WARM[.
#define BIASCOM_WARM 21 // Target WARM temperature for Comfort bias; must be in range ]BIASECO_WARM,MAX_TARGET_C-BAKE_UPLIFT-1].
#else // Default settings for DHW control.
// 55C+ centre value with boost to 60C+ for DHW Legionella control where needed.
// Note that the low end (~45C) is safe against scalding but may worry some for storage as a Legionella risk.
#define BIASECO_WARM 45 // Target WARM temperature for ECO bias; must be in range [MODECOM_WARM,MAX_TARGET_C].
#define BIASCOM_WARM 65 // Target WARM temperature for Comfort bias; must be in range ]MIN_TARGET_C,MODEECO_WARM].
#endif
// Default to a 'safe' temperature.
#define WARM max(BIASECO_WARM,SAFE_ROOM_TEMPERATURE)

// Scale can run from eco warm -1 to comfort warm + 1, eg: * 16 17 18 >19< 20 21 22 BOOST
#define TEMP_SCALE_MIN (BIASECO_WARM-1) // Bottom of range for adjustable-base-temperature systems.
#define TEMP_SCALE_MID ((BIASECO_WARM + BIASCOM_WARM + 1)/2) // Middle of range for adjustable-base-temperature systems; should be 'eco' baised.
#define TEMP_SCALE_MAX (BIASCOM_WARM+1) // Top of range for adjustable-base-temperature systems.

// Raise target by this many degrees in 'BAKE' mode (strictly positive).
#define BAKE_UPLIFT 5
// Maximum 'BAKE' minutes, ie time to crank heating up to BAKE setting (minutes, strictly positive, <255).
#define BAKE_MAX_M 30

// Initial minor setback degrees C (strictly positive).  Note that 1C heating setback may result in ~8% saving in the UK.
// This may be the maximum setback applied with a comfort bias for example.
#define SETBACK_DEFAULT 1
// Enhanced setback, eg in eco mode, for extra energy savings.  Not more than SETBACK_FULL.
#define SETBACK_ECO (1+SETBACK_DEFAULT)
// Full setback degrees C (strictly positive and significantly, ie several degrees, greater than SETBACK_DEFAULT, less than MIN_TARGET_C).
// Deeper setbacks increase energy savings at the cost of longer times to return to target temperatures.
// See also (recommending 13F/7C setback to 55F/12C): https://www.mge.com/images/pdf/brochures/residential/setbackthermostat.pdf
// See also (suggesting for an 8hr setback, 1F set-back = 1% energy savings): http://joneakes.com/jons-fixit-database/1270-How-far-back-should-a-set-back-thermostat-be-set
// This must set back to no more than than MIN_TARGET_C to avoid problems with unsigned arithmetic.
#define SETBACK_FULL 4
// Prolonged inactivity time deemed to indicate room(s) really unoccupied to trigger full setback (minutes, strictly positive).
#define SETBACK_FULL_M min(60, max(30, OTV0P2BASE::PseudoSensorOccupancyTracker::OCCUPATION_TIMEOUT_M))


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


// Forcing the warm mode to the specified state immediately.
// Iff forcing to FROST mode then any pending BAKE time is cancelled,
// else BAKE status is unchanged.
// Should be only be called once 'debounced' if coming from a button press for example.
// Is safe to call repeatedly from test routines, eg does not cause EEPROM wear.
void setWarmModeDebounced(const bool warm);
// If true then the unit is in 'warm' (heating) mode, else 'frost' protection mode.
// This is a 'debounced' value to reduce accidental triggering.
bool inWarmMode();
// Force to BAKE mode.
// Should ideally be only be called once 'debounced' if coming from a button press for example.
// Is safe to call repeatedly from test routines, eg does not cause EEPROM wear.
// Is thread-/ISR- safe.
void startBake();
// If true then the unit is in 'bake' mode, a subset of 'warm' mode which boosts the temperature target temporarily.
// This is a 'debounced' value to reduce accidental triggering.
bool inBakeMode();
// Should be only be called once 'debounced' if coming from a button press for example.
// Cancel 'bake' mode if active: does not force to FROST mode.
void cancelBakeDebounced();


#if defined(UNIT_TESTS)
// Support for unit tests to force particular apparent WARM setting (without EEPROM writes).
enum _TEST_basetemp_override
  {
    _btoUT_normal = 0, // No override
    _btoUT_min, // Minimum settable/reasonable temperature.
    _btoUT_mid, // Medium settable/reasonable temperature.
    _btoUT_max, // Minimum settable/reasonable temperature.
  };
#define _TEST_basetemp_override_MAX _btoUT_max // Max legit _TEST_basetemp_override_MAX value.
// Set the override value (or remove the override).
void _TEST_set_basetemp_override(_TEST_basetemp_override override);
#endif

// If true (the default) then the system has an 'Eco' energy-saving bias, else it has a 'comfort' bias.
// Several system parameters are adjusted depending on the bias,
// with 'eco' slanted toward saving energy, eg with lower target temperatures and shorter on-times.
// This is determined from user-settable temperature values.
bool hasEcoBias();

// Get (possibly dynamically-set) thresholds/parameters.
//#if defined(ENABLE_SETTABLE_TARGET_TEMPERATURES) || defined(TEMP_POT_AVAILABLE)
// Get 'FROST' protection target in C; no higher than getWARMTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Depends dynamically on current (last-read) temp-pot setting.
uint8_t getFROSTTargetC();
// Get 'WARM' target in C; no lower than getFROSTTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Depends dynamically on current (last-read) temp-pot setting.
uint8_t getWARMTargetC();
//#endif

#if defined(TEMP_POT_AVAILABLE)
// Expose internal calculation of WARM target based on user physical control for unit testing.
// Derived from temperature pot position, 0 for coldest (most eco), 255 for hotest (comfort).
// Temp ranges from eco-1C to comfort+1C levels across full (reduced jitter) [0,255] pot range.
// Everything beyond the lo/hi end-stop thresholds is forced to the appropriate end temperature.
uint8_t computeWARMTargetC(uint8_t pot, uint8_t loEndStop, uint8_t hiEndStop);
#endif


#if defined(ENABLE_SETTABLE_TARGET_TEMPERATURES)
// Set (non-volatile) 'FROST' protection target in C; no higher than getWARMTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Can also be used, even when a temperature pot is present, to set a floor setback temperature.
// Returns false if not set, eg because outside range [MIN_TARGET_C,MAX_TARGET_C], else returns true.
bool setFROSTTargetC(uint8_t tempC);
#endif
#if defined(ENABLE_SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
// Set 'WARM' target in C; no lower than getFROSTTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Returns false if not set, eg because below FROST setting or outside range [MIN_TARGET_C,MAX_TARGET_C], else returns true.
bool setWARMTargetC(uint8_t tempC);
#endif

// True if specified temperature is at or below 'eco' WARM target temperature, ie is eco-friendly.
#define isEcoTemperature(tempC) ((tempC) <= BIASECO_WARM)
// True if specified temperature is at or above 'comfort' WARM target temperature.
#define isComfortTemperature(tempC) ((tempC) >= BIASCOM_WARM)


#if defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX) || defined(ENABLE_DEFAULT_ALWAYS_RX)
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
// Single generic occupancy callback for occupied for this instance.
void genericMarkAsOccupied();
// Single generic occupancy callback for 'possibly occupied' for this instance.
void genericMarkAsPossiblyOccupied();

#if defined(ENABLE_SINGLETON_SCHEDULE)
#define SCHEDULER_AVAILABLE
// Customised scheduler for the current OpenTRV application.
class SimpleValveSchedule : public OTV0P2BASE::SimpleValveScheduleBase
    {
    public:
        // Allow scheduled on time to dynamically depend on comfort level.
        virtual uint8_t onTime()
            {
#if LEARNED_ON_PERIOD_M == LEARNED_ON_PERIOD_COMFORT_M
            // Simplify the logic where no variation in on time is required.
            return(LEARNED_ON_PERIOD_M);
#else // LEARNED_ON_PERIOD_M != LEARNED_ON_PERIOD_COMFORT_M
            // Variable 'on' time depending on how 'eco' the settings are.
            // Three-way split based on current WARM target temperature,
            // for a relatively gentle change in behaviour along the valve dial for example.
            const uint8_t wt = getWARMTargetC();
            if(isEcoTemperature(wt)) { return(LEARNED_ON_PERIOD_M); }
            else if(isComfortTemperature(wt)) { return(LEARNED_ON_PERIOD_COMFORT_M); }
#if defined(ENABLE_OCCUPANCY_SUPPORT)
            // If vacant for a long time (>1d) and not at maximum comfort end of scale
            // then truncate the on period to the minimum to attempt to save energy.
            else if(Occupancy.longVacant()) { return(LEARNED_ON_PERIOD_M); }
#endif
            else { return((LEARNED_ON_PERIOD_M + LEARNED_ON_PERIOD_COMFORT_M) / 2); }
#endif // LEARNED_ON_PERIOD_M == LEARNED_ON_PERIOD_COMFORT_M
            }
    };
// Singleton scheduler instance.
extern SimpleValveSchedule Scheduler;
#else
// Dummy scheduler to simplify coding.
extern OTV0P2BASE::NULLValveSchedule Scheduler;
#endif // defined(ENABLE_SINGLETON_SCHEDULE)


#if defined(ENABLE_LOCAL_TRV)
#define ENABLE_MODELLED_RAD_VALVE
// Internal model of radiator valve position, embodying control logic.
class ModelledRadValve : public OTRadValve::AbstractRadValve
  {
  private:
    // All input state for deciding where to set the radiator valve in normal operation.
    struct OTRadValve::ModelledRadValveInputState inputState;
    // All retained state for deciding where to set the radiator valve in normal operation.
    struct OTRadValve::ModelledRadValveState retainedState;

    // True if this node is calling for heat.
    // Marked volatile for thread-safe lock-free access.
    volatile bool callingForHeat;

    // True if the room/ambient temperature is below target, enough to likely call for heat.
    // Marked volatile for thread-safe lock-free access.
    volatile bool underTarget;

    // True if in glacial mode.
    // TODO: not fully implemented.
    bool glacial;

    // Cache of minValvePcReallyOpen value [0,99] to save some EEPROM access.
    // A value of 0 means not yet loaded from EEPROM.
    static uint8_t mVPRO_cache;

    // Compute target temperature and set heat demand for TRV and boiler; update state.
    // CALL REGULARLY APPROXIMATELY ONCE PER MINUTE TO ALLOW SIMPLE TIME-BASED CONTROLS.
    // Inputs are inWarmMode(), isRoomLit().
    // The inputs must be valid (and recent).
    // Values set are targetTempC, value (TRVPercentOpen).
    // This may also prepare data such as TX command sequences for the TRV, boiler, etc.
    // This routine may take significant CPU time; no I/O is done, only internal state is updated.
    // Returns true if valve target changed and thus messages may need to be recomputed/sent/etc.
    void computeCallForHeat();

  public:
    ModelledRadValve()
      : inputState(0),
        callingForHeat(false), underTarget(false),
#if defined(TRV_SLEW_GLACIAL)
        glacial(true)
#else
        glacial(false)
#endif
      { }

    // Force a read/poll/recomputation of the target position and call for heat.
    // Sets/clears changed flag if computed valve position changed.
    // Call at a fixed rate (1/60s).
    // Potentially expensive/slow.
    virtual uint8_t read() { computeCallForHeat(); return(value); }

    // Returns preferred poll interval (in seconds); non-zero.
    // Must be polled at near constant rate, about once per minute.
    virtual uint8_t preferredPollInterval_s() const { return(60); }

    // Returns a suggested (JSON) tag/field/key name including units of get(); NULL means no recommended tag.
    // The lifetime of the pointed-to text must be at least that of the Sensor instance.
    virtual const char *tag() const { return("v|%"); }


    // Returns true if (re)calibrating/(re)initialising/(re)syncing.
    // The target valve position is not lost while this is true.
    // By default there is no recalibration step.
    virtual bool isRecalibrating() const;

    // If possible exercise the valve to avoid pin sticking and recalibrate valve travel.
    // Default does nothing.
    virtual void recalibrate();

    // True if the controlled physical valve is thought to be at least partially open right now.
    // If multiple valves are controlled then is this true only if all are at least partially open.
    // Used to help avoid running boiler pump against closed valves.
    // The default is to use the check the current computed position
    // against the minimum open percentage.
    // True iff the valve(s) (if any) controlled by this unit are really open.
    //
    // When driving a remote wireless valve such as the FHT8V,
    // this waits until at least the command has been sent.
    // This also implies open to OTRadValve::DEFAULT_VALVE_PC_MIN_REALLY_OPEN or equivalent.
    // Must be exactly one definition/implementation supplied at link time.
    // If more than one valve is being controlled by this unit,
    // then this should return true if any of the valves are (significantly) open.
    virtual bool isControlledValveReallyOpen() const;

    // Get estimated minimum percentage open for significant flow [1,99] for this device.
    // Return global node value.
    virtual uint8_t getMinPercentOpen() const { return(getMinValvePcReallyOpen()); }

    // Get maximum allowed percent open [1,100] to limit maximum flow rate.
    // This may be important for systems such as district heat systems that charge by flow,
    // and other systems that prefer return temperatures to be as low as possible,
    // such as condensing boilers.
#if defined(TRV_MAX_PC_OPEN)
    uint8_t getMaxPercentageOpenAllowed() const { return(TRV_MAX_PC_OPEN); }
#else
    uint8_t getMaxPercentageOpenAllowed() const { return(100); } // TODO: make settable/persistent.
#endif

    // Enable/disable 'glacial' mode (default false/off).
    // For heat-pump, district-heating and similar slow-reponse and pay-by-volume environments.
    // Also may help with over-powerful or unbalanced radiators
    // with a significant risk of overshoot.
    void setGlacialMode(bool glacialOn) { glacial = glacialOn; }

    // Returns true if this valve control is in glacial mode.
    bool inGlacialMode() const { return(glacial); }

    // True if the computed valve position was changed by read().
    // Can be used to trigger rebuild of messages, force updates to actuators, etc.
    bool isValveMoved() const { return(retainedState.valveMoved); }

    // True if this unit is actively calling for heat.
    // This implies that the temperature is (significantly) under target,
    // the valve is really open,
    // and this needs more heat than can be passively drawn from an already-running boiler.
    // Thread-safe and ISR safe.
    bool isCallingForHeat() const { return(callingForHeat); }

    // True if the room/ambient temperature is below target, enough to likely call for heat.
    // This implies that the temperature is (significantly) under target,
    // the valve is really open,
    // and this needs more heat than can be passively drawn from an already-running boiler.
    // Thread-safe and ISR safe.
    bool isUnderTarget() const { return(underTarget); }

    // Get target temperature in C as computed by computeTargetTemperature().
    uint8_t getTargetTempC() const { return(inputState.targetTempC); }

    // Returns a suggested (JSON) tag/field/key name including units of getTargetTempC(); not NULL.
    // The lifetime of the pointed-to text must be at least that of this instance.
    const char *tagTTC() const { return("tT|C"); }

    // Stateless directly-testable version behind computeTargetTemperature().
    static uint8_t computeTargetTemp();

    // Compute/update target temperature and set up state for computeRequiredTRVPercentOpen().
    // Can be called as often as required though may be slowish/expensive.
    // Can be called after any UI/CLI/etc operation
    // that may cause the target temperature to change.
    // (Will also be called by computeCallForHeat().)
    // One aim is to allow reasonable energy savings (10--30%)
    // even if the device is left in WARM mode all the time,
    // using occupancy/light/etc to determine when temperature can be set back
    // without annoying users.
    //
    // Will clear any BAKE mode if the newly-computed target temperature is already exceeded.
    void computeTargetTemperature();

    // Computes optimal valve position given supplied input state including current position; [0,100].
    // Uses no state other than that passed as the arguments (thus unit testable).
    // This supplied 'retained' state may be updated.
    // Uses hysteresis and a proportional control and some other cleverness.
    // Is always willing to turn off quickly, but on slowly (AKA "slow start" algorithm),
    // and tries to eliminate unnecessary 'hunting' which makes noise and uses actuator energy.
    // Nominally called at a regular rate, once per minute.
    // All inputState values should be set to sensible values before starting.
    // Usually called by tick() which does required state updates afterwards.
    static uint8_t computeRequiredTRVPercentOpen(uint8_t valvePCOpen, const struct ModelledRadValveInputState &inputState, struct ModelledRadValveState &retainedState);

    // Get cumulative valve movement %; rolls at 8192 in range [0,8191], ie non-negative.
    uint16_t getCumulativeMovementPC() { return(retainedState.cumulativeMovementPC); }

    // Returns a suggested (JSON) tag/field/key name including units of getCumulativeMovementPC(); not NULL.
    // The lifetime of the pointed-to text must be at least that of this instance.
    const char *tagCMPC() const { return("vC|%"); }

    // Return minimum valve percentage open to be considered actually/significantly open; [1,100].
    // This is a value that has to mean all controlled valves are at least partially open if more than one valve.
    // At the boiler hub this is also the threshold percentage-open on eavesdropped requests that will call for heat.
    // If no override is set then OTRadValve::DEFAULT_VALVE_PC_MIN_REALLY_OPEN is used.
    static uint8_t getMinValvePcReallyOpen();

    // Set and cache minimum valve percentage open to be considered really open.
    // Applies to local valve and, at hub, to calls for remote calls for heat.
    // Any out-of-range value (eg >100) clears the override and OTRadValve::DEFAULT_VALVE_PC_MIN_REALLY_OPEN will be used.
    static void setMinValvePcReallyOpen(uint8_t percent);
  };
#define ENABLE_NOMINAL_RAD_VALVE
// Singleton implementation for entire node.
extern ModelledRadValve NominalRadValve;
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


#ifdef UNIT_TESTS
// Compute new linearly-smoothed value given old smoothed value and new value.
// Guaranteed not to produce a value higher than the max of the old smoothed value and the new value.
// Uses stochastic rounding to nearest to allow nominally sub-lsb values to have an effect over time.
// Usually only made public for unit testing.
uint8_t smoothStatsValue(uint8_t oldSmoothed, uint8_t newValue);
#endif

#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Clear and populate core stats structure with information from this node.
// Exactly what gets filled in will depend on sensors on the node,
// and may depend on stats TX security level (if collecting some sensitive items is also expensive).
void populateCoreStats(OTV0P2BASE::FullStatsMessageCore_t *content);
#endif // ENABLE_FS20_ENCODING_SUPPORT

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
