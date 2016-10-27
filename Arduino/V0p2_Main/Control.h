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
#include "V0p2_Sensors.h"

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


// WIP: temperature control object.
// Choose which subtype to use depending on enabled settings and board type.
#if defined(TEMP_POT_AVAILABLE) // Eg REV2/REV7.
  #if 0 && defined(HUMIDITY_SENSOR_SUPPORT) // Humidity sensing available.
  typedef OTRadValve::TempControlTempPot<&TempPot, static_cast<const OTV0P2BASE::HumiditySensorBase*>(&RelHumidity), PARAMS> TempControl_t;
  #else
  typedef OTRadValve::TempControlTempPot<&TempPot, static_cast<const OTV0P2BASE::HumiditySensorBase*>(NULL), PARAMS> TempControl_t;
  #endif // HUMIDITY_SENSOR_SUPPORT
#elif defined(ENABLE_SETTABLE_TARGET_TEMPERATURES) // Eg REV1.
typedef OTRadValve::TempControlSimpleEEPROMBacked<PARAMS> TempControl_t;
#else
#error No temperature control type selected.
// typedef OTRadValve::TempControlBase TempControl_t;
#endif
// defined(TEMP_POT_AVAILABLE) .. defined(ENABLE_SETTABLE_TARGET_TEMPERATURES)
extern TempControl_t tempControl;


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
#define isEcoTemperature(tempC) ((tempC) <= PARAMS::WARM_ECO)
// True if specified temperature is at or above 'comfort' WARM target temperature.
#define isComfortTemperature(tempC) ((tempC) >= PARAMS::WARM_COM)


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

    // The current automated setback (if any) in the direction of energy saving in C; non-negative.
    // Not intended for ISR/threaded access.
    uint8_t setbackC;

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
        setbackC(0),
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
    virtual bool isCallingForHeat() const { return(callingForHeat); }

    // True if the room/ambient temperature is below target, enough to likely call for heat.
    // This implies that the temperature is (significantly) under target,
    // the valve is really open,
    // and this needs more heat than can be passively drawn from an already-running boiler.
    // Thread-safe and ISR safe.
    virtual bool isUnderTarget() const { return(underTarget); }

    // Get target temperature in C as computed by computeTargetTemperature().
    uint8_t getTargetTempC() const { return(inputState.targetTempC); }

    // Returns a suggested (JSON) tag/field/key name including units of getTargetTempC(); not NULL.
    // The lifetime of the pointed-to text must be at least that of this instance.
    const char *tagTTC() const { return("tT|C"); }

    // Get the current automated setback (if any) in the direction of energy saving in C; non-negative.
    // For heating this is the number of C below the nominal user-set target temperature
    // that getTargetTempC() is; zero if no setback is in effect.
    // Generally will be 0 in FROST or BAKE modes.
    // Not ISR-/thread- safe.
    uint8_t getSetbackC() const { return(setbackC); }

    // Returns a suggested (JSON) tag/field/key name including units of getSetbackC(); not NULL.
    // It would often be appropriate to mark this as low priority since depth of setback matters more than speed.
    // The lifetime of the pointed-to text must be at least that of this instance.
    const char *tagTSC() const { return("tS|C"); }

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
    // It would often be appropriate to mark this as low priority since it can be computed from valve positions.
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
