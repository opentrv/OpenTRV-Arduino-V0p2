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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2015
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
// 55C+ with boost to 60C+ for DHW Legionella control where needed.
#define BIASECO_WARM 55 // Target WARM temperature for ECO bias; must be in range [MODECOM_WARM,MAX_TARGET_C].
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

// Initial minor setback degrees C (strictly positive).  Note that 1C heating setback may result in ~8% saving in UK.
// This may be the maximum setback applied with a comfort bias for example.
#define SETBACK_DEFAULT 1
// Enhanced setback in full-on eco mode for extra energy savings.  Not more than FULL_SETBACK.
#define SETBACK_ECO (1+SETBACK_DEFAULT)
// Full setback degrees C (strictly positive and significantly, ie several degrees, greater than SETBACK, less than MIN_TARGET_C).
// This must set back to no more than than MIN_TARGET_C to avoid problems with unsigned arithmetic.
#define SETBACK_FULL 3
// Prolonged inactivity time deemed to indicate room(s) really unoccupied to trigger full setback (minutes, strictly positive).
#define SETBACK_FULL_M 50

#ifdef LEARN_BUTTON_AVAILABLE
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

#ifdef SUPPORT_BAKE // IF DEFINED: this unit supports BAKE mode.
// Force to BAKE mode;
// Should be only be called once 'debounced' if coming from a button press for example.
// Is safe to call repeatedly from test routines, eg does not cause EEPROM wear.
void startBakeDebounced();
//// If true then the unit is in 'bake' mode, a subset of 'warm' mode which boosts the temperature target temporarily.
//bool inBakeMode();
// If true then the unit is in 'bake' mode, a subset of 'warm' mode which boosts the temperature target temporarily.
// This is a 'debounced' value to reduce accidental triggering.
bool inBakeMode();
// Cancel 'bake' mode if active.
// Should be only be called once 'debounced' if coming from a button press for example.
void cancelBakeDebounced();
#else
#define startBakeDebounced() {}
// NO-OP versions if BAKE mode not supported.
//#define inBakeMode() (false)
#define inBakeModeDebounced() (false)
#define cancelBakeDebounced() {}
#endif




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
//#if defined(SETTABLE_TARGET_TEMPERATURES) || defined(TEMP_POT_AVAILABLE)
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
uint8_t computeWARMTargetC(const uint8_t pot);
#endif


#if defined(SETTABLE_TARGET_TEMPERATURES)
// Set (non-volatile) 'FROST' protection target in C; no higher than getWARMTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Can also be used, even when a temperature pot is present, to set a floor setback temperature.
// Returns false if not set, eg because outside range [MIN_TARGET_C,MAX_TARGET_C], else returns true.
bool setFROSTTargetC(uint8_t tempC);
#endif
#if defined(SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
// Set 'WARM' target in C; no lower than getFROSTTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Returns false if not set, eg because below FROST setting or outside range [MIN_TARGET_C,MAX_TARGET_C], else returns true.
bool setWARMTargetC(uint8_t tempC);
#endif

// True if specified temperature is at or below 'eco' WARM target temperature, ie is eco-friendly.
#define isEcoTemperature(tempC) ((tempC) <= BIASECO_WARM)
// True if specified temperature is at or above 'comfort' WARM target temperature.
#define isComfortTemperature(tempC) ((tempC) >= BIASCOM_WARM)


#if defined(ENABLE_BOILER_HUB) || defined(ALLOW_STATS_RX) || defined(ENABLE_DEFAULT_ALWAYS_RX)
// Get minimum on (and off) time for pointer (minutes); zero if not in hub mode.
uint8_t getMinBoilerOnMinutes();
// Set minimum on (and off) time for pointer (minutes); zero to disable hub mode.
// Suggested minimum of 4 minutes for gas combi; much longer for heat pumps for example.
void setMinBoilerOnMinutes(uint8_t mins);
#else
#define getMinBoilerOnMinutes() (0) // Always disabled.
#define setMinBoilerOnMinutes(mins) {} // Do nothing.
#endif

#ifdef ENABLE_DEFAULT_ALWAYS_RX
// True: always in central hub/listen mode.
#define inHubMode() (true)
// True: always in stats hub/listen mode.
#define inStatsHubMode() (true)
#else
// True if in central hub/listen mode (possibly with local radiator also).
#define inHubMode() (0 != getMinBoilerOnMinutes())
// True if in stats hub/listen mode (minimum timeout).
#define inStatsHubMode() (1 == getMinBoilerOnMinutes())
#endif



// Simple mean filter.
// Find mean of group of ints where sum can be computed in an int without loss.
// TODO: needs a unit test or three.
template<size_t N> int smallIntMean(const int data[N])
  {
  // Extract mean.
  // Assume values and sum will be nowhere near the limits.
  int sum = 0;
  for(int8_t i = N; --i >= 0; ) { sum += data[i]; }
  // Compute rounded-up mean.
  return((sum + (int)(N/2)) / (int)N); // Avoid accidental computation as unsigned...
  }


// Delay in minutes after increasing flow before re-closing is allowed.
// This is to avoid excessive seeking/noise in the presence of strong draughts for example.
// Too large a value may cause significant temperature overshoots and possible energy wastage.
#define ANTISEEK_VALVE_RECLOSE_DELAY_M 5
// Delay in minutes after restricting flow before re-opening is allowed.
// This is to avoid excessive seeking/noise in the presence of strong draughts for example.
// Too large a value may cause significant temperature undershoots and discomfort/annoyance.
#define ANTISEEK_VALVE_REOPEN_DELAY_M (ANTISEEK_VALVE_RECLOSE_DELAY_M*2)

// Typical heat turn-down response time; in minutes, strictly positive.
#define TURN_DOWN_RESPONSE_TIME_M (ANTISEEK_VALVE_RECLOSE_DELAY_M + 3)

// Assumed daily budget in cumulative (%) valve movement for battery-powered devices.
#define DEFAULT_MAX_CUMULATIVE_PC_DAILY_VALVE_MOVEMENT 400


// All input state for computing valve movement.
// This is NOT altered in any way by computeRequiredTRVPercentOpen().
// Exposed to allow easier unit testing.
// All initial values set by the constructor are sane.
struct ModelledRadValveInputState
  {
  ModelledRadValveInputState(const int realTempC16) :
    targetTempC(FROST), 
    minPCOpen(DEFAULT_MIN_VALVE_PC_REALLY_OPEN), maxPCOpen(100),
    widenDeadband(false), glacial(false), hasEcoBias(false), inBakeMode(false)
    { setReferenceTemperatures(realTempC16); }

  // Calculate reference temperature from real temperature.
  void setReferenceTemperatures(const int currentTempC16);

  // Current target room temperature in C in range [MIN_TARGET_C,MAX_TARGET_C].
  uint8_t targetTempC;
  // Min % valve at which is considered to be actually open (allow the room to heat) [1,100].
  uint8_t minPCOpen;
  // Max % valve is allowed to be open [1,100].
  uint8_t maxPCOpen;

  // If true then allow a wider deadband (more temperature drift) to save energy and valve noise.
  // This is a strong hint that the system can work less strenuously to hit, and stay on, target.
  bool widenDeadband;
  // True if in glacial mode.
  bool glacial;
  // True if an eco bias is to be applied.
  bool hasEcoBias;
  // True if in BAKE mode.
  bool inBakeMode;

  // Reference (room) temperature in C/16; must be set before each valve position recalc.
  // Proportional control is in the region where (refTempC16>>4) == targetTempC.
  int refTempC16;
  };

// All retained state for computing valve movement, eg containing time-based state.
// Exposed to allow easier unit testing.
// All initial values set by the constructor are sane.
struct ModelledRadValveState
  {
  ModelledRadValveState() :
    initialised(false),
    isFiltering(false),
    valveMoved(false),
    cumulativeMovementPC(0),
    valveTurndownCountdownM(0), valveTurnupCountdownM(0)
    { }

  // Perform per-minute tasks such as counter and filter updates then recompute valve position.
  // The input state must be complete including target and reference temperatures
  // before calling this including the first time whereupon some further lazy initialisation is done.
  //   * valvePCOpenRef  current valve position UPDATED BY THIS ROUTINE, in range [0,100]
  void tick(volatile uint8_t &valvePCOpenRef, const ModelledRadValveInputState &inputState);

  // True once all deferred initialisation done during the first tick().
  // This takes care of setting state that depends on run-time data
  // such as real temperatures to propagate into all the filters.
  bool initialised;

  // If true then filtering is being applied to temperatures since they are fast-changing.
  bool isFiltering;

  // True if the computed valve position was changed by tick().
  bool valveMoved;

  // Cumulative valve movement count, as unsigned cumulative percent with rollover [0,8191].
  // This is a useful as a measure of battery consumption (slewing the valve)
  // and noise generated (and thus disturbance to humans) and of appropriate control damping.
  //
  // Keep as an unsigned 12-bit field (uint16_t x : 12) to ensure that
  // the value doesn't wrap round to -ve value
  // and can safely be sent/received in JSON by hosts with 16-bit signed ints,
  // and the maximum number of decimal digits used in its representation is limited to 4
  // and used efficiently (~80% use of the top digit).
  //
  // Daily allowance (in terms of battery/energy use) is assumed to be about 400% (DHD20141230),
  // so this should hold many times that value to avoid ambiguity from missed/infrequent readings,
  // especially given full slew (+100%) in nominally as little as 1 minute.
  uint16_t cumulativeMovementPC : 12;

  // Set non-zero when valve flow is constricted, and then counts down to zero.
  // Some or all attempts to open the valve are deferred while this is non-zero
  // to reduce valve hunting if there is string turbulence from the radiator
  // or maybe draughts from open windows/doors
  // causing measured temperatures to veer up and down.
  // This attempts to reduce excessive valve noise and energy use
  // and help to avoid boiler short-cycling.
  uint8_t valveTurndownCountdownM;
  // Mark flow as having been reduced.
  void valveTurndown() { valveTurndownCountdownM = ANTISEEK_VALVE_REOPEN_DELAY_M; }
  // If true then avoid turning up the heat yet.
  bool dontTurnup() { return(0 != valveTurndownCountdownM); }

  // Set non-zero when valve flow is increased, and then counts down to zero.
  // Some or all attempts to close the valve are deferred while this is non-zero
  // to reduce valve hunting if there is string turbulence from the radiator
  // or maybe draughts from open windows/doors
  // causing measured temperatures to veer up and down.
  // This attempts to reduce excessive valve noise and energy use
  // and help to avoid boiler short-cycling.
  uint8_t valveTurnupCountdownM;
  // Mark flow as having been increased.
  void valveTurnup() { valveTurnupCountdownM = ANTISEEK_VALVE_RECLOSE_DELAY_M; }
  // If true then avoid turning down the heat yet.
  bool dontTurndown() { return(0 != valveTurnupCountdownM); }

  // Length of filter memory in ticks; strictly positive.
  // Must be at least 4, and may be more efficient at a power of 2.
  static const size_t filterLength = 16;

  // Previous unadjusted temperatures, 0 being the newest, and following ones successively older.
  // These values have any target bias removed.
  // Half the filter size times the tick() interval gives an approximate time constant.
  // Note that full response time of a typical mechanical wax-based TRV is ~20mins.
  int prevRawTempC16[filterLength];

  // Get smoothed raw/unadjusted temperature from the most recent samples.
  int getSmoothedRecent();

  // get last change in temperature, +ve means rising.
  int getRawDelta() { return(prevRawTempC16[0] - prevRawTempC16[1]); }

//  // Compute an estimate of rate/velocity of temperature change in C/16 per minute/tick.
//  // A positive value indicates that temperature is rising.
//  // Based on comparing the most recent smoothed value with an older smoothed value.
//  int getVelocityC16PerTick();
  };

#if defined(LOCAL_TRV)
#define ENABLE_MODELLED_RAD_VALVE
// Internal model of radidator valve position, embodying control logic.
class ModelledRadValve : public AbstractRadValve
  {
  private:
    // All input state for deciding where to set the radiator valve in normal operation.
    struct ModelledRadValveInputState inputState;
    // All retained state for deciding where to set the radiator valve in normal operation.
    struct ModelledRadValveState retainedState;

    // True if this node is calling for heat.
    // Marked volatile for thread-safe lock-free access.
    volatile bool callingForHeat;

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
        callingForHeat(false),
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
    // This also implies open to DEFAULT_MIN_VALVE_PC_REALLY_OPEN or equivalent.
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

    // True if this unit is nominally calling for heat (temperature is under target).
    // Thread-safe and ISR safe.
    bool isCallingForHeat() const { return(callingForHeat); }

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
    // If no override is set then DEFAULT_MIN_VALVE_PC_REALLY_OPEN is used.
    static uint8_t getMinValvePcReallyOpen();

    // Set and cache minimum valve percentage open to be considered really open.
    // Applies to local valve and, at hub, to calls for remote calls for heat.
    // Any out-of-range value (eg >100) clears the override and DEFAULT_MIN_VALVE_PC_REALLY_OPEN will be used.
    static void setMinValvePcReallyOpen(uint8_t percent);
  };
#define ENABLE_NOMINAL_RAD_VALVE
// Singleton implementation for entire node.
extern ModelledRadValve NominalRadValve;
#elif defined(SLAVE_TRV)
// Valve which is put where it is told; no smarts of its own.
class SimpleSlaveRadValve : public AbstractRadValve
  {
  private:
    // Ticks left before comms timing out and valve should revert to 'safe' position.
    // Counts down to zero one tick per minute in the absence of valid calls to set().
    uint8_t ticksLeft;

  public:
    // Initial time to wait with valve (almost) closed for initial command from controller.
    // Helps avoid thrashing around during start-up when no heat may actually be required.
    // Controller is required to send at least every 15 mins, ie just less than this.
    static const uint8_t INITIAL_TIMEOUT_MINS = 16;

    // Valid calls to set() must happen at less than this interval (minutes, positive).
    // If this timeout triggers, a default valve position is used.
    // Controller is required to send at least every 15 mins, ie at most half this.
    static const uint8_t TIMEOUT_MINS = 30;

    // Default (safe) valve position in percent.
    // Similar to, but distinguishable from, eg FHT8V 'lost connection' 30% position.
    static const uint8_t SAFE_POSITION_PC = 33;

    // Create an instance with valve initially (almost) closed
    // and a few minutes for controller to be heard before reverting to 'safe' position.
    // This initial not-fully-closed position should help signal correct setup.
    SimpleSlaveRadValve() : ticksLeft(INITIAL_TIMEOUT_MINS) { value = 1; }

    // Get estimated minimum percentage open for significant flow for this device; strictly positive in range [1,99].
    // Set to just above the initial value given. 
    virtual uint8_t getMinPercentOpen() const { return(2); }

    // Returns true if this sensor/actuator value is potentially valid, eg in-range.
    virtual bool isValid(const uint8_t value) const { return(value <= 100); }

    // Set new target valve percent open.
    // Ignores invalid values.
    virtual bool set(const uint8_t newValue);

    // Do any regular work that needs doing.
    // Deals with timeout and reversion to 'safe' valve position if the controller goes quiet.
    // Call at a fixed rate (1/60s).
    // Potentially expensive/slow.
    virtual uint8_t read();

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
  };
#define ENABLE_NOMINAL_RAD_VALVE
// Singleton implementation for entire node.
extern SimpleSlaveRadValve NominalRadValve;
#endif



// Default maximum time to allow the boiler to run on to allow for lost call-for-heat transmissions etc.
// Should be (much) greater than the gap between transmissions (eg ~2m for FHT8V/FS20).
// Should be greater than the run-on time at the OpenTRV boiler unit and any further pump run-on time.
// Valves may have to linger open at minimum of this plus maybe an extra minute or so for timing skew
// for systems with poor/absent bypass to avoid overheating.
// Having too high a linger time value may cause excessive temperature overshoot.
#define DEFAULT_MAX_RUN_ON_TIME_M 5

// If defined then turn off valve very slowly after stopping call for heat (ie when shutting) which
// may allow comfortable bolier pump overrun in older systems with no/poor bypass to avoid overheating.
// In any case this should help reduce strain on circulation pumps, etc.
// ALWAYS IMPLEMENT LINGER AS OF 20141228
//#define VALVE_TURN_OFF_LINGER


#ifdef ENABLE_ANTICIPATION
// Returns true if system is in 'learn'/smart mode.
// If in 'smart' mode then the unit can anticipate user demand
// to pre-warm rooms, maintain customary temperatures, etc.
// Currently true if any simple schedule is set.
bool inSmartMode();
#endif



// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#ifdef OCCUPANCY_SUPPORT

// Number of minutes that room is regarded as occupied after markAsOccupied(); strictly positive.
// DHD20130528: no activity for ~30 minutes usually enough to declare room empty in my experience.
// Should probably be at least as long as, or a little longer than, the BAKE timeout.
// Should probably be significantly shorter than normal 'learn' on time to allow savings from that in empty rooms.
#define OCCUPATION_TIMEOUT_M (min(max(SETBACK_FULL_M, 30), 255))
// Threshold from 'likely' to 'probably'.
#define OCCUPATION_TIMEOUT_1_M ((OCCUPATION_TIMEOUT_M*2)/3)

// Occupancy measure as a % confidence that the room/area controlled by this unit has active human occupants.
class OccupancyTracker : public OTV0P2BASE::SimpleTSUint8Sensor
  {
  private:
    // Time until room regarded as unoccupied, in minutes; initially zero (ie treated as unoccupied at power-up).
    // Marked voilatile for thread-safe lock-free non-read-modify-write access to byte-wide value.
    // Compound operations must block interrupts.
    volatile uint8_t occupationCountdownM;

    // Non-zero if occpuancy system recently notified of activity.
    // Marked voilatile for thread-safe lock-free non-read-modify-write access to byte-wide value.
    // Compound operations must block interrupts.
    volatile uint8_t activityCountdownM;

    // Hours and minutes since room became vacant (doesn't roll back to zero from max hours); zero when room occupied.
    uint8_t vacancyH;
    uint8_t vacancyM;

  public:
    OccupancyTracker() : occupationCountdownM(0), activityCountdownM(0), vacancyH(0), vacancyM(0) { }

    // Force a read/poll of the occupancy and return the % likely occupied [0,100].
    // Potentially expensive/slow.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    // Poll at a fixed rate.
    virtual uint8_t read();

    // Returns true if this sensor reading value passed is potentially valid, eg in-range.
    // True if in range [0,100].
    virtual bool isValid(uint8_t value) const { return(value <= 100); }

    // This routine should be called once per minute.
    virtual uint8_t preferredPollInterval_s() const { return(60); }

    // Recommended JSON tag for full value; not NULL.
    virtual const char *tag() const { return("occ|%"); }

    // True if activity/occupancy recently reported (within last couple of minutes).
    // Includes weak and strong reports.
    // Thread-safe.
    bool reportedRecently() { return(0 != activityCountdownM); }

    // Returns true if the room appears to be likely occupied (with active users) now.
    // Operates on a timeout; calling markAsOccupied() restarts the timer.
    // Defaults to false (and API still exists) when OCCUPANCY_SUPPORT not defined.
    // Thread-safe.
    bool isLikelyOccupied() { return(0 != occupationCountdownM); }

    // Returns true if the room appears to be likely occupied (with active users) recently.
    // This uses the same timer as isOccupied() (restarted by markAsOccupied())
    // but returns to false somewhat sooner for example to allow ramping up more costly occupancy detection methods
    // and to allow some simple graduated occupancy responses.
    // Thread-safe.
    bool isLikelyRecentlyOccupied() { return(occupationCountdownM > OCCUPATION_TIMEOUT_1_M); }

    // False if room likely currently unoccupied (no active occupants).
    // Defaults to false (and API still exists) when OCCUPANCY_SUPPORT not defined.
    // This may require a substantial time after activity stops to become true.
    // This and isLikelyOccupied() cannot be true together; it is possible for neither to be true.
    // Thread-safe.
    bool isLikelyUnoccupied() { return(!isLikelyOccupied()); }

    // Call when very strong evidence of room occupation has occurred.
    // Do not call based on internal/synthetic events.
    // Such evidence may include operation of buttons (etc) on the unit or PIR.
    // Do not call from (for example) 'on' schedule change.
    // Makes occupation immediately visible.
    // Thread-safe and ISR-safe.
    void markAsOccupied() { value = 100; occupationCountdownM = OCCUPATION_TIMEOUT_M; activityCountdownM = 2; }

    // Call when some/weak evidence of room occupation, such as a light being turned on, or voice heard.
    // Do not call based on internal/synthetic events.
    // Doesn't force the room to appear recently occupied.
    // If the hardware allows this may immediately turn on the main GUI LED until normal GUI reverts it,
    // at least periodically.
    // Probably do not call on manual control operation to avoid interfering with UI operation.
    // Thread-safe.
    void markAsPossiblyOccupied();

    // Two-bit occupancy: (00 not disclosed,) 1 not occupied, 2 possibly occupied, 3 probably occupied.
    // 0 is not returned by this implementation.
    // Thread-safe.
    uint8_t twoBitOccupancyValue() { return(isLikelyRecentlyOccupied() ? 3 : (isLikelyOccupied() ? 2 : 1)); }

    // Recommended JSON tag for two-bit occupancy value; not NULL.
    const char *twoBitTag() { return("O"); }

    // Returns true if it is worth expending extra effort to check for occupancy.
    // This will happen when confidence in occupancy is not yet zero but is approaching,
    // so checking more thoroughly now can help maintain non-zero value if someone is present and active.
    // At other times more relaxed checking (eg lower power) can be used.
    bool increaseCheckForOccupancy() { return(!isLikelyRecentlyOccupied() && isLikelyOccupied() && !reportedRecently()); }

    // Get number of hours room vacant, zero when room occupied; does not wrap.
    // If forced to zero as soon as occupancy is detected.
    uint16_t getVacancyH() { return((value != 0) ? 0 : vacancyH); }

    // Recommended JSON tag for vacancy hours; not NULL.
    const char *vacHTag() { return("vac|h"); }

    // Threshold hours above which room is considered long vacant.
    // At least 24h in order to allow once-daily room programmes (including pre-warm) to operate reliably.
    static const uint8_t longVacantHThrH = 24;
    // Threshold hours above which room is considered long long vacant.
    // Longer than longVacantHThrH but much less than 3 days to try to capture some weekend-absence savings.
    // 8h less than 2d may capture full office savings for the whole day of Sunday
    // counting from from last occupancy at end of (working) day Friday for example.
    static const uint8_t longLongVacantHThrH = 39;

    // Returns true if room appears to have been vacant for over a day.
    // For a home or an office no sign of activity for this long suggests a weekend or a holiday for example.
    // At least 24h in order to allow once-daily room programmes (including pre-warm) to operate reliably.
    bool longVacant() { return(getVacancyH() > longVacantHThrH); }

    // Returns true if room appears to have been vacant for much longer than longVacant().
    // For a home or an office no sign of activity for this long suggests a long weekend or a holiday for example.
    // Longer than longVacant() but much less than 3 days to try to capture some weekend-absence savings.
    bool longLongVacant() { return(getVacancyH() > longLongVacantHThrH); }

    // Put directly into energy-conserving 'holiday mode' by making room appear to be 'long vacant'.
    // Be careful of retriggering presence immediately if this is set locally.
    // Set apparent vacancy to maximum to make setting obvious and to hide further vacancy from snooping.
    // Code elsewhere may wish to put the system in FROST mode also.
    void setHolidayMode() { activityCountdownM = 0; value = 0; occupationCountdownM = 0; vacancyH = 255U; }

#ifdef UNIT_TESTS
    // If true then mark as occupied else mark as (just) unoccupied.
    // Hides basic _TEST_set_() which would not behave as expected.
    virtual void _TEST_set_(const bool occupied)
      { if(occupied) { markAsOccupied(); } else { activityCountdownM = 0; value = 0; occupationCountdownM = 0; } }
//    // Set new value(s) for hours of vacancy, or marks as occupied is if zero vacancy.
//    // If a non-zero value is set it clears the %-occupancy-confidence value for some consistency.
//    // Makes this more usable as a mock for testing other components.
//    virtual void _TEST_set_vacH_(const uint8_t newVacH)
//      { vacancyM = 0; vacancyH = newVacH; if(0 != newVacH) { value = 0; occupationCountdownM = 0; } else { markAsOccupied(); } }
#endif
  };
#else
// Placeholder namespace with dummy static status methods to reduce code complexity.
class OccupancyTracker
  {
  public:
    static void markAsOccupied() {} // Defined as NO-OP for convenience when no general occupancy support.
    static void markAsPossiblyOccupied() {} // Defined as NO-OP for convenience when no general occupancy support.
    static bool isLikelyRecentlyOccupied() { return(false); } // Always false without OCCUPANCY_SUPPORT
    static bool isLikelyOccupied() { return(false); } // Always false without OCCUPANCY_SUPPORT
    static bool isLikelyUnoccupied() { return(false); } // Always false without OCCUPANCY_SUPPORT
    static uint8_t twoBitOccValue() { return(0); } // Always 0 without OCCUPANCY_SUPPORT.
    static uint16_t getVacancyH() { return(0); } // Always 0 without OCCUPANCY_SUPPORT.
    static bool longVacant() { return(false); } // Always false without OCCUPANCY_SUPPORT.
    static bool longLongVacant() { return(false); } // Always false without OCCUPANCY_SUPPORT.
  };
#endif
// Singleton implementation for entire node.
extern OccupancyTracker Occupancy;



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

// Returns true if specified hour is (conservatively) in the specifed outlier quartile for specified stats set.
// Returns false if a full set of stats not available, eg including the specified hour.
// Always returns false if all samples are the same.
//   * inTop  test for membership of the top quartile if true, bottom quartile if false
//   * statsSet  stats set number to use.
//   * hour  hour of day to use or ~0 for current hour.
bool inOutlierQuartile(uint8_t inTop, uint8_t statsSet, uint8_t hour = ~0);

// 'Unset'/invalid values for byte (eg raw EEPROM byte) and int (eg after decompression).
#define STATS_UNSET_BYTE 0xff
#define STATS_UNSET_INT 0x7fff

#ifdef ENABLE_ANTICIPATION
// Returns true iff room likely to be occupied and need warming at the specified hour's sample point based on collected stats.
// Used for predictively warming a room in smart mode and for choosing setback depths.
// Returns false if no good evidence to warm the room at the given time based on past history over about one week.
//   * hh hour to check for predictive warming [0,23]
bool shouldBeWarmedAtHour(const uint_least8_t hh);
#endif



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



// Clear and populate core stats structure with information from this node.
// Exactly what gets filled in will depend on sensors on the node,
// and may depend on stats TX security level (if collecting some sensitive items is also expensive).
void populateCoreStats(FullStatsMessageCore_t *content);

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

