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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2015
*/

/*
 * Header for main on-board sensors and actuators for V0p2 variants.
 */

#ifndef V0P2_ACTUATORS_H
#define V0P2_ACTUATORS_H

#include "AbstractRadValve.h"



// Generic (unit-testable) motor diver login using end-stop detection and simple shaft-encoder.
// Designed to be embedded in a motor controller instance.
// This used the sub-cycle clock for timing.
// This is sensitive to sub-cycle position, ie will try to avoid causing a main loop overrun.
class CurrentSenseValveMotorDirect : public HardwareMotorDriverInterfaceCallbackHandler
  {
  public:
    // Maximum time to move pin between fully retracted and extended and vv, seconds, strictly positive.
    // Set as a limit to allow a timeout when things go wrong. 
    static const uint8_t MAX_TRAVEL_S = 4 * 60; // 4 minutes.

  private:
    // Hardware interface instance, passed by reference.
    // Must have a lifetime exceeding that of this enclosing object.
    HardwareMotorDriverInterface * const hw;

  public:
    // Basic/coarse state of driver.
    // There may be microstates within most these basic states.
    //
    // Power-up sequence will often require something like:
    //   * withdrawing the pin completely (to make valve easy to fit)
    //   * waiting for some user activation step such as pressing a button to indicate valve fitted
    //   * running an initial calibration for the valve.
    //   * entering a normal state tracking the target %-open and periodically recalibrating/decalcinating.
    enum driverState
      {
      init = 0, // Power-up state.
      valvePinWithdrawing, // Retracting pin at power-up.
      valvePinWithdrawn, // Allows valve to be fitted; wait for user signal that valve has been fitted.
      valveCalibrating, // Calibrating full valve travel.
      valveNormal, // Normal operating state: values lower than this indicate that power-up is not complete.
      valveDecalcinating, // TODO: running decalcination cycle (and can recalibrate and mitigate valve seating issues).
      valveError // Error state can only normally be cleared by power-cycling.
      };

  private:
    // Major state of driver.
    // On power-up (or full reset) should be 0/init.
    // Stored as a uint8_t to save a little space and to make atomic operations easier.
    // Marked volatile so that individual reads are ISR-/thread- safe without a mutex.
    // Hold a mutex to perform compound operations such as read/modify/write.
    // Change state with changeState() which will do some other book-keeping.
    volatile /*driverState*/ uint8_t state;
    // Change state and perform some book-keeping.
    inline void changeState(const driverState newState) { state = (uint8_t)newState; clearPerState(); }

    // Data used only within one major state and not needing to be saved between state.
    // Thus it can be shared in a union to save space.
    // This can be cleared to all zeros with clearPerState(), so starts each state as all zeros.
    union
      {
      // State used while calibrating.
      struct
        {
        uint8_t calibState; // Current micro-state, starting at zero.
//        uint8_t runCount; // Completed round-trip calibration runs.
        uint16_t ticksFromOpenToClosed;
        uint16_t ticksFromClosedToOpen;
        } calibrating;
      } perState;
    inline void clearPerState() { if(sizeof(perState) > 0) { memset(&perState, 0, sizeof(perState)); } }

    // Flag set on signalHittingEndStop() callback from end-top / stall / high-current input.
    // Marked volatile for thread-safe lock-free access (with care).
    volatile bool endStopDetected;

//    // Set when valve needs recalibration, eg because dead-reckoning found to be significantly wrong.
//    // May also need recalibrating after (say) a few weeks to allow for battery/speed droop.
//    bool needsRecalibrating;

    // Set during calibration.
    uint16_t ticksFromOpenToClosed;
    uint16_t ticksFromClosedToOpen;
    // Ticks per percent in the open-to-closed direction; computed during tracking.
    uint8_t ticksPerPercentOpenToClosed;

    // Current sub-cycle ticks from fully-open (reference) end of travel, towards fully closed.
    // This is nominally ticks in the open-to-closed direction
    // since those may differ from the other direction.
    // Reset during calibration and upon hitting an end-stop.
    // Recalibration, full or partial, may be forced if this overflows or underflows significantly.
    // Significant underflow might be (say) the minimum valve-open percentage.
    // ISR-/thread- safe with a mutex.
    volatile uint16_t ticksFromOpen;
    // Maximum permitted value of ticksFromOpen;
    static const uint16_t MAX_TICKS_FROM_OPEN = ~0;

    // Current nominal percent open in range [0,100].
    uint8_t currentPC;

    // Target % open in range [0,100].
    // Maintained across all states; defaults to 'closed'/0.
    uint8_t targetPC;

  public:
    // Create an instance, passing in a reference to the non-NULL hardware driver.
    // The hardware driver instance lifetime must be longer than this instance.
    CurrentSenseValveMotorDirect(HardwareMotorDriverInterface * const hwDriver) :
        hw(hwDriver), targetPC(0)
        { changeState(init); }

    // Poll.
    // Regular poll every 1s or 2s,
    // though tolerates missed polls eg because of other time-critical activity.
    // May block for hundreds of milliseconds.
    void poll();

    // Get major state, mostly for testing.
    driverState getState() const { return((driverState) state); }

    // Get current target % open in range [0,100].
    uint8_t getTargetPC() { return(targetPC); }

    // Set current target % open in range [0,100].
    void setTargetPC(const uint8_t newPC) { targetPC = newPC; }

    // Minimally wiggles the motor to give tactile feedback and/or show to be working.
    // May take a significant fraction of a second.
    // Finishes with the motor turned off.
    virtual void wiggle();

    // Called when end stop hit, eg by overcurrent detection.
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    virtual void signalHittingEndStop(bool opening) { endStopDetected = true; }

    // Called when encountering leading edge of a mark in the shaft rotation in forward direction (falling edge in reverse).
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    virtual void signalShaftEncoderMarkStart(bool opening) { /* TODO */ }

    // Called with each motor run sub-cycle tick.
    // Is ISR-/thread- safe.
    virtual void signalRunSCTTick(bool opening);

    // Returns true iff not in error state and not (re)calibrating/(re)initialising/(re)syncing.
    // By default there is no recalibration step.
    virtual bool isInNormalRunState() const { return(state >= (uint8_t)valveNormal); }

    // Returns true if in an error state.
    // May be recoverable by forcing recalibration.
    virtual bool isInErrorState() const { return(state >= (uint8_t)valveError); }






//    // Call when given user signal that valve has been fitted (ie is fully on).
//    // Can be called while run() is in progress.
//    // Is ISR-/thread- safe.
//    void signalValveFitted();
//
//    // Set target % open.
//    // Can optionally be 'lazy' eg move more slowly or avoid tiny movements entirely.
//    void setTargetPercentOpen(uint8_t newTargetPC, bool beLazy = false) { targetPC = min(newTargetPC, 100); lazy = beLazy; }
//
//    // Used to run motor, adjust state, etc for up to specified maximum number of milliseconds.
//    // Returns true if more work remaining to get into target state.
//    bool run(uint16_t maxms, HardwareMotorDriverInterface &driver);
//
//    // Returns true iff not (re)calibrating/(re)initialising/(re)syncing.
//    // Initially false until power-up and at least initial calibration are complete.
//    bool isCalibrated() const { return((state > (uint8_t)valveCalibrating) && (0 != clicksFullTravel)); }
//
//    // Returns true if device can track movement between end stops.
//    // Without this at best the logic has to guess and the valve control logic
//    // should possibly be more concerned with nudging open/closed
//    // than trying to hit some arbitrary percentage open.
//    bool hasMovementTracker() const { return(0 != clicksFullTravel); }
//
//    // True iff power-up initialisation (eg including allowing user to fit to valve base) is done.
//    bool isPowerUpDone() const { return(state >= (uint8_t)valveNormal); }
//
//    // Get current motor drive status (off else direction of running).
//    HardwareMotorDriverInterface::motor_drive getMotorDriveStatus() const { return((HardwareMotorDriverInterface::motor_drive) motorDriveStatus); }
  };


#if defined(LOCAL_TRV) && defined(DIRECT_MOTOR_DRIVE_V1)
#define HAS_VALVEDIRECT
// Implementation for V1 (REV7/DORM1) motor.
// Usually not instantiated except within ValveMotorDirectV1.
// Creating multiple instances almost certainly a BAD IDEA.
class ValveMotorDirectV1HardwareDriver : public HardwareMotorDriverInterface
  {
  private:
    // Spin for up to the specified number of SCT ticks, monitoring current and position encoding.
    //   * maxRunTicks  maximum sub-cycle ticks to attempt to run/spin for); strictly positive
    //   * minTicksBeforeAbort  minimum ticks before abort for end-stop / high-current,
    //       don't attempt to run at all if less than this time available before (close to) end of sub-cycle;
    //       should be no greater than maxRunTicks
    //   * dir  direction to run motor (open or closed) or off if waiting for motor to stop
    //   * callback  handler to deliver end-stop and position-encoder callbacks to;
    //     non-null and callbacks must return very quickly
    // If too few ticks remain before the end of the sub-cycle for the minimum run,
    // then this will return true immediately.
    // Invokes callbacks for high current (end stop) and position (shaft) encoder where applicable.
    // Aborts early if high current is detected at the start,
    // or after the minimum run period.
    // Returns true if aborted early from too little time to start, or by high current (assumed end-stop hit).
    bool spinSCTTicks(uint8_t maxRunTicks, uint8_t minTicksBeforeAbort, motor_drive dir, HardwareMotorDriverInterfaceCallbackHandler &callback);

  public:
    // Detect if end-stop is reached or motor current otherwise very high.
    virtual bool isCurrentHigh(HardwareMotorDriverInterface::motor_drive mdir = motorDriveOpening) const;

    // Call to actually run/stop motor.
    // May take as much as (say) 200ms eg to change direction.
    // Stopping (removing power) should typically be very fast, << 100ms.
    //   * maxRunTicks  maximum sub-cycle ticks to attempt to run/spin for); zero will run for shortest reasonable time
    //   * dir  direction to run motor (or off/stop)
    //   * callback  callback handler
    virtual void motorRun(uint8_t maxRunTicks, motor_drive dir, HardwareMotorDriverInterfaceCallbackHandler &callback);
  };

// Actuator/driver for direct local (radiator) valve motor control.
class ValveMotorDirectV1 : public AbstractRadValve
  {
  private:
    // Driver for the V1/DORM1 hardware.
    ValveMotorDirectV1HardwareDriver driver;
    // Logic to manage state, etc.
    CurrentSenseValveMotorDirect logic;

  public:
    ValveMotorDirectV1() : logic(&driver) { }

    // Regular poll/update.
    virtual uint8_t read() { logic.poll(); return(value); }

//    // Handle simple interrupt.
//    // Fast and ISR (Interrupt Service Routines) safe.
//    // Returns true if interrupt was successfully handled and cleared
//    // else another interrupt handler in the chain may be called
//    // to attempt to clear the interrupt.
//    virtual bool handleInterruptSimple() { /* TODO */ }

    // Returns true iff not in error state and not (re)calibrating/(re)initialising/(re)syncing.
    virtual bool isInNormalRunState() const { return(logic.isInNormalRunState()); }

    // Returns true if in an error state,
    virtual bool isInErrorState() const { return(logic.isInErrorState()); }

    // Minimally wiggles the motor to give tactile feedback and/or show to be working.
    // May take a significant fraction of a second.
    // Finishes with the motor turned off.
    virtual void wiggle() { logic.wiggle(); }
  };
// Singleton implementation/instance.
extern ValveMotorDirectV1 ValveDirect;
#endif

















// Default minimum valve percentage open to be considered actually/significantly open; [1,100].
// Setting this above 0 delays calling for heat from a central boiler until water is likely able to flow.
// (It may however be possible to scavenge some heat if a particular valve opens below this and the circulation pump is already running, for example.)
// DHD20130522: FHT8V + valve heads I have been using are not typically open until around 6%; at least one opens at ~20%.
// Allowing valve to linger at just below this level without calling for heat when shutting
// may allow comfortable boiler pump overrun in older systems with no/poor bypass to avoid overheating.
// DHD20151014: may need reduction to <5 for use in high-pressure systems.
#define DEFAULT_MIN_VALVE_PC_REALLY_OPEN 11

// Default valve percentage at which significant heating power is being provided.
// For many valves much of the time this may be effectively fully open,
// ie no change beyond this makes significant difference to heat delivery.
// Should be significantly higher than DEFAULT_MIN_VALVE_PC_REALLY_OPEN.
// DHD20151014: may need boost to ~50 for tricky all-in-one units.
#define DEFAULT_VALVE_PC_MODERATELY_OPEN 35


#if defined(ENABLE_BOILER_HUB)
// Internal logic for simple on/off boiler output, fully testable.
class OnOffBoilerDriverLogic
  {
  public:
    // Maximum distinct radiators tracked by this system.
    // The algorithms uses will assume that this is a smallish number,
    // and may work best of a power of two.
    // Reasonable candidates are 8 or 16.
    static const uint8_t maxRadiators = 8;

    // Per-radiator data status.
    struct PerIDStatus
      {
      // ID of remote device; an empty slot is marked with the (invalid) ID of 0xffffu.
      uint16_t id;
      // Ticks until we expect new update from given valve else its data has expired when non-negative.
      // A zero or negative value means no active call for heat from the matching ID.
      // A negative value indicates that a signal is overdue by that number of ticks
      // or units greater than ticks to allow old entries to linger longer for tracking or performance reasons.
      // An empty slot is given up for an incoming new ID calling for heat.
      int8_t ticksUntilOff;
      // Last percent open for given valve.
      // A zero value means no active call for heat from the matching ID.
      // An empty slot is given up for an incoming new ID calling for heat.
      uint8_t percentOpen;
      };

  private:
    // True to call for heat from the boiler.
    bool callForHeat;

    // Number of ticks that boiler has been in current state, on or off, to avoid short-cycling.
    // The state cannot be changed until the specified minimum off/on ticks have been passed.
    // (This rule may be ignored when the boiler is currently on if all valves seem to have been turned off prematurely.)
    // This value does not roll back round to zero, ie will stop at maximum until reset.
    // The max representable value allows for several hours at 1 or 2 seconds per tick.
    uint8_t ticksInCurrentState;

    // Ticks minimum for boiler to stay in each state to avoid short-cycling; should be significantly positive but won't fail if otherwise.
    // Typically the equivalent of 2--10 minutes.
    uint8_t minTicksInEitherState;

    // Minimum individual valve percentage to be considered open [1,100].
    uint8_t minIndividualPC;

    // Minimum aggrigate valve percentage to be considered open, no lower than minIndividualPC; [1,100].
    uint8_t minAggregatePC;

    // 'Bad' (never valid as housecode or OpenTRV code) ID.
    static const uint16_t badID = 0xffffu;

#if defined(BOILER_RESPOND_TO_SPECIFIED_IDS_ONLY)
    // List of authorised IDs.
    // Entries beyond the last valid authorised ID are set to 0xffffu (never a valid ID);
    // if there are none then [0] is set to 0xffffu.
    // If no authorised IDs then no authorisation is done and all IDs are accepted
    // and kicked out expiry-first if there is a shortage of slots.
    uint16_t authedIDs[maxRadiators];
#endif

    // List of recently-heard IDs and ticks until their last call for heat will expire (matched by index).
    // If there any authorised IDs then only such IDs are admitted to the list.
    // The first empty empty slot is marked with the (invalid) ID of 0xffffu.
    // Marked volatile since may be updated by ISR.
    volatile PerIDStatus status[maxRadiators];

#if defined(OnOffBoilerDriverLogic_CLEANUP)
    // Do some incremental clean-up to speed up future operations.
    // Aim to free up at least one status slot if possible.
    void cleanup();
#endif

  public:
    OnOffBoilerDriverLogic() : callForHeat(false), ticksInCurrentState(0), minTicksInEitherState(60), minIndividualPC(DEFAULT_MIN_VALVE_PC_REALLY_OPEN), minAggregatePC(DEFAULT_VALVE_PC_MODERATELY_OPEN)
      {
#if defined(BOILER_RESPOND_TO_SPECIFIED_IDS_ONLY)
      authedIDs[0] = badID;
#endif
      status[0].id = badID;
      }

    // Set thresholds for per-value and minimum-aggregate percentages to fire the boiler.
    // Coerces values to be valid:
    // minIndividual in range [1,100] and minAggregate in range [minIndividual,100].
    void setThresholds(uint8_t minIndividual, uint8_t minAggregate);

    // Set minimum ticks for boiler to stay in each state to avoid short-cycling; should be significantly positive but won't fail if otherwise.
    // Typically the equivalent of 2--10 minutes (eg ~2+ for gas, ~8 for oil).
    void setMinTicksInEitherState(uint8_t minTicks) { minTicksInEitherState = minTicks; }

    // Called upon incoming notification of status or call for heat from given (valid) ID.
    // ISR-/thread- safe to allow for interrupt-driven comms, and as quick as possible.
    // Returns false if the signal is rejected, eg from an unauthorised ID.
    // The basic behaviour is that a signal with sufficient percent open
    // is good for 2 minutes (120s, 60 ticks) unless explicitly cancelled earlier,
    // for all valve types including FS20/FHT8V-style.
    // That may be slightly adjusted for IDs that indicate FS20 housecodes, etc.
    //   * id  is the two-byte ID or house code; 0xffffu is never valid
    //   * percentOpen  percentage open that the remote valve is reporting
    bool receiveSignal(uint16_t id, uint8_t percentOpen);

    // Iff true then call for heat from the boiler.
    bool isCallingForHeat() const { return(callForHeat); }

    // Poll every 2 seconds in real/virtual time to update state in particular the callForHeat value.
    // Not to be called from ISRs,
    // in part because this may perform occasional expensive-ish operations
    // such as incremental clean-up.
    // Because this does not assume a tick is in real time
    // this remains entirely unit testable,
    // and no use of wall-clock time is made within this or sibling class methods.
    void tick2s();

    // Fetches statuses of valves recently heard from and returns the count; 0 if none.
    // Optionally filters to return only those still live and apparently calling for heat.
    //   * valves  array to copy status to the start of; never null
    //   * size  size of valves[] in entries (not bytes), no more entries than that are used,
    //     and no more than maxRadiators entries are ever needed
    //   * onlyLiveAndCallingForHeat  if true retrieves only current entries
    //     'calling for heat' by percentage
    uint8_t valvesStatus(PerIDStatus valves[], uint8_t size, bool onlyLiveAndCallingForHeat) const;
  };


// Boiler output control (call-for-heat driver).
// Nominally drives on scale of [0,100]%
// but any non-zero value should be regarded as calling for heat from an on/off boiler,
// and only values of 0 and 100 may be produced.
// Implementations require poll() called at a fixed rate (every 2s).
class BoilerDriver : public OTV0P2BASE::SimpleTSUint8Actuator
  {
  private:
    OnOffBoilerDriverLogic logic;

  public:
    // Regular poll/update.
    virtual uint8_t read();

    // Preferred poll interval (2 seconds).
    virtual uint8_t preferredPollInterval_s() const { return(2); }

  };
// Singleton implementation/instance.
extern BoilerDriver BoilerControl;
#endif


#endif
