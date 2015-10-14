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





// Abstract class for motor drive.
// Supports abstract model plus remote (wireless) and local/direct implementations.
// Implementations may require poll() called at a fixed rate.
class AbstractRadValve : public OTV0P2BASE::SimpleTSUint8Actuator
  {
  public:
    // Returns true if this sensor reading value passed is valid, eg in range [0,100].
    virtual bool isValid(const uint8_t value) const { return(value <= 100); }

    // Returns true iff not (re)calibrating/(re)initialising/(re)syncing.
    // By default there is no recalibration step.
    virtual bool isCalibrated() const { return(true); }

    // True if the controlled physical valve is thought to be at least partially open right now.
    // If multiple valves are controlled then is this true only if all are at least partially open.
    // Used to help avoid running boiler pump against closed valves.
    // Must not be true while (re)calibrating.
    // The default is to use the check the current computed position
    // against the minimum open percentage.
    virtual bool isControlledValveReallyOpen() const { return(isCalibrated() && (value >= getMinPercentOpen())); }

    // Get estimated minimum percentage open for significant flow for this device; strictly positive in range [1,99].
    // Defaults to 1 which is minimum possible legitimate value.
    virtual uint8_t getMinPercentOpen() const { return(1); }
  };


// Generic callback handler for hardware valve motor driver.
class HardwareMotorDriverInterfaceCallbackHandler
  {
  protected:
    ~HardwareMotorDriverInterfaceCallbackHandler() {}

  public:
    // Called when end stop hit, eg by overcurrent detection.
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    virtual void signalHittingEndStop() = 0;
  
    // Called when encountering leading edge of a mark in the shaft rotation in forward direction (falling edge in reverse).
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    virtual void signalShaftEncoderMarkStart() = 0;
  };

// Interface for low-level hardware motor driver.
class HardwareMotorDriverInterface
  {
  protected:
    ~HardwareMotorDriverInterface() {}

  public:
    // Legal motor drive states.
    enum motor_drive
      {
      motorOff = 0, // Motor switched off (default).
      motorDriveClosing, // Drive towards the valve-closed position.
      motorDriveOpening, // Drive towards the valve-open position.
      motorStateInvalid // Higher than any valid state.
      };

    // Call to actually run/stop low-level motor.
    // May take as much as 200ms eg to change direction.
    // Stopping (removing power) should typically be very fast, << 100ms.
    //   * start  if true then this routine starts the motor from cold,
    //            else this runs the motor for a short continuation period;
    //            at least one continuation should be performed before testing
    //            for high current loads at end stops
    virtual void motorRun(motor_drive dir, bool start = true) = 0;
//
//    // Detect if end-stop is reached or motor current otherwise very high.
//    virtual bool isCurrentHigh();

    // Enable/disable end-stop detection and shaft-encoder.
    // Disabling should usually forces the motor off,
    // with a small pause for any residual movement to complete.
    virtual void enableFeedback(bool enable, HardwareMotorDriverInterfaceCallbackHandler &callback) = 0;

    // If true then enableFeedback(true) needs to be called in a fairly tight loop
    // while the motor is running and for a short while after
    // to capture end-stop hits, etc.
    virtual bool needsPoll() const = 0;
  };

// Generic (unit-testable) motor diver login using end-stop detection and simple shaft-encoder.
// Designed to be embedded in a motor controller instance.
class CurrentSenseValveMotorDirect : public HardwareMotorDriverInterfaceCallbackHandler
  {
  public:
    // Basic/coarse state of driver.
    // There are microstates within most of these basic states.
    //
    // Power-up sequence will often require something like:
    //   * withdrawing the pin completely (to make valve easy to fit)
    //   * waiting for some user activation step such as pressing a button to indicate valve fitted
    //   * running an initial calibration for the valve.
    //   * entering a normal state tracking the target %-open and periodically recalibrating/decalcinating.
    enum driverState
      {
      init = 0, // Power-up state.
      initPinWithdrawn, // Allows valve to be fitted.
      initCalibrating, // Calibrating full valve travel.
      normal, // Normal operating state: values lower than this indicate that power-up is not complete.
      decalcinating, // TODO: running decalcination cycle (and can recalibrate and mitigate valve seating issues).
      motorDriverError // Error state can only normally be cleared by power-cycling.
      };

  private:
    // Clicks from one end of the range to the other; 0 if not initialised or no movement tracker.
    uint16_t clicksFullTravel;

    // Current clicks from closed end of travel.
    // ISR-/thread- safe with a mutex.
    volatile uint16_t clicksFromClosed;

    // Measured (during calibration) sub-cycle ticks (1/128s) from open to closed.
    uint16_t ticksFromOpen;
    // Measured (during calibration) sub-cycle ticks (1/128s) from open to closed.
    uint16_t ticksFromClosed;

    // Basic state of driver.
    // Marked volatile so that individual reads are ISR-/thread- safe without a mutex.
    // Hold a mutex to do compound operations such as read/modify/write.
    volatile /*driverState*/ uint8_t state;

    // Nominal motor drive status, ie what it should be doing.
    // (Motor may not actually be running all the time that this indicates itself not off.)
    /*motor_drive*/ uint8_t motorDriveStatus;

    // Current nominal percent open in range [0,100].
    uint8_t currentPC;

    // Target % open in range [0,100].
    uint8_t targetPC;

    // True if movement can be 'lazy'.
    bool lazy;

    // Turn motor off, or on in a given drive direction.
    // Sets state accordingly.
    // Does not provide any monitoring of stall, position encoding, etc.
    // May take significant time and have to be done very carefully in concrete implementations.
    void setMotorDrive(HardwareMotorDriverInterface::motor_drive dir) { motorDriveStatus = min((uint8_t)dir, (uint8_t)HardwareMotorDriverInterface::motorStateInvalid - 1); }

  public:
    CurrentSenseValveMotorDirect() : state(init), motorDriveStatus(HardwareMotorDriverInterface::motorOff) { }

    // Called when end stop hit, eg by overcurrent detection.
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    virtual void signalHittingEndStop();

    // Called when encountering leading edge of a mark in the shaft rotation in forward direction (falling edge in reverse).
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    virtual void signalShaftEncoderMarkStart();

    // Call when given user signal that valve has been fitted (ie is fully on).
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    void signalValveFitted();

    // Set target % open.
    // Can optionally be 'lazy' eg move more slowly or avoid tiny movements entirely.
    void setTargetPercentOpen(uint8_t newTargetPC, bool beLazy = false) { targetPC = min(newTargetPC, 100); lazy = beLazy; }

    // Used to run motor, adjust state, etc for up to specified maximum number of milliseconds.
    // Returns true if more work remaining to get into target state.
    bool run(uint16_t maxms, HardwareMotorDriverInterface &driver);

    // Returns true iff not (re)calibrating/(re)initialising/(re)syncing.
    // Initially false until power-up and at least initial calibration are complete.
    bool isCalibrated() const { return((state > (uint8_t)initCalibrating) && (0 != clicksFullTravel)); }

    // Returns true if device can track movement between end stops.
    // Without this at best the logic has to guess and the valve control logic
    // should possibly be more concerned with nudging open/closed
    // than trying to hit some arbitrary percentage open.
    bool hasMovementTracker() const { return(0 != clicksFullTravel); }

    // True iff power-up initialisation (eg including allowing user to fit to valve base) is done.
    bool isPowerUpDone() const { return(state >= (uint8_t)motorNormal); }

    // Get current motor drive status (off else direction of running).
    HardwareMotorDriverInterface::motor_drive getMotorDriveStatus() const { return((HardwareMotorDriverInterface::motor_drive) motorDriveStatus); }
  };

#if defined(LOCAL_TRV) && defined(DIRECT_MOTOR_DRIVE_V1)
// Implementation for V1 (REV7/DORM1) motor.
// Usually not instantiated except within ValveMotorDirectV1.
// Creating multiple instances almost certainly a BAD IDEA.
class ValveMotorDirectV1HardwareDriver : public HardwareMotorDriverInterface
  {
  public:
    // Call to actually run/stop low-level motor.
    // May take as much as 200ms eg to change direction.
    // Stopping (removing power) should typically be very fast, << 100ms.
    //   * start  if true then this routine starts the motor from cold,
    //            else this runs the motor for a short continuation period;
    //            at least one continuation should be performed before testing
    //            for high current loads at end stops
    virtual void motorRun(motor_drive dir, bool start = true);

    // Detect if end-stop is reached or motor current otherwise very high.
    virtual bool isCurrentHigh(HardwareMotorDriverInterface::motor_drive mdir = motorDriveOpening) const;

    // Enable/disable end-stop detection and shaft-encoder.
    // Disabling should usually force the motor off,
    // with a small pause for any residual movement to complete.
    virtual void enableFeedback(bool enable, HardwareMotorDriverInterfaceCallbackHandler &callback);

    // If true then enableFeedback(true) needs to be called in a fairly tight loop
    // while the motor is running and for a short while after
    // to capture end-stop hits, etc.
    // Always true for this driver.
    virtual bool needsPoll() const { return(true); }
  };

// Actuator/driver for direct local (radiator) valve motor control.
class ValveMotorDirectV1 : public AbstractRadValve
  {
  private:
//    CurrentSenseValveMotorDirect logic;

  protected:
    // Turn motor off, or on in a given drive direction.
    // This routine is very careful to avoid setting outputs into any illegal/'bad' state.
    // Sets flags accordingly.
    // Does not provide any monitoring of stall, position encoding, etc.
    // May take significant time (~150ms) to gently stop motor.
//    void motorDrive(motor_drive dir) { logic.setMotorDrive(dir); }

  public:
    // Regular poll/update.
    virtual uint8_t read();

//    // Handle simple interrupt.
//    // Fast and ISR (Interrupt Service Routines) safe.
//    // Returns true if interrupt was successfully handled and cleared
//    // else another interrupt handler in the chain may be called
//    // to attempt to clear the interrupt.
//    virtual bool handleInterruptSimple();

    // Minimally wiggles the motor to give tactile feedback and/or show to be working.
    // Does not itself track movement against shaft encoder, etc, or check for stall.
    // May take a significant fraction of a second.
    // Finishes with the motor turned off.
    void wiggle();

//#if defined(ALT_MAIN_LOOP) && defined(DEBUG)
//  // Drive motor back and forth (toggle direction each call) just for testing/fun.
//  void flip();
//#endif
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
    // MAximum distinct radiators tracked by this system.
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
