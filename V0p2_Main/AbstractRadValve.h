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
 * Abstract/base radiator-valve class and supporting types for typical implementations.
 */

#ifndef ABSTRACTRADVALVE_H
#define ABSTRACTRADVALVE_H



// Abstract class for motor drive.
// Supports abstract model plus remote (wireless) and local/direct implementations.
// Implementations may require read() called at a fixed rate,
// though should tolerate calls being skipped when time is tight for other operations,
// since read() may take substantial time (hundreds of milliseconds).
class AbstractRadValve : public OTV0P2BASE::SimpleTSUint8Actuator
  {
  public:
    // Returns true if this target valve open % value passed is valid, ie in range [0,100].
    virtual bool isValid(const uint8_t value) const { return(value <= 100); }

    // Set new target valve percent open.
    // Ignores invalid values.
    // Some implementations may ignore/reject all attempts to directly set the values.
    // If true then the new target value was accepted.
    virtual bool set(const uint8_t newValue) { return(false); }

    // Returns true iff not in error state and not (re)calibrating/(re)initialising/(re)syncing.
    // By default there is no recalibration step.
    virtual bool isInNormalRunState() const { return(true); }

    // Returns true if in an error state.
    // May be recoverable by forcing recalibration.
    virtual bool isInErrorState() const { return(false); }

    // True if the controlled physical valve is thought to be at least partially open right now.
    // If multiple valves are controlled then is this true only if all are at least partially open.
    // Used to help avoid running boiler pump against closed valves.
    // Must not be true while (re)calibrating.
    // The default is to use the check the current computed position
    // against the minimum open percentage.
    virtual bool isControlledValveReallyOpen() const { return(isInNormalRunState() && (value >= getMinPercentOpen())); }

    // Get estimated minimum percentage open for significant flow for this device; strictly positive in range [1,99].
    // Defaults to 1 which is minimum possible legitimate value.
    virtual uint8_t getMinPercentOpen() const { return(1); }

    // Minimally wiggles the motor to give tactile feedback and/or show to be working.
    // May take a significant fraction of a second.
    // Finishes with the motor turned off.
    // By default does nothing.
    virtual void wiggle() { }
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
    virtual void signalHittingEndStop(bool opening) = 0;
  
    // Called when encountering leading edge of a mark in the shaft rotation in forward direction (falling edge in reverse).
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    virtual void signalShaftEncoderMarkStart(bool opening) = 0;

    // Called with each motor run sub-cycle tick.
    // Is ISR-/thread- safe.
    virtual void signalRunSCTTick(bool opening) = 0;
  };

// Interface for low-level hardware motor driver.
class HardwareMotorDriverInterface
  {
  public:
    // Legal motor drive states.
    enum motor_drive
      {
      motorOff = 0, // Motor switched off (default).
      motorDriveClosing, // Drive towards the valve-closed position.
      motorDriveOpening, // Drive towards the valve-open position.
      motorStateInvalid // Higher than any valid state.
      };

  protected:
    HardwareMotorDriverInterface() : last_dir((uint8_t)motorOff) { }

//    // Could attempt to force motor off at destruction...
//    ~HardwareMotorDriverInterface() : { }

    // Last recorded direction.
    // Helpful to record shaft-encoder and other behaviour correctly around direction changes.
    // Marked volatile and stored as uint8_t to help thread-safety, and potentially save space.
    volatile uint8_t last_dir;
//
//    // Spin for up to the specified number of SCT ticks, monitoring current and position encoding.
//    //   * maxRunTicks  maximum sub-cycle ticks to attempt to run/spin for); strictly positive
//    //   * minTicksBeforeAbort  minimum ticks before abort for end-stop / high-current,
//    //       don't attempt to run at all if less than this time available before (close to) end of sub-cycle;
//    //       should be no greater than maxRunTicks
//    //   * dir  direction to run motor (open or closed) or off if waiting for motor to stop
//    //   * callback  handler to deliver end-stop and position-encoder callbacks to;
//    //     non-null and callbacks must return very quickly
//    // If too few ticks remain before the end of the sub-cycle for the minimum run,
//    // then this will return true immediately.
//    // Invokes callbacks for high current (end stop) and position (shaft) encoder where applicable.
//    // Aborts early if high current is detected at the start,
//    // or after the minimum run period.
//    // Returns true if aborted early from too little time to start, or by high current (assumed end-stop hit).
//    virtual bool spinSCTTicks(uint8_t maxRunTicks, uint8_t minTicksBeforeAbort, motor_drive dir, HardwareMotorDriverInterfaceCallbackHandler &callback) = 0;

  public:
    // Detect (poll) if end-stop is reached or motor current otherwise very high.
    virtual bool isCurrentHigh(HardwareMotorDriverInterface::motor_drive mdir = motorDriveOpening) const = 0;

    // Call to actually run/stop motor.
    // May take as much as (say) 200ms eg to change direction.
    // Stopping (removing power) should typically be very fast, << 100ms.
    //   * maxRunTicks  maximum sub-cycle ticks to attempt to run/spin for); zero will run for shortest reasonable time
    //   * dir  direction to run motor (or off/stop)
    //   * callback  callback handler
    virtual void motorRun(uint8_t maxRunTicks, motor_drive dir, HardwareMotorDriverInterfaceCallbackHandler &callback) = 0;
  };

#endif
