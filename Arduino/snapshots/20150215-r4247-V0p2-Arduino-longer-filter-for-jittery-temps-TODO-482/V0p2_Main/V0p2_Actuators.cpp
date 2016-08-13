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
 V0p2 boards physical actuator support.
 */
#include <stdint.h>
#include <limits.h>
#include <util/atomic.h>

#include <Wire.h> // Arduino I2C library.

#include "Actuator.h"

#include "V0p2_Main.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "V0p2_Actuators.h" // I/O code access.

#include "Serial_IO.h"
#include "Power_Management.h"





#ifdef DIRECT_MOTOR_DRIVE_V1

// Call to actually run/stop low-level motor.
// May take as much as 200ms eg to change direction.
// Stopping (removing power) should typically be very fast, << 100ms.
void ValveMotorDirectV1HardwareDriver::motorRun(const motor_drive dir)
  {
  // *** MUST NEVER HAVE L AND R LOW AT THE SAME TIME else board may be destroyed at worst. ***
  // Operates as quickly as reasonably possible, eg to move to stall detection quickly...
  // TODO: consider making atomic to block some interrupt-related accidents...
  // TODO: note that the mapping between L/R and open/close not yet defined.
  // DHD20150205: 1st cut REV7 all-in-in-valve, seen looking down from valve into base, cw => close (ML=HIGH), ccw = open (MR=HIGH).
  switch(dir)
    {
    case motorDriveClosing:
      {
      // Pull one side high immediately *FIRST* for safety.
      // Stops motor if other side is not already low.
      // (Has no effect if motor is already running in the correct direction.)
      fastDigitalWrite(MOTOR_DRIVE_ML, HIGH);
      pinMode(MOTOR_DRIVE_ML, OUTPUT); // Ensure that the HIGH side is an output (can be done after, as else will be safe weak pull-up).
      nap(WDTO_120MS); // Let H-bridge respond and settle, and motor slow down.
      pinMode(MOTOR_DRIVE_MR, OUTPUT); // Ensure that the LOW side is an output.
      fastDigitalWrite(MOTOR_DRIVE_MR, LOW); // Pull LOW last.
      nap(WDTO_15MS); // Let H-bridge respond and settle.
//LED_HEATCALL_ON();
//LED_UI2_OFF();
      break; // Fall through to common case.
      }

    case motorDriveOpening:
      {
      // Pull one side high immediately *FIRST* for safety.
      // Stops motor if other side is not already low.
      // (Has no effect if motor is already running in the correct direction.)
      fastDigitalWrite(MOTOR_DRIVE_MR, HIGH); 
      pinMode(MOTOR_DRIVE_MR, OUTPUT); // Ensure that the HIGH side is an output (can be done after, as else will be safe weak pull-up).
      nap(WDTO_120MS); // Let H-bridge respond and settle, and motor slow down.
      pinMode(MOTOR_DRIVE_ML, OUTPUT); // Ensure that the LOW side is an output.
      fastDigitalWrite(MOTOR_DRIVE_ML, LOW); // Pull LOW last.
      nap(WDTO_15MS); // Let H-bridge respond and settle.
//LED_HEATCALL_OFF();
//LED_UI2_ON();
      break; // Fall through to common case.
      }

    case motorOff: default: // Explicit off, and default for safety.
      {
      // Everything off...
      fastDigitalWrite(MOTOR_DRIVE_MR, HIGH); // Belt and braces force pin logical output state high.
      pinMode(MOTOR_DRIVE_MR, INPUT_PULLUP); // Switch to weak pull-up; slow but possibly marginally safer.
      nap(WDTO_15MS); // Let H-bridge respond and settle.
      fastDigitalWrite(MOTOR_DRIVE_ML, HIGH); // Belt and braces force pin logical output state high.
      pinMode(MOTOR_DRIVE_ML, INPUT_PULLUP); // Switch to weak pull-up; slow but possibly marginally safer.
      nap(WDTO_15MS); // Let H-bridge respond and settle.
      return; // Return, not fall through.
      }
    }
  }


#define MI_NEEDS_ADC // Defined if MI output swing is not enough to use fast comparator.

// Enable/disable end-stop detection and shaft-encoder.
// Disabling should usually forces the motor off,
// with a small pause for any residual movement to complete.
void ValveMotorDirectV1HardwareDriver::enableFeedback(const bool enable, HardwareMotorDriverInterfaceCallbackHandler &callback)
  {
  // Check for high motor current indicating hitting an end-stop.
#if !defined(MI_NEEDS_ADC)
  const bool currentSense = analogueVsBandgapRead(MOTOR_DRIVE_MI_AIN, true);
#else
  // Measure motor current against (fixed) internal reference.
  const uint16_t mi = analogueNoiseReducedRead(MOTOR_DRIVE_MI_AIN, INTERNAL);
  const uint16_t miHigh = 250; // Typical *start* current 430 observed at 2.4V, REV7 board DHD20150205 (370@2.0V, 550@3.3V).
  const bool currentSense = (mi > miHigh) &&
    // Recheck the value read in case spiky.
    (analogueNoiseReducedRead(MOTOR_DRIVE_MI_AIN, INTERNAL) > miHigh) && (analogueNoiseReducedRead(MOTOR_DRIVE_MI_AIN, INTERNAL) > miHigh);
  if(mi > ((3*miHigh)/4)) { DEBUG_SERIAL_PRINT(mi); DEBUG_SERIAL_PRINTLN(); }
#endif
  if(currentSense) { LED_UI2_ON(); } else { LED_UI2_OFF(); }
  if(currentSense) { callback.signalHittingEndStop(); } 
  }


// Actuator/driver for direct local (radiator) valve motor control.
uint8_t ValveMotorDirectV1::read()
  {
  // Call the generic read() first.
//  AbstractCurrentSenseValveMotorDirectV1::read();

  // TODO

  }

//#if 1 && defined(ALT_MAIN_LOOP) && defined(DEBUG)
//// Drive motor back and forth (toggle direction each call) just for testing/fun.
//void ValveMotorDirectV1::flip()
//  {
//  static bool open;
//  open = !open;
//  motorDrive(open ? motorDriveOpening : motorDriveClosing);
//  }
//#endif

// Minimally wiggles the motor to give tactile feedback and/or show to be working.
// Does not itself track movement against shaft encoder, etc, or check for stall.
// May take a significant fraction of a second.
// Finishes with the motor turned off.
void ValveMotorDirectV1::wiggle()
  {
//  motorDrive(motorOff);
//  motorDrive(motorDriveOpening);
//  nap(WDTO_120MS);
//  motorDrive(motorDriveClosing);
//  nap(WDTO_120MS);
//  motorDrive(motorOff);
  }

//// Turn motor off, or on for a given drive direction.
//// This routine is very careful to avoid setting outputs into any illegal/'bad' state.
//// Sets flags accordingly.
//// Does not provide any monitoring of stall, position encoding, etc.
//// May take significant time (~150ms) to gently stop motor.
//void ValveMotorDirectV1::motorDrive(const motor_drive dir)
//  {
//  // *** MUST NEVER HAVE L AND R LOW AT THE SAME TIME else board may be destroyed at worst. ***
//  // Operates as quickly as reasonably possible, eg to move to stall detection quickly...
//  // TODO: consider making atomic to block some interrupt-related accidents...
//  // TODO: note that the mapping between L/R and open/close not yet defined.
//  switch(dir)
//    {
//    case motorDriveOpening:
//      {
//      fastDigitalWrite(MOTOR_DRIVE_ML, HIGH); // Pull one side high immediately *FIRST* for safety.
//      nap(WDTO_120MS); // Let H-bridge respond and settle, and motor slow down.
//      pinMode(MOTOR_DRIVE_MR, OUTPUT); // Ensure that the LOW side is an output.
//      fastDigitalWrite(MOTOR_DRIVE_MR, LOW); // Pull other side side low after.
//      nap(WDTO_15MS); // Let H-bridge respond and settle.
////LED_HEATCALL_ON();
////LED_UI2_OFF();
//      break; // Fall through to common case.
//      }
//
//    case motorDriveClosing:
//      {
//      fastDigitalWrite(MOTOR_DRIVE_MR, HIGH); // Pull one side high immediately *FIRST* for safety.
//      nap(WDTO_120MS); // Let H-bridge respond and settle, and motor slow down.
//      pinMode(MOTOR_DRIVE_ML, OUTPUT); // Ensure that the LOW side is an output.
//      fastDigitalWrite(MOTOR_DRIVE_ML, LOW); // Pull other side side low after.
//      nap(WDTO_15MS); // Let H-bridge respond and settle.
////LED_HEATCALL_OFF();
////LED_UI2_ON();
//      break; // Fall through to common case.
//      }
//
//    case motorOff: default: // Explicit off, and default for safety.
//      {
//      // Everything off...
//      fastDigitalWrite(MOTOR_DRIVE_MR, HIGH); // Belt and braces force pin logical output state high.
//      pinMode(MOTOR_DRIVE_MR, INPUT_PULLUP); // Switch to weak pull-up; slow but possibly marginally safer.
//      nap(WDTO_15MS); // Let H-bridge respond and settle.
//      fastDigitalWrite(MOTOR_DRIVE_ML, HIGH); // Belt and braces force pin logical output state high.
//      pinMode(MOTOR_DRIVE_ML, INPUT_PULLUP); // Switch to weak pull-up; slow but possibly marginally safer.
//      nap(WDTO_15MS); // Let H-bridge respond and settle.
//      motorDriveStatus = motorOff; // Ensure value state even if 'dir' invalid.
//      return; // Return, not fall through.
//      }
//    }
//
//  // If state has changed to new 'active' state,
//  // force both lines to outputs (which may be relatively slow)
//  // and update this instance's state.
//  if(motorDriveStatus != dir) 
//    {
//    pinMode(MOTOR_DRIVE_ML, OUTPUT);
//    pinMode(MOTOR_DRIVE_MR, OUTPUT);
//    motorDriveStatus = dir;
//    }
//  }

// Singleton implementation/instance.
ValveMotorDirectV1 ValveDirect;
#endif

