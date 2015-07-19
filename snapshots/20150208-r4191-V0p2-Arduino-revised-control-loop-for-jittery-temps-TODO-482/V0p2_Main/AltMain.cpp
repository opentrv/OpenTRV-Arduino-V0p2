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
  Alternate POST / setup and loop / main for non-OpenTRV code running on OpenTRV h/w platform.

  Also for rapid prototyping without dead-weight of OpenTRV intricate timing, etc!
 */


// Arduino libraries.
//#include <Wire.h>

#include "V0p2_Main.h"

#include "V0p2_Generic_Config.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

#include "Control.h"
#include "EEPROM_Utils.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "RTC_Support.h"
#include "Power_Management.h"
#include "PRNG.h"
#include "RFM22_Radio.h"
#include "Security.h"
#include "Serial_IO.h"
#include "UI_Minimal.h"




// Link in support for alternate Power On Self-Test and main loop if required.
#if defined(ALT_MAIN_LOOP)

// Called from startup() after some initial setup has been done.
// Can abort with panic() if need be.
void POSTalt()
  {
#ifdef USE_MODULE_RFM22RADIOSIMPLE
#if !defined(RFM22_IS_ACTUALLY_RFM23) && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("(Using RFM22.)");
#endif
  // Initialise the radio, if configured, ASAP, because it can suck a lot of power until properly initialised.
  RFM22PowerOnInit();
  // Check that the radio is correctly connected; panic if not...
  if(!RFM22CheckConnected()) { panic(); }
  // Configure the radio.
  RFM22RegisterBlockSetup(FHT8V_RFM22_Reg_Values);
  // Put the radio in low-power standby mode.
  RFM22ModeStandbyAndClearState();
#endif
  // Force initialisation into low-power state.
  const int heat = TemperatureC16.read();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("temp: ");
  DEBUG_SERIAL_PRINT(heat);
  DEBUG_SERIAL_PRINTLN();
#endif
  const int light = AmbLight.read();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("light: ");
  DEBUG_SERIAL_PRINT(light);
  DEBUG_SERIAL_PRINTLN();
#endif
  }


//// Version of loopAlt() used to verify 32768Hz xtal clock speed.
//// Outputs a '*' every 2s.
//// Avoids wakeup from sleep and (eg) RTC interrupt to minimise jitter.
//void loopAlt()
//  {
//  const uint8_t magicTick = 0x3f;
//  // Wait until magic tick has passed if not already done so.
//  while(magicTick == getSubCycleTime()) { }
//  // Wait until magic tick some time after start of underlying 2s cycle.
//  while(magicTick != getSubCycleTime()) { }
//  // Signal to host ASAP with minimum jitter.
//  DEBUG_SERIAL_PRINT_FLASHSTRING("*");
//  }


#if defined(DIRECT_MOTOR_DRIVE_V1) && defined(ALT_MAIN_LOOP) && defined(DEBUG)
// Static instance of motor driver.  BE CAREFUL.
static ValveMotorDirectV1HardwareDriver hd;

class callbacks : public HardwareMotorDriverInterfaceCallbackHandler
  {
  public:
    // Set true when end-stop hit.
    volatile bool hitEndStop;

    // Called when end stop hit, eg by overcurrent detection.
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    virtual void signalHittingEndStop() { hitEndStop = true; }
  
    // Called when encountering leading edge of a mark in the shaft rotation in forward direction (falling edge in reverse).
    // Can be called while run() is in progress.
    // Is ISR-/thread- safe.
    virtual void signalShaftEncoderMarkStart() { }
  };
static callbacks cb;
#endif

// Called from loop().
void loopAlt()
  {
  // Sleep in low-power mode (waiting for interrupts) until seconds roll.
  // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
  // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*E"); // End-of-cycle sleep.
#endif

#if defined(WAKEUP_32768HZ_XTAL) // Normal 32768Hz crystal driving main timing.
  powerDownSerial(); // Ensure that serial I/O is off.
  // Power down most stuff (except radio for hub RX).
  minimisePowerWithoutSleep();
//  RFM22ModeStandbyAndClearState();
  static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = getSecondsLT()))
    {
    sleepUntilInt(); // Normal long minimal-power sleep until wake-up interrupt.
//    DEBUG_SERIAL_PRINTLN_FLASHSTRING("w"); // Wakeup.
    }
  TIME_LSD = newTLSD;
#else // Keep running on main RC clock, simulating normal-ish sleep length.
  delay(2000);
#endif

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif


  // START LOOP BODY
  // ===============

  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*");



//  const bool neededWaking = powerUpSerialIfDisabled();

#if defined(DIRECT_MOTOR_DRIVE_V1) && defined(ALT_MAIN_LOOP) && defined(DEBUG)
  // Ensure that end-stop current-sense feedback is enabled before starting the motor.
  cb.hitEndStop = false;
  hd.enableFeedback(true, cb);
  // Ensure that the motor is running...
  static bool open = true;
  if(open) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("opening"); } else { DEBUG_SERIAL_PRINTLN_FLASHSTRING("closing"); }
  hd.motorRun(open ? HardwareMotorDriverInterface::motorDriveOpening : HardwareMotorDriverInterface::motorDriveClosing);
  // Try to ride through any start-up transients...
  nap(WDTO_120MS);
  nap(WDTO_120MS);
  nap(WDTO_120MS);
  nap(WDTO_120MS);
  // Spin the motor for up to about 1.5s polling for end-stop hit, etc.
  while(!cb.hitEndStop && (getSubCycleTime() < 0xc0))
    {
    hd.enableFeedback(true, cb);
    }
  // Stop motor.
  hd.motorRun(HardwareMotorDriverInterface::motorOff);
  // Iff the end-stop was hit then reverse the motor.
  if(cb.hitEndStop)
    {
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("Hit end stop; reversing...");
    open = !open;
    }
#endif

//  static bool boilerOut;
//  boilerOut = !boilerOut;
//  if(boilerOut) { LED_HEATCALL_ON() } else { LED_HEATCALL_OFF(); }
//  fastDigitalWrite(OUT_HEATCALL, boilerOut ? HIGH : LOW);
//#if defined(LED_UI2_L)
//  if(boilerOut) { LED_UI2_OFF() } else { LED_UI2_ON(); }
//#endif

//  // Force any pending output before return / possible UART power-down.
//  flushSerialSCTSensitive();
//
//  if(neededWaking) { powerDownSerial(); }
  }



#endif
