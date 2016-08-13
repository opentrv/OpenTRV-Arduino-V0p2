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

Author(s) / Copyright (s): Damon Hart-Davis 2014
*/

/*
  Alternate POST / setup and loop / main for non-OpenTRV code running on OpenTRV h/w platform.

  Also for rapid prototyping without dead-weight of OpenTRV intricate timing, etc!
 */


// Arduino libraries.
#include <Wire.h>

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
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("light: ");
  DEBUG_SERIAL_PRINT(light);
  DEBUG_SERIAL_PRINTLN();
#endif
  }




// Called from loop().
void loopAlt()
  {
  // Sleep in low-power mode (waiting for interrupts) until seconds roll.
  // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
  // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*E"); // End-of-cycle sleep.
#endif
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
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif


  // START LOOP BODY
  // ===============

  DEBUG_SERIAL_PRINTLN_FLASHSTRING("tick...");

  const bool neededWaking = powerUpSerialIfDisabled();


#ifdef DIRECT_MOTOR_DRIVE_V1
  ValveDirect.read();
#endif


  // Force any pending output before return / possible UART power-down.
  flushSerialSCTSensitive();

  if(neededWaking) { powerDownSerial(); }
  }



#endif
