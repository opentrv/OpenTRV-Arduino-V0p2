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


// Arduino libraries imported here (even for use in other .cpp files).
#include <Wire.h>

#include "V0p2_Main.h"

#include "V0p2_Generic_Config.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

#include "Ambient_Light_Sensor.h"
#include "Control.h"
#include "EEPROM_Utils.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "RTC_Support.h"
#include "Power_Management.h"
#include "PRNG.h"
#include "RFM22_Radio.h"
#include "Security.h"
#include "Sensor_SHT21.h"
#include "Serial_IO.h"
#include "Temperature_Sensor.h"
#include "Temp_Pot.h"
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

  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Setting up interrupts");
  cli();
  PCICR = 0x05;
  PCMSK0 = 0b00000011; // PB; PCINT  0--7    (LEARN1 and Radio)
  PCMSK1 = 0b00000000; // PC; PCINT  8--15
  PCMSK2 = 0b00101001; // PD; PCINT 16--24   (LEARN2 and MODE, RX)
  sei();	
  }



static volatile uint8_t interruptCount;



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
  static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = getSecondsLT()))
    {
    sleepUntilInt(); // Normal long minimal-power sleep until wake-up interrupt.
    }
  TIME_LSD = newTLSD;
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif


  // START LOOP BODY
  // ===============
 
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("tick...");



  DEBUG_SERIAL_PRINTLN_FLASHSTRING("int count: ");
  DEBUG_SERIAL_PRINT(interruptCount);
  DEBUG_SERIAL_PRINTLN();





  }

// Interrupt service routine for I/O port transition changes.
ISR(PCINT0_vect)
  {
  ++interruptCount;
  }

ISR(PCINT1_vect)
  {
  ++interruptCount;
  }

ISR(PCINT2_vect)
  {
  ++interruptCount;
  }

#endif
