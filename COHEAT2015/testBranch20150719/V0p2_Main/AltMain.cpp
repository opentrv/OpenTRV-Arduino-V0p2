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


//// Mask for Port D input change interrupts.
//#define MASK_PD_BASIC 0b00000001 // Just RX.
//#if defined(ENABLE_VOICE_SENSOR)
//#if VOICE_NIRQ > 7
//#error voice interrupt on wrong port
//#endif
//#define VOICE_INT_MASK (1 << (VOICE_NIRQ&7))
//#define MASK_PD (MASK_PD_BASIC | VOICE_INT_MASK)
//#else
//#define MASK_PD MASK_PD_BASIC // Just RX.
//#endif


// Called from startup() after some initial setup has been done.
// Can abort with panic() if need be.
void POSTalt()
  {
#if defined(USE_MODULE_RFM22RADIOSIMPLE) 
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  static const OTRadioLink::OTRadioChannelConfig RFMConfig(FHT8V_RFM22_Reg_Values, true, true, true);
  RFM23B.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!RFM23B.configure(1, &RFMConfig) || !RFM23B.begin()) { panic(); }
#endif

  // Force initialisation into low-power state.
  const int heat = TemperatureC16.read();
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("temp: ");
  DEBUG_SERIAL_PRINT(heat);
  DEBUG_SERIAL_PRINTLN();
#endif
//  const int light = AmbLight.read();
//#if 0 && defined(DEBUG)
//  DEBUG_SERIAL_PRINT_FLASHSTRING("light: ");
//  DEBUG_SERIAL_PRINT(light);
//  DEBUG_SERIAL_PRINTLN();
//#endif



  // Trailing setup for the run
  // --------------------------

  // Set up async edge interrupts.
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    //PCICR = 0x05;
    //PCMSK0 = 0b00000011; // PB; PCINT  0--7    (LEARN1 and Radio)
    //PCMSK1 = 0b00000000; // PC; PCINT  8--15
    //PCMSK2 = 0b00101001; // PD; PCINT 16--24   (LEARN2 and MODE, RX)
//    PCICR = 0x4; // 0x4 enables PD/PCMSK2.
//    PCMSK2 = MASK_PD; // PD; PCINT 16--24 (0b1 is PCINT16/RX)
    }



  RFM23B.listen(true);


  }

#if defined(ALT_MAIN_LOOP) // Do not define handlers here when alt main is in use.
// Previous state of port D pins to help detect changes.
static volatile uint8_t prevStatePD;
// Interrupt service routine for PD I/O port transition changes (including RX).
ISR(PCINT2_vect)
  {
//  const uint8_t pins = PIND;
//  const uint8_t changes = pins ^ prevStatePD;
//  prevStatePD = pins;
//
//#if defined(ENABLE_VOICE_SENSOR)
////  // Voice detection is a falling edge.
////  // Handler routine not required/expected to 'clear' this interrupt.
////  // FIXME: ensure that Voice.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
////  if((changes & VOICE_INT_MASK) && !(pins & VOICE_INT_MASK))
//  // Voice detection is a RISING edge.
//  // Handler routine not required/expected to 'clear' this interrupt.
//  // FIXME: ensure that Voice.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
//  if((changes & VOICE_INT_MASK) && (pins & VOICE_INT_MASK))
//    { Voice.handleInterruptSimple(); }
//#endif
//
//  // TODO: MODE button and other things...
//
//  // If an interrupt arrived from no other masked source then wake the CLI.
//  // The will ensure that the CLI is active, eg from RX activity,
//  // eg it is possible to wake the CLI subsystem with an extra CR or LF.
//  // It is OK to trigger this from other things such as button presses.
//  // FIXME: ensure that resetCLIActiveTimer() is inlineable to minimise ISR prologue/epilogue time and space.
//  if(!(changes & MASK_PD & ~1)) { resetCLIActiveTimer(); }
  }
#endif






// Position to move the valve to [0,100].
static uint8_t valvePosition = 42; // <<<<<<<< YOUR STUFF SETS THIS!




// Called from loop().
void loopAlt()
  {
  // Sleep in low-power mode (waiting for interrupts) until seconds roll.
  // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
  // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*E"); // End-of-cycle sleep.
#endif

#if !defined(MIN_ENERGY_BOOT)
  powerDownSerial(); // Ensure that serial I/O is off.
  // Power down most stuff (except radio for hub RX).
  minimisePowerWithoutSleep();
//  RFM22ModeStandbyAndClearState();
#endif
  static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = getSecondsLT()))
    {
    nap(WDTO_15MS, true);
//    sleepUntilInt(); // Normal long minimal-power sleep until wake-up interrupt.
//    DEBUG_SERIAL_PRINTLN_FLASHSTRING("w"); // Wakeup.
    RFM23B.poll();
    while(0 != RFM23B.getRXMsgsQueued())
      {
      uint8_t buf[65];
      const uint8_t msglen = RFM23B.getRXMsg(buf, sizeof(buf));
      const bool neededWaking = powerUpSerialIfDisabled();
      OTRadioLink::dumpRXMsg(buf, msglen);
      Serial.flush();
      if(neededWaking) { powerDownSerial(); }
      }
    }
  TIME_LSD = newTLSD;

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif


  // START LOOP BODY
  // ===============

//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*");

  // Power up serail for the loop body.
  // May just want to turn it on in POSTalt() and leave it on...
  const bool neededWaking = powerUpSerialIfDisabled();


#if defined(USE_MODULE_FHT8VSIMPLE)
  // Try for double TX for more robust conversation with valve?
  const bool doubleTXForFTH8V = false;
  // FHT8V is highest priority and runs first.
  // ---------- HALF SECOND #0 -----------
  bool useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_First(doubleTXForFTH8V); // Time for extra TX before UI.
//  if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@0"); }
#endif











// EXPERIMENTAL TEST OF NEW RADIO CODE
#if 1 && defined(DEBUG)
//    DEBUG_SERIAL_PRINT_FLASHSTRING("listening to channel: ");
//    DEBUG_SERIAL_PRINT(RFM23B.getListenChannel());
//    DEBUG_SERIAL_PRINTLN();

//    RFM23B.listen(false);
//    DEBUG_SERIAL_PRINT_FLASHSTRING("MODE ");
//    DEBUG_SERIAL_PRINT(RFM23B.getMode());
//    DEBUG_SERIAL_PRINTLN();
//    RFM23B.listen(true);
//    DEBUG_SERIAL_PRINT_FLASHSTRING("MODE ");
//    DEBUG_SERIAL_PRINT(RFM23B.getMode());
//    DEBUG_SERIAL_PRINTLN();
//    RFM23B.poll();
    for(uint8_t lastErr; 0 != (lastErr = RFM23B.getRXErr()); )
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING("err ");
      DEBUG_SERIAL_PRINT(lastErr);
      DEBUG_SERIAL_PRINTLN();
      }
    DEBUG_SERIAL_PRINT_FLASHSTRING("RSSI ");
    DEBUG_SERIAL_PRINT(RFM23B.getRSSI());
    DEBUG_SERIAL_PRINTLN();
    static uint8_t oldDroppedRecent;
    const uint8_t droppedRecent = RFM23B.getRXMsgsDroppedRecent();
    if(droppedRecent != oldDroppedRecent)
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING("?DROPPED recent: ");
      DEBUG_SERIAL_PRINT(droppedRecent);
      DEBUG_SERIAL_PRINTLN();
      oldDroppedRecent = droppedRecent;
      }
#endif





#if defined(USE_MODULE_FHT8VSIMPLE)
  if(0 == TIME_LSD)
    {
    // Once per minute regenerate valve-setting command ready to transmit.
    FHT8VCreateValveSetCmdFrame(valvePosition);
    }
#endif







#if defined(USE_MODULE_FHT8VSIMPLE)
  if(useExtraFHT8VTXSlots)
    {
    // Time for extra TX before other actions, but don't bother if minimising power in frost mode.
    // ---------- HALF SECOND #1 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(doubleTXForFTH8V); 
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@1"); }
    }
#endif





#if defined(USE_MODULE_FHT8VSIMPLE) && defined(TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // ---------- HALF SECOND #2 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(doubleTXForFTH8V); 
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@2"); }
    }
#endif





#if defined(USE_MODULE_FHT8VSIMPLE) && defined(TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // ---------- HALF SECOND #3 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(doubleTXForFTH8V); 
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@3"); }
    }
#endif














  // Force any pending output before return / possible UART power-down.
  flushSerialSCTSensitive();
  if(neededWaking) { powerDownSerial(); }
  }



#endif
