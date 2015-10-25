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

#include "V0p2_Main.h"

#include "V0p2_Generic_Config.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

// Arduino libraries.
//#include <Wire.h>
#ifdef ALLOW_CC1_SUPPORT
#include <OTProtocolCC.h>
#endif

#include "Control.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "Power_Management.h"
#include "RFM22_Radio.h"
#include "Security.h"
#include "Serial_IO.h"
#include "UI_Minimal.h"




// Link in support for alternate Power On Self-Test and main loop if required.
#if defined(ALT_MAIN_LOOP)

// Mask for Port B input change interrupts.
#define MASK_PB_BASIC 0b00000000 // Nothing.
#ifdef PIN_RFM_NIRQ
  #if (PIN_RFM_NIRQ < 8) || (PIN_RFM_NIRQ > 15)
    #error PIN_RFM_NIRQ expected to be on port B
  #endif
  #define RFM23B_INT_MASK (1 << (PIN_RFM_NIRQ&7))
  #define MASK_PB (MASK_PB_BASIC | RFM23B_INT_MASK)
#else
  #define MASK_PB MASK_PB_BASIC
#endif


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
    //PCMSK0 = PB; PCINT  0--7    (LEARN1 and Radio)
    //PCMSK1 = PC; PCINT  8--15
    //PCMSK2 = PD; PCINT 16--24   (LEARN2 and MODE, RX)

    PCICR =
#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
        1 | // 0x1 enables PB/PCMSK0.
#endif
#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
        2 | // 0x2 enables PC/PCMSK1.
#endif
#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
        4 | // 0x4 enables PD/PCMSK2.
#endif
        0;

#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
    PCMSK0 = MASK_PB;
#endif
#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
    PCMSK1 = MASK_PC;
#endif
#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
    PCMSK2 = MASK_PD;
#endif
    }


  RFM23B.listen(true);
  }


#if defined(ALT_MAIN_LOOP)

#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
//// Interrupt count.  Marked volatile so safe to read without a lock as is a single byte.
//static volatile uint8_t intCountPB;
// Previous state of port B pins to help detect changes.
static volatile uint8_t prevStatePB;
// Interrupt service routine for PB I/O port transition changes.
ISR(PCINT0_vect)
  {
//  ++intCountPB;
  const uint8_t pins = PINB;
  const uint8_t changes = pins ^ prevStatePB;
  prevStatePB = pins;

#if defined(RFM23B_INT_MASK)
  // RFM23B nIRQ falling edge is of interest.
  // Handler routine not required/expected to 'clear' this interrupt.
  // TODO: try to ensure that OTRFM23BLink.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
  if((changes & RFM23B_INT_MASK) && !(pins & RFM23B_INT_MASK))
    { RFM23B.handleInterruptSimple(); }
#endif

  }
#endif

#if defined(MASK_PC) && (MASK_PC != 0) // If PB interrupts required.
// Previous state of port C pins to help detect changes.
static volatile uint8_t prevStatePC;
// Interrupt service routine for PC I/O port transition changes.
ISR(PCINT1_vect)
  {
//  const uint8_t pins = PINC;
//  const uint8_t changes = pins ^ prevStatePC;
//  prevStatePC = pins;
//
// ...
  }
#endif

#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
// Previous state of port D pins to help detect changes.
static volatile uint8_t prevStatePD;
// Interrupt service routine for PD I/O port transition changes (including RX).
ISR(PCINT2_vect)
  {
//  const uint8_t pins = PIND;
//  const uint8_t changes = pins ^ prevStatePD;
//  prevStatePD = pins;
//
// ....
  }
#endif

#endif // ALT_MAIN







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
  OTV0P2BASE::powerDownSerial(); // Ensure that serial I/O is off.
  // Power down most stuff (except radio for hub RX).
  minimisePowerWithoutSleep();
#endif
  static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = OTV0P2BASE::getSecondsLT()))
    {
    // Poll I/O and process message incrementally (in this otherwise idle time)
    // before sleep and on wakeup in case some IO needs further processing now,
    // eg work was accrued during the previous major slow/outer loop
    // or the in a previous orbit of this loop sleep or nap was terminated by an I/O interrupt.
    // Come back and have another go if work was done, until the next tick at most.
    if(handleQueuedMessages(&Serial, true, &RFM23B)) { continue; }

// If missing h/w interrupts for anything that needs rapid response
// then AVOID the lowest-power long sleep.
#if CONFIG_IMPLIES_MAY_NEED_CONTINUOUS_RX && !defined(PIN_RFM_NIRQ)
#define MUST_POLL_FREQUENTLY true
#else
#define MUST_POLL_FREQUENTLY false
#endif
    if(MUST_POLL_FREQUENTLY /** && in hub mode */ )
      {
      // No h/w interrupt wakeup on receipt of frame,
      // so can only sleep for a short time between explicit poll()s,
      // though allow wake on interrupt anyway to minimise loop timing jitter.
      OTV0P2BASE::nap(WDTO_15MS, true);
      }
    else
      {
      // Normal long minimal-power sleep until wake-up interrupt.
      // Rely on interrupt to force fall through to I/O poll() below.
      OTV0P2BASE::sleepUntilInt();
      }
//    DEBUG_SERIAL_PRINTLN_FLASHSTRING("w"); // Wakeup.

//    idle15AndPoll(); // Attempt to crash the board!

    }
  TIME_LSD = newTLSD;

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif


  // START LOOP BODY
  // ===============

//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*");

//  // Power up serial for the loop body.
//  // May just want to turn it on in POSTalt() and leave it on...
//  const bool neededWaking = powerUpSerialIfDisabled();


//#if defined(USE_MODULE_FHT8VSIMPLE)
//  // Try for double TX for more robust conversation with valve?
//  const bool doubleTXForFTH8V = false;
//  // FHT8V is highest priority and runs first.
//  // ---------- HALF SECOND #0 -----------
//  bool useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_First(doubleTXForFTH8V); // Time for extra TX before UI.
////  if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@0"); }
//#endif

















//#if defined(USE_MODULE_FHT8VSIMPLE)
//  if(useExtraFHT8VTXSlots)
//    {
//    // Time for extra TX before other actions, but don't bother if minimising power in frost mode.
//    // ---------- HALF SECOND #1 -----------
//    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(doubleTXForFTH8V); 
////    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@1"); }
//    }
//#endif





//#if defined(USE_MODULE_FHT8VSIMPLE) && defined(V0P2BASE_TWO_S_TICK_RTC_SUPPORT)
//  if(useExtraFHT8VTXSlots)
//    {
//    // ---------- HALF SECOND #2 -----------
//    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(doubleTXForFTH8V); 
////    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@2"); }
//    }
//#endif





//#if defined(USE_MODULE_FHT8VSIMPLE) && defined(V0P2BASE_TWO_S_TICK_RTC_SUPPORT)
//  if(useExtraFHT8VTXSlots)
//    {
//    // ---------- HALF SECOND #3 -----------
//    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(doubleTXForFTH8V); 
////    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@3"); }
//    }
//#endif


#ifdef HAS_DORM1_VALVE_DRIVE
  // Yank valve to random target every minute to try to upset it!
  if(0 == TIME_LSD) { ValveDirect.set(OTV0P2BASE::randRNG8() % 101); }

  // Provide regular poll to motor driver.
  // May take significant time to run
  // so don't call when timing is critical or not much left,
  // eg around critical TXes.
  ValveDirect.read();
#endif



//  // Reading shaft encoder.
//  // Measure motor count against (fixed) internal reference.
//  power_intermittent_peripherals_enable(true);
//  const uint16_t mc = analogueNoiseReducedRead(MOTOR_DRIVE_MC_AIN, INTERNAL);
//  void power_intermittent_peripherals_disable();
//  DEBUG_SERIAL_PRINT_FLASHSTRING("Count input: ");
//  DEBUG_SERIAL_PRINT(mc);
//  DEBUG_SERIAL_PRINTLN();


//  // Command-Line Interface (CLI) polling.
//  // If a reasonable chunk of the minor cycle remains after all other work is done
//  // AND the CLI is / should be active OR a status line has just been output
//  // then poll/prompt the user for input
//  // using a timeout which should safely avoid overrun, ie missing the next basic tick,
//  // and which should also allow some energy-saving sleep.
//#if 1 // && defined(SUPPORT_CLI)
//  if(true)
//    {
//    const uint8_t sct = getSubCycleTime();
//    const uint8_t listenTime = max(GSCT_MAX/16, CLI_POLL_MIN_SCT);
//    if(sct < (GSCT_MAX - 2*listenTime))
//      // Don't listen beyond the last 16th of the cycle,
//      // or a minimal time if only prodding for interaction with automated front-end,
//      // as listening for UART RX uses lots of power.
//      { pollCLI(OTV0P2BASE::randRNG8NextBoolean() ? (GSCT_MAX-listenTime) : (sct+CLI_POLL_MIN_SCT), 0 == TIME_LSD); }
//    }
//#endif











//  // Force any pending output before return / possible UART power-down.
//  flushSerialSCTSensitive();
//  if(neededWaking) { OTV0P2BASE::powerDownSerial(); }
  }



#endif
