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

  DHD20151208: TEST CODE FOR REV7 PRODUCTION 2015Q4.
 */


#include "V0p2_Main.h"

#include "V0p2_Generic_Config.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

// Arduino libraries.
//#include <Wire.h>
#ifdef ALLOW_CC1_SUPPORT
#include <OTProtocolCC.h>
#endif
#include <OTV0p2Base.h>
#include <OTRadioLink.h>

#include "Power_Management.h"
#include "RFM22_Radio.h"
#include "Serial_IO.h"
#include "V0p2_Sensors.h"



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


// Mask for Port D input change interrupts.
#define MASK_PD_BASIC 0b00000001 // Just RX.
#if defined(ENABLE_VOICE_SENSOR)
#if VOICE_NIRQ > 7
#error voice interrupt on wrong port
#endif
#define VOICE_INT_MASK (1 << (VOICE_NIRQ&7))
#define MASK_PD (MASK_PD_BASIC | VOICE_INT_MASK)
#else
#define MASK_PD MASK_PD_BASIC // Just RX.
#endif

static const OTRadioLink::OTRadioChannelConfig RFMConfig(OTRadValve::FHT8VRadValveBase::FHT8V_RFM23_Reg_Values, true, true, true);

// Called from startup() after some initial setup has been done.
// Can abort with panic() if need be.
void POSTalt()
  {

#if defined(USE_MODULE_RFM22RADIOSIMPLE) 
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  RFM23B.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!RFM23B.configure(1, &RFMConfig) || !RFM23B.begin()) { panic(F("PANIC!")); }
#endif


//  // Force initialisation into low-power state.
//  const int heat = TemperatureC16.read();
//#if 1 && defined(DEBUG)
//  DEBUG_SERIAL_PRINT_FLASHSTRING("temp: ");
//  DEBUG_SERIAL_PRINT(heat);
//  DEBUG_SERIAL_PRINTLN();
//#endif
//  const int light = AmbLight.read();
//#if 1 && defined(DEBUG)
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

#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
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

  const uint8_t pins = PIND;
  const uint8_t changes = pins ^ prevStatePD;
  prevStatePD = pins;

#if defined(ENABLE_VOICE_SENSOR)
  //  // Voice detection is a falling edge.
  //  // Handler routine not required/expected to 'clear' this interrupt.
  //  // FIXME: ensure that Voice.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
    // Voice detection is a RISING edge.
    if((changes & VOICE_INT_MASK) && (pins & VOICE_INT_MASK)) {
      Voice.handleInterruptSimple();
    }
#endif // ENABLE_VOICE_SENSOR

//    // If an interrupt arrived from no other masked source then wake the CLI.
//    // The will ensure that the CLI is active, eg from RX activity,
//    // eg it is possible to wake the CLI subsystem with an extra CR or LF.
//    // It is OK to trigger this from other things such as button presses.
//    // FIXME: ensure that resetCLIActiveTimer() is inlineable to minimise ISR prologue/epilogue time and space.
//    if(!(changes & MASK_PD & ~1)) { resetCLIActiveTimer(); }
  }
#endif // defined(MASK_PD) && (MASK_PD != 0)

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
    OTV0P2BASE::sleepUntilInt();
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

  static bool soakTestMode;

  // Flash lights, read sensors.
  LED_HEATCALL_ON();
  LED_UI2_ON();
  const bool m = (LOW == fastDigitalRead(BUTTON_MODE_L));
  const bool l1 = (LOW == fastDigitalRead(BUTTON_LEARN_L));
  const bool l2 = (LOW == fastDigitalRead(BUTTON_LEARN2_L));
  if(m || l1 || l2)
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("button(s): ");
    DEBUG_SERIAL_PRINT(m ? 'm' : ' ');
    DEBUG_SERIAL_PRINT(l1 ? 'l' : ' ');
    DEBUG_SERIAL_PRINT(l2 ? '2' : ' ');
    DEBUG_SERIAL_PRINTLN();
    }
  // Toggle soak-test mode with mode button held down.
  if(m)
    {
    soakTestMode = !soakTestMode;
    DEBUG_SERIAL_PRINT_FLASHSTRING("soak test mode: ");
    DEBUG_SERIAL_PRINT(soakTestMode);
    DEBUG_SERIAL_PRINTLN();
    }
  // Open or close as directed by learn buttons; only one can be true.
  const bool open = l1 && !l2;
  const bool close = !l1 && l2;
  if(open || close)
    {
    soakTestMode = !soakTestMode;
    DEBUG_SERIAL_PRINT_FLASHSTRING("manual valve open: ");
    DEBUG_SERIAL_PRINT(open);
    DEBUG_SERIAL_PRINTLN();
    }
  LED_UI2_OFF();




//#if defined(USE_MODULE_FHT8VSIMPLE)
//  // Try for double TX for more robust conversation with valve?
//  const bool doubleTXForFTH8V = false;
//  // FHT8V is highest priority and runs first.
//  // ---------- HALF SECOND #0 -----------
//  bool useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_First(doubleTXForFTH8V); // Time for extra TX before UI.
////  if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@0"); }
//#endif



  switch(TIME_LSD >> 1) // 30s cycle to reduce boredom!
    {
    case 1:
      {
      const int light = AmbLight.read();
      DEBUG_SERIAL_PRINT_FLASHSTRING("light: ");
      DEBUG_SERIAL_PRINT(light);
      DEBUG_SERIAL_PRINTLN();
      break;
      }

    case 2:
      {
      const int heat = TemperatureC16.read();
      DEBUG_SERIAL_PRINT_FLASHSTRING("temp: ");
      DEBUG_SERIAL_PRINT(heat);
      DEBUG_SERIAL_PRINTLN();
      break;
      }

    case 3:
      {
      const int rh = RelHumidity.read();
      DEBUG_SERIAL_PRINT_FLASHSTRING("RH%: ");
      DEBUG_SERIAL_PRINT(rh);
      DEBUG_SERIAL_PRINTLN();
      break;
      }

    case 4:
      {
      const int tp = TempPot.read();
      DEBUG_SERIAL_PRINT_FLASHSTRING("dial: ");
      DEBUG_SERIAL_PRINT(tp);
      DEBUG_SERIAL_PRINTLN();
      break;
      }
  }

 LED_HEATCALL_OFF();




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


//#ifdef HAS_DORM1_VALVE_DRIVE
//  // Move valve to new target every minute to try to upset it!
//  // Targets at key thresholds and random.
//  if(0 == TIME_LSD)
//    {
//    switch(OTV0P2BASE::randRNG8() & 1)
//      {
//      case 0: ValveDirect.set(OTRadValve::DEFAULT_VALVE_PC_MIN_REALLY_OPEN-1); break; // Nominally shut.
//      case 1: ValveDirect.set(OTRadValve::DEFAULT_VALVE_PC_MODERATELY_OPEN); break; // Nominally open.
//      // Random.
////      default: ValveDirect.set(OTV0P2BASE::randRNG8() % 101); break;
//      }
//    }
//
//  // Simulate human doing the right thing after fitting valve when required.
//  if(ValveDirect.isWaitingForValveToBeFitted()) { ValveDirect.signalValveFitted(); }
//
//  // Provide regular poll to motor driver.
//  // May take significant time to run
//  // so don't call when timing is critical or not much left,
//  // eg around critical TXes.
//  const uint8_t pc = ValveDirect.read();
//  DEBUG_SERIAL_PRINT_FLASHSTRING("Pos%: ");
//  DEBUG_SERIAL_PRINT(pc);
//  DEBUG_SERIAL_PRINTLN();
//#endif



//  // Reading shaft encoder.
//  // Measure motor count against (fixed) internal reference.
//  power_intermittent_peripherals_enable(true);
//  const uint16_t mc = analogueNoiseReducedRead(MOTOR_DRIVE_MC_AIN, INTERNAL);
//  void power_intermittent_peripherals_disable();
//  DEBUG_SERIAL_PRINT_FLASHSTRING("Count input: ");
//  DEBUG_SERIAL_PRINT(mc);
//  DEBUG_SERIAL_PRINTLN();














//  // Force any pending output before return / possible UART power-down.
//  flushSerialSCTSensitive();
//  if(neededWaking) { OTV0P2BASE::powerDownSerial(); }
  }



#endif
