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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2017
                           Deniz Erbilgin 2015--2017
*/

/******************************************************************************
 * NOTES
 *****************************************************************************/
/**
 * @brief   Template for building a V0p2 app.
 * @TODO    Figure out how detailed this should be, e.g.
 *          - Do we want a template for setup and loop functions?
 *          - What generic functions should be included/assumed necessary?
 *            e.g. pollIO, CLI stuff etc.
 */

/******************************************************************************
 * INCLUDES
 *****************************************************************************/
 
////// GLOBAL flags that alter system build and behaviour.

// If defined, do extra checks and serial logging.  Will take more code space and power.
#undef DEBUG
// Ensure that OpenTRV 'standard' UART speed is set unless explicitly overridden.
#define BAUD 4800

// *** Global flag for REVx configuration here *** //
// e.g.
// #define CONFIG_REV10_SECURE_BHR // REV10: secure stats relay and boiler hub.

// Get defaults for valve applications.
#include <OTV0p2_valve_ENABLE_defaults.h>

// *** Main board config imported here *** //

// --------------------------------------------
// Fixups to apply after loading the target config.
#include <OTV0p2_valve_ENABLE_fixups.h>
// I/O pin allocation and setup: include ahead of I/O module headers.
#include <OTV0p2_Board_IO_Config.h>


////// MAIN LIBRARIES

#include <Arduino.h>
#include <OTV0p2Base.h>
#include <OTRadioLink.h>


////// ADDITIONAL LIBRARIES/PERIPHERALS

// #define RFM23B_IRQ_CONTROL  // Enable RFM23B IRQ control code.
// #include <OTRFM23BLink.h>
// #include <OTAESGCM.h>


////// ADDITIONAL USER HEADERS


/******************************************************************************
 * GENERAL CONSTANTS AND VARIABLES
 * Core constants and variables required for V0p2 loop.
 *****************************************************************************/



/******************************************************************************
 * PERIPHERALS
 * Peripheral drivers and config
 *****************************************************************************/
 
////// RADIOS


////// SENSORS


////// ACTUATORS


////// CONTROL


////// MESSAGING


/******************************************************************************
 * INTERRUPT SERVICE ROUTINES
 *****************************************************************************/

/**
 * Example pin change ISR for a generic peripheral "periph" on pin bank "x".
 */
// static volatile uint8_t prevStatePx;
// Interrupt service routine for Px I/O port transition changes.
// ISR(PCINTx_vect)
// {
//     // Basic profiling info
//     OTV0P2BASE::MemoryChecks::recordPC();
// 
//     const uint8_t pins = PINx;
//     const uint8_t changes = pins ^ prevStatePx;
//     prevStatePBx= pins;
// 
//     // RFM23B nIRQ falling edge is of interest.
//     // Handler routine not required/expected to 'clear' this interrupt.
//     if((changes & periph_INT_MASK) && !(pins & periph_INT_MASK))
//         { periph._handleInterrupt(); }
// }


/******************************************************************************
 * LOCAL FUNCTIONS
 *****************************************************************************/
 
////// PANIC

// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
/**
 * Example panic routines with one radio.
 */
// static void panic()
// {
//     // Reset radio and go into low-power mode here
//     PrimaryRadio.panicShutdown();
//     
//     // Power down almost everything else...
//     OTV0P2BASE::minimisePowerWithoutSleep();
//     pinMode(OTV0P2BASE::LED_HEATCALL_L, OUTPUT);
// 
//     // Flash LED.
//     for( ; ; ) {
//         OTV0P2BASE::LED_HEATCALL_ON();
//         OTV0P2BASE::nap(WDTO_15MS);
//         OTV0P2BASE::LED_HEATCALL_OFF();
//         OTV0P2BASE::nap(WDTO_120MS);
//     }
// }
// 
// // Panic with fixed message.
// static void panic(const __FlashStringHelper *s)
// {
//     OTV0P2BASE::serialPrintlnAndFlush(); // Start new line to highlight error.  // May fail.
//     OTV0P2BASE::serialPrintAndFlush('!'); // Indicate error with leading '!' // May fail.
//     OTV0P2BASE::serialPrintlnAndFlush(s); // Print supplied detail text. // May fail.
//     panic();
// }


////// DIAGNOSTICS
// e.g. stack check


////// SENSORS



////// CONTROL

/**
 * Example IO Poll.
 */
// // Call this to do an I/O poll if needed; returns true if something useful definitely happened.
// // This call should typically take << 1ms at 1MHz CPU.
// // Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// // Should also do nothing that interacts with Serial.
// // Limits actual poll rate to something like once every 8ms, unless force is true.
// //   * force if true then force full poll on every call (ie do not internally rate-limit)
// // Note that radio poll() can be for TX as well as RX activity.
// // Not thread-safe, eg not to be called from within an ISR.
// bool pollIO(const bool force = false)
// {
//     static volatile uint8_t _pO_lastPoll;
//     // Poll RX at most about every ~8ms.
//     const uint8_t sct = OTV0P2BASE::getSubCycleTime();
//     if(force || (sct != _pO_lastPoll)) {
//         _pO_lastPoll = sct;
//         // Poll for inbound frames.
//         // If RX is not interrupt-driven then
//         // there will usually be little time to do this
//         // before getting an RX overrun or dropped frame
//         PrimaryRadio.poll();
//     }
//     return(false);
// }


////// MESSAGING
// e.g. void bareStatsTX()


////// UI
// e.g. void pollCLI()


/******************************************************************************
 * SETUP
 *****************************************************************************/

// Setup routine: runs once after reset.
void setup()
{

}


/******************************************************************************
 * MAIN LOOP
 *****************************************************************************/

void loop()
{

}

