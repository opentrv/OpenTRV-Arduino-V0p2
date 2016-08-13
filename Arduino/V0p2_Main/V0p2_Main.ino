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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2016
                           Deniz Erbilgin 2015--2016
*/

/*
  V0p2 (V0.2) core.

  DHD20130417: hardware setup on bare board.
    * 1MHz CPU clock (from 8MHz internal RC clock with /8 prescaler) ATmega328P running at 1.8V--5V (typically 2V--3.3V).
    * Fuse set for BOD-managed additional clock settle time, ie as fast a restart from sleep as possible.
    * All unused pins unconnected and nominally floating (though driven low as output where possible).
    * 32768Hz xtal between pins XTAL1 and XTAL2, async timer 2, for accurate timekeeping and low-power sleep.
    * All unused system modules turned off.

  Basic AVR power consumption ticking an (empty) control loop at ~0.5Hz should be ~1uA.
 */

#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT) || defined(ENABLE_SECURE_RADIO_BEACON)
#include <OTAESGCM.h>
#endif

#include "V0p2_Main.h"

#include "V0p2_Generic_Config.h"
#include <OTV0p2_Board_IO_Config.h> // I/O pin allocation and setup: include ahead of I/O module headers.

// Arduino libraries imported here (even for use in other .cpp files).
#include <SPI.h>
#include <Wire.h>
#include <OTRadioLink.h>
#include <OTSIM900Link.h>
#include <OTRN2483Link.h>
#include <OTRadValve.h>

#include "V0p2_Sensors.h"
#include "Control.h"
#include "UI_Minimal.h"


// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic()
  {
#ifdef ENABLE_RADIO_PRIMARY_MODULE
  // Reset radio and go into low-power mode.
  PrimaryRadio.panicShutdown();
#endif
#ifdef ENABLE_RADIO_SECONDARY_MODULE
  // Reset radio and go into low-power mode.
  SecondaryRadio.panicShutdown();
#endif
  // Power down almost everything else...
  OTV0P2BASE::minimisePowerWithoutSleep();
#ifdef LED_HEATCALL
  pinMode(LED_HEATCALL, OUTPUT);
#else
  pinMode(LED_HEATCALL_L, OUTPUT);
#endif
  for( ; ; )
    {
    LED_HEATCALL_ON();
    tinyPause();
    LED_HEATCALL_OFF();
    bigPause();
    }
  }

// Panic with fixed message.
void panic(const __FlashStringHelper *s)
  {
  OTV0P2BASE::serialPrintlnAndFlush(); // Start new line to highlight error.  // May fail.
  OTV0P2BASE::serialPrintAndFlush('!'); // Indicate error with leading '!' // May fail.
  OTV0P2BASE::serialPrintlnAndFlush(s); // Print supplied detail text. // May fail.
  panic();
  }


// Signal position in basic POST sequence as a small positive integer, or zero for done/none.
// Simple count of position in ON flashes.
// LED is assumed to be ON upon entry, and is left ON at exit.
//
// See video which shows the boot sequence: http://gallery.hd.org/_c/energy-matters/_more2013/_more12/OpenTRV-V0p2-breadboard-POST-Power-On-Self-Test-LED-sequence-as-of-20131202-bootloader-then-five-sections-then-flicker-for-SPI-drive-of-RFM23-1-DHD.mov.html
//   * Two quick flashes from the Arduino bootloader then the LED comes on.
//   * Each of the 5 main sections of Power On Self Test is 1 second LED on, 0.5 second off, n short flashes separated by 0.25s off, then 0.5s off, then 1s on.
//     The value of n is 1, 2, 3, 4, 5.
//   * The LED should then go off except for optional faint flickers as the radio is being driven if set up to do so.
#ifndef ALT_MAIN_LOOP
#define PP_OFF_MS 250
static void posPOST(const uint8_t position, const __FlashStringHelper *s = NULL)
  {
  OTV0P2BASE::sleepLowPowerMs(1000);
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("posPOST: "); // Can only be used once serial is set up.
  DEBUG_SERIAL_PRINT(position);
  if(NULL != s)
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING(": ");
    DEBUG_SERIAL_PRINT(s);
    }
  DEBUG_SERIAL_PRINTLN();
//#else
//  OTV0P2BASE::serialPrintlnAndFlush(s);
#endif
//  pinMode(LED_HEATCALL, OUTPUT);
  LED_HEATCALL_OFF();

  // Skip much of lightshow if '0'/end/none position.
  if(position > 0)
    {
    OTV0P2BASE::sleepLowPowerMs(2*PP_OFF_MS); // TODO: use this time to gather entropy.
    for(int i = position; --i >= 0; )
      {
      LED_HEATCALL_ON();
      tinyPause();
      LED_HEATCALL_OFF();
      OTV0P2BASE::sleepLowPowerMs(PP_OFF_MS); // TODO: use this time to gather entropy.
      }
    }

  OTV0P2BASE::sleepLowPowerMs(PP_OFF_MS); // TODO: use this time to gather entropy.
  LED_HEATCALL_ON();
  OTV0P2BASE::sleepLowPowerMs(1000); // TODO: use this time to gather entropy.
  }
#endif // ALT_MAIN_LOOP

// Rearrange date into sensible most-significant-first order, and make it fully numeric.
// FIXME: would be better to have this in PROGMEM (Flash) rather than RAM, eg as F() constant.
static const char _YYYYMmmDD[] =
  {
  __DATE__[7], __DATE__[8], __DATE__[9], __DATE__[10],
  '/',
  __DATE__[0], __DATE__[1], __DATE__[2],
  '/',
  ((' ' == __DATE__[4]) ? '0' : __DATE__[4]), __DATE__[5],
  '\0'
  };
// Version (code/board) information printed as one line to serial (with line-end, and flushed); machine- and human- parseable.
// Format: "board VX.X REVY YYYY/Mmm/DD HH:MM:SS".
void serialPrintlnBuildVersion()
  {
  OTV0P2BASE::serialPrintAndFlush(F("board V0.2 REV"));
  OTV0P2BASE::serialPrintAndFlush(V0p2_REV);
  OTV0P2BASE::serialPrintAndFlush(' ');
  OTV0P2BASE::serialPrintAndFlush(_YYYYMmmDD);
  OTV0P2BASE::serialPrintlnAndFlush(F(" " __TIME__));
  }


// Pick an appropriate radio config for RFM23 (if it is the primary radio).
#ifdef ENABLE_RADIO_PRIMARY_RFM23B
// OTRadioChannelConfig(const void *_config, bool _isFull, bool _isRX, bool _isTX, bool _isAuth = false, bool _isEnc = false, bool _isUnframed = false)
#if defined(ENABLE_FAST_FRAMED_CARRIER_SUPPORT)
#define RADIO_CONFIG_NAME "GFSK"
// Nodes talking on fast GFSK channel 0.
static const uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] =
  {
  // GFSK channel 0 full config, RX/TX, not in itself secure.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true),
  };
#else // !defined(ENABLE_FAST_FRAMED_CARRIER_SUPPORT)
#define RADIO_CONFIG_NAME "OOK"
// Nodes talking (including to to FHT8V) on slow OOK.
static const uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] =
  {
  // FS20/FHT8V compatible channel 0 partial/minimal single-channel register config; RX/TX, not secure, unframed.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::FHT8V_RFM23_Reg_Values, false, true, true, false, false, true)
  };
#endif
#endif // ENABLE_RADIO_PRIMARY_RFM23B


#if defined(ENABLE_RADIO_SECONDARY_SIM900)
static const OTRadioLink::OTRadioChannelConfig SecondaryRadioConfig(&SIM900Config, true);
#elif defined(ENABLE_RADIO_SECONDARY_MODULE)
static const OTRadioLink::OTRadioChannelConfig SecondaryRadioConfig(NULL, true);
#endif // ENABLE_RADIO_SECONDARY_SIM900


#if defined(ENABLE_STATS_RX) && defined(ENABLE_FS20_ENCODING_SUPPORT)
// If using FS20-based non-secured messaging...
// If a stats hub then be prepared to accept a wide variety of binary and JSON message types.
// It may yet be good to trim the smaller message types down to size in particular to help queueing.
// It may also be kinder to some of the older FS20+trailer handling routines to not over-trim!
// This does duplicate some of the handling work elsewhere, thus allowing room for confusion.
// Allows all messages through by default for diagnostic purposes.
static bool FilterRXISR(const volatile uint8_t *buf, volatile uint8_t &buflen)
  {
  const uint8_t initialBuflen = buflen;
  if(initialBuflen < 1) { return(true); } // Accept everything, even empty message with no type byte.
  // Fall through for message types not handled specifically.
  switch(buf[0])
    {
    case OTRadioLink::FTp2_FullStatsIDL: case OTRadioLink::FTp2_FullStatsIDH:
      {
      // Maximum size is 8 including trailing CRC; fall through for possible further zeros trim.
      buflen = min(initialBuflen, 8); // OTRadioLink::V0P2_MESSAGING_LEADING_FULL_STATS_MAX_BYTES_ON_WIRE);
      break;
      }
    case OTRadioLink::FTp2_JSONRaw:
      {
      // Maximum size is 56 including trailing CRC; fall through for possible further zeros trim.
      buflen = min(initialBuflen, OTV0P2BASE::MSG_JSON_ABS_MAX_LENGTH + 1);
      break;
      }
    case OTRadioLink::FTp2_FS20_native:
      {
      // Maxmimum size is 53 including trailing stats+CRC; fall through for possible further zeros trim.
      buflen = min(initialBuflen, 53);
      break;
      }
    }
  #if defined(CONFIG_TRAILING_ZEROS_FILTER_RX)
  // By default apply heuristic trim of trailing zeros to almost all message types.
  return(OTRadioLink::frameFilterTrailingZeros(buf, buflen)); // This will accept all messages.
  #else
  return(true); // Accept all messages.
  #endif
  }
#elif defined(CONFIG_TRAILING_ZEROS_FILTER_RX)
// Useful general heuristic to improve queueing, etc.
#define FilterRXISR (OTRadioLink::frameFilterTrailingZeros)
#else
// NO RADIO RX FILTERING BY DEFAULT
#define NO_RX_FILTER
#define FilterRXISR NULL
#endif

// Optional Power-On Self Test routines.
// Aborts with a call to panic() if a test fails.
#if !defined(ALT_MAIN_LOOP)
void optionalPOST()
  {
  // Have 32678Hz clock at least running before going any further.
#if defined(ENABLE_WAKEUP_32768HZ_XTAL)
  if(!::OTV0P2BASE::HWTEST::check32768HzOsc()) { panic(F("xtal")); } // Async clock not running correctly.
#else
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("(No xtal.)");
#endif

  // Signal that xtal is running AND give it time to settle.
  posPOST(0 /*, F("about to test radio module") */);

// FIXME  This section needs refactoring
#ifdef ENABLE_RADIO_PRIMARY_RFM23B
// TODO-547: why does nested SPI enable break things?
//  const bool neededToWakeSPI = OTV0P2BASE::powerUpSPIIfDisabled();
//  DEBUG_SERIAL_PRINT(neededToWakeSPI);
//  DEBUG_SERIAL_PRINTLN();
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  PrimaryRadio.preinit(NULL);
#if 1 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  // Print out some info on the radio config.
  DEBUG_SERIAL_PRINT_FLASHSTRING("R1 #chan=");
  DEBUG_SERIAL_PRINT(nPrimaryRadioChannels);
  #if defined(ENABLE_CONTINUOUS_RX)
  DEBUG_SERIAL_PRINT_FLASHSTRING(" contRX"); // Show that continuous RX is enabled (eg battery draining for non-hub nodes).
  #endif
  DEBUG_SERIAL_PRINT_FLASHSTRING(" name=");
  DEBUG_SERIAL_PRINTLN_FLASHSTRING(RADIO_CONFIG_NAME);
#endif
  // Check that the radio is correctly connected; panic if not...
  if(!PrimaryRadio.configure(nPrimaryRadioChannels, RFM23BConfigs) || !PrimaryRadio.begin()) { panic(F("r1")); }
  // Apply filtering, if any, while we're having fun...
#ifndef NO_RX_FILTER
  PrimaryRadio.setFilterRXISR(FilterRXISR);
#endif // NO_RX_FILTER
#endif // ENABLE_RADIO_PRIMARY_RFM23B

#ifdef ENABLE_RADIO_SECONDARY_MODULE
#ifdef ENABLE_RADIO_SIM900
  // Turn power on for SIM900 with PFET for secondary power control.
  fastDigitalWrite(A3, 0);
  pinMode(A3, OUTPUT);
#endif // ENABLE_RADIO_SIM900
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  SecondaryRadio.preinit(NULL);
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("R2");
#endif
  // Check that the radio is correctly connected; panic if not...
  if(!SecondaryRadio.configure(1, &SecondaryRadioConfig) || !SecondaryRadio.begin()) { panic(F("r2")); }
  // Assume no RX nor filtering on secondary radio.
#endif // ENABLE_RADIO_SECONDARY_MODULE

//  posPOST(1, F("Radio OK, checking buttons/sensors and xtal"));

// Buttons should not be activated DURING boot for user-facing boards; an activated button implies a fault.
#if ((V0p2_REV >= 1) && (V0p2_REV <= 4)) || ((V0p2_REV >= 7) && (V0p2_REV <= 8))
#if (9 != V0p2_REV) || !defined(CONFIG_REV9_cut1) // Usual tests for stuck control buttons.
  // Check buttons not stuck in the activated position.
  if((fastDigitalRead(BUTTON_MODE_L) == LOW)
#if defined(BUTTON_LEARN_L)
     || (fastDigitalRead(BUTTON_LEARN_L) == LOW)
#endif
#if defined(BUTTON_LEARN2_L) && (9 != V0p2_REV) // This input is not momentary with REV9.
     || (fastDigitalRead(BUTTON_LEARN2_L) == LOW)
#endif
    )
    { panic(F("b")); }
//#else
//    DEBUG_SERIAL_PRINT(fastDigitalRead(BUTTON_MODE_L)); DEBUG_SERIAL_PRINTLN(); // Should be BOOSTSWITCH_L for REV9.
//    DEBUG_SERIAL_PRINT(fastDigitalRead(BUTTON_LEARN_L)); DEBUG_SERIAL_PRINTLN();
//    DEBUG_SERIAL_PRINT(fastDigitalRead(BUTTON_LEARN2_L)); DEBUG_SERIAL_PRINTLN(); // AKA WINDOW SWITCH
#endif
#endif // Select user-facing boards.

// Save space (and time) by avoiding the second POST sequence; LED will be turned off anyway.
//  // Single/main POST checkpoint for speed.
//  posPOST(1 /* , F("POST OK") */ );
  }
#endif // !defined(ALT_MAIN_LOOP)


// Setup routine: runs once after reset.
// Does some limited board self-test and will panic() if anything is obviously broken.
void setup()
  {
  // Set appropriate low-power states, interrupts, etc, ASAP.
  OTV0P2BASE::powerSetup();

//#if defined(ENABLE_MIN_ENERGY_BOOT)
//  nap(WDTO_120MS); // Sleep to let power supply recover a little.
//#endif

  // IO setup for safety, and to avoid pins floating.
  OTV0P2BASE::IOSetup();

//#if defined(ENABLE_MIN_ENERGY_BOOT)
//  nap(WDTO_120MS); // Sleep to let power supply recover a little.
//#endif

#if !defined(ENABLE_MIN_ENERGY_BOOT)
  // Restore previous RTC state if available.
  OTV0P2BASE::restoreRTC();
  // TODO: consider code to calibrate the internal RC oscillator against the xtal, eg to keep serial comms happy, eg http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=36237&start=0
#endif

#if !defined(ENABLE_MIN_ENERGY_BOOT)
#if defined(LED_UI2_EXISTS) && defined(ENABLE_UI_LED_2_IF_AVAILABLE)
  LED_UI2_ON();
#endif
  OTV0P2BASE::serialPrintAndFlush(F("\r\nOpenTRV: ")); // Leading CRLF to clear leading junk, eg from bootloader.
    serialPrintlnBuildVersion();
#if defined(LED_UI2_EXISTS) && defined(ENABLE_UI_LED_2_IF_AVAILABLE)
  OTV0P2BASE::nap(WDTO_120MS); // Sleep to let UI2 LED be seen.
  LED_UI2_OFF();
#endif
#endif

#if !defined(ENABLE_MIN_ENERGY_BOOT)
  // Count resets to detect unexpected crashes/restarts.
  const uint8_t oldResetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
  eeprom_write_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT, 1 + oldResetCount);
#endif

#if defined(DEBUG) && !defined(ENABLE_MIN_ENERGY_BOOT)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("DEBUG");
#endif

#if defined(DEBUG) && !defined(ENABLE_MIN_ENERGY_BOOT)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Resets: ");
  DEBUG_SERIAL_PRINT(oldResetCount);
  DEBUG_SERIAL_PRINTLN();
#if !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
  const uint8_t overruns = (~eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_OVERRUN_COUNTER)) & 0xff;
  if(0 != overruns)
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("Overruns: ");
    DEBUG_SERIAL_PRINT(overruns);
    DEBUG_SERIAL_PRINTLN();
    }
#endif
#if 0 && defined(DEBUG)
  // Compute approx free RAM: see http://jeelabs.org/2011/05/22/atmega-memory-use/
  DEBUG_SERIAL_PRINT_FLASHSTRING("Free RAM: ");
  extern int __heap_start, *__brkval;
  int x;
  DEBUG_SERIAL_PRINT((int) &x - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
  DEBUG_SERIAL_PRINTLN();
#endif
#if defined(ALT_MAIN_LOOP)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("ALTERNATE MAIN LOOP...");
#elif defined(UNIT_TESTS)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("UNIT TESTS...");
#endif
#endif

// Do not do normal POST if running alternate main loop.
// POST may take too long and do unwanted things,
// especially for non-standard hardware setup.
#if defined(ALT_MAIN_LOOP)
  POSTalt(); // Do alternate POST and setup if required.
#else
  optionalPOST();
#endif

  // Collect full set of environmental values before entering loop() in normal mode.
  // This should also help ensure that sensors are properly initialised.

  // No external sensors are *assumed* present if running alt main loop.
  // This may mean that the alt loop/POST will have to initialise them explicitly,
  // and the initial seed entropy may be marginally reduced also.
#if !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
  const int heat = TemperatureC16.read();
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  DEBUG_SERIAL_PRINT_FLASHSTRING("T: ");
  DEBUG_SERIAL_PRINT(heat);
  DEBUG_SERIAL_PRINTLN();
#endif
#if defined(ENABLE_AMBLIGHT_SENSOR)
  const int light = AmbLight.read();
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  DEBUG_SERIAL_PRINT_FLASHSTRING("L: ");
  DEBUG_SERIAL_PRINT(light);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif
#if defined(HUMIDITY_SENSOR_SUPPORT)
  const uint8_t rh = RelHumidity.read();
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  DEBUG_SERIAL_PRINT_FLASHSTRING("RH%: ");
  DEBUG_SERIAL_PRINT(rh);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif
#if defined(TEMP_POT_AVAILABLE)
  const int tempPot = TempPot.read();
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  DEBUG_SERIAL_PRINT_FLASHSTRING("temp pot: ");
  DEBUG_SERIAL_PRINT(tempPot);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif
#endif

#if !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
  const uint16_t Vcc = Supply_cV.read();
#if 1 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  // Get current power supply voltage (internal sensor).
  DEBUG_SERIAL_PRINT_FLASHSTRING("Vcc: ");
  DEBUG_SERIAL_PRINT(Vcc);
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("cV");
#endif
#if 0 && defined(DEBUG)
  // Get internal temperature measurement (internal sensor).
  const int intTempC16 = OTV0P2BASE::readInternalTemperatureC16();
  DEBUG_SERIAL_PRINT_FLASHSTRING("Int temp: ");
  DEBUG_SERIAL_PRINT((intTempC16 + 8) >> 4);
  DEBUG_SERIAL_PRINT_FLASHSTRING("C / ");
  DEBUG_SERIAL_PRINT(intTempC16);
  DEBUG_SERIAL_PRINTLN();
#endif
#if !defined(ENABLE_MIN_ENERGY_BOOT)
  OTV0P2BASE::seedPRNGs();
#endif
#endif

#if !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Computing initial target/demand...");
#endif
#if defined(ENABLE_NOMINAL_RAD_VALVE)
  // Update targets, output to TRV and boiler, etc, to be sensible before main loop starts.
  NominalRadValve.read();
#endif
#endif

  // Ensure that the unique node ID is set up (mainly on first use).
  // Have one attempt (don't want to stress an already failing EEPROM) to force-reset if not good, then panic.
  // Needs to have had entropy gathered, etc.
  if(!OTV0P2BASE::ensureIDCreated())
    {
    if(!OTV0P2BASE::ensureIDCreated(true)) // Force reset.
      { panic(F("ID")); }
    }

  // Initialised: turn main/heatcall UI LED off.
  LED_HEATCALL_OFF();

#if defined(ENABLE_CLI) && defined(ENABLE_CLI_HELP) && !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS) && !defined(ENABLE_TRIMMED_MEMORY)
  // Help user get to CLI.
  OTV0P2BASE::serialPrintlnAndFlush(F("At CLI > prompt enter ? for help"));
#endif

#if !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
#if !defined(ENABLE_TRIMMED_MEMORY)
  // Report initial status.
  serialStatusReport();
#endif
  // Do OpenTRV-specific (late) setup.
  setupOpenTRV();
#endif
  }


//========================================
// MAIN LOOP
//========================================

void loop()
  {
#if defined(EST_CPU_DUTYCYCLE)
  const unsigned long usStart = micros();
#endif

#if defined(UNIT_TESTS) // Run unit tests *instead* of normal loop() code.
  loopUnitTest();
#elif defined(ALT_MAIN_LOOP) // Run alternative main loop.
  loopAlt();
#else // Normal OpenTRV usage.
  loopOpenTRV();
#endif

#if defined(EST_CPU_DUTYCYCLE)
  const unsigned long usEnd = micros();
  // Nominal loop time should be 2s x 1MHz clock, ie 2,000,000 if CPU running all the time.
  // Should generally be <2000 (<0.1%) for leaf, <20000 (<1%) for hub.
  const unsigned long usApparentTaken = usEnd - usStart;
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("us apparent: ");
  DEBUG_SERIAL_PRINT(usApparentTaken);
  DEBUG_SERIAL_PRINTLN();
// Leaf sample 2014/11/22...
//us apparent: 4544
//us apparent: 25280
//us apparent: 9280
//us apparent: 68416
//=F0%@19C7;IC2E0;V3230;X0;R81;T8 1 W255 0 F255 0 W255 0 F255 0;S12 12 16 e;o34
//
//>
//us apparent: 339136
//us apparent: 1600
//us apparent: 2112
//us apparent: 2880
//us apparent: 3008
//us apparent: 1408
//us apparent: 1344
//Bare stats TX
//us apparent: 74368
//{"@":"c2e0","T|C16":311,"H|%":81,"L":238,"B|cV":323}
//us apparent: 119232
//us apparent: 1408
//...
//us apparent: 3008
//us apparent: 1344
//us apparent: 3264
//us apparent: 1408
//us apparent: 3776
//us apparent: 2816
#endif
#endif
  }
