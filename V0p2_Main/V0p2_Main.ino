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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2015
                           Deniz Erbilgin 2015
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

#ifdef ALLOW_CC1_SUPPORT
#include <OTProtocolCC.h>
#endif

#include <util/crc16.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h> // for radio config

#include "V0p2_Main.h"

#include "V0p2_Generic_Config.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

// Arduino libraries imported here (even for use in other .cpp files).
#include <SPI.h>
#include <Wire.h>
#include <OTRadioLink.h>
//#include <OTNullRadioLink.h> // as in separate library to OTRadioLink
#include <OTSIM900Link.h>
#include <OTRadValve.h>

#include "V0p2_Sensors.h"
#include "V0p2_Actuators.h"
#include "Control.h"
#include "Power_Management.h"
#include "Radio.h"
#include "Serial_IO.h"
#include "UI_Minimal.h"


// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic()
  {
#ifdef USE_MODULE_RFM22RADIOSIMPLE
  // Reset radio and go into low-power mode.
  PrimaryRadio.panicShutdown();
#endif
  // Power down almost everything else...
  minimisePowerWithoutSleep();
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
  OTV0P2BASE::serialPrintlnAndFlush(s); // May fail.
  panic();
  }


// Compute a CRC of all of SRAM as a hash that should contain some entropy, especially after power-up.
#if !defined(RAMSTART)
#define RAMSTART (0x100)
#endif
static uint16_t sramCRC()
  {
  uint16_t result = ~0U;
  for(uint8_t *p = (uint8_t *)RAMSTART; p <= (uint8_t *)RAMEND; ++p)
    { result = _crc_ccitt_update(result, *p); }
  return(result);
  }
// Compute a CRC of all of EEPROM as a hash that may contain some entropy, particularly across restarts.
static uint16_t eeCRC()
  {
  uint16_t result = ~0U;
  for(uint8_t *p = (uint8_t *)0; p <= (uint8_t *)E2END; ++p)
    {
    const uint8_t v = eeprom_read_byte(p);
    result = _crc_ccitt_update(result, v);
    }
  return(result);
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
static void posPOST(const uint8_t position, const __FlashStringHelper *s)
  {
  OTV0P2BASE::sleepLowPowerMs(1000);
#ifdef DEBUG
  DEBUG_SERIAL_PRINT_FLASHSTRING("posPOST: "); // Can only be used once serial is set up.
  DEBUG_SERIAL_PRINT(position);
  DEBUG_SERIAL_PRINT_FLASHSTRING(": ");
  DEBUG_SERIAL_PRINT(s);
  DEBUG_SERIAL_PRINTLN();
#else
  OTV0P2BASE::serialPrintlnAndFlush(s);
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

// Rearrange date into sensible most-significant-first order.  (Would like it also to be fully numeric, but whatever...)
// FIXME: would be better to have this in PROGMEM (Flash) rather than RAM.
static const char _YYYYMmmDD[] =
  {
  __DATE__[7], __DATE__[8], __DATE__[9], __DATE__[10],
  '/',
  __DATE__[0], __DATE__[1], __DATE__[2],
  '/',
  __DATE__[4], __DATE__[5],
  '\0'
  };
// Version (code/board) information printed as one line to serial (with line-end, and flushed); machine- and human- parseable.
// Format: "board VXXXX REVY; code YYYY/Mmm/DD HH:MM:SS".
void serialPrintlnBuildVersion()
  {
  OTV0P2BASE::serialPrintAndFlush(F("board V0.2 REV"));
  OTV0P2BASE::serialPrintAndFlush(V0p2_REV);
  OTV0P2BASE::serialPrintAndFlush(F(" "));
  OTV0P2BASE::serialPrintAndFlush(_YYYYMmmDD);
  OTV0P2BASE::serialPrintAndFlush(F(" " __TIME__));
  OTV0P2BASE::serialPrintlnAndFlush();
  }

// FIXME deal with this
static const OTRadioLink::OTRadioChannelConfig RFMConfig(OTRadValve::FHT8VRadValveBase::FHT8V_RFM23_Reg_Values, true, true, true);
//static const OTRadioLink::OTRadioChannelConfig SecondaryRadioConfig(&SIM900Config, true, true, true);
static const OTRadioLink::OTRadioChannelConfig SecondaryRadioConfig(NULL, true, true, true);

#if defined(ALLOW_CC1_SUPPORT_RELAY)
// For a CC1 relay, ignore everything except FTp2_CC1PollAndCmd messages.
// With care (not accessing EEPROM for example) this could also reject anything with wrong house code.
static bool FilterRXISR(const volatile uint8_t *buf, volatile uint8_t &buflen)
  {
  if((buflen < 8) || (OTRadioLink::FTp2_CC1PollAndCmd != buf[0])) { return(false); }
  buflen = 8; // Truncate message to correct size for efficiency.
  return(true); // Accept message.
  }
#elif defined(ALLOW_CC1_SUPPORT_HUB)
// For a CC1 hub, ignore everything except FTp2_CC1Alert and FTp2_CC1PollResponse messages.
static bool FilterRXISR(const volatile uint8_t *buf, volatile uint8_t &buflen)
  {
  if(buflen < 8) { return(false); }
  const uint8_t t = buf[0];
  if((OTRadioLink::FTp2_CC1Alert != t) && (OTRadioLink::FTp2_CC1PollResponse != t)) { return(false); }
  buflen = 8; // Truncate message to correct size for efficiency.
  return(true); // Accept message.
  }
#elif defined(ALLOW_STATS_RX) && defined(ENABLE_FS20_ENCODING_SUPPORT)
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
      // Maxmimum size is 8 including trailing CRC; fall through for possible further zeros trim.
      buflen = min(initialBuflen, 8); // OTRadioLink::V0P2_MESSAGING_LEADING_FULL_STATS_MAX_BYTES_ON_WIRE);
      break;
      }
    case OTRadioLink::FTp2_JSONRaw:
      {
      // Maxmimum size is 56 including trailing CRC; fall through for possible further zeros trim.
      buflen = min(initialBuflen, MSG_JSON_ABS_MAX_LENGTH + 1);
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
#define FilterRXISR NULL
#endif

// Optional Power-On Self Test routines.
// Aborts with a call to panic() if a test fails.
void optionalPOST()
  {
  // Capture early sub-cycle time to help ensure that the 32768Hz async clock is actually running.
  const uint8_t earlySCT = OTV0P2BASE::getSubCycleTime();

//  posPOST(1, F("about to test radio module"));

// FIXME  This section needs refactoring
#ifdef USE_MODULE_RFM22RADIOSIMPLE
// TODO-547: why does nested SPI enable break things?
//  const bool neededToWakeSPI = OTV0P2BASE::powerUpSPIIfDisabled();
//  DEBUG_SERIAL_PRINT(neededToWakeSPI);
//  DEBUG_SERIAL_PRINTLN();
#if !defined(RFM22_IS_ACTUALLY_RFM23) && defined(DEBUG) && !defined(MIN_ENERGY_BOOT)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("(Using RFM22.)");
#endif // !defined(RFM22_IS_ACTUALLY_RFM23) && defined(DEBUG) && !defined(MIN_ENERGY_BOOT)

#ifdef ENABLE_RADIO_SIM900
fastDigitalWrite(A3, 0);
pinMode(A3, OUTPUT);
#endif // ENABLE_RADIO_SIM900

  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  PrimaryRadio.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!PrimaryRadio.configure(1, &RFMConfig) || !PrimaryRadio.begin()) { panic(); }
  // Apply filtering, if any, while we're having fun...
  PrimaryRadio.setFilterRXISR(FilterRXISR);
//  if(neededToWakeSPI) { OTV0P2BASE::powerDownSPI(); }
#endif // USE_MODULE_RFM22RADIOSIMPLE

#ifdef ENABLE_RADIO_SECONDARY_MODULE
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  SecondaryRadio.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!SecondaryRadio.configure(1, &SecondaryRadioConfig) || !SecondaryRadio.begin()) { panic(); }
  // Apply filtering, if any, while we're having fun...
  SecondaryRadio.setFilterRXISR(FilterRXISR);
#endif // ENABLE_RADIO_SECONDARY_MODULE


//  posPOST(1, F("Radio OK, checking buttons/sensors and xtal"));

// Buttons should not be activated DURING boot; activated button implies fault.
//#if (9 != V0p2_REV) || !defined(CONFIG_REV9_cut1) // Usual tests for stuck control buttons.
//  // Check buttons not stuck in the activated position.
//  if((fastDigitalRead(BUTTON_MODE_L) == LOW)
//#if defined(BUTTON_LEARN_L)
//     || (fastDigitalRead(BUTTON_LEARN_L) == LOW)
//#endif
//#if defined(BUTTON_LEARN2_L) && (9 != V0p2_REV) // This input is not momentary with REV9.
//     || (fastDigitalRead(BUTTON_LEARN2_L) == LOW)
//#endif
//    )
//    { panic(F("button stuck")); }
//#else
//    DEBUG_SERIAL_PRINT(fastDigitalRead(BUTTON_MODE_L)); DEBUG_SERIAL_PRINTLN(); // Should be BOOSTSWITCH_L for REV9.
//    DEBUG_SERIAL_PRINT(fastDigitalRead(BUTTON_LEARN_L)); DEBUG_SERIAL_PRINTLN();
//    DEBUG_SERIAL_PRINT(fastDigitalRead(BUTTON_LEARN2_L)); DEBUG_SERIAL_PRINTLN(); // AKA WINDOW SWITCH
//#endif

#if defined(WAKEUP_32768HZ_XTAL)
  // Check that the 32768Hz async clock is actually running having done significant CPU-intensive work.
  const uint8_t laterSCT = OTV0P2BASE::getSubCycleTime();
  if(laterSCT == earlySCT)
    {
#if defined(WAKEUP_32768HZ_XTAL)
    // Allow extra time for 32768Hz crystal to start reliably, see: http://www.atmel.com/Images/doc1259.pdf
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("Sleeping to let 32768Hz clock start...");
#endif
    // Time spent here should not be a whole multiple of basic cycle time to avoid a spuriously-stationary async clock reading!
    // Allow several seconds to start.
    // Attempt to capture some entropy while waiting, implicitly from oscillator start-up time if nothing else.
    for(uint8_t i = 255; (--i > 0) && (earlySCT == OTV0P2BASE::getSubCycleTime()); )
      {
      OTV0P2BASE::addEntropyToPool(OTV0P2BASE::clockJitterWDT() ^ OTV0P2BASE::noisyADCRead(), 1); // Conservatively hope for at least 1 bit from combined sources!
      OTV0P2BASE::nap(WDTO_15MS); // Ensure lower mount of ~3s until loop finishes.
      OTV0P2BASE::captureEntropy1(); // Have other fun, though likely largely ineffective at this stage.
      }
#endif
    const uint8_t latestSCT = OTV0P2BASE::getSubCycleTime();
    if(latestSCT == earlySCT)
      {
#if 0 && defined(DEBUG)
      DEBUG_SERIAL_PRINTLN_FLASHSTRING("32768Hz clock may not be running!");
#endif
      panic(F("Xtal dead")); // Async clock not running.
      }
    }
//  posPOST(2, F("slow RTC clock OK"));
#else
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("(No xtal.)");
#endif

  // Single POST checkpoint for speed.
#if defined(WAKEUP_32768HZ_XTAL)
  posPOST(0, F("Radio, xtal, buttons OK"));
#else
  posPOST(0, F("Radio, buttons OK"));
#endif
  }


// Setup routine: runs once after reset.
// Does some limited board self-test and will panic() if anything is obviously broken.
void setup()
  {
  // Set appropriate low-power states, interrupts, etc, ASAP.
  powerSetup();

#if defined(MIN_ENERGY_BOOT)
  nap(WDTO_120MS); // Sleep to let power supply recover a little.
#endif

  // IO setup for safety, and to avoid pins floating.
  IOSetup();

#if defined(MIN_ENERGY_BOOT)
  nap(WDTO_120MS); // Sleep to let power supply recover a little.
#endif

#if !defined(MIN_ENERGY_BOOT)
  // Restore previous RTC state if available.
  OTV0P2BASE::restoreRTC();
  // TODO: consider code to calibrate the internal RC oscillator against the xtal, eg to keep serial comms happy, eg http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=36237&start=0
#endif

#if !defined(MIN_ENERGY_BOOT)
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

#if !defined(MIN_ENERGY_BOOT)
  // Count resets to detect unexpected crashes/restarts.
  const uint8_t oldResetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
  eeprom_write_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT, 1 + oldResetCount);
#endif

#if defined(DEBUG) && !defined(MIN_ENERGY_BOOT)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("DEBUG");
#endif

#if defined(DEBUG) && !defined(MIN_ENERGY_BOOT)
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

  // Collect full set of environmental values before entering loop().
  // This should also help ensure that sensors are properly initialised.

  // No external sensors are *assumed* present if running alt main loop.
  // This may mean that the alt loop/POST will have to initialise them explicitly,
  // and the initial seed entropy may be marginally reduced also.
#if !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
  const int light = AmbLight.read();
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("L: ");
  DEBUG_SERIAL_PRINT(light);
  DEBUG_SERIAL_PRINTLN();
#endif
//  // Assume 0 or full-scale values unlikely.
//  if((0 == light) || (light >= 1023)) { panic(F("LDR fault")); }
  const int heat = TemperatureC16.read();
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("T: ");
  DEBUG_SERIAL_PRINT(heat);
  DEBUG_SERIAL_PRINTLN();
#endif
#ifdef HUMIDITY_SENSOR_SUPPORT
  const uint8_t rh = RelHumidity.read();
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("RH%: ");
  DEBUG_SERIAL_PRINT(rh);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif
#if defined(TEMP_POT_AVAILABLE)
  const int tempPot = TempPot.read();
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("temp pot: ");
  DEBUG_SERIAL_PRINT(tempPot);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif
#endif


#if !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
  // Get current power supply voltage (internal sensor).
  const uint16_t Vcc = Supply_mV.read();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Vcc: ");
  DEBUG_SERIAL_PRINT(Vcc);
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("mV");
#endif
  // Get internal temperature measurement (internal sensor).
  const int intTempC16 = readInternalTemperatureC16();
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Int temp: ");
  DEBUG_SERIAL_PRINT((intTempC16 + 8) >> 4);
  DEBUG_SERIAL_PRINT_FLASHSTRING("C / ");
  DEBUG_SERIAL_PRINT(intTempC16);
  DEBUG_SERIAL_PRINTLN();
#endif

#if !defined(MIN_ENERGY_BOOT)
  // Seed PRNG(s) with available environmental values and clock time/jitter for some entropy.
  // Also sweeps over SRAM and EEPROM (see RAMEND and E2END), especially for non-volatile state and uninitialised areas of SRAM.
  // TODO: add better PRNG with entropy pool (eg for crypto).
  // TODO: add RFM22B WUT clock jitter, RSSI, temperature and battery voltage measures.
  const uint16_t srseed = sramCRC();
  const uint16_t eeseed = eeCRC();
  // DHD20130430: maybe as much as 16 bits of entropy on each reset in seed1, concentrated in the least-significant bits.
  const uint16_t s16 = (__DATE__[5]) ^
                       Vcc ^
                       (intTempC16 << 1) ^
#if !defined(ALT_MAIN_LOOP)
                       (heat << 2) ^
#if defined(TEMP_POT_AVAILABLE)
                       ((((uint16_t)tempPot) << 3) + tempPot) ^
#endif
                       (light << 4) ^
#if defined(HUMIDITY_SENSOR_SUPPORT)
                       ((((uint16_t)rh) << 8) - rh) ^
#endif
#endif
                       (OTV0P2BASE::getMinutesSinceMidnightLT() << 5) ^
                       (((uint16_t)OTV0P2BASE::getSubCycleTime()) << 6);
  //const long seed1 = ((((long) clockJitterRTC()) << 13) ^ (((long)clockJitterWDT()) << 21) ^ (((long)(srseed^eeseed)) << 16)) + s16;
  // Seed simple/fast/small built-in PRNG.  (Smaller and faster than srandom()/random().)
  const uint8_t nar1 = OTV0P2BASE::noisyADCRead();
  OTV0P2BASE::seedRNG8(nar1 ^ (uint8_t) s16, oldResetCount - (uint8_t)((s16+eeseed) >> 8), ::OTV0P2BASE::clockJitterWDT() ^ (uint8_t)srseed);
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("nar ");
  DEBUG_SERIAL_PRINTFMT(nar1, BIN);
  DEBUG_SERIAL_PRINTLN();
#endif
  // TODO: seed other/better PRNGs.
  // Feed in mainly persistent/nonvolatile state explicitly.
  OTV0P2BASE::addEntropyToPool(oldResetCount ^ eeseed, 0);
  OTV0P2BASE::addEntropyToPool((uint8_t)(eeseed >> 8) + nar1, 0);
  OTV0P2BASE::addEntropyToPool((uint8_t)s16 ^ (uint8_t)(s16 >> 8), 0);
  for(uint8_t i = 0; i < V0P2BASE_EE_LEN_SEED; ++i)
    { OTV0P2BASE::addEntropyToPool(eeprom_read_byte((uint8_t *)(V0P2BASE_EE_START_SEED + i)), 0); }
  OTV0P2BASE::addEntropyToPool(OTV0P2BASE::noisyADCRead(), 1); // Conservative first push of noise into pool.
  // Carry a few bits of entropy over a reset by picking one of the four designated EEPROM bytes at random;
  // if zero, erase to 0xff, else AND in part of the seed including some of the previous EEPROM hash (and write).
  // This amounts to about a quarter of an erase/write cycle per reset/restart per byte, or 400k restarts endurance!
  // These 4 bytes should be picked up as part of the hash/CRC of EEPROM above, next time,
  // essentially forming a longish-cycle (poor) PRNG even with little new real entropy each time.
  OTV0P2BASE::seedRNG8(nar1 ^ (uint8_t) s16, oldResetCount - (uint8_t)((s16+eeseed) >> 8), ::OTV0P2BASE::clockJitterWDT() ^ (uint8_t)srseed);
  uint8_t *const erp = (uint8_t *)(V0P2BASE_EE_START_SEED + (3&((s16)^((eeseed>>8)+(__TIME__[7]))))); // Use some new and some eeseed bits to choose which byte to updated.
  const uint8_t erv = eeprom_read_byte(erp);
  if(0 == erv) { OTV0P2BASE::eeprom_smart_erase_byte(erp); }
  else
    {
    OTV0P2BASE::eeprom_smart_clear_bits(erp,
#if !defined(NO_clockJitterEntropyByte)
      ::OTV0P2BASE::clockJitterEntropyByte()
#else
      (::OTV0P2BASE::clockJitterWDT() ^ nar1) // Less good fall-back when clockJitterEntropyByte() not available with more actual entropy.
#endif
      + ((uint8_t)eeseed)); // Nominally include disjoint set of eeseed bits in choice of which to clear.
    }
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("srseed ");
  DEBUG_SERIAL_PRINTFMT(srseed, BIN);
  DEBUG_SERIAL_PRINTLN();
  DEBUG_SERIAL_PRINT_FLASHSTRING("eeseed ");
  DEBUG_SERIAL_PRINTFMT(eeseed, BIN);
  DEBUG_SERIAL_PRINTLN();
  DEBUG_SERIAL_PRINT_FLASHSTRING("RNG8 ");
  DEBUG_SERIAL_PRINTFMT(randRNG8(), BIN);
  DEBUG_SERIAL_PRINTLN();
  DEBUG_SERIAL_PRINT_FLASHSTRING("erv ");
  DEBUG_SERIAL_PRINTFMT(erv, BIN);
  DEBUG_SERIAL_PRINTLN();
#endif
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
  if(!ensureIDCreated())
    {
    if(!ensureIDCreated(true)) // Force reset.
      { panic(F("!Bad ID: can't fix")); }
    }


  // Initialised: turn heatcall UI LED off.
//  pinMode(LED_HEATCALL, OUTPUT);
  LED_HEATCALL_OFF();

#if defined(SUPPORT_CLI) && !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
  // Help user get to CLI.
  OTV0P2BASE::serialPrintlnAndFlush(F("At CLI > prompt enter ? for help"));
#endif

#if !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
  // Report initial status.
  serialStatusReport();
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
