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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2014
                           Gary Gladman 2014
*/

/*
  V0p2 (V0.2) core.

  DHD20130417: hardware setup on bare board.
    * 1MHz CPU clock (from 8MHz internal RC clock with /8 prescaler) ATmega328P running at 1.8V--5V (typically 2V--3.3V).
    * Fuse set for BOD-managed additional clock settle time, ie as fast a restart from sleep as possible.
    * All unused pins unconnected and nominally floating (though driven low as output where possible).
    * 32768Hz xtal between pins 9 and 10, async timer 2, for accurate timekeeping and low-power sleep.
    * All unused system modules turned off.

  Basic AVR power consumption ticking an (empty) control loop at ~0.5Hz should be ~1uA.
 */

// Arduino libraries imported here (even for use in other .cpp files).
#include <SPI.h>
#include <Wire.h>

#include <util/crc16.h>
#include <avr/eeprom.h>

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
#include "Serial_IO.h"
#include "Temperature_Sensor.h"
#include "Temp_Pot.h"
#include "UI_Minimal.h"
#include "Unit_Tests.h"





// Controller's view of Least Significiant Digits of the current (local) time, in this case whole seconds.
// See PICAXE V0.1/V0.09/DHD201302L0 code.
#define TIME_LSD_IS_BINARY // TIME_LSD is in binary (cf BCD).
#define TIME_CYCLE_S 60 // TIME_LSD ranges from 0 to TIME_CYCLE_S-1, also major cycle length.
static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.


// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic()
  {
#ifdef USE_MODULE_RFM22RADIOSIMPLE
  // Reset radio and go into low-power mode.
  RFM22PowerOnInit();
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
  serialPrintlnAndFlush(s); // May fail.
  panic();  
  }


#ifdef DEBUG
// Optional Power-On Self Test routines.
// Aborts with a call to panic() if a test fails.
static void optionalPOST()
  {
  }
#endif

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

// Signal position in basic POST sequence (as a small positive integer.
// Simple count of position in ON flashes.
// LED is assumed to be ON upon entry, and is left ON at exit.
//
// See video which shows the boot sequence: http://gallery.hd.org/_c/energy-matters/_more2013/_more12/OpenTRV-V0p2-breadboard-POST-Power-On-Self-Test-LED-sequence-as-of-20131202-bootloader-then-five-sections-then-flicker-for-SPI-drive-of-RFM23-1-DHD.mov.html
//   * Two quick flashes from the Arduino bootloader then the LED comes on.
//   * Each of the 5 main sections of Power On Self Test is 1 second LED on, 0.5 second off, n short flashes separated by 0.25s off, then 0.5s off, then 1s on.
//     The value of n is 1, 2, 3, 4, 5.
//   * The LED should then go off except for optional faint flickers as the radio is being driven if set up to do so.
#define PP_OFF_MS 250
static void posPOST(const uint8_t position, const __FlashStringHelper *s)
  {
  sleepLowPowerMs(1000);
#ifdef DEBUG
  DEBUG_SERIAL_PRINT_FLASHSTRING("posPOST: "); // Can only be used once serial is set up.
  DEBUG_SERIAL_PRINT(position);
  DEBUG_SERIAL_PRINT_FLASHSTRING(": ");
  DEBUG_SERIAL_PRINT(s);
  DEBUG_SERIAL_PRINTLN();
#else
  serialPrintlnAndFlush(s);
#endif
//  pinMode(LED_HEATCALL, OUTPUT);
  LED_HEATCALL_OFF();
  sleepLowPowerMs(2*PP_OFF_MS); // TODO: use this time to gather entropy.
  
  int i = position;
  while(--i >= 0)
    {
    LED_HEATCALL_ON();
    tinyPause();
    LED_HEATCALL_OFF();
    sleepLowPowerMs(PP_OFF_MS); // TODO: use this time to gather entropy.
    }

  sleepLowPowerMs(PP_OFF_MS); // TODO: use this time to gather entropy.
  LED_HEATCALL_ON();
  sleepLowPowerMs(1000); // TODO: use this time to gather entropy.
  }

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
  serialPrintAndFlush(F("board V0.2 REV"));
  serialPrintAndFlush(V0p2_REV);
  serialPrintAndFlush(F("; code "));
  serialPrintAndFlush(_YYYYMmmDD);
  serialPrintAndFlush(F(" " __TIME__));
  serialPrintlnAndFlush();
  }


// Setup routine: runs once after reset.
// Does some limited board self-test and will panic() if anything is obviously broken.
void setup()
  {
  // Set appropriate low-power states, interrupts, etc, ASAP.
  powerSetup();

  // Capture early sub-cycle time to help ensure that the 32768Hz async clock is actually running.
  const uint8_t earlySCT = getSubCycleTime();

  IOSetup();

  // Restore previous RTC state if available.
  RTC::restoreRTC();
  // TODO: consider code to calibrate the internal RC oscillator against the xtal, eg to keep serial comms happy, eg http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=36237&start=0

  serialPrintAndFlush(F("\r\nOpenTRV booting: ")); // Leading CRLF to clear leading junk, eg from bootloader.
  serialPrintlnBuildVersion();

  // Count resets to detect unexpected crashes/restarts.
  const uint8_t oldResetCount = eeprom_read_byte((uint8_t *)EE_START_RESET_COUNT);
  eeprom_write_byte((uint8_t *)EE_START_RESET_COUNT, 1 + oldResetCount);

#ifdef DEBUG
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("DEBUG mode with serial logging");
#endif
#ifdef DEBUG
  DEBUG_SERIAL_PRINT_FLASHSTRING("Resets: ");
  DEBUG_SERIAL_PRINT(oldResetCount);
  DEBUG_SERIAL_PRINTLN();
  const uint8_t overruns = (~eeprom_read_byte((uint8_t *)EE_START_OVERRUN_COUNTER)) & 0xff;
  if(0 != overruns)
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("Overruns: ");
    DEBUG_SERIAL_PRINT(overruns);
    DEBUG_SERIAL_PRINTLN();
    }
  // Compute approx free RAM: see http://jeelabs.org/2011/05/22/atmega-memory-use/
  DEBUG_SERIAL_PRINT_FLASHSTRING("Free RAM: ");
  extern int __heap_start, *__brkval;
  int x;
  DEBUG_SERIAL_PRINT((int) &x - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
  DEBUG_SERIAL_PRINTLN();
#ifdef UNIT_TESTS
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("UNIT TESTS WILL BE RUN...");
#endif
#endif

  posPOST(1, F("about to test RFM23"));

#ifdef USE_MODULE_RFM22RADIOSIMPLE
#if !defined(RFM22_IS_ACTUALLY_RFM23) && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("(Using RFM22.)");
#endif
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  RFM22PowerOnInit();
  // Check that the radio is correctly connected; panic if not...
  if(!RFM22CheckConnected()) { panic(); }
  // Configure the radio.
  RFM22RegisterBlockSetup(FHT8V_RFM22_Reg_Values);
  // Put the radio in low-power standby mode.
  RFM22ModeStandbyAndClearState();
#endif

  posPOST(2, F("RFM23 OK"));


#ifdef DEBUG
  // Do additional POST.
  optionalPOST();
#endif

  // Get current power supply voltage.
  const uint16_t Vcc = readBatterymV();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Vcc: ");
  DEBUG_SERIAL_PRINT(Vcc);
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("mV");
#endif
  // Get internal temperature measurement.
  const int intTempC16 = readInternalTemperatureC16();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Int temp: ");
  DEBUG_SERIAL_PRINT((intTempC16 + 8) >> 4);
  DEBUG_SERIAL_PRINT_FLASHSTRING("C / ");
  DEBUG_SERIAL_PRINT(intTempC16);
  DEBUG_SERIAL_PRINTLN();
#endif

  posPOST(3, F("internal sensors OK, next light/temp/other"));

#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Gathering initial inputs and computing target/demand...");
#endif
  // Collect full set of environmental values before entering loop().
  // This should also help ensure that sensors are properly initialised.
  const lightLevel_t light = Light::readAmbientLight();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("light: ");
  DEBUG_SERIAL_PRINT(light);
  DEBUG_SERIAL_PRINTLN();
#endif
//  // Assume 0 or full-scale values unlikely.
//  if((0 == light) || (light >= 1023)) { panic(F("LDR fault")); }

  const tempC16_t heat = Temp::readTemperatureC16();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("temp: ");
  DEBUG_SERIAL_PRINT(heat);
  DEBUG_SERIAL_PRINTLN();
#endif

#if defined(TEMP_POT_AVAILABLE)
  const int tempPot = readTempPot();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("temp pot: ");
  DEBUG_SERIAL_PRINT(tempPot);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif

  // Check buttons not stuck enabled.
  if(fastDigitalRead(BUTTON_MODE_L) == LOW) { panic(F("M stuck")); }
#if defined(BUTTON_LEARN_L)
  if(fastDigitalRead(BUTTON_LEARN_L) == LOW) { panic(F("L stuck")); }
#endif
#if defined(BUTTON_LEARN2_L)
  if(fastDigitalRead(BUTTON_LEARN2_L) == LOW) { panic(F("L2 stuck")); }
#endif


  posPOST(4, F("light/temp OK"));

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Gathering initial inputs and computing target/demand...");
#endif
  // Update targets, output to TRV and boiler, etc, to be sensible before main loop starts.
  computeCallForHeat();

#if defined(USE_MODULE_FHT8VSIMPLE)
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Creating initial FHT8V frame...");
#endif
  // Unconditionally ensure that a valid FHT8V TRV command frame has been computed and stored
  // in case this unit is actually controlling a local valve.
  FHT8VCreateValveSetCmdFrame();
#endif

  // Seed PRNG(s) with available environmental values and clock time/jitter for some entropy.
  // Also sweeps over SRAM and EEPROM (see RAMEND and E2END), especially for non-volatile state and uninitialised areas of SRAM.
  // TODO: add better PRNG with entropy pool (eg for crypto).
  // TODO: add RFM22B WUT clock jitter, RSSI, temperature and battery voltage measures.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Starting seed computation...");
#endif
  const uint16_t srseed = sramCRC();
  const uint16_t eeseed = eeCRC();
  // Check that the 32768Hz async clock is actually running having done significant CPU-intensive work.
  const uint8_t laterSCT = getSubCycleTime();
  if(laterSCT == earlySCT)
    {
#if defined(WAKEUP_32768HZ_XTAL)
    // Allow (an extra) 1s+ for 32768Hz crystal to start reliably, see: http://www.atmel.com/Images/doc1259.pdf
#if 1 && defined(DEBUG)
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("Sleeping to let async 32768Hz clock start...");
#endif
    // Time spent here should not be a whole multiple of basic cycle time to avoid spuriously stationary async clock reading!
    for(int i = 20; (--i >= 0) && (earlySCT == getSubCycleTime()); )
      {
      sleepLowPowerMs(691);
      captureEntropy1();
      }
#endif
    const uint8_t latestSCT = getSubCycleTime();
    if(latestSCT == earlySCT)
      {
#if 1 && defined(DEBUG)
      DEBUG_SERIAL_PRINTLN_FLASHSTRING("Async 32768Hz clock may not be running!");
#endif
      panic(F("XTAL dead")); // Async clock not running.
      }
    }
  posPOST(5, F("slow RTC clock OK"));
  // DHD20130430: maybe as much as 16 bits of entropy on each reset in seed1, concentrated in the least-significant bits.
  const uint16_t s16 = (__DATE__[5]) ^ ((RTC::getMinutesSinceMidnightLT() << 5) ^ (((int)getSubCycleTime()) << 7) ^ (heat << 2) ^ (light << 6) ^ Vcc ^ intTempC16); /* This fits in an int (16 bits). */
  //const long seed1 = ((((long) clockJitterRTC()) << 13) ^ (((long)clockJitterWDT()) << 21) ^ (((long)(srseed^eeseed)) << 16)) + s16;
  // Seed simple/fast/small built-in PRNG.  (Smaller and faster than srandom()/random().)
  const uint8_t nar1 = noisyADCRead();
  seedRNG8(nar1 ^ (uint8_t) s16, oldResetCount - (uint8_t)((s16+eeseed) >> 8), clockJitterWDT() ^ (uint8_t)srseed);
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("nar ");
  DEBUG_SERIAL_PRINTFMT(nar1, BIN);
  DEBUG_SERIAL_PRINTLN();
#endif
  // TODO: seed other/better PRNGs.
  // Feed in mainly persistent/nonvolatile state explicitly. 
  addEntropyToPool(oldResetCount ^ eeseed, 0);
  addEntropyToPool((uint8_t)(eeseed >> 8) + nar1, 0);
  addEntropyToPool((uint8_t)s16 ^ (uint8_t)(s16 >> 8) , 0);
  for(uint8_t i = 0; i < EE_LEN_SEED; ++i)
    { addEntropyToPool(eeprom_read_byte((uint8_t *)(EE_START_SEED + i)), 0); }
  addEntropyToPool(noisyADCRead(), 4); // Conservative first push of noise into pool.
  // Carry a few bits of entropy over a reset by picking one of the four designated EEPROM bytes at random;
  // if zero, erase to 0xff, else AND in part of the seed including some of the previous EEPROM hash (and write).
  // This amounts to about a quarter of an erase/write cycle per reset/restart per byte, or 400k restarts endurance!
  // These 4 bytes should be picked up as part of the hash/CRC of EEPROM above, next time,
  // essentially forming a longish-cycle (poor) PRNG even with little new real entropy each time.
  seedRNG8(nar1 ^ (uint8_t) s16, oldResetCount - (uint8_t)((s16+eeseed) >> 8), clockJitterWDT() ^ (uint8_t)srseed);
  uint8_t *const erp = (uint8_t *)(EE_START_SEED + (3&((s16)^((eeseed>>8)+(__TIME__[7]))))); // Use some new and some eeseed bits to choose which byte to updated.
  const uint8_t erv = eeprom_read_byte(erp);
  if(0 == erv) { eeprom_smart_erase_byte(erp); }
  else { eeprom_smart_clear_bits(erp, clockJitterEntropyByte() + ((uint8_t)eeseed)); } // Nominally include disjoint set of eeseed bits in choice of which to clear.
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

  // Initialised: turn heatcall UI LED off.
//  pinMode(LED_HEATCALL, OUTPUT);
  LED_HEATCALL_OFF();
  
#if defined(SUPPORT_CLI)
  // Help user get to CLI.
  serialPrintlnAndFlush(F("? at CLI prompt for help"));
#endif

  // Ensure ID set up (mainly on first use).
  // Have one attempt (don't want to bang already failign EEPROM) to force-reset if not good, then panic.
  if(!ensureIDCreated())
    {
    if(!ensureIDCreated(true)) // Force reset.
      { panic(F("Invalid ID and cannot reset, sorry.")); }
    }

  // Report initial status.
  serialStatusReport();

  // Set appropriate loop() values just before entering it.
  TIME_LSD = RTC::getSecondsLT();
  }






// Call this to do an I/O poll if needed; returns true if something useful happened.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// Limits actual poll rate to something like once every 32ms, unless force is true.
//   * force if true then force full poll on every call (ie do not internally rate-limit)
bool pollIO(const bool force)
  {
#if defined(ENABLE_BOILER_HUB) && defined(USE_MODULE_FHT8VSIMPLE)
  if(inHubMode())
    {
    static volatile uint8_t _pO_lastPoll;

    // Poll RX at most about every ~32ms to help approx match spil rate when called in loop with 30ms nap.
    const uint8_t sct = getSubCycleTime();
    if(force || ((0 == (sct & 3)) && (sct != _pO_lastPoll)))
      {
      _pO_lastPoll = sct;
      if(FHT8VCallForHeatPoll()) // Check if call-for-heat has been overheard.
        { return(true); }
      }
    }
#endif
  return(false);
  }





//========================================
// MAIN LOOP
//========================================

#ifdef UNIT_TESTS // Run unit tests *instead* of normal loop() code. 
void loop() { unitTestLoop(); }
#else

// 'Elapsed minutes' count of minute/major cycles; cheaper than accessing RTC and not tied to real time.
static uint8_t minuteCount;

#if defined(ENABLE_BOILER_HUB)
// Ticks until locally-controlled boiler should be turned off; boiler should be on while this is positive.
// Ticks are the mail loop time, 1s or 2s.
// Used in hub mode only.
static uint16_t boilerCountdownTicks;
// Minutes since boiler last on as result of remote call for heat.
// Reducing listening if quiet for a while helps reduce self-heating temperature error
// (~2C as of 2013/12/24 at 100% RX, ~100mW heat dissipation in V0.2 REV1 box) and saves some energy.
// Time thresholds could be affected by eco/comfort switch.
#define RX_REDUCE_MIN_M 20 // Minimum minutes quiet before considering reducing RX duty cycle listening for call for heat; [1--255], 10--60 typical.
// IF DEFINED then give backoff threshold to minimise duty cycle.
// #define RX_REDUCE_MAX_M 240 // Minutes quiet before considering maximally reducing RX duty cycle; ]RX_REDUCE_MIN_M--255], 30--240 typical.
static uint8_t boilerNoCallM;
#endif

// The main control loop routine runs forever.
// Note: exiting loop() and re-entering can take a little while, handling Arduino background tasks such as serial.
void loop()
  {
#if 0 && defined(DEBUG) // Indicate loop start.
  DEBUG_SERIAL_PRINT('L');
  DEBUG_SERIAL_PRINT(TIME_LSD);
  DEBUG_SERIAL_PRINTLN();
#endif

  // Set up some variables before sleeping to minimise delay/jitter after the RTC tick.
  bool showStatus = false; // Show status at end of loop?

  // Use the zeroth second in each minute to force extra deep device sleeps/resets, etc.
  const bool second0 = (0 == TIME_LSD);
  if(second0)
  {
    // Tick minute.
    RTC::tickMinuteRTC();
  }
  // Sensor readings are taken late in each minute (where they are taken)
  // and if possible noise and heat and light should be minimised in this part of each minute to improve readings.
  // const bool sensorReading30s = (TIME_LSD >= 30);
  // Sensor readings and (stats transmissions) are nominally on a 4-minute cycle.
  const uint8_t minuteFrom4 = (minuteCount & 3);
  // The 0th minute in each group of four is always used for measuring where possible (possibly amongst others)
  // and where possible locally-generated noise and heat and light should be minimised in this minute to give the best possible readings.
  // This is the first (0th) minute in each group of four.
  const bool minute0From4ForSensors = (0 == minuteFrom4);
  // True the minute after all sensors should have been sampled.
  const bool minute1From4AfterSensors = (1 == minuteFrom4);

  // Note last-measured battery status.
  const bool batteryLow = isBatteryLow();

  // Run some tasks less often when not demanding heat (at the valve or boiler), so as to conserve battery/energy.
  const bool conserveBattery =
    (batteryLow || !inWarmModeDebounced()) && // Don't spare the batteries unless low, or in FROST mode (which should be most of the time).
#if defined(ENABLE_BOILER_HUB)
    (0 == boilerCountdownTicks) && // Unless the boiler is off, stay responsive.
#endif
    (!isControlledValveOpen()) &&  // Run at full speed until the FHT8V valve should actually have shut and the boiler gone off.
    (0 == getTRVPercentOpen()); // Run at full speed until not nominally demanding heat, eg even during FROST mode or pre-heating.

  // Try if very near to end of cycle and thus causing an overrun.
  // Conversely, if not true, should have time to savely log outputs, etc.
  const uint8_t nearOverrunThreshold = GSCT_MAX - 8; // ~64ms/~32 serial TX chars of grace time...
  bool tooNearOverrun = false; // Set flag that can be checked later.

  // Is this unit currently in central hub listener mode?
  const bool hubMode = inHubMode();

  // Check (early) for any remote stats arriving to dump.
  // This is designed to be easy to pick up by reading the serial output.
  // The output is terse to avoid taking too long and possibly delaying other stuff too far.
  // Avoid doing this at all if too near the end of the cycle and risking overrun,
  // leaving any message queued, hoping it does not get overwritten.
  if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }
  else
    {
    FullStatsMessageCore_t stats;
    getLastCoreStats(&stats);
    if(stats.containsID)
      {
      // Dump (remote) stats field '@<hexnodeID>;TnnCh[P;]'
      // where the T field shows temperature in C with a hex digit after the binary point indicated by C
      // and the optional P field indicates low power.
      serialPrintAndFlush(LINE_START_CHAR_RSTATS);
      serialPrintAndFlush((((uint16_t)stats.id0) << 8) | stats.id1, HEX);
      if(stats.containsTempAndPower)
        {
        serialPrintAndFlush(F(";T"));
        serialPrintAndFlush(stats.tempAndPower.tempC16 >> 4, DEC);
        serialPrintAndFlush('C');
        serialPrintAndFlush(stats.tempAndPower.tempC16 & 0xf, HEX);
        if(stats.tempAndPower.powerLow) { serialPrintAndFlush(F(";P")); } // Insert power-low field if needed.
        }
      if(stats.containsAmbL)
        {
        serialPrintAndFlush(F(";L"));
        serialPrintAndFlush((uint16_t) (stats.ambL << 2));  // revert to light scaling
        }
      if(0 != stats.occ)
        {
        serialPrintAndFlush(F(";O"));
        serialPrintAndFlush(stats.occ);
        }
      serialPrintlnAndFlush();
      }
    }

  // IF IN CENTRAL HUB MODE: listen out for OpenTRV units calling for heat.
  // Power optimisation 1: when >> 1 TX cycle (of ~2mins) need not listen, ie can avoid enabling receiver.
  // Power optimisation 2: TODO: when (say) >>30m since last call for heat then only sample listen for (say) 3 minute in 10 (not at a TX cycle multiple).
  // TODO: These optimisation are more important when hub unit is running a local valve
  // to avoid temperature over-estimates from self-heating,
  // and could be disabled if no local valve is being run to provide better response to remote nodes.
  bool hubModeBoilerOn = false; // If true then remote call for heat is in progress.
#if defined(USE_MODULE_FHT8VSIMPLE)
  bool needsToEavesdrop = false; // By default assume no need to eavesdrop.
#endif
  if(hubMode)
    {
#if defined(USE_MODULE_FHT8VSIMPLE)
    // Final poll to to cover up to end of previous minor loop.
    // Keep time from here to following SetupToEavesdropOnFHT8V() as short as possible to avoid missing remote calls.
    FHT8VCallForHeatPoll();

    // Fetch and clear current pending sample house code calling for heat.
    const uint16_t hcRequest = FHT8VCallForHeatHeardGetAndClear();
    const bool heardIt = (hcRequest != ((uint16_t)~0));
    // Don't log call for hear if near overrun,
    // and leave any error queued for next time.
    if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }
    else
      {
      if(heardIt)
        {
        DEBUG_SERIAL_TIMESTAMP();
        DEBUG_SERIAL_PRINT(' ');
        serialPrintAndFlush(F("CfH ")); // Call for heat from 
        serialPrintAndFlush((hcRequest >> 8) & 0xff);
        serialPrintAndFlush(' ');
        serialPrintAndFlush(hcRequest & 0xff);
        serialPrintlnAndFlush();
        }
      else
        {
        // Check for error if nothing received.
        const uint8_t err = FHT8VLastRXErrGetAndClear();
        if(0 != err)
          {
          serialPrintAndFlush(F("!RXerr F"));
          serialPrintAndFlush(err);
          serialPrintlnAndFlush();
          }
        }
      }

    // Record call for heat, both to start boiler-on cycle and to defer need to listen again. 
    // Optimisation: may be able to stop RX if boiler is on for local demand (can measure local temp better: less self-heating).
    if(heardIt)
      {
      if(0 == boilerCountdownTicks)
        {
        if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }
        else { serialPrintlnAndFlush(F("RCfH1")); } // Remote call for heat on.
        }
      boilerCountdownTicks = getMinBoilerOnMinutes() * (60/MAIN_TICK_S);
      boilerNoCallM = 0; // No time has passed since the last call.
      }
    // Else count down towards boiler off.
    else if(boilerCountdownTicks > 0)
      {
      if(0 == --boilerCountdownTicks)
        {
        if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }
        else { serialPrintlnAndFlush(F("RCfH0")); } // Remote call for heat off
        }
      }
    // Else already off so count up quiet minutes...
    else if(second0 && (boilerNoCallM < (uint8_t)~0)) { ++boilerNoCallM; }         

    // Turn boiler output on or off in response to calls for heat.
    hubModeBoilerOn = (boilerCountdownTicks > 0);

    // If not running a local TRV, and this without local temperature measurement problems from self-heating,
    // then just listen all the time for maximum simplicity and responsiveness at some cost in extra power consumption.
    // (At least as long as power is not running low for some reasons.)
    if(!localFHT8VTRVEnabled() && !batteryLow)
      { needsToEavesdrop = true; }
    // Try to avoid listening in the 'quiet' sensor minute in order to minimise noise and power consumption and self-heating.
    // Optimisation: if just heard a call need not listen on this next cycle.
    // Optimisation: if boiler timeout is a long time away (>> one FHT8V TX cycle, ~2 minutes excl quiet minute), then can avoid listening for now.
    //    Longish period without any RX listening may allow hub unit to cool and get better sample of local temperature if marginal.
    // Aim to listen in one stretch for greater than full FHT8V TX cycle of ~2m to avoid missing a call for heat.
    // MUST listen for all of final 2 mins of boiler-on to avoid missing TX (without forcing boiler over-run).
    else if((boilerCountdownTicks <= ((MAX_FHT8V_TX_CYCLE_HS+1)/(2*MAIN_TICK_S))) && // Don't miss a final TX that would keep the boiler on...
       (boilerCountdownTicks != 0)) // But don't force unit to listen/RX all the time if no recent call for heat.
      { needsToEavesdrop = true; }
    else if((!heardIt) &&
       (!minute0From4ForSensors) &&
       (boilerCountdownTicks <= (RX_REDUCE_MIN_M*(60/MAIN_TICK_S)))) // Listen eagerly for fresh calls for heat for last few minutes before turning boiler off.
      {
#if defined(RX_REDUCE_MAX_M) && defined(LOCAL_TRV)
      // Skip the minute before the 'quiet' minute also in very quiet mode to improve local temp measurement.
      // (Should still catch at least one TX per 4 minutes at worst.)
      needsToEavesdrop =
          ((boilerNoCallM <= RX_REDUCE_MAX_M) || (3 != (minuteCount & 3)));
#else
      needsToEavesdrop = true;
#endif
      }

#endif
    }

#if defined(USE_MODULE_FHT8VSIMPLE)
  // Act on eavesdropping need, setting up or clearing down hooks as required.
  if(needsToEavesdrop)
    {
    // Ensure radio is in RX mode rather than standby, and possibly hook up interrupts if available (REV1 board).
    SetupToEavesdropOnFHT8V(second0); // Start listening (if not already so).
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("hub listen, on/cd ");
    DEBUG_SERIAL_PRINT(boilerCountdownTicks);
    DEBUG_SERIAL_PRINT_FLASHSTRING("t quiet ");
    DEBUG_SERIAL_PRINT(boilerNoCallM);
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("m");
#endif
    }
  else
    {
    // Power down and clear radio state (if currently eavesdropping).
    StopEavesdropOnFHT8V(second0);
    // Clear any RX state so that nothing stale is carried forward.
    FHT8VCallForHeatHeardGetAndClear();
    }
#endif


  // Set BOILER_OUT as appropriate for local and/or remote calls for heat.
  // FIXME: local valve-driven boiler on does not obey normal on/off run-time rules.
  fastDigitalWrite(OUT_HEATCALL, ((hubModeBoilerOn || isControlledValveOpen()) ? HIGH : LOW));


  // Sleep in low-power mode (waiting for interrupts) until seconds roll.
  // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
  // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*E"); // End-of-cycle sleep.
#endif
  powerDownSerial(); // Ensure that serial I/O is off.
  // Power down most stuff (except radio for hub RX).
  minimisePowerWithoutSleep();
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = RTC::getSecondsLT()))
    {
#if defined(ENABLE_BOILER_HUB) && defined(USE_MODULE_FHT8VSIMPLE) // Deal with FHT8V eavesdropping if needed.
    // Poll for RX of remote calls-for-heat if needed.
    if(needsToEavesdrop) { nap30AndPoll(); continue; }
#endif
#if defined(USE_MODULE_RFM22RADIOSIMPLE) // Force radio to power-saving standby state if appropriate.
    // Force radio to known-low-power state from time to time (not every time to avoid unnecessary SPI work, LED flicker, etc.)
    if(batteryLow || second0) { RFM22ModeStandbyAndClearState(); }
#endif
    sleepUntilInt(); // Normal long minimal-power sleep until wake-up interrupt.
    }
  TIME_LSD = newTLSD;
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif

#if defined(ENABLE_BOILER_HUB) && defined(USE_MODULE_FHT8VSIMPLE) // Deal with FHT8V eavesdropping if needed.
  // Check RSSI...
  if(needsToEavesdrop)
    {
    const uint8_t rssi = RFM22RSSI();
    static uint8_t lastRSSI;
    if((rssi > 0) && (lastRSSI != rssi))
      {
      lastRSSI = rssi;
      addEntropyToPool(rssi, 0); // Probably some real entropy but don't assume it.
#if 0 && defined(DEBUG)
      DEBUG_SERIAL_PRINT_FLASHSTRING("RSSI=");
      DEBUG_SERIAL_PRINT(rssi);
      DEBUG_SERIAL_PRINTLN();
#endif
      }
    }
#endif

#if 0 && defined(DEBUG) // Show CPU cycles.
  DEBUG_SERIAL_PRINT('C');
  DEBUG_SERIAL_PRINT(cycleCountCPU());
  DEBUG_SERIAL_PRINTLN();
#endif


  // START LOOP BODY
  // ===============


  // Warn if too near overrun before.
  if(tooNearOverrun) { serialPrintlnAndFlush(F("?near overrun")); }


  // Get current power supply voltage.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Vcc: ");
  DEBUG_SERIAL_PRINT(readBatterymV());
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("mV");
#endif


#if defined(USE_MODULE_FHT8VSIMPLE)
  // FHT8V is highest priority and runs first.
  // ---------- HALF SECOND #0 -----------
  bool useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_First(!conserveBattery); // Time for extra TX before UI.
//  if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@0"); }
#endif


  // High-priority UI handing, every other/even second.
  // Show status if the user changed something significant.
  // Must take ~300ms or less so as not to run over into next half second if two TXs are done.
  bool recompute = false; // Set true an extra recompute of target temperature should be done.
#if !defined(TWO_S_TICK_RTC_SUPPORT)
  if(0 == (TIME_LSD & 1))
#endif
    {
    if(tickUI(TIME_LSD))
      {
      showStatus = true;
      recompute = true;
      }
    }
 
  
  if(recompute || recentUIControlUse())
    {
    computeTargetTemperature(); // Force recompute of temperature for (UI) responsiveness.
    }


#if defined(USE_MODULE_FHT8VSIMPLE)
  if(useExtraFHT8VTXSlots)
    {
    // Time for extra TX before other actions, but don't bother if minimising power in frost mode.
    // ---------- HALF SECOND #1 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(!conserveBattery); 
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@1"); }
    }
#endif


// While debugging switchover to phototransistor...
//  DEBUG_SERIAL_PRINT('L');
//  DEBUG_SERIAL_PRINT(Light::readAmbientLight());
//  DEBUG_SERIAL_PRINTLN();


  // DO SCHEDULING

  // Once-per-minute tasks: all must take << 0.3s.
  // Run tasks spread throughout the minute to be as kind to batteries (etc) as possible.
  // Only when runAll is true run less-critical tasks that be skipped sometimes when particularly conserving energy.
  // TODO: coordinate temperature reading with time when radio and other heat-generating items are off for more accurate readings.
  // TODO: ensure only take ambient light reading at times when all LEDs are off.
  const bool runAll = (!conserveBattery) || minute0From4ForSensors;

  switch(TIME_LSD) // With TWO_S_TICK_RTC_SUPPORT only even seconds are available.
    {
    case 0:
      {
      // Tasks that must be run every minute.
      ++minuteCount;
      checkUserSchedule(); // Force to user's programmed settings, if any, at the correct time.
      // Ensure that the RTC has been persisted promptly when necessary.
      RTC::persistRTC();
      break;
      }

    // Churn/reseed PRNG(s) a little to improve unpredictability in use: should be lightweight.
    case 2: { if(runAll) { seedRNG8(minuteCount ^ cycleCountCPU() ^ (uint8_t)getBatterymV(), getSubCycleTime() ^ (uint8_t)Light::getAmbientLight(), (uint8_t)Temp::getTemperatureC16()); } break; }
    // Force read of battery voltage; measure and recompute status less often when already thought to be low, eg when conserving.
    case 4: { if(runAll) { readBatterymV(); } break; }

#if defined(USE_MODULE_FHT8VSIMPLE) && defined(FHT8V_ALLOW_EXTRA_TXES)
    // Re-transmit slot for additional comms with boiler hub, eg when valve wide open, to help ensure reliable/fast call for heat.
    // This is entirely optional, and just improves odds of an urgent call for heat being acted on quickly,
    // so it can be cancelled with any reasonable excuse to save some energy and bandwidth.
    // This should always be safe, ie not done unless valve actually open, etc,
    // though it may cause the boiler to overrun a little into a closing valve under some circumstances.
    case 8:
      {
      if((!hubMode) && // Hub doesn't need to send extra TXes to itself!
         (!batteryLow) && // Don't send if battery is low.
         (!useExtraFHT8VTXSlots) && // Don't send if there's an immediately pending TX.
         inWarmModeDebounced() && // Only do extra TX if still in a warming mode, ie don't TX if mode just changed to FROST.
         (isControlledValveOpen()) && // Valve should be open already so we won't hurt the pump/boiler with call for heat.
         ((getTRVPercentOpen() >= 75) || inBakeModeDebounced())) // Valve fairly wide open, eg for BAKE or because boiler not hearing us reliably.
          {
#if 1 && defined(DEBUG)
          DEBUG_SERIAL_PRINTLN_FLASHSTRING("Extra FTH8V TX");
#endif
          pollIO(); // Deal with any pending I/O.
          sleepLowPowerLessThanMs(1 | (randRNG8() & 0x7f)); // Sleep randomly up to 127ms to spread transmissions and help avoid collisions.
          pollIO(); // Deal with any pending I/O.
          FHT8VDoSafeExtraTXToHub(); // Takes up to ~80ms.
          }
      break;
      }
#endif

    // Regular transmission of stats if NOT driving a local valve and not a hub, ie needing to stay in RX mode.
    case 10:
      {
      if(hubMode) { break; } // Hub can't allow random switch to TX mode.
      if(!enableTrailingMinimalStatsPayload()) { break; } // Not allowed to send stuff like this.
      
#if defined(USE_MODULE_FHT8VSIMPLE)
      if(localFHT8VTRVEnabled()) { break; } // Do only piggy-back stats TXes if driving an FHT8V to save bandwidth/conflicts.
#endif
      // Generally only attempt stats TX in the minute after all sensors should have been polled (so that readings are fresh).
      if(minute1From4AfterSensors ||
        (!batteryLow && (0 == (0x24 & randRNG8())))) // Occasional additional TX when not conserving power.
        {
        // Gather core stats.
        FullStatsMessageCore_t content;
        populateCoreStats(&content);

        // Create (insecure) message.
        uint8_t buf[RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES + FullStatsMessageCore_MAX_BYTES_ON_WIRE + 1];
        uint8_t *bptr = buf;
        // Start with RFM23-friendly preamble which ends with with the aacccccc sync word.
        memset(bptr, RFM22_PREAMBLE_BYTE, RFM22_PREAMBLE_BYTES);
        bptr += RFM22_PREAMBLE_BYTES;
        memset(bptr, RFM22_SYNC_BYTE, RFM22_SYNC_MIN_BYTES);
        bptr += RFM22_SYNC_MIN_BYTES;
        const uint8_t *msg1 = encodeFullStatsMessageCore(bptr, sizeof(buf) - (bptr-buf), getStatsTXLevel(), false, &content);
        if(NULL == msg1)
          {
DEBUG_SERIAL_PRINTLN_FLASHSTRING("Msg gen err!");
          break;
          }

        pollIO(); // Deal with any pending I/O.
        sleepLowPowerLessThanMs(1 | (randRNG8() & 0x7f)); // Sleep randomly up to 127ms to spread transmissions and help avoid collisions.
        pollIO(); // Deal with any pending I/O.

        // TODO: put in listen before TX to reduce collisions (CSMA)
        // Send message!
        // Assume RFM22/23 support for now.
        RFM22QueueCmdToFF(buf);
        RFM22TXFIFO(); // Send it!  Approx 1.6ms/byte.     
        RFM22ModeStandbyAndClearState(); // Go to standby to conserve energy.
DEBUG_SERIAL_PRINTLN_FLASHSTRING("Bare stats TX");
        }
      break;
      }

    // Read all environmental inputs, late in the cycle.
#ifdef TEMP_POT_AVAILABLE
    // Sample the user-selected WARM temperature target.
    case 50: { if(runAll) { readTempPot(); } break; }
#endif
    // Sample ambient light levels.
    case 52: { if(runAll) { Light::readAmbientLight(); } break; }
    // At a hub, sample temperature as late as possible in (and only in the 'quiet') minute, to reduce valve hunting from self-heating.
    case 54: { if(hubMode ? minute0From4ForSensors : runAll) { Temp::readTemperatureC16(); } break; }

    // Compute targets and heat demand based on environmental inputs.
    // Note: ensure that valve-shut message is always conveyed quickly to valve even in slow/'conserve' mode.
    // Also drives OUT_HEATCALL to control local boiler if in central hub mode.
    case 56:
      {
      static bool boilerOn; // Internal record of current boiler-out state.

      // Recompute target, valve position and call for heat, etc.
      // Should be called once per minute to work correctly.
      if(computeCallForHeat() ||
         (minute1From4AfterSensors && enableTrailingMinimalStatsPayload()))
        {
        // If there was a change in target valve position,
        // or periodically in a minute after all sensors should have been read,
        // precompute some or all of any outgoing frames/stats/etc ready for transmission.
#if defined(USE_MODULE_FHT8VSIMPLE)
        // Recompute FHT8V command to send if target valve setting has changed...
        if(localFHT8VTRVEnabled()) { FHT8VCreateValveSetCmdFrame(); }
#endif
        }

#if defined(ENABLE_BOILER_HUB)
      // Track how long since remote call for heat last heard.
      if(hubMode)
        {
        if(boilerCountdownTicks != 0)
          {
#if 1 && defined(DEBUG)
          DEBUG_SERIAL_PRINT_FLASHSTRING("Boiler on, s left: ");
          DEBUG_SERIAL_PRINT(boilerCountdownTicks * MAIN_TICK_S);
          DEBUG_SERIAL_PRINTLN();
#endif
          }
        }
#endif

      // Show current status if appropriate.
      if(runAll) { showStatus = true; }
      break;
      }

    // Stats samples; should never be missed.
    case 58:
      {
      // Take full stats sample as near the end of the hour as reasonably possible (without danger of overrun),
      // and with other optional non-full samples evenly spaced throughout the hour (if not low on battery).
      if(minute0From4ForSensors) // Hope to take lowest-noise samples on the special minute out of each 4.
        {
        const uint_least8_t mm = RTC::getMinutesLT();
        switch(mm)
          {
          case 16: case 17: case 18: case 19:
          case 36: case 37: case 38: case 39:
            { if(!batteryLow) { sampleStats(false); } break; } // Skip sub-samples if short of juice.
          case 56: case 57: case 58: case 59:
            { sampleStats(true); break; } // Always take the full sample at end of hour.
          }
        }
      break;
      }
    }

#if defined(USE_MODULE_FHT8VSIMPLE) && defined(TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // Time for extra TX before other actions, but don't bother if minimising power in frost mode.
    // ---------- HALF SECOND #2 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(!conserveBattery); 
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@2"); }
    }
#endif

  // Generate periodic status reports.
  if(showStatus) { serialStatusReport(); }

#if defined(USE_MODULE_FHT8VSIMPLE) && defined(TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // ---------- HALF SECOND #3 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(!conserveBattery); 
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@3"); }
    }
#endif

  // Command-Line Interface (CLI) polling.
  // If a reasonable chunk of the minor cycle remains after all other work is done
  // AND the CLI is / should be active OR a status line has just been output
  // then poll/prompt the user for input
  // using a timeout which should safely avoid missing the next basic tick
  // and which should also allow some energy-saving sleep.
#if 1 && defined(SUPPORT_CLI)
  const bool humanCLIUse = isCLIActive(); // Keeping CLI active for human interaction rather than for automated interaction.
  if(showStatus || humanCLIUse)
    {
    const uint8_t sct = getSubCycleTime();
    const uint8_t listenTime = max(GSCT_MAX/16, CLI_POLL_MIN_SCT);
    if(sct < (GSCT_MAX - 2*listenTime))
      // Don't listen beyond the last 16th of the cycle,
      // or a minimal time if only prodding for interaction with automated front-end,
      // as listening for UART RX uses lots of power.
      { pollCLI(humanCLIUse ? (GSCT_MAX-listenTime) : (sct+CLI_POLL_MIN_SCT)); }
    }
#endif



#if 0 && defined(DEBUG)
  const int tDone = getSubCycleTime();
  if(tDone > 1) // Ignore for trivial 1-click time.
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("done in "); // Indicates what fraction of available loop time was used / 256.
    DEBUG_SERIAL_PRINT(tDone);
    DEBUG_SERIAL_PRINT_FLASHSTRING(" @ ");
    DEBUG_SERIAL_TIMESTAMP();
    DEBUG_SERIAL_PRINTLN();
    }
#endif

  // Detect and handle (actual or near) overrun, if it happens, though it should not.
  if(TIME_LSD != RTC::getSecondsLT())
    {
    // Increment the overrun counter (stored inverted, so 0xff initialised => 0 overruns).
    const uint8_t orc = 1 + ~eeprom_read_byte((uint8_t *)EE_START_OVERRUN_COUNTER);
    eeprom_smart_update_byte((uint8_t *)EE_START_OVERRUN_COUNTER, ~orc);
#if 1 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("!ERROR: loop overrun ");
    DEBUG_SERIAL_PRINT(orc);
    DEBUG_SERIAL_PRINTLN();
#endif
#if defined(USE_MODULE_FHT8VSIMPLE)
    FHT8VSyncAndTXReset(); // Assume that sync with valve may have been lost, so re-sync.
#endif
    TIME_LSD = RTC::getSecondsLT(); // Prepare to sleep until start of next full minor cycle.
    }
#if 0 && defined(DEBUG) // Expect to pick up near overrun at start of next loop.
  else if(getSubCycleTime() >= nearOverrunThreshold)
    {
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("?O"); // Near overrun.  Note 2ms/char to send...
    }
#endif
  }

#endif // defined(UNIT_TESTS)
