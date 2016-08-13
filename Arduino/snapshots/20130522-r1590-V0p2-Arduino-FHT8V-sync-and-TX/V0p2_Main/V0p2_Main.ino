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

Author(s) / Copyright (s): Damon Hart-Davis 2013
*/

/*
  V0p2 (V0.2) core.

  DHD20130417: hardware setup on bare board.
    * 1MHz CPU clock (from 8MHz internal RC clock with /8 prescaler) ATmega328P running at 1.8V--5V.
    * Fuse set for BOD-managed additional clock settle time, ie as fast restart from sleep as possible.
    * All unused pins unconnected and nominally floating (though driven low as output where possible).
    * 32768Hz xtal between pins 9 and 10, async timer 2, for accurate timekeeping and low-power sleep.
    * All unused system modules turned off.

  Basic AVR power consumption ticking an (empty) control loop at ~1Hz should be ~1uA.
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
#include "Serial_Debug.h"
#include "Temperature_Sensor.h"
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
  minimsePowerWithoutSleep();
  pinMode(LED_HEATCALL, OUTPUT);
  for( ; ; )
    {
    fastDigitalWrite(LED_HEATCALL, HIGH);
    tinyPause();
    fastDigitalWrite(LED_HEATCALL, LOW);
    bigPause();
    }
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
  restoreRTC();
  // TODO: consider code to calibrate the internal RC oscillator against the xtal, eg to keep serial comms happy, eg http://www.avrfreaks.net/index.php?name=PNphpBB2&file=printview&t=36237&start=0

#ifdef DEBUG
  // If debugging, (re)enable USART and enable Serial subsystem.
  power_usart0_enable();
  Serial.begin(BAUD);
  DEBUG_SERIAL_PRINTLN();
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("DEBUG mode with serial logging...");
#ifdef UNIT_TESTS
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("UNIT TESTS WILL BE RUN...");
#endif
#endif

#ifdef USE_MODULE_RFM22RADIOSIMPLE
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  RFM22PowerOnInit();
  // Check that the radio is correctly connected; panic if not...
  if(!RFM22CheckConnected()) { panic(); }
  // Configure the radio.
  RFM22RegisterBlockSetup(FHT8V_RFM22_Reg_Values);
  // Put the radio in low-power standby mode.
  RFM22ModeStandbyAndClearState();
#endif

#ifdef DEBUG
  // Do additional POST.
  optionalPOST();
#endif

  // Get current power supply voltage.
  const uint16_t Vcc = powerSupplyVoltage();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Vcc: ");
  DEBUG_SERIAL_PRINT(Vcc);
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("mV");
#endif

  // Get internal temperature measurement.
  const int intTempC = internalTemperatureC();
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Int temp: ");
  DEBUG_SERIAL_PRINT(intTempC);
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("C");
#endif

#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Gathering initial inputs and computing target/demand...");
#endif
  // Collect full set of environmental values before entering loop().
  // This should also help ensure that sensors are properly initialised.
  const int heat = readTemperatureC16();
  const int light = readAmbientLight();
  // Update targets, output to TRV and boiler, etc, to be sensible before main loop starts.
  computeTargetAndDemand();

#if defined(USE_MODULE_FHT8VSIMPLE)
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Creating initial FHT8V frame...");
#endif
  // Unconditionally ensure that a valid FHT8V TRV command frame has been computed and stored.
  FHT8VCreateValveSetCmdFrame();
#endif

  // Seed PRNG(s) with available environmental values and clock time/jitter for some entropy.
  // Also sweeps over SRAM and EEPROM (see RAMEND and E2END), especially for non-volatile state and unitialised areas of SRAM.
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
#if 1 && defined(WAKEUP_32768HZ_XTAL)
    // Allow (an extra) 1s+ for 32768Hz crystal to start reliably, see: http://www.atmel.com/Images/doc1259.pdf
#if 1 && defined(DEBUG)
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("Sleeping to let async 32768Hz clock start...");
#endif
    sleepLowPowerMs(1500); // Should not be a whole multiple of basic cycle time to avoid spuriously stationary async clock reading!
#endif
    const uint8_t latestSCT = getSubCycleTime();
    if(latestSCT == earlySCT)
      {
#if 1 && defined(DEBUG)
      DEBUG_SERIAL_PRINTLN_FLASHSTRING("Async 32768Hz clock may not be running!");
#endif
      panic(); // Async clock not running.
      }
    }
  // DHD20130430: maybe as much as 16 bits of entropy on each reset in seed1, concentrated in the least-significant bits.
  const uint16_t s16 = ((getMinutesSinceMidnightLT() << 5) ^ (((int)getSubCycleTime()) << 7) ^ heat ^ (light << 6) ^ Vcc ^ intTempC); /* This fits in an int (16 bits). */
  //const long seed1 = ((((long) clockJitterRTC()) << 13) ^ (((long)clockJitterWDT()) << 21) ^ (((long)(srseed^eeseed)) << 16)) + s16;
  // Seed simple/fast/small built-in PRNG.  (Smaller and faster than srandom()/random().)
  seedRNG8((uint8_t) s16, (uint8_t)((s16+eeseed) >> 8), clockJitterWDT() ^ (uint8_t)srseed);
  // TODO: seed other/better PRNGs.
  // Capture a few bits of this seed over a reset by picking one of the two designated EEPROM bytes at random;
  // if zero, erase to 0xff, else AND in part of the seed including some of the previous EEPROM hash (and write).
  // This amounts to about a quarter of an erase/write cycle per reset/restart per byte, or 400k restarts endurance!
  // These bytes should be picked up as part of the hash/CRC of EEPROM above, next time,
  // essentially forming a longish-cycle (poor) PRNG even with little new real entropy each time.
  uint8_t *const erp = (uint8_t *)(EE_START_SEED + (1&((s16) ^ (eeseed>>8)))); // Use some new and some eeseed bits to choose byte to updated.
  const uint8_t erv = eeprom_read_byte(erp);
  if(0 == erv) { eeprom_smart_erase_byte(erp); }
  else { eeprom_smart_clear_bits(erp, clockJitterEntropyByte() ^ ((uint8_t)eeseed)); } // Include disjoint set of eeseed bits in choice of which to clear.
#if 1 && defined(DEBUG)
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

  // Initialised: turn heatcall UI LED off (and make it an output).
  pinMode(LED_HEATCALL, OUTPUT);
  fastDigitalWrite(LED_HEATCALL, LOW);

  // Report initial status.
  serialStatusReport();

  // Set appropriate loop() values just before entering it.
  TIME_LSD = getSecondsLT();
  }


#ifdef UNIT_TESTS // Run unit tests *instead* of normal loop() code. 
void loop() { unitTestLoop(); }
#else

// 'Elapsed minutes' count of minute/major cycles; cheaper than accessing RTC and not tied to real time.
static uint8_t minuteCount;

// The main control loop routine runs over and over again forever.
// Note: exiting loop() and re-entering can take a little while, handling Arduino background tasks such as serial.
void loop()
  {
  // Set up some variables before sleeping to minimise delay/jitter after the RTC tick.
  const bool conserveEnergy = !inWarmMode(); // Expect to spend most time in FROST mode, so save energy then.
  bool showStatus = false; // Show status at end of loop?

  // Sleep in low-power mode (waiting for interrupts) until seconds roll.
  // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
  // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
  minimsePowerWithoutSleep();
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = getSecondsLT())) { sleepUntilInt(); }
  TIME_LSD = newTLSD;


  // START LOOP BODY


#if defined(USE_MODULE_FHT8VSIMPLE)
  // FHT8V is highest priority and runs first.
  // ---------- HALF SECOND #0 -----------
  bool useExtraFHT8VTXSlots = FHT8VPollSyncAndTX_First(!conserveEnergy); // Time for extra TX before UI.
#endif


  // High-priority UI handing, every other/even second.
  // Show status if the user changed something significant.
  // Must take ~300ms or less so as not to run over into next half second if two TXs are done.
#if !defined(TWO_S_TICK_RTC_SUPPORT)
  if(0 == (TIME_LSD & 1))
#endif
    { if(tickUI(TIME_LSD)) { showStatus = true; } }


#if defined(USE_MODULE_FHT8VSIMPLE)
  if(useExtraFHT8VTXSlots)
    {
    // Time for extra TX before other actions, but don't bother if minimising power in frost mode.
    // ---------- HALF SECOND #1 -----------
    useExtraFHT8VTXSlots = FHT8VPollSyncAndTX_Next(!conserveEnergy); 
    }
#endif


  // DO SCHEDULING

  // Once-per-minute tasks: all must take << 0.3s.

  // TODO: set up prioritised task set as per PICAXE V0.09 version...
  if(0 == TIME_LSD)
    {
    // Tasks that must be run every minute.
    ++minuteCount;
    checkUserSchedule(); // Force to user's programmed settings, if any, at the correct time.
    // Ensure that RTC is persisted promptly when needed.
    persistRTC();
    }

  // Run tasks spread throughout the minute
  // that can be run somewhat less often when particularly conserving energy.
  // Runs only each 4 minutes in 'conserving' mode.
  if((!conserveEnergy) || (0 == (minuteCount & 3))) // DHD20130522: Arduino IDE / gcc generates poor code here...
    {
    switch(TIME_LSD) // With TWO_S_TICK_RTC_SUPPORT only even seconds are available.
      {
      // Read all environmental inputs.
      case 2: { readAmbientLight(); break; }
      case 4: { readTemperatureC16(); break; }

      // Compute targets and heat demand based on environmental inputs.
      case 6:
        {
        if(computeTargetAndDemand())
          {
#if defined(USE_MODULE_FHT8VSIMPLE)
          // Recompute FHT8V command to send if target valve setting has changed...
          FHT8VCreateValveSetCmdFrame();
#endif
          }

#if 0 && defined(BOILER_HUB)
        if boilerCountdownS != 0 then ; Remote calls for heat are still active.
            high OUT_HEATCALL
        else
#endif
          if(0 != getTRVPercentOpen()) // Local call for heat given local TRV is at least partly open/on.  (TODO: modulating!)
            { fastDigitalWrite(OUT_HEATCALL, HIGH); }
          else // Stop calling for heat from the boiler.
            { fastDigitalWrite(OUT_HEATCALL, LOW); }
#if 0 && defined(BOILER_HUB)
        endif
#endif
        break;
        }

      // Trailing less-critical/housekeeping tasks.
      case 8: { showStatus = true; break; }
      // Churn/reseed PRNG(s) a little to improve unpredictability when actually used.
      case 10: { seedRNG8(minuteCount ^ cycleCountCPU(), getSubCycleTime(), (uint8_t)getTemperatureC16()); break; }
      }
    }

#if defined(USE_MODULE_FHT8VSIMPLE) && defined(TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // Time for extra TX before other actions, but don't bother if minimising power in frost mode.
    // ---------- HALF SECOND #2 -----------
    useExtraFHT8VTXSlots = FHT8VPollSyncAndTX_Next(!conserveEnergy); 
    }
#endif

  // Generate periodic status reports.
  if(showStatus) { serialStatusReport(); }

#if defined(USE_MODULE_FHT8VSIMPLE) && defined(TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // ---------- HALF SECOND #3 -----------
    useExtraFHT8VTXSlots = FHT8VPollSyncAndTX_Next(!conserveEnergy); 
    }
#endif

  // CLI
  // If a reasonable chunk of the minor cycle remains after all other work is done
  // AND the CLI is / should be active
  // then poll/prompt the user for input here
  // using a timeout which should safely avoid missing the next basic tick
  // and which should also allow some energy-saving sleep.
  // TODO: be clever and if getSubCycleTime() has gone backwards then assume end tick has been missed and return from loop() without sleeping.
#if defined(SUPPORT_CLI)
  if(isCLIActive() && (getSubCycleTime() < (GSCT_MAX-(GSCT_MAX/8))))
    { pollCLI(msRemainingThisBasicCycle() / 2); }
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
  }

#endif // defined(UNIT_TESTS)
