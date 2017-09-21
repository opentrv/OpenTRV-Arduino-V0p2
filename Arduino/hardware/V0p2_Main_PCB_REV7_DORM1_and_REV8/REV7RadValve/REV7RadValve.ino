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

#include "V0p2_Main.h"
#include <OTAESGCM.h>

// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic()
  {
  // Reset radio and go into low-power mode.
  PrimaryRadio.panicShutdown();
  // Power down almost everything else...
  OTV0P2BASE::minimisePowerWithoutSleep();
  pinMode(OTV0P2BASE::LED_HEATCALL_L, OUTPUT);
  for( ; ; )
    {
    OTV0P2BASE::LED_HEATCALL_ON();
    OTV0P2BASE::nap(WDTO_15MS);
    OTV0P2BASE::LED_HEATCALL_OFF();
    OTV0P2BASE::nap(WDTO_120MS);
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
  OTV0P2BASE::LED_HEATCALL_OFF();

  // Skip much of lightshow if '0'/end/none position.
  if(position > 0)
    {
    OTV0P2BASE::sleepLowPowerMs(2*PP_OFF_MS); // TODO: use this time to gather entropy.
    for(int i = position; --i >= 0; )
      {
      OTV0P2BASE::LED_HEATCALL_ON();
      OTV0P2BASE::nap(WDTO_15MS);
      OTV0P2BASE::LED_HEATCALL_OFF();
      OTV0P2BASE::sleepLowPowerMs(PP_OFF_MS); // TODO: use this time to gather entropy.
      }
    }

  OTV0P2BASE::sleepLowPowerMs(PP_OFF_MS); // TODO: use this time to gather entropy.
  OTV0P2BASE::LED_HEATCALL_ON();
  OTV0P2BASE::sleepLowPowerMs(1000); // TODO: use this time to gather entropy.
  }


// Pick an appropriate radio config for RFM23 (if it is the primary radio).
// OTRadioChannelConfig(const void *_config, bool _isFull, bool _isRX, bool _isTX, bool _isAuth = false, bool _isEnc = false, bool _isUnframed = false)
#define RADIO_CONFIG_NAME "GFSK"
// Nodes talking on fast GFSK channel 0.
static constexpr uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] =
  {
  // GFSK channel 0 full config, RX/TX, not in itself secure.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true),
  };

// NO RADIO RX FILTERING BY DEFAULT
#define NO_RX_FILTER
#define FilterRXISR NULL

void optionalPOST()
  {
  // Have 32678Hz clock at least running before going any further.
  // Check that the slow clock is running reasonably OK, and tune the fast one to it.
  if(!::OTV0P2BASE::HWTEST::calibrateInternalOscWithExtOsc()) { panic(F("Xtal")); } // Async clock not running or can't tune.

  // Signal that xtal is running AND give it time to settle.
  posPOST(0 /*, F("about to test radio module") */);

// FIXME  This section needs refactoring
// TODO-547: why does nested SPI enable break things?
//  const bool neededToWakeSPI = OTV0P2BASE::powerUpSPIIfDisabled();
//  DEBUG_SERIAL_PRINT(neededToWakeSPI);
//  DEBUG_SERIAL_PRINTLN();
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  PrimaryRadio.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!PrimaryRadio.configure(nPrimaryRadioChannels, RFM23BConfigs) || !PrimaryRadio.begin()) { panic(F("r1")); }

//  posPOST(1, F("Radio OK, checking buttons/sensors and xtal"));

// Buttons should not be activated DURING boot for user-facing boards; an activated button implies a fault.
  // Check buttons not stuck in the activated position.
  if(fastDigitalRead(BUTTON_MODE_L) == LOW) { panic(F("b")); }

// Save space (and time) by avoiding the second POST sequence; LED will be turned off anyway.
//  // Single/main POST checkpoint for speed.
//  posPOST(1 /* , F("POST OK") */ );
  }

/////// SENSORS

// Sensor for supply (eg battery) voltage in millivolts.
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;

TempPot_t TempPot;

AmbientLight AmbLight;

// Singleton implementation/instance.
OTV0P2BASE::HumiditySensorSHT21 RelHumidity;

// Ambient/room temperature sensor, usually on main board.
OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.

////////////////////////// Actuators

// DORM1/REV7 direct drive actuator.
// Singleton implementation/instance.
// Suppress unnecessary activity when room dark, eg to avoid disturbance if device crashes/restarts,
// unless recent UI use because value is being fitted/adjusted.
ValveDirect_t ValveDirect([](){return((!valveUI.veryRecentUIControlUse()) && AmbLight.isRoomDark());});

////////////////////////// CONTROL

// Singleton non-volatile stats store instance.
OTV0P2BASE::EEPROMByHourByteStats eeStats;

// Stats updater singleton.
StatsU_t statsU;

// Singleton scheduler instance.
Scheduler_t Scheduler;

// Radiator valve mode (FROST, WARM, BAKE).
OTRadValve::ValveMode valveMode;

// Temperature control object.
TempControl_t tempControl;

// Manage EEPROM access for hub mode and boiler control.
OTRadValve::OTHubManager<enableDefaultAlwaysRX, enableRadioRX, allowGetMinBoilerOnMFromEEPROM> hubManager;

// Singleton implementation for entire node.
OccupancyTracker Occupancy;
/////// RADIOS
// To allow BoilerHub::remoteCallForHeatRX access to minuteCount in Control.cpp
extern uint8_t minuteCount; // XXX

// Brings in necessary radio libs.
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTV0P2BASE::fnmax(uint8_t(2), uint8_t(OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY)) - 1;
static constexpr int8_t RFM23B_IRQ_PIN = PIN_RFM_NIRQ;
static constexpr bool RFM23B_allowRX = false;
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> RFM23B;

// Assigns radio to PrimaryRadio alias
OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B;

// RFM22 is apparently SPI mode 0 for Arduino library pov.

// When RX not enabled, switch in dummy version (base class implements stubs)
OTRadioLink::OTMessageQueueHandlerNull messageQueue;

//========================================
// STUFF
//========================================
static OTV0P2BASE::EEPROMByHourByteStats ebhs;
// Create setback lockout if needed.
typedef bool(*setbackLockout_t)();
// If allowing setback lockout, eg for testing, then inject suitable lambda.
static bool setbackLockout() { return(0 != OTRadValve::getSetbackLockout()); }
// Algorithm for computing target temperature.
constexpr OTRadValve::ModelledRadValveComputeTargetTempBasic<
  PARAMS,
  &valveMode,
  decltype(TemperatureC16),   &TemperatureC16,
  decltype(tempControl),      &tempControl,
  decltype(Occupancy),        &Occupancy,
  decltype(AmbLight),         &AmbLight,
  decltype(valveUI),          &valveUI,
  decltype(Scheduler),        &Scheduler,
  decltype(ebhs),             &ebhs,
  decltype(RelHumidity),      &RelHumidity,
  setbackLockout>
  cttBasic;
// Internal model of controlled radiator valve position.
OTRadValve::ModelledRadValve NominalRadValve(
  &cttBasic,
  &valveMode,
  &tempControl,
  &ValveDirect,
    false,
    100);

// Valve physical UI controller.
valveUI_t valveUI(
  &valveMode,
  &tempControl,
  &NominalRadValve,
  &Occupancy,
  &AmbLight,
  &TempPot,
  &Supply_cV,
  OTV0P2BASE::LED_HEATCALL_ON,
  OTV0P2BASE::LED_HEATCALL_OFF,
  OTV0P2BASE::LED_HEATCALL_ON_ISR_SAFE);

//========================================
// FUNCTIONS
//========================================

// CLI

// Dump some brief CLI usage instructions to serial TX, which must be up and running.
// If this gets too big there is a risk of overrunning and missing the next tick...
static void dumpCLIUsage(const uint8_t stopBy)
{
    OTV0P2BASE::CLI::InvalidIgnored(); // Minimal placeholder.
    Serial.println();
}

// Used to poll user side for CLI input until specified sub-cycle time.
// Commands should be sent terminated by CR *or* LF; both may prevent 'E' (exit) from working properly.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// Times itself out after at least a minute or two of inactivity.
// NOT RENTRANT (eg uses static state for speed and code space).
void pollCLI(const uint8_t maxSCT, const bool startOfMinute, const OTV0P2BASE::ScratchSpace &s)
{
    // Perform any once-per-minute operations.
    if(startOfMinute) { OTV0P2BASE::CLI::countDownCLI(); }
    const bool neededWaking = OTV0P2BASE::powerUpSerialIfDisabled<V0P2_UART_BAUD>();
    // Wait for input command line from the user (received characters may already have been queued)...
    // Read a line up to a terminating CR, either on its own or as part of CRLF.
    // (Note that command content and timing may be useful to fold into PRNG entropy pool.)
    // A static buffer generates better code but permanently consumes previous SRAM.
    const uint8_t n = OTV0P2BASE::CLI::promptAndReadCommandLine(maxSCT, s, [](){pollIO();});
    char *buf = (char *)s.buf;
    if(n > 0) {
        // Got plausible input so keep the CLI awake a little longer.
        OTV0P2BASE::CLI::resetCLIActiveTimer();
        // Process the input received, with action based on the first char...
        switch(buf[0]) {
            // Explicit request for help, or unrecognised first character.
            // Avoid showing status as may already be rather a lot of output.
            default: case '?': { dumpCLIUsage(maxSCT); break; }
            // Exit/deactivate CLI immediately.
            // This should be followed by JUST CR ('\r') OR LF ('\n')
            // else the second will wake the CLI up again.
            case 'E': { OTV0P2BASE::CLI::makeCLIInactive(); break; }
            // Show/set generic parameter values (eg "G N [M]").
            case 'G': { OTV0P2BASE::CLI::GenericParam().doCommand(buf, n); break; }
            // Reset or display ID.
            case 'I': { OTV0P2BASE::CLI::NodeIDWithSet().doCommand(buf, n); break; }
            // Status line stats print and TX.
            case 'S': {
                Serial.print(F("Resets: "));
                const uint8_t resetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
                Serial.print(resetCount);
                Serial.println();
                // Show stack headroom.
                OTV0P2BASE::serialPrintAndFlush(F("SH ")); OTV0P2BASE::serialPrintAndFlush(OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd()); OTV0P2BASE::serialPrintlnAndFlush();
                // Default light-weight print and TX of stats.
                bareStatsTX();
                break; // Note that status is by default printed after processing input line.
            }
            // Switch to FROST mode OR set FROST/setback temperature (even with temp pot available).
            // With F! force to frost and holiday (long-vacant) mode.  Useful for testing and for remote CLI use.
            case 'F': {
                valveMode.setWarmModeDebounced(false); // No parameter supplied; switch to FROST mode.
                break;
            }
            /**
            * Set secret key.
            * @note  The OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond
            *        function pointer MUST be passed here to ensure safe handling of the key and the Tx message
            *        counter.
            */
            case 'K': { OTV0P2BASE::CLI::SetSecretKey(OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond).doCommand(buf, n); break; }
            // Switch to WARM (not BAKE) mode OR set WARM temperature.
            case 'W':{
                valveMode.cancelBakeDebounced(); // Ensure BAKE mode not entered.
                valveMode.setWarmModeDebounced(true); // No parameter supplied; switch to WARM mode.
                break;
            }
            // Zap/erase learned statistics.
            case 'Z': { OTV0P2BASE::CLI::ZapStats().doCommand(buf, n); break; }
        }
        // Else show ack of command received.
        Serial.println(F("OK"));
    } else { Serial.println(); } // Terminate empty/partial CLI input line after timeout.
    // Force any pending output before return / possible UART power-down.
    OTV0P2BASE::flushSerialSCTSensitive();
    if(neededWaking) { OTV0P2BASE::powerDownSerial(); }
}



//========================================
// SETUP
//========================================

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


  // Restore previous RTC state if available.
  OTV0P2BASE::restoreRTC();

  OTV0P2BASE::serialPrintAndFlush(F("\r\nOpenTRV: ")); // Leading CRLF to clear leading junk, eg from bootloader.
  V0p2Base_serialPrintlnBuildVersion();

  // Count resets to detect unexpected crashes/restarts.
  const uint8_t oldResetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
  eeprom_write_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT, 1 + oldResetCount);

#if defined(DEBUG) && !defined(ENABLE_MIN_ENERGY_BOOT)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("DEBUG");
#endif

#if defined(DEBUG) && !defined(ENABLE_MIN_ENERGY_BOOT)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Resets: ");
  DEBUG_SERIAL_PRINT(oldResetCount);
  DEBUG_SERIAL_PRINTLN();
  const uint8_t overruns = (~eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_OVERRUN_COUNTER)) & 0xff;
  if(0 != overruns)
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("Overruns: ");
    DEBUG_SERIAL_PRINT(overruns);
    DEBUG_SERIAL_PRINTLN();
    }
#if 0 && defined(DEBUG)
  // Compute approx free RAM: see http://jeelabs.org/2011/05/22/atmega-memory-use/
  DEBUG_SERIAL_PRINT_FLASHSTRING("Free RAM: ");
  extern int __heap_start, *__brkval;
  int x;
  DEBUG_SERIAL_PRINT((int) &x - (__brkval == 0 ? (int) &__heap_start : (int) __brkval));
  DEBUG_SERIAL_PRINTLN();
#endif
#endif

  optionalPOST();

  // Collect full set of environmental values before entering loop() in normal mode.
  // This should also help ensure that sensors are properly initialised.

  // No external sensors are *assumed* present if running alt main loop.
  // This may mean that the alt loop/POST will have to initialise them explicitly,
  // and the initial seed entropy may be marginally reduced also.
  const int heat = TemperatureC16.read();
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  DEBUG_SERIAL_PRINT_FLASHSTRING("T: ");
  DEBUG_SERIAL_PRINT(heat);
  DEBUG_SERIAL_PRINTLN();
#endif
  const int light = AmbLight.read();
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  DEBUG_SERIAL_PRINT_FLASHSTRING("L: ");
  DEBUG_SERIAL_PRINT(light);
  DEBUG_SERIAL_PRINTLN();
#endif
  const uint8_t rh = RelHumidity.read();
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  DEBUG_SERIAL_PRINT_FLASHSTRING("RH%: ");
  DEBUG_SERIAL_PRINT(rh);
  DEBUG_SERIAL_PRINTLN();
#endif
#if defined(TEMP_POT_AVAILABLE)
  const int tempPot = TempPot.read();
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  DEBUG_SERIAL_PRINT_FLASHSTRING("temp pot: ");
  DEBUG_SERIAL_PRINT(tempPot);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif

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
  OTV0P2BASE::seedPRNGs();

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Computing initial target/demand...");
#endif
  // Update targets, output to TRV and boiler, etc, to be sensible before main loop starts.
  NominalRadValve.read();

  // Ensure that the unique node ID is set up (mainly on first use).
  // Have one attempt (don't want to stress an already failing EEPROM) to force-reset if not good, then panic.
  // Needs to have had entropy gathered, etc.
  if(!OTV0P2BASE::ensureIDCreated())
    {
    if(!OTV0P2BASE::ensureIDCreated(true)) // Force reset.
      { panic(F("ID")); }
    }

  // Initialised: turn main/heatcall UI LED off.
  OTV0P2BASE::LED_HEATCALL_OFF();

  // Do OpenTRV-specific (late) setup.
  setupOpenTRV();
  }

//========================================
// MAIN LOOP
//========================================
#if 1
// More detailed stack usage output
inline void stackCheck()
{
	// Force restart if SPAM/heap/stack likely corrupt.
    OTV0P2BASE::MemoryChecks::forceResetIfStackOverflow();
    
    // Print max stack usage each cycle
    const int16_t minsp = OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd();
    const uint8_t location = OTV0P2BASE::MemoryChecks::getLocation();
    OTV0P2BASE::serialPrintAndFlush(F("minsp: "));
    OTV0P2BASE::serialPrintAndFlush(minsp, HEX);
    OTV0P2BASE::serialPrintAndFlush(F(" loc:"));
    OTV0P2BASE::serialPrintAndFlush(location);
    OTV0P2BASE::serialPrintlnAndFlush();

    OTV0P2BASE::MemoryChecks::resetMinSP();
}
#endif

void loop()
  {
#if defined(EST_CPU_DUTYCYCLE)
  const unsigned long usStart = micros();
#endif

#if 1
  // Force restart if SPAM/heap/stack likely corrupt.
  OTV0P2BASE::MemoryChecks::forceResetIfStackOverflow();

  // Complain and keep complaining when getting near stack overflow.
  // TODO: make DEBUG-only when confident all configs OK.
  const int16_t minsp = OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd();
  if(minsp < 64) { OTV0P2BASE::serialPrintlnAndFlush(F("!SH")); }
#else
  stackCheck();
#endif

  loopOpenTRV();

#if defined(EST_CPU_DUTYCYCLE)
  const unsigned long usEnd = micros();
  // Nominal loop time should be 2s x 1MHz clock, ie 2,000,000 if CPU running all the time.
  // Should generally be <2000 (us) (<0.1%) for leaf, <20000 (<1%) for hub.
  // Example output.
//us apparent: 4544
//us apparent: 25280
//us apparent: 9280
  const unsigned long usApparentTaken = usEnd - usStart;
#if 1 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("us apparent: ");
  DEBUG_SERIAL_PRINT(usApparentTaken);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif
  }
