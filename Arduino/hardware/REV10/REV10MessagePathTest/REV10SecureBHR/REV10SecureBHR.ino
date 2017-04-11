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

#include "REV10SecureBHR.h"


// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic()
  {
  // Reset radio and go into low-power mode.
  PrimaryRadio.panicShutdown();
  // Reset radio and go into low-power mode.
  SecondaryRadio.panicShutdown();
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
// Nodes talking on fast GFSK channel 0.
static constexpr uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] =
  {
  // GFSK channel 0 full config, RX/TX, not in itself secure.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true),
  };

static const OTRadioLink::OTRadioChannelConfig SecondaryRadioConfig(&SIM900Config, true);

void optionalPOST()
  {
  // Have 32678Hz clock at least running before going any further.
  // Check that the slow clock is running reasonably OK, and tune the fast one to it.
  if(!::OTV0P2BASE::HWTEST::calibrateInternalOscWithExtOsc()) { panic(F("Xtal")); } // Async clock not running or can't tune.

  // Signal that xtal is running AND give it time to settle.
  posPOST(0 /*, F("about to test radio module") */);

// FIXME  This section needs refactoring
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  PrimaryRadio.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!PrimaryRadio.configure(nPrimaryRadioChannels, RFM23BConfigs) || !PrimaryRadio.begin()) { panic(F("r1")); }

  // Turn power on for SIM900 with PFET for secondary power control.
  fastDigitalWrite(A3, 0);
  pinMode(A3, OUTPUT);
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  SecondaryRadio.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!SecondaryRadio.configure(1, &SecondaryRadioConfig) || !SecondaryRadio.begin()) { panic(F("r2")); }
  }


/////// SENSORS

// Sensor for supply (eg battery) voltage in millivolts.
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;

OTV0P2BASE::RoomTemperatureC16_TMP112 TemperatureC16;

////////////////////////// CONTROL

// Singleton non-volatile stats store instance.
OTV0P2BASE::EEPROMByHourByteStats eeStats;

// Stats updater singleton.
StatsU_t statsU;

// Radiator valve mode (FROST, WARM, BAKE).
OTRadValve::ValveMode valveMode;

// Temperature control object.
TempControl_t tempControl;


//========================================
// SETUP
//========================================

// Setup routine: runs once after reset.
// Does some limited board self-test and will panic() if anything is obviously broken.
void setup()
  {
  // Set appropriate low-power states, interrupts, etc, ASAP.
  OTV0P2BASE::powerSetup();

  // IO setup for safety, and to avoid pins floating.
  OTV0P2BASE::IOSetup();

  // Restore previous RTC state if available.
  OTV0P2BASE::restoreRTC();

  OTV0P2BASE::serialPrintAndFlush(F("\r\nOpenTRV: ")); // Leading CRLF to clear leading junk, eg from bootloader.
    V0p2Base_serialPrintlnBuildVersion();

  // Count resets to detect unexpected crashes/restarts.
  const uint8_t oldResetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
  eeprom_write_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT, 1 + oldResetCount);

  optionalPOST();

  // Collect full set of environmental values before entering loop() in normal mode.
  // This should also help ensure that sensors are properly initialised.

  // No external sensors are *assumed* present if running alt main loop.
  // This may mean that the alt loop/POST will have to initialise them explicitly,
  // and the initial seed entropy may be marginally reduced also.
  const int heat = TemperatureC16.read();


  const uint16_t Vcc = Supply_cV.read();

  OTV0P2BASE::seedPRNGs();

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

void loop()
  {

  // Force restart if SPAM/heap/stack likely corrupt.
  OTV0P2BASE::MemoryChecks::forceResetIfStackOverflow();

  // Complain and keep complaining when getting near stack overflow.
  // TODO: make DEBUG-only when confident all configs OK.
  const int16_t minsp = OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd();
  if(minsp < 64) { OTV0P2BASE::serialPrintlnAndFlush(F("!SH")); }

  loopOpenTRV();
  }
