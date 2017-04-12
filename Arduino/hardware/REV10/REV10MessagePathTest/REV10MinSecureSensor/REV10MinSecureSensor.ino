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
  Test of minimum relay code path.

  DHD20130417: hardware setup on bare board.
    * 1MHz CPU clock (from 8MHz internal RC clock with /8 prescaler) ATmega328P running at 1.8V--5V (typically 2V--3.3V).
    * Fuse set for BOD-managed additional clock settle time, ie as fast a restart from sleep as possible.
    * All unused pins unconnected and nominally floating (though driven low as output where possible).
    * 32768Hz xtal between pins XTAL1 and XTAL2, async timer 2, for accurate timekeeping and low-power sleep.
    * All unused system modules turned off.

  Basic AVR power consumption ticking an (empty) control loop at ~0.5Hz should be ~1uA.
 */

#include "REV10SecureBHR.h"
#include "ipAddress.h"  // IP adress in seperate header to avoid accidentally committing.

/////// RADIOS
//For EEPROM: TODO make a spec for how config should be stored in EEPROM to make changing them easy
//- Set the first field of SIM900LinkConfig to true.
//- The configs are stored as \0 terminated strings starting at 0x300.
//- You can program the eeprom using ./OTRadioLink/dev/utils/sim900eepromWrite.ino
//  static const void *SIM900_PIN      = (void *)0x0300;
//  static const void *SIM900_APN      = (void *)0x0305;
//  static const void *SIM900_UDP_ADDR = (void *)0x031B;
//  static const void *SIM900_UDP_PORT = (void *)0x0329;
//  const OTSIM900Link::OTSIM900LinkConfig_t SIM900Config(
//                                                  true,
//                                                  SIM900_PIN,
//                                                  SIM900_APN,
//                                                  SIM900_UDP_ADDR,
//                                                  SIM900_UDP_PORT);
//For Flash:
//- Set the first field of SIM900LinkConfig to false.
//- The configs are stored as \0 terminated strings.
//- Where multiple options are available, uncomment whichever you want
  static const char SIM900_PIN[5] PROGMEM       = "1111";

// APN Configs - Uncomment based on what SIM you are using
//  static const char SIM900_APN[] PROGMEM      = "\"everywhere\",\"eesecure\",\"secure\""; // EE
//static const char SIM900_APN[] PROGMEM      = "\"arkessa.net\",\"arkessa\",\"arkessa\""; // Arkessa
static const char SIM900_APN[] PROGMEM      = "\"mobiledata\""; // GeoSIM

// UDP Configs - Edit SIM900_UDP_ADDR for relevant server. NOTE: The server IP address should never be committed to GitHub.
// IP adress in seperate header to avoid accidentally committing.
//static const char SIM900_UDP_ADDR[16] PROGMEM = ""; // Of form "1.2.3.4". 
static const char SIM900_UDP_PORT[5] PROGMEM = "9999";             // Standard port for OpenTRV servers
const OTSIM900Link::OTSIM900LinkConfig_t SIM900Config(
                                                false,
                                                SIM900_PIN,
                                                SIM900_APN,
                                                SIM900_UDP_ADDR,
                                                SIM900_UDP_PORT);

// Pick an appropriate radio config for RFM23 (if it is the primary radio).
// Nodes talking on fast GFSK channel 0.
static constexpr uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] =
  {
  // GFSK channel 0 full config, RX/TX, not in itself secure.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true),
  };

static const OTRadioLink::OTRadioChannelConfig SecondaryRadioConfig(&SIM900Config, true);

// Brings in necessary radio libs.
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
static constexpr int8_t RFM23B_IRQ_PIN = PIN_RFM_NIRQ;
static constexpr bool RFM23B_allowRX = true;
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> RFM23B;
OTSIM900Link::OTSIM900Link<8, 5, RADIO_POWER_PIN, OTV0P2BASE::getSecondsLT> SIM900; // (REGULATOR_POWERUP, RADIO_POWER_PIN);

// Assigns radio to PrimaryRadio alias
OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B;

// Assign radio to SecondaryRadio alias.
OTRadioLink::OTRadioLink &SecondaryRadio = SIM900;

/////// SENSORS

// Sensor for supply (eg battery) voltage in millivolts.
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;

OTV0P2BASE::RoomTemperatureC16_TMP112 TemperatureC16;

// 
OTV0P2BASE::EEPROMByHourByteStats eeStats;
StatsU_t statsU;

//========================================
// LOCAL FUNCTIONS
//========================================
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


//========================================
// SETUP
//========================================

// Setup routine: runs once after reset.
// Does some limited board self-test and will panic() if anything is obviously broken.
static constexpr uint8_t PP_OFF_MS = 250;
void setup()
  {
  //----------- Low level setup
  // Set appropriate low-power states, interrupts, etc, ASAP.
  OTV0P2BASE::powerSetup();

  // IO setup for safety, and to avoid pins floating.
  OTV0P2BASE::IOSetup();
  
  OTV0P2BASE::serialPrintAndFlush(F("\r\nOpenTRV: ")); // Leading CRLF to clear leading junk, eg from bootloader.
    V0p2Base_serialPrintlnBuildVersion();
  
  // Count resets to detect unexpected crashes/restarts.
  const uint8_t oldResetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
  eeprom_write_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT, 1 + oldResetCount);
  
  // Have 32678Hz clock at least running before going any further.
  // Check that the slow clock is running reasonably OK, and tune the fast one to it.
  if(!::OTV0P2BASE::HWTEST::calibrateInternalOscWithExtOsc()) { panic(F("Xtal")); } // Async clock not running or can't tune.

  //----------- Flash UI
  // Signal that xtal is running AND give it time to settle.
  OTV0P2BASE::sleepLowPowerMs(1000);
  OTV0P2BASE::LED_HEATCALL_OFF();

  OTV0P2BASE::sleepLowPowerMs(PP_OFF_MS); // TODO: use this time to gather entropy.
  OTV0P2BASE::LED_HEATCALL_ON();
  OTV0P2BASE::sleepLowPowerMs(1000); // TODO: use this time to gather entropy.


  //----------- Init Radio
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  PrimaryRadio.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!PrimaryRadio.configure(nPrimaryRadioChannels, RFM23BConfigs) || !PrimaryRadio.begin()) { panic(F("r1")); }

  // Turn power on for SIM900 with PFET for secondary power control.
  fastDigitalWrite(A3, 0); // todo move into sim900link
  pinMode(A3, OUTPUT);
  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  SecondaryRadio.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!SecondaryRadio.configure(1, &SecondaryRadioConfig) || !SecondaryRadio.begin()) { panic(F("r2")); }

  //----------- Init sensors
  // Collect full set of environmental values before entering loop() in normal mode.
  // This should also help ensure that sensors are properly initialised.
  const int heat = TemperatureC16.read();
  const uint16_t Vcc = Supply_cV.read();

  OTV0P2BASE::seedPRNGs();

  //----------- Ensure has ID
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
