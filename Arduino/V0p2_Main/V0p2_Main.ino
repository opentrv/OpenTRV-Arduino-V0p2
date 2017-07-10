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

#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT) || defined(ENABLE_SECURE_RADIO_BEACON)
#include <OTAESGCM.h>
#endif

#include "V0p2_Main.h"

// Sat at the top to ease changing stuff when programming.
// TODO move this out into a config header that isn't committed to git..
#ifdef ENABLE_RADIO_SIM900
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
static const char SIM900_UDP_ADDR[16] PROGMEM = ""; // Of form "1.2.3.4".
static const char SIM900_UDP_PORT[5] PROGMEM = "9999";             // Standard port for OpenTRV servers
const OTSIM900Link::OTSIM900LinkConfig_t SIM900Config(
                                              false,
                                              SIM900_PIN,
                                              SIM900_APN,
                                              SIM900_UDP_ADDR,
                                              SIM900_UDP_PORT);
#endif // ENABLE_RADIO_SIM900

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
#ifndef V0P2BASE_LED_HEATCALL_IS_L
  pinMode(OTV0P2BASE::LED_HEATCALL, OUTPUT);
#else
  pinMode(OTV0P2BASE::LED_HEATCALL_L, OUTPUT);
#endif
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
#ifdef ENABLE_RADIO_PRIMARY_RFM23B
// OTRadioChannelConfig(const void *_config, bool _isFull, bool _isRX, bool _isTX, bool _isAuth = false, bool _isEnc = false, bool _isUnframed = false)
#if defined(ENABLE_FAST_FRAMED_CARRIER_SUPPORT)
#define RADIO_CONFIG_NAME "GFSK"
// Nodes talking on fast GFSK channel 0.
static constexpr uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] =
  {
  // GFSK channel 0 full config, RX/TX, not in itself secure.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true),
  };
#else // !defined(ENABLE_FAST_FRAMED_CARRIER_SUPPORT)
#define RADIO_CONFIG_NAME "OOK"
// Nodes talking (including to to FHT8V) on slow OOK.
static constexpr uint8_t nPrimaryRadioChannels = 1;
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

// If in stats or boiler hub mode, and with an FS20 OOK carrier, then apply a trailing-zeros RX filter.
#if (defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)) && defined(ENABLE_FHT8VSIMPLE_RX)
#define CONFIG_TRAILING_ZEROS_FILTER_RX
#endif

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

void optionalPOST()
  {
  // Have 32678Hz clock at least running before going any further.
#if defined(ENABLE_WAKEUP_32768HZ_XTAL)
#ifdef ENABLE_TUNE_FAST_OSC_TO_RTC_SOURCE
  // Check that the slow clock is running reasonably OK, and tune the fast one to it.
  if(!::OTV0P2BASE::HWTEST::calibrateInternalOscWithExtOsc()) { panic(F("Xtal")); } // Async clock not running or can't tune.
//    if(!::OTV0P2BASE::HWTEST::check32768HzOsc()) { panic(F("xtal")); } // Async clock not running correctly.
#else
  // Just check that the slow clock is running reasonably OK.
  if(!::OTV0P2BASE::HWTEST::check32768HzOsc()) { panic(F("xtal")); } // Async clock not running correctly.
#endif // ENABLE_TUNE_FAST_OSC_TO_RTC_SOURCE

#else
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("(No xtal.)");
#endif // defined(ENABLE_WAKEUP_32768HZ_XTAL)

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

/////// SENSORS

// Sensor for supply (eg battery) voltage in millivolts.
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;

#ifdef TEMP_POT_AVAILABLE
TempPot_t TempPot;
#endif

#ifdef ENABLE_AMBLIGHT_SENSOR
AmbientLight AmbLight;
#endif // ENABLE_AMBLIGHT_SENSOR

#if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
OTV0P2BASE::MinimalOneWire<> MinOW_DEFAULT;
#endif

#if defined(SENSOR_EXTERNAL_DS18B20_ENABLE_0) // Enable sensor zero.
OTV0P2BASE::TemperatureC16_DS18B20 extDS18B20_0(MinOW_DEFAULT, 0);
#endif

#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
// Singleton implementation/instance.
OTV0P2BASE::HumiditySensorSHT21 RelHumidity;
#else
OTV0P2BASE::DummyHumiditySensorSHT21 RelHumidity;
#endif

// Ambient/room temperature sensor, usually on main board.
#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.
#elif defined(ENABLE_PRIMARY_TEMP_SENSOR_DS18B20)
#if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
// DSB18B20 temperature impl, with slightly reduced precision to improve speed.
OTV0P2BASE::TemperatureC16_DS18B20 TemperatureC16(MinOW_DEFAULT, OTV0P2BASE::TemperatureC16_DS18B20::MAX_PRECISION - 1);
#endif
#else // Don't use TMP112 if SHT21 or DS18B20 are selected.
OTV0P2BASE::RoomTemperatureC16_TMP112 TemperatureC16;
#endif

#ifdef ENABLE_VOICE_SENSOR
OTV0P2BASE::VoiceDetectionQM1 Voice;
#endif

////////////////////////// Actuators

// DORM1/REV7 direct drive actuator.
#ifdef HAS_DORM1_VALVE_DRIVE
// Singleton implementation/instance.
// Suppress unnecessary activity when room dark, eg to avoid disturbance if device crashes/restarts,
// unless recent UI use because value is being fitted/adjusted.
ValveDirect_t ValveDirect([](){return((!valveUI.veryRecentUIControlUse()) && AmbLight.isRoomDark());});
#endif

// FHT8V radio-controlled actuator.
#ifdef ENABLE_FHT8VSIMPLE
// Function to append stats trailer (and 0xff) to FHT8V/FS20 TX buffer.
// Assume enough space in buffer for largest possible stats message.
#if defined(ENABLE_STATS_TX)
uint8_t *appendStatsToTXBufferWithFF(uint8_t *bptr, const uint8_t bufSize)
{
  OTV0P2BASE::FullStatsMessageCore_t trailer;
  OTRadValve::populateCoreStats(&trailer,
                    (localFHT8VTRVEnabled() ? &FHT8V : NULL),
                    TemperatureC16.get(),
                    Supply_cV.isSupplyVoltageLow(),
                    AmbLight.get(),
                    Occupancy.twoBitOccupancyValue());
  // Ensure that no ID is encoded in the message sent on the air since it would be a repeat from the FHT8V frame.
  trailer.containsID = false;

#if defined(ENABLE_MINIMAL_STATS_TXRX)
  // As bandwidth optimisation just write minimal trailer if only temp&power available.
  if (trailer.containsTempAndPower &&
      !trailer.containsID && !trailer.containsAmbL)
  {
    writeTrailingMinimalStatsPayload(bptr, &(trailer.tempAndPower));
    bptr += 3;
    *bptr = (uint8_t)0xff; // Terminate TX bytes.
  }
  else
#endif
  {
    // Assume enough space in buffer for largest possible stats message.
    bptr = OTV0P2BASE::encodeFullStatsMessageCore(bptr, bufSize, OTV0P2BASE::getStatsTXLevel(), false, &trailer);
  }
  return (bptr);
}
#else
#define appendStatsToTXBufferWithFF NULL // Do not append stats.
#endif
#endif // ENABLE_FHT8VSIMPLE

#ifdef ENABLE_FHT8VSIMPLE
OTRadValve::FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V(appendStatsToTXBufferWithFF);
#endif // ENABLE_FHT8VSIMPLE

#ifdef ENABLE_BOILER_HUB
OTRadValve::BoilerLogic::OnOffBoilerDriverLogic<decltype(hubManager), hubManager, OUT_HEATCALL> BoilerHub;
#endif // ENABLE_BOILER_HUB

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

#ifdef ENABLE_OCCUPANCY_SUPPORT
// Singleton implementation for entire node.
OccupancyTracker Occupancy;
#endif

// Mechanism to generate '=' stats line, if enabled.
#if defined(ENABLE_SERIAL_STATUS_REPORT)
StatsLine_t statsLine;
#endif // defined(ENABLE_SERIAL_STATUS_REPORT)
/////// RADIOS
// To allow BoilerHub::remoteCallForHeatRX access to minuteCount in Control.cpp
extern uint8_t minuteCount; // XXX

#if defined(ENABLE_RADIO_NULL)
OTRadioLink::OTNullRadioLink NullRadio;
#endif

// Brings in necessary radio libs.
#ifdef ENABLE_RADIO_RFM23B
#if defined(ENABLE_TRIMMED_MEMORY) && !defined(ENABLE_DEFAULT_ALWAYS_RX) && !defined(ENABLE_CONTINUOUS_RX)
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTV0P2BASE::fnmax(uint8_t(2), uint8_t(OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY)) - 1;
#else
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
#endif
#if defined(PIN_RFM_NIRQ)
static constexpr int8_t RFM23B_IRQ_PIN = PIN_RFM_NIRQ;
#else
static constexpr int8_t RFM23B_IRQ_PIN = -1;
#endif
#if defined(ENABLE_RADIO_RX)
static constexpr bool RFM23B_allowRX = true;
#else
static constexpr bool RFM23B_allowRX = false;
#endif
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> RFM23B;
#endif // ENABLE_RADIO_RFM23B
#ifdef ENABLE_RADIO_SIM900
OTSIM900Link::OTSIM900Link<8, 5, RADIO_POWER_PIN, OTV0P2BASE::getSecondsLT> SIM900; // (REGULATOR_POWERUP, RADIO_POWER_PIN);
#endif
#ifdef ENABLE_RADIO_RN2483
OTRN2483Link::OTRN2483Link RN2483(RADIO_POWER_PIN, SOFTSERIAL_RX_PIN, SOFTSERIAL_TX_PIN);
#endif // ENABLE_RADIO_RN2483

// Assigns radio to PrimaryRadio alias
#if defined(ENABLE_RADIO_PRIMARY_RFM23B)
OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B;
#elif defined(RADIO_PRIMARY_SIM900)
OTRadioLink::OTRadioLink &PrimaryRadio = SIM900;
#else
OTRadioLink::OTRadioLink &PrimaryRadio = NullRadio;
#endif // ENABLE_RADIO_PRIMARY_RFM23B

// Assign radio to SecondaryRadio alias.
#ifdef ENABLE_RADIO_SECONDARY_MODULE
#if defined(RADIO_SECONDARY_RFM23B)
OTRadioLink::OTRadioLink &SecondaryRadio = RFM23B;
#elif defined(ENABLE_RADIO_SECONDARY_SIM900)
OTRadioLink::OTRadioLink &SecondaryRadio = SIM900;
#elif defined(ENABLE_RADIO_SECONDARY_RN2483)
OTRadioLink::OTRadioLink &SecondaryRadio = RN2483;
#else
OTRadioLink::OTRadioLink &SecondaryRadio = NullRadio;
#endif // RADIO_SECONDARY_RFM23B
#endif // ENABLE_RADIO_SECONDARY_MODULE

// RFM22 is apparently SPI mode 0 for Arduino library pov.


// Setup frame RX handlers
#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT) && defined(ENABLE_RADIO_RX)
// Define queue handler
// Currently 4 possible cases for RXing secure frames:
// - Both relay and boiler hub present (e.g. CONFIG_REV10_AS_BHR)
// - Just relay present (e.g. CONFIG_REV10_AS_GSM_RELAY_ONLY)
// - Just boiler hub (e.g. CONFIG_REV8_SECURE_BHR)
// - Unit acting as stats-hub (e.g. CONFIG_REV11_SECURE_STATSHUB)
#if defined(ENABLE_RADIO_SECONDARY_SIM900) && defined(ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY) && defined(ENABLE_BOILER_HUB)
// relay + bh
inline bool decodeAndHandleSecureFrame(volatile const uint8_t * const msg)
{
  return OTRadioLink::decodeAndHandleOTSecureOFrame<OTRadioLink::SimpleSecureFrame32or0BodyRXV0p2,
                                                    OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleDec_DEFAULT_STATELESS,
                                                   OTV0P2BASE::getPrimaryBuilding16ByteSecretKey,
                                                   OTRadioLink::relayFrameOperation<decltype(SIM900), SIM900>,
                                                   OTRadioLink::boilerFrameOperation<decltype(BoilerHub), BoilerHub, minuteCount>
                                                  >(msg);
}
#elif defined(ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY)
// relay
inline bool decodeAndHandleSecureFrame(volatile const uint8_t * const msg)
{
  return OTRadioLink::decodeAndHandleOTSecureOFrame<OTRadioLink::SimpleSecureFrame32or0BodyRXV0p2,
                                                    OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleDec_DEFAULT_STATELESS,
                                                   OTV0P2BASE::getPrimaryBuilding16ByteSecretKey,
                                                   OTRadioLink::relayFrameOperation<decltype(SIM900), SIM900>
                                                  >(msg);
}
#elif defined(ENABLE_BOILER_HUB)
// bh
inline bool decodeAndHandleSecureFrame(volatile const uint8_t * const msg)
{
  return OTRadioLink::decodeAndHandleOTSecureOFrame<OTRadioLink::SimpleSecureFrame32or0BodyRXV0p2,
                                                    OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleDec_DEFAULT_STATELESS,
                                                   OTV0P2BASE::getPrimaryBuilding16ByteSecretKey,
                                                   OTRadioLink::boilerFrameOperation<decltype(BoilerHub), BoilerHub, minuteCount>
                                                  >(msg);
}
#else
// serial
inline bool decodeAndHandleSecureFrame(volatile const uint8_t * const msg)
{
  return OTRadioLink::decodeAndHandleOTSecureOFrame<OTRadioLink::SimpleSecureFrame32or0BodyRXV0p2,
                                                    OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleDec_DEFAULT_STATELESS,
                                                   OTV0P2BASE::getPrimaryBuilding16ByteSecretKey,
                                                   OTRadioLink::serialFrameOperation<decltype(Serial), Serial>
                                                  >(msg);
}
#endif // defined(ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY) && (ENABLE_BOILER_HUB)
OTRadioLink::OTMessageQueueHandler< pollIO, V0P2_UART_BAUD,
                                    decodeAndHandleSecureFrame, OTRadioLink::decodeAndHandleDummyFrame
                                   > actualMessageQueue;  //TODO change baud
OTRadioLink::OTMessageQueueHandlerBase &messageQueue = actualMessageQueue;
#else // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
// When RX not enabled, switch in dummy version (base class implements stubs)
OTRadioLink::OTMessageQueueHandlerNull messageQueue;
#endif  // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT




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

//#if defined(ENABLE_MIN_ENERGY_BOOT)
//  nap(WDTO_120MS); // Sleep to let power supply recover a little.
//#endif

#if !defined(ENABLE_MIN_ENERGY_BOOT)
  // Restore previous RTC state if available.
  OTV0P2BASE::restoreRTC();
#endif

#if !defined(ENABLE_MIN_ENERGY_BOOT)
#if defined(LED_UI2_EXISTS) && defined(ENABLE_UI_LED_2_IF_AVAILABLE)
  LED_UI2_ON();
#endif
  OTV0P2BASE::serialPrintAndFlush(F("\r\nOpenTRV: ")); // Leading CRLF to clear leading junk, eg from bootloader.
    V0p2Base_serialPrintlnBuildVersion();
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

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Computing initial target/demand...");
#endif
#if defined(ENABLE_NOMINAL_RAD_VALVE)
  // Update targets, output to TRV and boiler, etc, to be sensible before main loop starts.
  NominalRadValve.read();
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
  OTV0P2BASE::LED_HEATCALL_OFF();

#if defined(ENABLE_CLI) && defined(ENABLE_CLI_HELP) && !defined(ENABLE_TRIMMED_MEMORY)
  // Help user get to CLI.
  OTV0P2BASE::serialPrintlnAndFlush(F("At CLI > prompt enter ? for help"));
#endif

#if !defined(ENABLE_TRIMMED_MEMORY)
  // Report initial status.
  serialStatusReport();
#endif
  // Do OpenTRV-specific (late) setup.
  setupOpenTRV();
  }

//========================================
// MAIN LOOP
//========================================

void loop()
  {
#if defined(EST_CPU_DUTYCYCLE)
  const unsigned long usStart = micros();
#endif

  // Force restart if SPAM/heap/stack likely corrupt.
  OTV0P2BASE::MemoryChecks::forceResetIfStackOverflow();

  // Complain and keep complaining when getting near stack overflow.
  // TODO: make DEBUG-only when confident all configs OK.
  const int16_t minsp = OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd();
  if(minsp < 64) { OTV0P2BASE::serialPrintlnAndFlush(F("!SH")); }

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
