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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2017
                           Gary Gladman 2015
                           Mike Stirling 2013
*/

/*
 Generic messaging and radio/comms support for OpenTRV.
 */
#include "V0p2_Main.h"
#include "OTRadioLink.h"

// To allow BoilerHub::remoteCallForHeatRX access to minuteCount in Control.cpp
extern uint8_t minuteCount; // XXX

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

// Define frame handlers:
//OTRadioLink::OTSerialHandler<decltype(Serial), Serial> serialHandler;
#ifdef ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
OTRadioLink::OTRadioHandler<decltype(SIM900), SIM900> radioHandler;
//OTRadioLink::OTRadioHandler<decltype(NullRadio), NullRadio> radioHandler;
#else
OTRadioLink::OTSerialHandler<decltype(Serial), Serial> serialHandler;
#endif //ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
#ifdef ENABLE_BOILER_HUB
OTRadioLink::OTBoilerHandler<decltype(BoilerHub), BoilerHub, minuteCount> boilerHandler;
#endif // ENABLE_BOILER_HUB

#ifdef ENABLE_RADIO_RX
constexpr uint8_t allowInsecureRX = false;

#if defined(ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY) && defined(ENABLE_BOILER_HUB)
OTRadioLink::OTMessageQueueHandler2<decltype(radioHandler), radioHandler, 'O',
                                   decltype(boilerHandler), boilerHandler, 'O',
                                   pollIO, V0P2_UART_BAUD,
                                   allowInsecureRX> actualMessageQueue;
#elif defined(ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY)
OTRadioLink::OTMessageQueueHandler<decltype(radioHandler), radioHandler, 'O',
                                   pollIO, V0P2_UART_BAUD,
                                   allowInsecureRX> actualMessageQueue;
#elif defined(ENABLE_BOILER_HUB)
OTRadioLink::OTMessageQueueHandler<decltype(boilerHandler), boilerHandler, 'O',
                                   pollIO, V0P2_UART_BAUD,
                                   allowInsecureRX> actualMessageQueue;
#else
OTRadioLink::OTMessageQueueHandler<decltype(serialHandler), serialHandler, 'O',
                                   pollIO, V0P2_UART_BAUD,
                                   allowInsecureRX> actualMessageQueue;
#endif // defined(ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY) && (ENABLE_BOILER_HUB)
OTRadioLink::OTMessageQueueHandlerBase &messageQueue = actualMessageQueue;

#else // ENABLE_RADIO_RX
OTRadioLink::OTMessageQueueHandlerBase messageQueue;
#endif  // ENABLE_RADIO_RX






