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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2016
                           Gary Gladman 2015
                           Mike Stirling 2013
*/

/*
 Generic messaging and radio/comms support for OpenTRV.
 */

#include "Messaging.h"

#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

#include <OTRadioLink.h>
#ifdef ALLOW_CC1_SUPPORT
#include <OTProtocolCC.h>
#endif

#include <util/atomic.h>

#include "Control.h"
#include "UI_Minimal.h"

#include "V0p2_Sensors.h"


#ifdef ENABLE_RADIO_SIM900
//For EEPROM:
//- Set the first field of SIM900LinkConfig to true.
//- The configs are stored as \0 terminated strings starting at 0x300.
//- You can program the eeprom using ./OTRadioLink/dev/utils/sim900eepromWrite.ino
//  static const void *SIM900_PIN      = (void *)0x0300; // TODO confirm this address
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
// - APNs - concirrus:  "internet.cxn"
//        - id:         "id"
  static const char SIM900_PIN[5] PROGMEM       = "1111";
  static const char SIM900_APN[] PROGMEM      = "\"everywhere\",\"eesecure\",\"secure\"";
  static const char SIM900_UDP_ADDR[14] PROGMEM = "46.101.52.242"; // ORS server
  static const char SIM900_UDP_PORT[5] PROGMEM = "9999";
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

// Brings in necessary radio libs
#ifdef ENABLE_RADIO_RFM23B
#ifdef PIN_RFM_NIRQ
OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS, PIN_RFM_NIRQ> RFM23B;
#else
OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS, -1> RFM23B;
#endif
#endif // ENABLE_RADIO_RFM23B
#ifdef ENABLE_RADIO_SIM900
OTSIM900Link::OTSIM900Link SIM900(A3, A2, 8, 5);
#endif
#ifdef ENABLE_RADIO_RN2483
OTRN2483Link::OTRN2483Link RN2483;
#endif // ENABLE_RADIO_RN2483

// Assigns radio to PrimaryRadio alias
#if defined(RADIO_PRIMARY_RFM23B)
OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B;
#elif defined(RADIO_PRIMARY_SIM900)
OTRadioLink::OTRadioLink &PrimaryRadio = SIM900;
#else
OTRadioLink::OTRadioLink &PrimaryRadio = NullRadio;
#endif // RADIO_PRIMARY_RFM23B

// Assign radio to SecondaryRadio alias.
#ifdef ENABLE_RADIO_SECONDARY_MODULE
#if defined(RADIO_SECONDARY_RFM23B)
OTRadioLink::OTRadioLink &SecondaryRadio = RFM23B;
#elif defined(RADIO_SECONDARY_SIM900)
OTRadioLink::OTRadioLink &SecondaryRadio = SIM900;
#elif defined(RADIO_SECONDARY_RN2483)
OTRadioLink::OTRadioLink &SecondaryRadio = RN2483;
#else
OTRadioLink::OTRadioLink &SecondaryRadio = NullRadio;
#endif // RADIO_SECONDARY_RFM23B
#endif // ENABLE_RADIO_SECONDARY_MODULE

// RFM22 is apparently SPI mode 0 for Arduino library pov.

// Send the underlying stats binary/text 'whitened' message.
// This must be terminated with an 0xff (which is not sent),
// and no longer than STATS_MSG_MAX_LEN bytes long in total (excluding the terminating 0xff).
// This must not contain any 0xff and should not contain long runs of 0x00 bytes.
// The message to be sent must be written at an offset of STATS_MSG_START_OFFSET from the start of the buffer.
// This routine will alter the content of the buffer for transmission,
// and the buffer should not be re-used as is.
//   * doubleTX  double TX to increase chance of successful reception
//   * RFM23BfriendlyPremable  if true then add an extra preamble
//     to allow RFM23B-based receiver to RX this
// This will use whichever transmission medium/carrier/etc is available.
//#define STATS_MSG_START_OFFSET (RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES)
//#define STATS_MSG_MAX_LEN (64 - STATS_MSG_START_OFFSET)
void RFM22RawStatsTXFFTerminated(uint8_t * const buf, const bool doubleTX, bool RFM23BFramed)
  {
  if(RFM23BFramed) RFM22RXPreambleAdd(buf);     // Only needed for RFM23B. This should be made more clear when refactoring
  const uint8_t buflen = OTRadioLink::frameLenFFTerminated(buf);
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("buflen=");
    DEBUG_SERIAL_PRINT(buflen);
    DEBUG_SERIAL_PRINTLN();
#endif // DEBUG
  if(!PrimaryRadio.queueToSend(buf, buflen, 0, (doubleTX ? OTRadioLink::OTRadioLink::TXmax : OTRadioLink::OTRadioLink::TXnormal)))
    {
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("!TX failed");
#endif
    } // DEBUG
  //DEBUG_SERIAL_PRINTLN_FLASHSTRING("RS");
  }


#ifdef ALLOW_CC1_SUPPORT_RELAY
#include <OTProtocolCC.h>
#include <OTRadValve.h>
#include "V0p2_Sensors.h"
// Send a CC1 Alert message with this unit's house code via the RFM23B.
bool sendCC1AlertByRFM23B()
  {
  OTProtocolCC::CC1Alert a = OTProtocolCC::CC1Alert::make(FHT8VGetHC1(), FHT8VGetHC2());
  if(a.isValid()) // Might be invalid if house codes are, eg if house codes not set.
    {
    uint8_t txbuf[STATS_MSG_START_OFFSET + OTProtocolCC::CC1Alert::primary_frame_bytes+1]; // More than large enough for preamble + sync + alert message.
    uint8_t *const bptr = RFM22RXPreambleAdd(txbuf);
    const uint8_t bodylen = a.encodeSimple(bptr, sizeof(txbuf) - STATS_MSG_START_OFFSET, true);
    const uint8_t buflen = STATS_MSG_START_OFFSET + bodylen;
#if 0 && defined(DEBUG)
OTRadioLink::printRXMsg(p, txbuf, buflen);
#endif
    // Send loud since the hub may be relatively far away,
    // there is no 'ACK', and these messages should not be sent very often.
    // Should be consistent with automatically-generated alerts to help with diagnosis.
    return(PrimaryRadio.sendRaw(txbuf, buflen, 0, OTRadioLink::OTRadioLink::TXmax));
    }
  return(false); // Failed.
  }
#endif


#if defined(LISTEN_FOR_FTp2_FS20_native) // defined(ENABLE_RADIO_RX) && (defined(ENABLE_BOILER_HUB) || defined(ALLOW_STATS_RX)) && defined(ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX) // Listen for calls for heat from remote valves...
static void decodeAndHandleFTp2_FS20_native(Print *p, const bool secure, const uint8_t * const msg, const uint8_t msglen)
{
#if 0 && defined(DEBUG)
  OTRadioLink::printRXMsg(p, msg, msglen);
#endif

  // Decode the FS20/FHT8V command into the buffer/struct.
  OTRadValve::FHT8VRadValveBase::fht8v_msg_t command;
  uint8_t const *lastByte = msg+msglen-1;
  uint8_t const *trailer = OTRadValve::FHT8VRadValveBase::FHT8VDecodeBitStream(msg, lastByte, &command);

#if defined(ENABLE_BOILER_HUB)
  // Potentially accept as call for heat only if command is 0x26 (38).
  // Later filter on the valve being open enough for some water flow to be likely
  // (for individual valves, and in aggregate)
  // and for the housecode being accepted.
  if(0x26 == command.command)
    {
    const uint16_t compoundHC = (((uint16_t)command.hc1) << 8) | command.hc2;
#if 0 && defined(DEBUG)
    p->print("FS20 RX 0x26 "); // Just notes that a 'valve %' FS20 command has been overheard.
    p->print(command.hc1); p->print(' ');
    p->println(command.hc2);
#endif
    const uint8_t percentOpen = OTRadValve::FHT8VRadValveBase::convert255ScaleToPercent(command.extension);
    remoteCallForHeatRX(compoundHC, percentOpen);
    }
#endif

  if(NULL != trailer)
    {
#if 0 && defined(DEBUG)
p->print("FS20 msg HC "); p->print(command.hc1); p->print(' '); p->println(command.hc2);
#endif
#if defined(ALLOW_STATS_RX) // Only look for the trailer if supported.
    // If whole FHT8V frame was OK then check if there is a valid stats trailer.

    // Check for 'core' stats trailer.
    if(OTV0P2BASE::MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS == (trailer[0] & OTV0P2BASE::MESSAGING_FULL_STATS_FLAGS_HEADER_MASK))
      {
      OTV0P2BASE::FullStatsMessageCore_t content;
      const uint8_t *tail = decodeFullStatsMessageCore(trailer, lastByte-trailer+1, OTV0P2BASE::stTXalwaysAll, false, &content);
      if(NULL != tail)
        {
        // Received trailing stats frame!

        // If ID is present then make sure it matches that implied by the FHT8V frame (else reject this trailer)
        // else file it in from the FHT8C frame.
        bool allGood = true;
        if(content.containsID)
          {
          if((content.id0 != command.hc1) || (content.id1 != command.hc2))
            { allGood = false; }
          }
        else
          {
          content.id0 = command.hc1;
          content.id1 = command.hc2;
          content.containsID = true;
          }

#if 0 && defined(DEBUG)
/*if(allGood)*/ { p->println("FS20 ts"); }
#endif
        // If frame looks good then capture it.
        if(allGood) { outputCoreStats(p, false, &content); }
//            else { setLastRXErr(FHT8VRXErr_BAD_RX_SUBFRAME); }
        // TODO: record error with mismatched ID.
        }
      }
#if defined(ENABLE_MINIMAL_STATS_TXRX)
    // Check for minimal stats trailer.
    else if((trailer + MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES <= lastByte) && // Enough space for minimum-stats trailer.
       (MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS == (trailer[0] & MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MASK)))
      {
      if(verifyHeaderAndCRCForTrailingMinimalStatsPayload(trailer)) // Valid header and CRC.
        {
#if 0 && defined(DEBUG)
        p->println("FS20 tsm"); // Just notes that a 'valve %' FS20 command has been overheard.
#endif
        trailingMinimalStatsPayload_t payload;
        extractTrailingMinimalStatsPayload(trailer, &payload);
        }
      }
#endif
#endif
    }
  return;
  }
#endif


#ifdef ENABLE_RADIO_RX
// Decode and handle inbound raw message.
// A message may contain trailing garbage at the end; the decoder/router should cope.
// The buffer may be reused when this returns,
// so a copy should be taken of anything that needs to be retained.
// If secure is true then this message arrived over a secure channel.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This routine is NOT allowed to alter the contents of the buffer passed.
static void decodeAndHandleRawRXedMessage(Print *p, const bool secure, const uint8_t * const msg, const uint8_t msglen)
  {
  // TODO: consider extracting hash of all message data (good/bad) and injecting into entropy pool.
#if 0 && defined(DEBUG)
  OTRadioLink::printRXMsg(p, msg, msglen);
#endif
  if(msglen < 2) { return; } // Too short to be useful, so ignore.

  const uint8_t firstByte = msg[0];

   // Length-first OpenTRV secureable-frame format...
#ifdef ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
  // Don't try to parse any apparently-truncated message.
  // (It might be in a different format for example.)
  if(firstByte <= msglen)
    {  
    switch(msg[1]) // Switch on type.
      {
#ifdef ENABLE_OTSECUREFRAME_INSECURE_RX_PERMITTED // Allow parsing of insecure frame version...
      case 'O': // Non-secure basic OpenTRV secureable frame...
          {
          // Do some simple validation of structure and number ranges.
          const uint8_t fl = firstByte + 1; // (Full) frame length, including the length byte itself.
          if(fl < 8) { break; } // Too short to be valid.
          const uint8_t il = msg[2] & 0xf;
          if(0 == il) { break; } // Anonymous sender (zero-length ID) not (yet) permitted.
          //
          // TODO
          //
          break;
          }
#endif // ENABLE_OTSECUREFRAME_INSECURE_RX_PERMITTED

      // Reject unrecognised type, though potentially fall through to recognise other encodings.
      default: break;
      }
  }
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT

#ifdef ENABLE_FS20_ENCODING_SUPPORT
  switch(firstByte)
    {
    default:
    case OTRadioLink::FTp2_NONE: // Also zero-length with leading length byte.
      break;

#ifdef ALLOW_CC1_SUPPORT_HUB
    // Handle alert message (at hub).
    // Dump onto serial to be seen by the attached host.
    case OTRadioLink::FTp2_CC1Alert:
      {
      OTProtocolCC::CC1Alert a;
      a.OTProtocolCC::CC1Alert::decodeSimple(msg, msglen);
      // After decode instance should be valid and with correct (source) house code.
      if(a.isValid())
        {
        // Pass message to host to deal with as "! hc1 hc2" after prefix indicating relayed (CC1 alert) message.
        p->print(F("+CC1 ! ")); p->print(a.getHC1()); p->print(' '); p->println(a.getHC2());
        }
      return;
      }
#endif

#ifdef ALLOW_CC1_SUPPORT_HUB
    // Handle poll-response message (at hub).
    // Dump onto serial to be seen by the attached host.
    case OTRadioLink::FTp2_CC1PollResponse:
      {
      OTProtocolCC::CC1PollResponse a;
      a.OTProtocolCC::CC1PollResponse::decodeSimple(msg, msglen);
      // After decode instance should be valid and with correct (source) house code.
      if(a.isValid())
        {
        // Pass message to host to deal with as:
        //     * hc1 hc2 rh tp tr al s w sy
        // after prefix indicating relayed (CC1) message.
        // (Parameters in same order as make() factory method, see below.)
//   * House code (hc1, hc2) of valve controller that the poll/command is being sent to.
//   * relative-humidity    [0,50] 0-100 in 2% steps (rh)
//   * temperature-ds18b20  [0,199] 0.000-99.999C in 1/2 C steps, pipe temp (tp)
//   * temperature-opentrv  [0,199] 0.000-49.999C in 1/4 C steps, room temp (tr)
//   * ambient-light        [1,62] no units, dark to light (al)
//   * switch               [false,true] activation toggle, helps async poll detect intermittent use (s)
//   * window               [false,true] false=closed,true=open (w)
//   * syncing              [false,true] if true, (re)syncing to FHT8V (sy)
// Returns instance; check isValid().
//            static CC1PollResponse make(uint8_t hc1, uint8_t hc2,
//                                        uint8_t rh,
//                                        uint8_t tp, uint8_t tr,
//                                        uint8_t al,
//                                        bool s, bool w, bool sy);
        p->print(F("+CC1 * "));
            p->print(a.getHC1()); p->print(' '); p->print(a.getHC2()); p->print(' ');
            p->print(a.getRH()); p->print(' ');
            p->print(a.getTP()); p->print(' '); p->print(a.getTR()); p->print(' ');
            p->print(a.getAL()); p->print(' ');
            p->print(a.getS()); p->print(' '); p->print(a.getW()); p->print(' ');
               p->println(a.getSY());
        }
      return;
      }
#endif

#ifdef ALLOW_CC1_SUPPORT_RELAY
    // Handle poll/cmd message (at relay).
    // IFF this message is addressed to this (target) unit's house code
    // then action the commands and respond (quickly) with a poll response.
    case OTRadioLink::FTp2_CC1PollAndCmd:
      {
      OTProtocolCC::CC1PollAndCommand c;
      c.OTProtocolCC::CC1PollAndCommand::decodeSimple(msg, msglen);
      // After decode instance should be valid and with correct house code.
      if(c.isValid())
        {
//        p->print(F("+CC1 * ")); p->print(a.getHC1()); p->print(' '); p->println(a.getHC2());
        // Process the message only if it is targetted at this node.
        const uint8_t hc1 = FHT8VGetHC1();
        const uint8_t hc2 = FHT8VGetHC2();
        if((c.getHC1() == hc1) && (c.getHC2() == hc2))
          {
          // Act on the incoming command.
          // Set LEDs.
          setLEDsCO(c.getLC(), c.getLT(), c.getLF(), true);
          // Set radiator valve position.
          NominalRadValve.set(c.getRP());

          // Respond to the hub with sensor data.
          // Can use read() for very freshest values at risk of some delay/cost.
#ifdef HUMIDITY_SENSOR_SUPPORT
          const uint8_t rh = RelHumidity.read() >> 1; // Scale from [0,100] to [0,50] for TX.
#else
          const uint8_t rh = 0; // RH% not available.
#endif
          const uint8_t tp = (uint8_t) constrain(extDS18B20_0.read() >> 3, 0, 199); // Scale to to 1/2C [0,100[ for TX.
          const uint8_t tr = (uint8_t) constrain(TemperatureC16.read() >> 2, 0, 199); // Scale from 1/16C to 1/4C [0,50[ for TX.
          const uint8_t al = AmbLight.read() >> 2; // Scale from [0,255] to [1,62] for TX (allow value coercion at extremes).
          const bool s = getSwitchToggleStateCO();
          const bool w = (fastDigitalRead(BUTTON_LEARN2_L) != LOW); // BUTTON_LEARN2_L high means open circuit means door/window open.
          const bool sy = !NominalRadValve.isInNormalRunState(); // Assume only non-normal FHT8V state is 'syncing'.
          OTProtocolCC::CC1PollResponse r =
              OTProtocolCC::CC1PollResponse::make(hc1, hc2, rh, tp, tr, al, s, w, sy);
          // Send message back to hub.
          // Hub can poll again if it does not see the response.
          // TODO: may need to insert a delay to allow hub to be ready if use of read() above is not enough.
          uint8_t txbuf[STATS_MSG_START_OFFSET + OTProtocolCC::CC1PollResponse::primary_frame_bytes+1]; // More than large enough for preamble + sync + alert message.
          uint8_t *const bptr = RFM22RXPreambleAdd(txbuf);
          const uint8_t bodylen = r.encodeSimple(bptr, sizeof(txbuf) - STATS_MSG_START_OFFSET, true);
          const uint8_t buflen = STATS_MSG_START_OFFSET + bodylen;
#if 0 && defined(DEBUG)
OTRadioLink::printRXMsg(p, txbuf, buflen);
#endif
          if(PrimaryRadio.sendRaw(txbuf, buflen)) // Send at default volume...  One going missing won't hurt that much.
            {
#if 1 && defined(DEBUG)
            p->println(F("polled")); // Done it!
#endif
            }
          }
        }
      return;
      }
#endif

#if defined(ALLOW_STATS_RX) && defined(ENABLE_FS20_ENCODING_SUPPORT) && defined(ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX)
    // Stand-alone stats message.
    case OTRadioLink::FTp2_FullStatsIDL: case OTRadioLink::FTp2_FullStatsIDH:
      {
#if 0 && defined(DEBUG)
DEBUG_SERIAL_PRINTLN_FLASHSTRING("Stats IDx");
#endif
      // May be binary stats frame, so attempt to decode...
      OTV0P2BASE::FullStatsMessageCore_t content;
      // (TODO: should reject non-secure messages when expecting secure ones...)
      const uint8_t *tail = OTV0P2BASE::decodeFullStatsMessageCore(msg, msglen, OTV0P2BASE::stTXalwaysAll, false, &content);
      if(NULL != tail)
         {
         if(content.containsID)
           {
#if 0 && defined(DEBUG)
           DEBUG_SERIAL_PRINT_FLASHSTRING("Stats HC ");
           DEBUG_SERIAL_PRINTFMT(content.id0, HEX);
           DEBUG_SERIAL_PRINT(' ');
           DEBUG_SERIAL_PRINTFMT(content.id1, HEX);
           DEBUG_SERIAL_PRINTLN();
#endif
//           recordCoreStats(false, &content);
           OTV0P2BASE::outputCoreStats(&Serial, secure, &content);
           }
         }
      return;
      }
#endif

#if defined(LISTEN_FOR_FTp2_FS20_native) // defined(ALLOW_STATS_RX) && defined(ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX) && defined(ENABLE_FS20_ENCODING_SUPPORT) // Listen for calls for heat from remote valves...
    case OTRadioLink::FTp2_FS20_native:
      {
      decodeAndHandleFTp2_FS20_native(p, secure, msg, msglen);
      return;
      }
#endif

#if defined(ALLOW_STATS_RX) && defined(ENABLE_FS20_ENCODING_SUPPORT)
    case OTRadioLink::FTp2_JSONRaw:
      {
      if(-1 != OTV0P2BASE::checkJSONMsgRXCRC(msg, msglen))
        {
#ifdef ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
        // Initial pass for Brent.
        // Strip trailing high bit and CRC.  Not very nice, but it'll have to do.
        uint8_t buf[OTV0P2BASE::MSG_JSON_ABS_MAX_LENGTH + 1];
        uint8_t buflen = 0;
        while(buflen < sizeof(buf))
          {
          const uint8_t b = msg[buflen];
          if(('}' | 0x80) == b) { buf[buflen++] = '}'; break; } // End of JSON found.
          buf[buflen++] = b;
          }
        // FIXME should only relay authenticated (and encrypted) traffic.
        // Relay stats frame over secondary radio.
        SecondaryRadio.queueToSend(buf, buflen); 
#else // Don't write to console/Serial also if relayed.
        // Write out the JSON message.
        OTV0P2BASE::outputJSONStats(&Serial, secure, msg, msglen);
        // Attempt to ensure that trailing characters are pushed out fully.
        OTV0P2BASE::flushSerialProductive();
#endif // ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
        }
      return;
      }
#endif
    }
#endif // ENABLE_FS20_ENCODING_SUPPORT

  // Unparseable frame: drop it.
#if 0 && defined(DEBUG)
  p->print(F("!RX bad msg prefix ")); OTRadioLink::printRXMsg(p, msg, min(msglen, 8));
#endif
  return;
  }
#endif // ENABLE_RADIO_RX

#ifdef ENABLE_RADIO_RX
// Incrementally process I/O and queued messages, including from the radio link.
// This may mean printing them to Serial (which the passed Print object usually is),
// or adjusting system parameters,
// or relaying them elsewhere, for example.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This will attempt to process messages in such a way
// as to avoid internal overflows or other resource exhaustion.
bool handleQueuedMessages(Print *p, bool wakeSerialIfNeeded, OTRadioLink::OTRadioLink *rl)
  {
  bool workDone = false;
  bool neededWaking = false; // Set true once this routine wakes Serial.

  // Deal with any I/O that is queued.
  pollIO(true);

  // Check for activity on the radio link.
  rl->poll();
  uint8_t msglen;
  const volatile uint8_t *pb;
  if(NULL != (pb = rl->peekRXMsg(msglen)))
    {
    if(!neededWaking && wakeSerialIfNeeded && OTV0P2BASE::powerUpSerialIfDisabled<V0P2_UART_BAUD>()) { neededWaking = true; } // FIXME
    // Don't currently regard anything arriving over the air as 'secure'.
    // FIXME: cast away volatile to process the message content.
    decodeAndHandleRawRXedMessage(p, false, (const uint8_t *)pb, msglen);
    rl->removeRXMsg();
    // Note that some work has been done.
    workDone = true;
    }

  // Turn off serial at end, if this routine woke it.
  if(neededWaking) { OTV0P2BASE::flushSerialProductive(); OTV0P2BASE::powerDownSerial(); }
  return(workDone);
  }
#endif // ENABLE_RADIO_RX

