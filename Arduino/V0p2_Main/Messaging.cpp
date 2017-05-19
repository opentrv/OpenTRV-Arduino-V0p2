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
//OTSIM900Link::OTSIM900Link SIM900(REGULATOR_POWERUP, RADIO_POWER_PIN, SOFTSERIAL_RX_PIN, SOFTSERIAL_TX_PIN);
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

#if defined(ENABLE_RFM23B_FS20_RAW_PREAMBLE)
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
#endif // defined(ENABLE_RFM23B_FS20_RAW_PREAMBLE)


#if defined(ENABLE_RADIO_RX) && defined(ENABLE_FHT8VSIMPLE_RX) // (defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)) && defined(ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX) // Listen for calls for heat from remote valves...
// Handle FS20/FHT8V traffic including binary stats.
// Returns true on success, false otherwise.
static bool decodeAndHandleFTp2_FS20_native(Print *p, const bool secure, const uint8_t * const msg, const uint8_t msglen)
{
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
    const uint8_t percentOpen = OTRadValve::FHT8VRadValveUtil::convert255ScaleToPercent(command.extension);
    remoteCallForHeatRX(compoundHC, percentOpen);
    }
#endif

  if(NULL != trailer)
    {
#if 0 && defined(DEBUG)
p->print("FS20 msg HC "); p->print(command.hc1); p->print(' '); p->println(command.hc2);
#endif
#if defined(ENABLE_STATS_RX) // Only look for the trailer if supported.
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
#endif // defined(ENABLE_STATS_RX)
    }
  return(true);
  }
#endif

#ifdef ENABLE_RADIO_RX
// Decode and handle inbound raw message (msg[-1] contains the count of bytes received).
// A message may contain trailing garbage at the end; the decoder/router should cope.
// The buffer may be reused when this returns,
// so a copy should be taken of anything that needs to be retained.
// If secure is true then this message arrived over an inherently secure channel.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This routine is NOT allowed to alter in any way the content of the buffer passed.
static void decodeAndHandleRawRXedMessage(Print *p, const bool secure, const uint8_t * const msg)
  {
  const uint8_t msglen = msg[-1];

  // TODO: consider extracting hash of all message data (good/bad) and injecting into entropy pool.
#if 0 && defined(DEBUG)
  OTRadioLink::printRXMsg(p, msg-1, msglen+1); // Print len+frame.
#endif

  if(msglen < 2) { return; } // Too short to be useful, so ignore.

   // Length-first OpenTRV secureable-frame format...
#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT) // && defined(ENABLE_FAST_FRAMED_CARRIER_SUPPORT)
constexpr bool allowOTSecureFrameRX = false; // ENABLE_OTSECUREFRAME_INSECURE_RX_PERMITTED  FIXME!
constexpr bool enableRadioRelay = true; // ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY  FIXME!
  if(OTRadioLink::decodeAndHandleOTSecureableFrame<allowOTSecureFrameRX, enableRadioRelay>(p, secure, msg, SecondaryRadio)) { return; }  // XXX
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT

  const uint8_t firstByte = msg[0];

#ifdef ENABLE_FS20_ENCODING_SUPPORT
  switch(firstByte)
    {
    default: // Reject unrecognised leading type byte.
    case OTRadioLink::FTp2_NONE: // Reject zero-length with leading length byte.
      break;

#if defined(ENABLE_STATS_RX) && defined(ENABLE_FS20_ENCODING_SUPPORT) && defined(ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX)
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

#if defined(ENABLE_FHT8VSIMPLE_RX) // defined(ENABLE_STATS_RX) && defined(ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX) && defined(ENABLE_FS20_ENCODING_SUPPORT) // Listen for calls for heat from remote valves...
    case OTRadioLink::FTp2_FS20_native:
      {
      decodeAndHandleFTp2_FS20_native(p, secure, msg, msglen);
      return;
      }
#endif

#if defined(ENABLE_STATS_RX) && defined(ENABLE_FS20_ENCODING_SUPPORT)
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

  // Unparseable frame: drop it; possibly log it as an error.
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  p->print(F("!RX bad msg, len+prefix: ")); OTRadioLink::printRXMsg(p, msg-1, min(msglen+1, 8));
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
// as to avoid internal overflows or other resource exhaustion,
// which may mean deferring work at certain times
// such as the end of minor cycle.
// The Print object pointer must not be NULL.
bool handleQueuedMessages(Print *p, bool wakeSerialIfNeeded, OTRadioLink::OTRadioLink *rl)
  {
  // Avoid starting any potentially-slow processing very late in the minor cycle.
  // This is to reduce the risk of loop overruns
  // at the risk of delaying some processing
  // or even dropping some incoming messages if queues fill up.
  // Decoding (and printing to serial) a secure 'O' frame takes ~60 ticks (~0.47s).
  // Allow for up to 0.5s of such processing worst-case,
  // ie don't start processing anything later that 0.5s before the minor cycle end.
  const uint8_t sctStart = OTV0P2BASE::getSubCycleTime();
  if(sctStart >= ((OTV0P2BASE::GSCT_MAX/4)*3)) { return(false); }

  // Deal with any I/O that is queued.
  bool workDone = pollIO(true);

  // Check for activity on the radio link.
  rl->poll();

  bool neededWaking = false; // Set true once this routine wakes Serial.
  const volatile uint8_t *pb;
  if(NULL != (pb = rl->peekRXMsg()))
    {
    if(!neededWaking && wakeSerialIfNeeded && OTV0P2BASE::powerUpSerialIfDisabled<V0P2_UART_BAUD>()) { neededWaking = true; } // FIXME
    // Don't currently regard anything arriving over the air as 'secure'.
    // FIXME: shouldn't have to cast away volatile to process the message content.
    decodeAndHandleRawRXedMessage(p, false, (const uint8_t *)pb);
    rl->removeRXMsg();
    // Note that some work has been done.
    workDone = true;
    }

  // Turn off serial at end, if this routine woke it.
  if(neededWaking) { OTV0P2BASE::flushSerialProductive(); OTV0P2BASE::powerDownSerial(); }

#if 0 && defined(DEBUG)
  const uint8_t sctEnd = OTV0P2BASE::getSubCycleTime();
  const uint8_t ticks = sctEnd - sctStart;
  if(ticks > 1)
    {
    OTV0P2BASE::serialPrintAndFlush(ticks);
    OTV0P2BASE::serialPrintlnAndFlush();
    }
#endif

  return(workDone);
  }
#endif // ENABLE_RADIO_RX

