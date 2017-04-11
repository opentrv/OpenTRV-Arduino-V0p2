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
#include "REV10SecureBHR.h"

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


// Brings in necessary radio libs.
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
static constexpr int8_t RFM23B_IRQ_PIN = PIN_RFM_NIRQ;
static constexpr bool RFM23B_allowRX = true;
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> RFM23B;
//OTSIM900Link::OTSIM900Link SIM900(REGULATOR_POWERUP, RADIO_POWER_PIN, SOFTSERIAL_RX_PIN, SOFTSERIAL_TX_PIN);
OTSIM900Link::OTSIM900Link<8, 5, RADIO_POWER_PIN, OTV0P2BASE::getSecondsLT> SIM900; // (REGULATOR_POWERUP, RADIO_POWER_PIN);

// Assigns radio to PrimaryRadio alias
OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B;

// Assign radio to SecondaryRadio alias.
OTRadioLink::OTRadioLink &SecondaryRadio = SIM900;

// Handle FS20/FHT8V traffic including binary stats.
// Returns true on successful frame type match, false if no suitable frame was found/decoded and another parser should be tried.
static bool decodeAndHandleOTSecureableFrame(Print *p, const bool secure, const uint8_t * const msg)
  {
  const uint8_t msglen = msg[-1];
  const uint8_t firstByte = msg[0];

  // Validate structure of header/frame first.
  // This is quick and checks for insane/dangerous values throughout.
  OTRadioLink::SecurableFrameHeader sfh;
  const uint8_t l = sfh.checkAndDecodeSmallFrameHeader(msg-1, msglen+1);
  // If isOK flag is set false for any reason, frame is broken/unsafe/unauth.
  bool isOK = (l > 0);
  // If failed this early and this badly, let someone else try parsing the message buffer...
  if(!isOK) { return(false); }

  // Buffer for receiving secure frame body.
  // (Non-secure frame bodies should be read directly from the frame buffer.)
  uint8_t secBodyBuf[OTRadioLink::ENC_BODY_SMALL_FIXED_PTEXT_MAX_SIZE];
  uint8_t decryptedBodyOutSize = 0;

  // Validate integrity of frame (CRC for non-secure, auth for secure).
  const bool secureFrame = sfh.isSecure();
  // Body length after any decryption, etc.
  uint8_t receivedBodyLength;
  // TODO: validate entire message, eg including auth, or CRC if insecure msg rcvd&allowed.
  // Validate (authenticate) and decrypt body of secure frames.
  uint8_t key[16];
  if(secureFrame && isOK)
    {
    // Get the 'building' key.
    if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key))
      {
      isOK = false;
      OTV0P2BASE::serialPrintlnAndFlush(F("!RX key"));
      }
    }
  uint8_t senderNodeID[OTV0P2BASE::OpenTRV_Node_ID_Bytes];
  if(secureFrame && isOK)
    {
    // Look up full ID in associations table,
    // validate RX message counter,
    // authenticate and decrypt,
    // update RX message counter.
    isOK = (0 != OTRadioLink::SimpleSecureFrame32or0BodyRXV0p2::getInstance().decodeSecureSmallFrameSafely(&sfh, msg-1, msglen+1,
                                            OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleDec_DEFAULT_STATELESS,
                                            NULL, key,
                                            secBodyBuf, sizeof(secBodyBuf), decryptedBodyOutSize,
                                            senderNodeID,
                                            true));
    if(!isOK)
      {
      // Useful brief network diagnostics: a couple of bytes of the claimed ID of rejected frames.
      // Warnings rather than errors because there may legitimately be multiple disjoint networks.
      OTV0P2BASE::serialPrintAndFlush(F("?RX auth")); // Missing association or failed auth.
      if(sfh.getIl() > 0) { OTV0P2BASE::serialPrintAndFlush(' '); OTV0P2BASE::serialPrintAndFlush(sfh.id[0], HEX); }
      if(sfh.getIl() > 1) { OTV0P2BASE::serialPrintAndFlush(' '); OTV0P2BASE::serialPrintAndFlush(sfh.id[1], HEX); }
      OTV0P2BASE::serialPrintlnAndFlush();
      }
    }

  if(!isOK) { return(false); } // Stop if not OK.

  switch(firstByte) // Switch on type.
    {
    case 'O' | 0x80: // Basic OpenTRV secure frame...
      {
      if(decryptedBodyOutSize < 2)
        {
        break;
        }
      // If acting as a boiler hub
      // then extract the valve %age and pass to boiler controller
      // but use only if valid.
      // Ignore explicit call-for-heat flag for now.
      const uint8_t percentOpen = secBodyBuf[0];
      if(percentOpen <= 100) { remoteCallForHeatRX(0, percentOpen); } // todo call for heat valve id not passed in.
      // If the frame contains JSON stats
      // then forward entire secure frame as-is across the secondary radio relay link,
      // else print directly to console/Serial.
      if((0 != (secBodyBuf[1] & 0x10)) && (decryptedBodyOutSize > 3) && ('{' == secBodyBuf[2]))
        {
        SecondaryRadio.queueToSend(msg, msglen); 
//        // Write out the JSON message, inserting synthetic ID/@ and seq/+.
//        Serial.print(F("{\"@\":\""));
//        for(int i = 0; i < OTV0P2BASE::OpenTRV_Node_ID_Bytes; ++i) { Serial.print(senderNodeID[i], HEX); }
//        Serial.print(F("\",\"+\":"));
//        Serial.print(sfh.getSeq());
//        Serial.print(',');
//        Serial.write(secBodyBuf + 3, decryptedBodyOutSize - 3);
//        Serial.println('}');
////        OTV0P2BASE::outputJSONStats(&Serial, secure, msg, msglen);
//        // Attempt to ensure that trailing characters are pushed out fully.
//        OTV0P2BASE::flushSerialProductive();
        }
      return(true);
      }

    // Reject unrecognised type, though fall through potentially to recognise other encodings.
    default: break;
    }

  // Failed to parse; let another handler try.
  return(false);
  }

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
  if(msglen < 2) { return; } // Too short to be useful, so ignore.

   // Length-first OpenTRV secureable-frame format...
  if(decodeAndHandleOTSecureableFrame(p, secure, msg)) { return; }
  const uint8_t firstByte = msg[0];
  // Unparseable frame: drop it; possibly log it as an error.
  return;
  }

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
  return(workDone);
  }

