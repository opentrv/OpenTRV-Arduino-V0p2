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
*/

/*
 Generic messaging support for OpenTRV.
 */

#include "Messaging.h"

#include <OTRadioLink.h>
#ifdef ALLOW_CC1_SUPPORT
#include <OTProtocolCC.h>
#endif

#include <util/atomic.h>

#include "Control.h"
#include "Radio.h"
#include "UI_Minimal.h"

#include "V0p2_Sensors.h"


#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Return true if header/structure and CRC looks valid for (3-byte) buffered stats payload.
bool verifyHeaderAndCRCForTrailingMinimalStatsPayload(uint8_t const *const buf)
  {
  return((MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS == ((buf[0]) & MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MASK)) && // Plausible header.
         (0 == (buf[1] & 0x80)) && // Top bit is clear on this byte also.
         (buf[2] == OTV0P2BASE::crc7_5B_update(buf[0], buf[1]))); // CRC validates, top bit implicitly zero.
  }
#endif // ENABLE_FS20_ENCODING_SUPPORT

#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Store minimal stats payload into (2-byte) buffer from payload struct (without CRC); values are coerced to fit as necessary..
//   * payload  must be non-NULL
// Used for minimal and full packet forms,
void writeTrailingMinimalStatsPayloadBody(uint8_t *buf, const trailingMinimalStatsPayload_t *payload)
  {
#ifdef DEBUG
  if(NULL == payload) { panic(); }
#endif
  // Temperatures coerced to fit between MESSAGING_TRAILING_MINIMAL_STATS_TEMP_BIAS (-20C) and 0x7ff_MESSAGING_TRAILING_MINIMAL_STATS_TEMP_BIAS (107Cf).
#if MESSAGING_TRAILING_MINIMAL_STATS_TEMP_BIAS > 0
#error MESSAGING_TRAILING_MINIMAL_STATS_TEMP_BIAS must be negative
#endif
  const int16_t bitmask = 0x7ff;
  const int16_t minTempRepresentable = MESSAGING_TRAILING_MINIMAL_STATS_TEMP_BIAS;
  const int16_t maxTempRepresentable = bitmask + MESSAGING_TRAILING_MINIMAL_STATS_TEMP_BIAS;
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("payload->tempC16: ");
  DEBUG_SERIAL_PRINTFMT(payload->tempC16, DEC);
  DEBUG_SERIAL_PRINT_FLASHSTRING(" min=");
  DEBUG_SERIAL_PRINTFMT(minTempRepresentable, DEC);
  DEBUG_SERIAL_PRINT_FLASHSTRING(" max=");
  DEBUG_SERIAL_PRINTFMT(maxTempRepresentable, DEC);
  DEBUG_SERIAL_PRINTLN();
#endif
  int16_t temp16Cbiased = payload->tempC16;
  if(temp16Cbiased < minTempRepresentable) { temp16Cbiased = minTempRepresentable; }
  else if(temp16Cbiased > maxTempRepresentable) { temp16Cbiased = maxTempRepresentable; }
  temp16Cbiased -= MESSAGING_TRAILING_MINIMAL_STATS_TEMP_BIAS; // Should now be strictly positive.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("temp16Cbiased: ");
  DEBUG_SERIAL_PRINTFMT(temp16Cbiased, DEC);
  DEBUG_SERIAL_PRINTLN();
#endif
  const uint8_t byte0 = MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS | (payload->powerLow ? 0x10 : 0) | (temp16Cbiased & 0xf);
  const uint8_t byte1 = (uint8_t) (temp16Cbiased >> 4);
  buf[0] = byte0;
  buf[1] = byte1;
#if 0 && defined(DEBUG)
  for(uint8_t i = 0; i < 2; ++i) { if(0 != (buf[i] & 0x80)) { panic(); } } // MSBits should be clear.
#endif
  }
#endif // ENABLE_FS20_ENCODING_SUPPORT

#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Store minimal stats payload into (3-byte) buffer from payload struct and append CRC; values are coerced to fit as necessary..
//   * payload  must be non-NULL
void writeTrailingMinimalStatsPayload(uint8_t *buf, const trailingMinimalStatsPayload_t *payload)
  {
  writeTrailingMinimalStatsPayloadBody(buf, payload);
  buf[2] = OTV0P2BASE::crc7_5B_update(buf[0], buf[1]);
#if 0 && defined(DEBUG)
  for(uint8_t i = 0; i < 3; ++i) { if(0 != (buf[i] & 0x80)) { panic(); } } // MSBits should be clear.
#endif
  }
#endif // ENABLE_FS20_ENCODING_SUPPORT

#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Extract payload from valid (3-byte) header+payload+CRC into payload struct; only 2 bytes are actually read.
// Input bytes (eg header and check value) must already have been validated.
void extractTrailingMinimalStatsPayload(const uint8_t *const buf, trailingMinimalStatsPayload_t *const payload)
  {
#ifdef DEBUG
  if(NULL == payload) { panic(); }
#endif
  payload->powerLow = (0 != (buf[0] & 0x10));
  payload->tempC16 = ((((int16_t) buf[1]) << 4) | (buf[0] & 0xf)) + MESSAGING_TRAILING_MINIMAL_STATS_TEMP_BIAS;
  }
#endif // ENABLE_FS20_ENCODING_SUPPORT


#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Send (valid) core binary stats to specified print channel, followed by "\r\n".
// This does NOT attempt to flush output nor wait after writing.
// Will only write stats with a source ID.
void outputCoreStats(Print *p, bool secure, const FullStatsMessageCore_t *stats)
  {
  if(stats->containsID)
    {
    // Dump (remote) stats field '@<hexnodeID>;TnnCh[P;]'
    // where the T field shows temperature in C with a hex digit after the binary point indicated by C
    // and the optional P field indicates low power.
    p->print((char) OTV0P2BASE::SERLINE_START_CHAR_RSTATS);
    p->print((((uint16_t)stats->id0) << 8) | stats->id1, HEX);
    if(stats->containsTempAndPower)
      {
      p->print(F(";T"));
      p->print(stats->tempAndPower.tempC16 >> 4, DEC);
      p->print('C');
      p->print(stats->tempAndPower.tempC16 & 0xf, HEX);
      if(stats->tempAndPower.powerLow) { p->print(F(";P")); } // Insert power-low field if needed.
      }
    if(stats->containsAmbL)
      {
      p->print(F(";L"));
      p->print(stats->ambL);
      }
    if(0 != stats->occ)
      {
      p->print(F(";O"));
      p->print(stats->occ);
      }
    p->println();
    }
  }
#endif // ENABLE_FS20_ENCODING_SUPPORT

#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Send (valid) minimal binary stats to specified print channel, followed by "\r\n".
// This does NOT attempt to flush output nor wait after writing.
void outputMinimalStats(Print *p, bool secure, uint8_t id0, uint8_t id1, const trailingMinimalStatsPayload_t *stats)
    {
    // Convert to full stats for output.
    FullStatsMessageCore_t fullstats;
    clearFullStatsMessageCore(&fullstats);
    fullstats.id0 = id0;
    fullstats.id1 = id1;
    fullstats.containsID = true;
    memcpy((void *)&fullstats.tempAndPower, stats, sizeof(fullstats.tempAndPower));
    fullstats.containsTempAndPower = true;
    outputCoreStats(p, secure, &fullstats);
    }
#endif // ENABLE_FS20_ENCODING_SUPPORT


#ifdef ENABLE_FS20_ENCODING_SUPPORT

// Send core/common 'full' stats message.
//   * content contains data to be sent in the message; must be non-NULL
// Note that up to 7 bytes of payload is optimal for the CRC used.
// If successful, returns pointer to terminating 0xff at end of message.
// Returns NULL if failed (eg because of bad inputs or insufficient buffer space);
// part of the message may have have been written in this case and in particular the previous terminating 0xff may have been overwritten.
uint8_t *encodeFullStatsMessageCore(uint8_t * const buf, const uint8_t buflen, const OTV0P2BASE::stats_TX_level secLevel, const bool secureChannel,
    const FullStatsMessageCore_t * const content)
  {
  if(NULL == buf) { return(NULL); } // Could be an assert/panic instead at a pinch.
  if(NULL == content) { return(NULL); } // Could be an assert/panic instead at a pinch.
  if(secureChannel) { return(NULL); } // TODO: cannot create secure message yet.
//  if(buflen < FullStatsMessageCore_MIN_BYTES_ON_WIRE+1) { return(NULL); } // Need space for at least the shortest possible message + terminating 0xff.
//  if(buflen < FullStatsMessageCore_MAX_BYTES_ON_WIRE+1) { return(NULL); } // Need space for longest possible message + terminating 0xff.

  // Compute message payload length (excluding CRC and terminator).
  // Fail immediately (return NULL) if not enough space for message content.
  const uint8_t payloadLength =
      1 + // Initial header.
      (content->containsID ? 2 : 0) +
      (content->containsTempAndPower ? 2 : 0) +
      1 + // Flags header.
      (content->containsAmbL ? 1 : 0);
  if(buflen < payloadLength + 2)  { return(NULL); }

  // Validate some more detail.
  // ID
  if(content->containsID)
    {
    if((content->id0 == (uint8_t)0xff) || (content->id1 == (uint8_t)0xff)) { return(NULL); } // ID bytes cannot be 0xff.
    if((content->id0 & 0x80) != (content->id1 & 0x80)) { return(NULL); } // ID top bits don't match.
    }
  // Ambient light.
  if(content->containsAmbL)
    {
    if((content->ambL == 0) || (content->ambL == (uint8_t)0xff)) { return(NULL); } // Forbidden values.
    }

  // WRITE THE MESSAGE!
  // Pointer to next byte to write in message.
  register uint8_t *b = buf;

  // Construct the header.
  // * byte 0 :  |  0  |  1  |  1  |  1  |  R0 | IDP | IDH | SEC |   header, 1x reserved 0 bit, ID Present, ID High, SECure
//#define MESSAGING_FULL_STATS_HEADER_MSBS 0x70
//#define MESSAGING_FULL_STATS_HEADER_MASK 0xf0
//#define MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT 4
//#define MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH 2
//#define MESSAGING_FULL_STATS_HEADER_BITS_ID_SECURE 1
  const uint8_t header = MESSAGING_FULL_STATS_HEADER_MSBS |
      (content->containsID ? MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT : 0) |
      ((content->containsID && (0 != (content->id0 & 0x80))) ? MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH : 0) |
      0; // TODO: cannot do secure messages yet.
  *b++ = header;
 
  // Insert ID if requested.
  if(content->containsID)
    {
    *b++ = content->id0 & 0x7f;
    *b++ = content->id1 & 0x7f;
    }

  // Insert basic temperature and power status if requested.
  if(content->containsTempAndPower)
    {
    writeTrailingMinimalStatsPayloadBody(b, &(content->tempAndPower));
    b += 2;
    }
 
  // Always insert flags header , and downstream optional values.
// Flags indicating which optional elements are present:
// AMBient Light, Relative Humidity %.
// OC1/OC2 = Occupancy: 00 not disclosed, 01 probably, 10 possibly, 11 not occupied recently.
// IF EXT is 1 a futher flags byte follows.
// * byte b+2: |  0  |  1  |  1  | EXT | ABML| RH% | OC1 | OC2 |   EXTension-follows flag, plus optional section flags.
//#define MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS 0x60
//#define MESSAGING_FULL_STATS_FLAGS_HEADER_MASK 0xe0
//#define MESSAGING_FULL_STATS_FLAGS_HEADER_AMBL 8
//#define MESSAGING_FULL_STATS_FLAGS_HEADER_RHP 4
  // Omit occupancy data unless encoding for a secure channel or at a very permissive stats TX security level.
  const uint8_t flagsHeader = MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS |
    (content->containsAmbL ? MESSAGING_FULL_STATS_FLAGS_HEADER_AMBL : 0) |
    ((secureChannel || (secLevel <= OTV0P2BASE::stTXalwaysAll)) ? (content->occ & 3) : 0);
  *b++ = flagsHeader;
  // Now insert extra fields as flagged.
  if(content->containsAmbL)
    { *b++ = content->ambL; }
  // TODO: RH% etc

  // Finish off message by computing and appending the CRC and then terminating 0xff (and return pointer to 0xff).
  // Assumes that b now points just beyond the end of the payload.
  uint8_t crc = MESSAGING_FULL_STATS_CRC_INIT; // Initialisation.
  for(const uint8_t *p = buf; p < b; ) { crc = OTV0P2BASE::crc7_5B_update(crc, *p++); }
  *b++ = crc;
  *b = 0xff;
#if 0 && defined(DEBUG)
  if(b - buf != payloadLength + 1) { panic(F("msg gen err")); }
#endif
  return(b);
  }

// Decode core/common 'full' stats message.
// If successful returns pointer to next byte of message, ie just after full stats message decoded.
// Returns NULL if failed (eg because of corrupt/insufficient message data) and state of 'content' result is undefined.
// This will avoid copying into the result data (possibly tainted) that has arrived at an inappropriate security level.
//   * content will contain data decoded from the message; must be non-NULL
const uint8_t *decodeFullStatsMessageCore(const uint8_t * const buf, const uint8_t buflen, const OTV0P2BASE::stats_TX_level secLevel, const bool secureChannel,
    FullStatsMessageCore_t * const content)
  {
//DEBUG_SERIAL_PRINTLN_FLASHSTRING("dFSMC");
  if(NULL == buf) { return(NULL); } // Could be an assert/panic instead at a pinch.
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk c");
  if(NULL == content) { return(NULL); } // Could be an assert/panic instead at a pinch.
  if(buflen < FullStatsMessageCore_MIN_BYTES_ON_WIRE)
    {
#if 0
    DEBUG_SERIAL_PRINT_FLASHSTRING("buf too small: ");
    DEBUG_SERIAL_PRINT(buflen);
    DEBUG_SERIAL_PRINTLN();
#endif
    return(NULL); // Not long enough for even a minimal message to be present...
    }

//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" clr");
  // Conservatively clear the result completely.
  clearFullStatsMessageCore(content);

  // READ THE MESSAGE!
  // Pointer to next byte to read in message.
  register const uint8_t *b = buf;

  // Validate the message header and start to fill in structure.
  const uint8_t header = *b++;
  // Deonstruct the header.
  // * byte 0 :  |  0  |  1  |  1  |  1  |  R0 | IDP | IDH | SEC |   header, 1x reserved 0 bit, ID Present, ID High, SECure
//#define MESSAGING_FULL_STATS_HEADER_MSBS 0x70
//#define MESSAGING_FULL_STATS_HEADER_MASK 0x70
//#define MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT 4
//#define MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH 2
//#define MESSAGING_FULL_STATS_HEADER_BITS_ID_SECURE 0x80
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk msb");
  if(MESSAGING_FULL_STATS_HEADER_MSBS != (header & MESSAGING_FULL_STATS_HEADER_MASK)) { return(NULL); } // Bad header.
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk sec");
  if(0 != (header & MESSAGING_FULL_STATS_HEADER_BITS_ID_SECURE)) { return(NULL); } // TODO: cannot do secure messages yet.
  // Extract ID if present.
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk ID");
  const bool containsID = (0 != (header & MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT));
  if(containsID)
    {
    content->containsID = true;
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" containsID");
    const uint8_t idHigh = ((0 != (header & MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH)) ? 0x80 : 0);
    content->id0 = *b++ | idHigh;
    content->id1 = *b++ | idHigh;
    }

  // If next header is temp/power then extract it, else must be the flags header.
  if(b - buf >= buflen) { return(NULL); } // Fail if next byte not available.
  if(MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS == (*b & MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MASK))
    {
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk msh");
    if(b - buf >= buflen-1) { return(NULL); } // Fail if 2 bytes not available for this section.
    if(0 != (0x80 & b[1])) { return(NULL); } // Following byte does not have msb correctly cleared.
    extractTrailingMinimalStatsPayload(b, &(content->tempAndPower));
    b += 2;
    content->containsTempAndPower = true;
    }

  // If next header is flags then extract it.
  // FIXME: risk of misinterpretting CRC.
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk flg");
  if(b - buf >= buflen) { return(NULL); } // Fail if next byte not available.
  if(MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS != (*b & MESSAGING_FULL_STATS_FLAGS_HEADER_MASK)) { return(NULL); } // Corrupt message.
  const uint8_t flagsHeader = *b++;
  content->occ = flagsHeader & 3;
  const bool containsAmbL = (0 != (flagsHeader & MESSAGING_FULL_STATS_FLAGS_HEADER_AMBL));
  if(containsAmbL)
    {
    if(b - buf >= buflen) { return(NULL); } // Fail if next byte not available.
    const uint8_t ambL = *b++;
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk aml");
    if((0 == ambL) || (ambL == (uint8_t)0xff)) { return(NULL); } // Illegal value.
    content->ambL = ambL;
    content->containsAmbL = true;
    }

  // Finish off by computing and checking the CRC (and return pointer to just after CRC).
  // Assumes that b now points just beyond the end of the payload.
  if(b - buf >= buflen) { return(NULL); } // Fail if next byte not available.
  uint8_t crc = MESSAGING_FULL_STATS_CRC_INIT; // Initialisation.
  for(const uint8_t *p = buf; p < b; ) { crc = OTV0P2BASE::crc7_5B_update(crc, *p++); }
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk CRC");
  if(crc != *b++) { return(NULL); } // Bad CRC.

  return(b); // Point to just after CRC.
  }

#endif // ENABLE_FS20_ENCODING_SUPPORT


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
  // and the housecode being accepted.
  if(0x26 == command.command)
    {
    const uint16_t compoundHC = (((uint16_t)command.hc1) << 8) | command.hc2;
#if 0 && defined(DEBUG)
    p->print("FS20 RX 0x26 "); // Just notes that a 'valve %' FS20 command has been overheard.
    p->print(command.hc1); p->print(' ');
    p->println(command.hc2);
#endif
    // Process the common 'valve closed' and valve open cases efficiently.
    // Nominally conversion to % should be (uint8_t) ((command.extension * 100) / 255)
    // but approximation with /256, ie >>8, probably fine.
    const uint8_t percentOpen =
        (0 == command.extension) ? 0 :
        ((255 == command.extension) ? 100 :
        ((uint8_t) ((command.extension * (int)100) >> 8)));
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
    if(MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS == (trailer[0] & MESSAGING_FULL_STATS_FLAGS_HEADER_MASK))
      {
      FullStatsMessageCore_t content;
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
//        if(allGood) { recordCoreStats(false, &content); }
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
        // FIMXE // recordMinimalStats(true, command.hc1, command.hc2, &payload); // Record stats; local loopback is secure.
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
      FullStatsMessageCore_t content;
      // (TODO: should reject non-secure messages when expecting secure ones...)
      const uint8_t *tail = decodeFullStatsMessageCore(msg, msglen, OTV0P2BASE::stTXalwaysAll, false, &content);
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
           outputCoreStats(&Serial, secure, &content);
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
      if(-1 != checkJSONMsgRXCRC(msg, msglen))
        {
#ifdef ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
        // Initial pass for Brent.
        // Strip trailing high bit and CRC.  Not very nice, but it'll have to do.
        uint8_t buf[MSG_JSON_ABS_MAX_LENGTH + 1];
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
        outputJSONStats(&Serial, secure, msg, msglen);
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

