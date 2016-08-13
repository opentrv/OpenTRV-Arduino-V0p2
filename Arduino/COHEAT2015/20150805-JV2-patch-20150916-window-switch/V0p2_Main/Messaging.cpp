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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2015
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

#include "EEPROM_Utils.h"
#include "Control.h"
#include "Power_Management.h"
#include "RFM22_Radio.h"
#include "Security.h"
#include "Serial_IO.h"
#include "UI_Minimal.h"

#ifdef USE_MODULE_FHT8VSIMPLE
#include "FHT8V_Wireless_Rad_Valve.h"
#endif

//// Update 'C2' 8-bit CRC with next byte.
//// Usually initialised with 0xff.
//// Should work well from 10--119 bits (2--~14 bytes); best 27-50, 52, 56-119 bits.
//// See: http://users.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
//// Also: http://en.wikipedia.org/wiki/Cyclic_redundancy_check
//#define C2_POLYNOMIAL 0x97
//uint8_t crc8_C2_update(uint8_t crc, const uint8_t datum)
//  {
//  crc ^= datum;
//  for(uint8_t i = 0; ++i <= 8; )
//    {
//    if(crc & 0x80)
//      { crc = (crc << 1) ^ C2_POLYNOMIAL; }
//    else
//      { crc <<= 1; }
//    }
//  return(crc);
//  }





// Return true if header/structure and CRC looks valid for (3-byte) buffered stats payload.
bool verifyHeaderAndCRCForTrailingMinimalStatsPayload(uint8_t const *const buf)
  {
  return((MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS == ((buf[0]) & MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MASK)) && // Plausible header.
         (0 == (buf[1] & 0x80)) && // Top bit is clear on this byte also.
         (buf[2] == OTRadioLink::crc7_5B_update(buf[0], buf[1]))); // CRC validates, top bit implicitly zero.
  }

// Store minimal stats payload into (2-byte) buffer from payload struct (without CRC); values are coerced to fit as necessary..
//   * payload  must be non-null
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

// Store minimal stats payload into (3-byte) buffer from payload struct and append CRC; values are coerced to fit as necessary..
//   * payload  must be non-null
void writeTrailingMinimalStatsPayload(uint8_t *buf, const trailingMinimalStatsPayload_t *payload)
  {
  writeTrailingMinimalStatsPayloadBody(buf, payload);
  buf[2] = OTRadioLink::crc7_5B_update(buf[0], buf[1]);
#if 0 && defined(DEBUG)
  for(uint8_t i = 0; i < 3; ++i) { if(0 != (buf[i] & 0x80)) { panic(); } } // MSBits should be clear.
#endif
  }

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


#if defined(ALLOW_STATS_RX)
#ifndef getInboundStatsQueueOverrun
// Count of dropped inbound stats message due to insufficient queue space.
// Must only be accessed under a lock (ATOMIC_BLOCK).
static uint16_t inboundStatsQueueOverrun = 0;

// Get count of dropped inbound stats messages due to insufficient queue space.
uint16_t getInboundStatsQueueOverrun()
  {
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    { return(inboundStatsQueueOverrun); }
  }
#endif
#endif


// Last JSON (\0-terminated) stats record received, or with first byte \0 if none.
// Should only be accessed under a lock for thread safety.
static /* volatile */ char jsonStats[MSG_JSON_MAX_LENGTH + 1];

// Record stats (local or remote) in JSON (ie non-empty, {}-surrounded, \0-terminated text) format.
// If secure is true then this message arrived over a secure channel.
// The supplied buffer's content is not altered.
// The supplied JSON should already have been somewhat validated.
// Is thread/ISR-safe and moderately fast (though will require a data copy).
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
#if defined(ALLOW_STATS_RX)
#ifndef recordJSONStats
void recordJSONStats(bool secure, const char *json)
  {
#if 0 && defined(DEBUG)
  if(NULL == json) { panic(); }
  if('\0' == *json) { panic(); }
#endif
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    if('\0' != *jsonStats) { ++inboundStatsQueueOverrun; } // Dropped a frame.
    // Atomically overwrite existing buffer with new non-empty stats message.
    strncpy(jsonStats, json, MSG_JSON_MAX_LENGTH+1); // FIXME: will pad redundantly with trailing nulls.
    // Drop over-length message,
    if('\0' != jsonStats[sizeof(jsonStats) - 1]) { *jsonStats = '\0'; }
    }
  }
#endif
#endif

// Gets (and clears) the last JSON record received, if any,
// filling in the supplied buffer
// else leaving it starting with '\0' if none available.
// The buffer must be at least MSG_JSON_MAX_LENGTH+1 chars.
#if defined(ALLOW_STATS_RX)
#ifndef getLastJSONStats
void getLastJSONStats(char *buf)
  {
#if 0 && defined(DEBUG)
  if(NULL == buf) { panic(); }
#endif
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    if('\0' == *jsonStats)
      { *buf = '\0'; } // No message available.
    else
      {
      // Copy the message to the receiver.
      strcpy(buf, jsonStats);
      // Clear the buffer.
      *jsonStats = '\0';
      }
    }
  }
#endif
#endif


// Last core stats record received, or with no ID set if none.
// Should only be accessed under a lock for thread safety.
static /* volatile */ FullStatsMessageCore_t coreStats; // Start up showing no record set.

// Record minimal incoming stats from given ID (if each byte < 100, then may be FHT8V-compatible house code).
// Is thread/ISR-safe and fast.
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
#if defined(ALLOW_STATS_RX) && defined(ALLOW_MINIMAL_STATS_TXRX)
#ifndef recordMinimalStats
void recordMinimalStats(const bool secure, const uint8_t id0, const uint8_t id1, const trailingMinimalStatsPayload_t * const payload)
  {
#if 0 && defined(DEBUG)
  if(NULL == payload) { panic(); }
#endif
   ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    if(coreStats.containsID) { ++inboundStatsQueueOverrun; } // Dropped a frame.
    clearFullStatsMessageCore(&coreStats);
    coreStats.id0 = id0;
    coreStats.id1 = id1;
    coreStats.containsID = true;
    memcpy((void *)&coreStats.tempAndPower, payload, sizeof(coreStats.tempAndPower));
    coreStats.containsTempAndPower = true;
    }
  }
#endif
#endif

// Record core incoming stats; ID must be set as a minimum.
// Is thread/ISR-safe and fast.
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
#if defined(ALLOW_STATS_RX)
#ifndef recordCoreStats
void recordCoreStats(const bool secure, const FullStatsMessageCore_t * const stats)
  {
#if 0 && defined(DEBUG)
  if(NULL == payload) { panic(); }
#endif  // TODO
   if(!stats->containsID) { return; } // Ignore if no ID.
   ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    if(coreStats.containsID) { ++inboundStatsQueueOverrun; } // Dropped a frame.
    memcpy((void *)&coreStats, stats, sizeof(coreStats));
    }
  }
#endif
#endif

// Gets (and clears) the last core stats record received, if any, returning true and filling in the stats struct.
// If no minimal stats record has been received since the last call then the ID will be absent and the rest undefined.
#if defined(ALLOW_STATS_RX)
#ifndef getLastCoreStats
void getLastCoreStats(FullStatsMessageCore_t *stats)
  {
#if 0 && defined(DEBUG)
  if(NULL == stats) { panic(); }
#endif
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    if(!coreStats.containsID)
      { stats->containsID = false; } // Nothing there; just clear containsID field in response for speed.
    else
      {
      // Copy everything.
      memcpy(stats, (void *)&coreStats, sizeof(*stats));
      coreStats.containsID = false; // Mark stats as read.
      }
    }
  }
#endif
#endif


#if defined(ALLOW_STATS_TX)
#if !defined(enableTrailingStatsPayload)
// Returns true if an unencrypted minimal trailing static payload and similar (eg bare stats transmission) is permitted.
// True if the TX_ENABLE value is no higher than stTXmostUnsec.
// Some filtering may be required even if this is true.
// TODO: allow cacheing in RAM for speed.
bool enableTrailingStatsPayload() { return(eeprom_read_byte((uint8_t *)EE_START_STATS_TX_ENABLE) <= stTXmostUnsec); }
#endif
#endif



// Coerce any ID bytes to valid values if unset (0xff) or if forced,
// by filling with valid values (0x80--0xfe) from decent entropy gathered on the fly.
// Will moan about invalid values and return false but not attempt to reset,
// eg in case underlying EEPROM cell is worn/failing.
// Returns true iff all values good.
bool ensureIDCreated(const bool force)
  {
  bool allGood = true;
  for(uint8_t i = 0; i < EE_LEN_ID; ++i)
    {
    uint8_t * const loc = i + (uint8_t *)EE_START_ID;
    if(force || (0xff == eeprom_read_byte(loc))) // Byte is unset or change is being forced.
        {
        serialPrintAndFlush(F("Setting ID byte "));
        serialPrintAndFlush(i);
        serialPrintAndFlush(' ');
        const uint8_t envNoise = ((i & 1) ? TemperatureC16.get() : ((uint8_t)AmbLight.getRaw()));
        for( ; ; )
          {
          // Try to make decently-randomised 'unique-ish' ID with mixture of sources.
          // Is not confidential, and will be transmitted in the clear.
          // System will typically not have been running long when this is invoked.
          const uint8_t newValue = 0x80 | (getSecureRandomByte() ^ envNoise);
          if(0xff == newValue) { continue; } // Reject unusable value.
          eeprom_smart_update_byte(loc, newValue);
          serialPrintAndFlush(newValue, HEX);
          break;
          }
        serialPrintlnAndFlush();
        }
    // Validate.
    const uint8_t v2 = eeprom_read_byte(loc);
    if(!validIDByte(v2))
        {
        allGood = false;
        serialPrintAndFlush(F("Invalid byte "));
        serialPrintAndFlush(i);
        serialPrintAndFlush(F(" ... "));
        serialPrintAndFlush(v2, HEX);
        serialPrintlnAndFlush();
        }
     }
  return(allGood);
  }


// Send core/common 'full' stats message.
//   * content contains data to be sent in the message; must be non-null
// Note that up to 7 bytes of payload is optimal for the CRC used.
// If successful, returns pointer to terminating 0xff at end of message.
// Returns null if failed (eg because of bad inputs or insufficient buffer space);
// part of the message may have have been written in this case and in particular the previous terminating 0xff may have been overwritten.
uint8_t *encodeFullStatsMessageCore(uint8_t * const buf, const uint8_t buflen, const stats_TX_level secLevel, const bool secureChannel,
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
    ((secureChannel || (secLevel <= stTXalwaysAll)) ? (content->occ & 3) : 0);
  *b++ = flagsHeader;
  // Now insert extra fields as flagged.
  if(content->containsAmbL)
    { *b++ = content->ambL; }
  // TODO: RH% etc

  // Finish off message by computing and appending the CRC and then terminating 0xff (and return pointer to 0xff).
  // Assumes that b now points just beyond the end of the payload.
  uint8_t crc = MESSAGING_FULL_STATS_CRC_INIT; // Initialisation.
  for(const uint8_t *p = buf; p < b; ) { crc = OTRadioLink::crc7_5B_update(crc, *p++); }
  *b++ = crc;
  *b = 0xff;
#if 0 && defined(DEBUG)
  if(b - buf != payloadLength + 1) { panic(F("msg gen err")); }
#endif
  return(b);
  }

// Decode core/common 'full' stats message.
// If successful returns pointer to next byte of message, ie just after full stats message decoded.
// Returns null if failed (eg because of corrupt message data) and state of 'content' result is undefined.
// This will avoid copying into the result data (possibly tainted) that has arrived at an inappropriate security level.
//   * content will contain data decoded from the message; must be non-null
const uint8_t *decodeFullStatsMessageCore(const uint8_t * const buf, const uint8_t buflen, const stats_TX_level secLevel, const bool secureChannel,
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
  if(MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS == (*b & MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MASK))
    {
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk msh");
    if(0 != (0x80 & b[1])) { return(NULL); } // Following byte does not have msb correctly cleared.
    extractTrailingMinimalStatsPayload(b, &(content->tempAndPower));
    b += 2;
    content->containsTempAndPower = true;
    }

  // If next header is flags then extract it.
  // FIXME: risk of misinterpretting CRC.
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk flg");
  if(MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS != (*b & MESSAGING_FULL_STATS_FLAGS_HEADER_MASK)) { return(NULL); } // Corrupt message.
  const uint8_t flagsHeader = *b++;
  content->occ = flagsHeader & 3;
  const bool containsAmbL = (0 != (flagsHeader & MESSAGING_FULL_STATS_FLAGS_HEADER_AMBL));
  if(containsAmbL)
    {
    const uint8_t ambL = *b++;
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk aml");
    if((0 == ambL) || (ambL == (uint8_t)0xff)) { return(NULL); } // Illegal value.
    content->ambL = ambL;
    content->containsAmbL = true;
    }

  // Finish off by computing and checking the CRC (and return pointer to just after CRC).
  // Assumes that b now points just beyond the end of the payload.
  uint8_t crc = MESSAGING_FULL_STATS_CRC_INIT; // Initialisation.
  for(const uint8_t *p = buf; p < b; ) { crc = OTRadioLink::crc7_5B_update(crc, *p++); }
//DEBUG_SERIAL_PRINTLN_FLASHSTRING(" chk CRC");
  if(crc != *b++) { return(NULL); } // Bad CRC.

  return(b); // Point to just after CRC.
  }





// Returns true unless the buffer clearly does not contain a possible valid raw JSON message.
// This message is expected to be one object wrapped in '{' and '}'
// and containing only ASCII printable/non-control characters in the range [32,126].
// The message must be no longer than MSG_JSON_MAX_LENGTH excluding trailing null.
// This only does a quick validation for egregious errors.
bool quickValidateRawSimpleJSONMessage(const char * const buf)
  {
  if('{' != buf[0]) { return(false); }
  // Scan up to maximum length for terminating '}'.
  const char *p = buf + 1;
  for(int i = 1; i < MSG_JSON_MAX_LENGTH; ++i)
    {
    const char c = *p++;
    // With a terminating '}' (followed by '\0') the message is superficially valid.
    if(('}' == c) && ('\0' == *p)) { return(true); }
    // Non-printable/control character makes the message invalid.
    if((c < 32) || (c > 126)) { return(false); }
    // Premature end of message renders it invalid.
    if('\0' == c) { return(false); }
    }
  return(false); // Bad (unterminated) message.
  }

// Adjusts null-terminated text JSON message up to MSG_JSON_MAX_LENGTH bytes (not counting trailing '\0') for TX.
// Sets high-bit on final '}' to make it unique, checking that all others are clear.
// Computes and returns 0x5B 7-bit CRC in range [0,127]
// or 0xff if the JSON message obviously invalid and should not be TXed.
// The CRC is initialised with the initial '{' character.
// NOTE: adjusts content in place.
#define adjustJSONMsgForTXAndComputeCRC_ERR 0xff // Error return value.
uint8_t adjustJSONMsgForTXAndComputeCRC(char * const bptr)
  {
  // Do initial quick validation before computing CRC, etc,
  if(!quickValidateRawSimpleJSONMessage(bptr)) { return(adjustJSONMsgForTXAndComputeCRC_ERR); }
//  if('{' != *bptr) { return(adjustJSONMsgForTXAndComputeCRC_ERR); }
  bool seenTrailingClosingBrace = false;
  uint8_t crc = '{';
  for(char *p = bptr; *++p; ) // Skip first char ('{'); loop until '\0'.
    {
    const char c = *p;
//    if(c & 0x80) { return(adjustJSONMsgForTXAndComputeCRC_ERR); } // No high-bits should be set!
    if(('}' == c) && ('\0' == *(p+1)))
      {
      seenTrailingClosingBrace = true;
      const char newC = c | 0x80;
      *p = newC; // Set high bit.
      crc = OTRadioLink::crc7_5B_update(crc, (uint8_t)newC); // Update CRC.
      return(crc);
      }
    crc = OTRadioLink::crc7_5B_update(crc, (uint8_t)c); // Update CRC.
    }
  if(!seenTrailingClosingBrace) { return(adjustJSONMsgForTXAndComputeCRC_ERR); } // Missing ending '}'.
  return(crc);
  }

// IF DEFINED: allow raw ASCII JSON message terminated with '}' and '\0' in adjustJSONMsgForRXAndCheckCRC().
// This has no error checking other than none of the values straying out of the printable range.
// Only intended as a transitional measure!
//#define ALLOW_RAW_JSON_RX

// Extract/adjust raw RXed putative JSON message up to MSG_JSON_ABS_MAX_LENGTH chars.
// Returns length including bounding '{' and '}' iff message superficially valid
// (essentially as checked by quickValidateRawSimpleJSONMessage() for an in-memory message)
// and that the CRC matches as computed by adjustJSONMsgForTXAndComputeCRC(),
// else returns -1.
// Strips the high-bit off the final '}' and replaces the CRC with a '\0'
// iff the message appeared OK
// to allow easy handling with string functions.
//  * bptr  pointer to first byte/char (which must be '{')
//  * bufLen  remaining bytes in buffer starting at bptr
// NOTE: adjusts content in place iff the message appears to be valid JSON.
#define adjustJSONMsgForRXAndCheckCRC_ERR -1
int8_t adjustJSONMsgForRXAndCheckCRC(char * const bptr, const uint8_t bufLen)
  {
  if('{' != *bptr) { return(adjustJSONMsgForRXAndCheckCRC_ERR); }
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("adjustJSONMsgForRXAndCheckCRC()... {");
#endif
  uint8_t crc = '{';
  // Scan up to maximum length for terminating '}'-with-high-bit.
  const uint8_t ml = min(MSG_JSON_ABS_MAX_LENGTH, bufLen);
  char *p = bptr + 1;
  for(int i = 1; i < ml; ++i)
    {
    const char c = *p++;
    crc = OTRadioLink::crc7_5B_update(crc, (uint8_t)c); // Update CRC.
#ifdef ALLOW_RAW_JSON_RX
    if(('}' == c) && ('\0' == *p))
      {
      // Return raw message as-is!
#if 0 && defined(DEBUG)
      DEBUG_SERIAL_PRINTLN_FLASHSTRING("} OK raw");
#endif
      return(i+1);
      }
#endif
    // With a terminating '}' (followed by '\0') the message is superficially valid.
    if((((char)('}' | 0x80)) == c) && (crc == (uint8_t)*p))
      {
      *(p - 1) = '}';
      *p = '\0'; // Null terminate for use as a text string.
#if 0 && defined(DEBUG)
      DEBUG_SERIAL_PRINTLN_FLASHSTRING("} OK with CRC");
#endif
      return(i+1);
      }
    // Non-printable/control character makes the message invalid.
    if((c < 32) || (c > 126))
      {
#if 0 && defined(DEBUG)
      DEBUG_SERIAL_PRINT_FLASHSTRING(" bad: char 0x");
      DEBUG_SERIAL_PRINTFMT(c, HEX);
      DEBUG_SERIAL_PRINTLN();
#endif
      return(adjustJSONMsgForRXAndCheckCRC_ERR);
      }
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINT(c);
#endif
    }
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING(" bad: unterminated");
#endif
  return(adjustJSONMsgForRXAndCheckCRC_ERR); // Bad (unterminated) message.
  }


// Print a single char to a bounded buffer; returns 1 if successful, else 0 if full.
size_t BufPrint::write(const uint8_t c)
  {
  if(size < capacity) { b[size++] = c; b[size] = '\0'; return(1); }
  return(0);
  }

// Returns true iff if a valid key for our subset of JSON.
// Rejects keys containing " or \ or any chars outside the range [32,126]
// to avoid having to escape anything.
bool isValidKey(const SimpleStatsKey key)
  {
  if(NULL == key) { return(false); } 
  for(const char *s = key; '\0' != *s; ++s)
    {
    const char c = *s;
    if((c < 32) || (c > 126) || ('"' == c) || ('\\' == c)) { return(false); }
    }
  return(true);
  }

// Returns pointer to stats tuple with given (non-NULL) key if present, else NULL.
// Does a simple linear search.
SimpleStatsRotationBase::DescValueTuple * SimpleStatsRotationBase::findByKey(const SimpleStatsKey key) const
  {
  for(int i = 0; i < nStats; ++i)
    {
    DescValueTuple * const p = stats + i;
    if(0 == strcmp(p->descriptor.key, key)) { return(p); }
    }
  return(NULL); // Not found.
  }

// Remove given stat and properties.
// True iff the item existed and was removed.
bool SimpleStatsRotationBase::remove(const SimpleStatsKey key)
  {
  DescValueTuple *p = findByKey(key);
  if(NULL == p) { return(false); }
  // If it needs to be removed and is not the last item
  // then move the last item down into its slot.
  const bool lastItem = ((p - stats) == (nStats - 1));
  if(!lastItem) { *p = stats[nStats-1]; }
  // We got rid of one!
  // TODO: possibly explicitly destroy/overwrite the removed one at the end.
  --nStats;
  return(true);
  }

// Create/update stat/key with specified descriptor/properties.
// The name is taken from the descriptor.
bool SimpleStatsRotationBase::putDescriptor(const GenericStatsDescriptor &descriptor)
  {
  if(!isValidKey(descriptor.key)) { return(false); }
  DescValueTuple *p = findByKey(descriptor.key);
  // If item already exists, update its properties.
  if(NULL != p) { p->descriptor = descriptor; }
  // Else if not yet at capacity then add this new item at the end.
  // Don't mark it as changed since its value may not yet be meaningful
  else if(nStats < capacity)
    {
    p = stats + (nStats++);
    *p = DescValueTuple();
    p->descriptor = descriptor;
    }
  // Else failed: no space to add a new item.
  else { return(false); }
  return(true); // OK
  }
    
// Create/update value for given stat/key.
// If properties not already set and not supplied then stat will get defaults.
// If descriptor is supplied then its key must match (and the descriptor will be copied).
// True if successful, false otherwise (eg capacity already reached).
bool SimpleStatsRotationBase::put(const SimpleStatsKey key, const int newValue)
  {
  if(!isValidKey(key))
    {
#if 0 && defined(DEBUG)
DEBUG_SERIAL_PRINT_FLASHSTRING("Bad JSON key ");
DEBUG_SERIAL_PRINT(key);
DEBUG_SERIAL_PRINTLN();
#endif
    return(false);
    }

  DescValueTuple *p = findByKey(key);
  // If item already exists, update it.
  if(NULL != p)
    {
    // Update the value and mark as changed if changed.
    if(p->value != newValue)
      {
      p->value = newValue;
      p->flags.changed = true;
      }
    // Update done!
    return(true);
    }

  // If not yet at capacity then add this new item at the end.
  // Mark it as changed to prioritise seeing it in the JSON output.
  if(nStats < capacity)
    {
    p = stats + (nStats++);
    *p = DescValueTuple();
    p->value = newValue;
    p->flags.changed = true;
    // Copy descriptor .
    p->descriptor = GenericStatsDescriptor(key);
    // Addition of new field done!
    return(true);
    }

#if 1 && defined(DEBUG)
DEBUG_SERIAL_PRINT_FLASHSTRING("Too many keys: ");
DEBUG_SERIAL_PRINT(key);
DEBUG_SERIAL_PRINTLN();
#endif
  return(false); // FAILED: full.
  }

#if defined(ALLOW_JSON_OUTPUT)
// Print an object field "name":value to the given buffer.
size_t SimpleStatsRotationBase::print(BufPrint &bp, const SimpleStatsRotationBase::DescValueTuple &s, bool &commaPending) const
  {
  size_t w = 0;
  if(commaPending) { w += bp.print(','); }
  w += bp.print('"');
  w += bp.print(s.descriptor.key); // Assumed not to need escaping in any way.
  w += bp.print('"');
  w += bp.print(':');
  w += bp.print(s.value);
  commaPending = true;
  return(w);
  }
#endif


// True if any changed values are pending (not yet written out).
bool SimpleStatsRotationBase::changedValue()
  {
  DescValueTuple const *p = stats + nStats;
  for(int8_t i = nStats; --i > 0; )
    { if((--p)->flags.changed) { return(true); } }
  return(false);
  }

#if defined(ALLOW_JSON_OUTPUT)
// Write stats in JSON format to provided buffer; returns a non-zero value if successful.
// Output starts with an "@" (ID) string field,
// then and optional count (if enabled),
// then the tracked stats as space permits,
// attempting to give priority to high-priority and changed values,
// allowing a potentially large set of values to my multiplexed over time
// into a constrained size/bandwidth message.
//
//   * buf  is the byte/char buffer to write the JSON to; never NULL
//   * bufSize is the capacity of the buffer starting at buf in bytes;
//       should be two (2) greater than the largest JSON output to be generates
//       to allow for a trailing null and one extra byte/char to ensure that the message is not over-large
//   * sensitivity  threshold below which (sensitive) stats will not be included; 0 means include everything
//   * maximise  if true attempt to maximise the number of stats squeezed into each frame,
//       potentially at the cost of signficant CPU time
//   * suppressClearChanged  if true then 'changed' flag for included fields is not cleared by this
//       allowing them to continue to be treated as higher priority
uint8_t SimpleStatsRotationBase::writeJSON(uint8_t *const buf, const uint8_t bufSize, const uint8_t sensitivity,
                                           const bool maximise, const bool suppressClearChanged)
  {
#ifdef DEBUG
  if(NULL == buf) { panic(0); } // Should never happen.
#endif
  // Minimum size is for {"@":""} plus null plus extra padding char/byte to check for overrun.
  if(bufSize < 10) { return(0); } // Failed.

  // Write/print to buffer passed in.
  BufPrint bp((char *)buf, bufSize);
  // True if field has been written and will need a ',' if another field is written.
  bool commaPending = false;

  // Start object.
  bp.print('{');

  // Write ID first.
  // If an explicit ID is supplied then use it
  // else compute it taking the housecode by preference if it is set.
  bp.print(F("\"@\":\""));

#ifdef USE_MODULE_FHT8VSIMPLE
  if(NULL != id) { bp.print(id); } // Value has to be 'safe' (eg no " nor \ in it).
  else
    {
    if(localFHT8VTRVEnabled())
      {
      const uint8_t hc1 = FHT8VGetHC1();
      const uint8_t hc2 = FHT8VGetHC2();
      bp.print(hexDigit(hc1 >> 4));
      bp.print(hexDigit(hc1));
      bp.print(hexDigit(hc2 >> 4));
      bp.print(hexDigit(hc2));
      }
    else
#endif
      {
      const uint8_t id1 = eeprom_read_byte(0 + (uint8_t *)EE_START_ID);
      const uint8_t id2 = eeprom_read_byte(1 + (uint8_t *)EE_START_ID);
      bp.print(hexDigit(id1 >> 4));
      bp.print(hexDigit(id1));
      bp.print(hexDigit(id2 >> 4));
      bp.print(hexDigit(id2));
      }
    }

  bp.print('"');
  commaPending = true;

  // Write count next iff enabled.
  if(c.enabled)
    {
    if(commaPending) { bp.print(','); commaPending = false; }
    bp.print(F("\"+\":"));
    bp.print(c.count);
    commaPending = true;
    }

  // Be prepared to rewind back to logical start of buffer.
  bp.setMark();

  bool gotHiPri = false;
  uint8_t hiPriIndex = 0;
  bool gotLoPri = false;
  uint8_t loPriIndex = 0;
  if(nStats != 0)
    {
    // High-pri/changed stats.
    // Only do this on a portion of runs to let 'normal' stats get a look-in.
    // This happens on even-numbered runs (eg including the first, typically).
    // Write at most one high-priority item.
    if(0 == (c.count & 1))
      {
      uint8_t next = lastTXedHiPri;
      for(int i = nStats; --i >= 0; )
        {
        // Wrap around the end of the stats.
        if(++next >= nStats) { next = 0; }
        // Skip stat if too sensitive to include in this output.
        DescValueTuple &s = stats[next];
        if(sensitivity > s.descriptor.sensitivity) { continue; }
        // Skip stat if neither changed nor high-priority.
        if(!s.descriptor.highPriority && !s.flags.changed) { continue; }
        // Found suitable stat to include in output.
        hiPriIndex = next;
        gotHiPri = true;
        // Add to JSON output.
        print(bp, s, commaPending);
        // If successful, ie still space for the closing "}\0" without running over-length
        // then mark this as a fall-back, else rewind and discard this item.
        if(bp.getSize() > bufSize - 3) { bp.rewind(); break; }
        else
          {
          bp.setMark();
          lastTXed = lastTXedHiPri = hiPriIndex;
          if(!suppressClearChanged) { stats[hiPriIndex].flags.changed = false; }
          break;
          }
        /* if(!maximise) */ { break; }
        }
      }

    // Insert normal-priority stats if space left.
    // Rotate through all eligible stats round-robin,
    // adding one to the end of the current message if possible,
    // checking first the item indexed after the previous one sent.
//    if(!gotHiPri)
      {
      uint8_t next = lastTXedLoPri;
      for(int i = nStats; --i >= 0; )
        {
        // Wrap around the end of the stats.
        if(++next >= nStats) { next = 0; }
        // Avoid re-transmitting the very last thing TXed unless there in only one item!
        if((lastTXed == next) && (nStats > 1)) { continue; }
        // Avoid transmitting the hi-pri item just sent if any.
        if(gotHiPri && (hiPriIndex == next)) { continue; }
        // Skip stat if too sensitive to include in this output.
        DescValueTuple &s = stats[next];
        if(sensitivity > s.descriptor.sensitivity) { continue; }
        // Found suitable stat to include in output.
        loPriIndex = next;
        gotLoPri = true;
        // Add to JSON output.
        print(bp, s, commaPending);
        // If successful then mark this as a fall-back, else rewind and discard this item.
        // If successful, ie still space for the closing "}\0" without running over-length
        // then mark this as a fall-back, else rewind and discard this item.
        if(bp.getSize() > bufSize - 3) { bp.rewind(); break; }
        else
          {
          bp.setMark();
          lastTXed = lastTXedLoPri = loPriIndex;
          if(!suppressClearChanged) { stats[loPriIndex].flags.changed = false; }
          }
        if(!maximise) { break; }
        }
      }
    }

  // TODO: maximise.

  // Terminate object.
  bp.print('}');
#if 0
  DEBUG_SERIAL_PRINT_FLASHSTRING("JSON: ");
  DEBUG_SERIAL_PRINT((char *)buf);
  DEBUG_SERIAL_PRINTLN();
#endif
//  if(w >= (size_t)(bufSize-1))
  if(bp.isFull())
    {
    // Overrun, so failed/aborted.
    // Shouldn't really be possible unless buffer far far too small.
    *buf = '\0';
    return(0);
    }

  // On successfully creating output, update some internal state including success count.
  ++c.count;

  return(bp.getSize()); // Success!
  }
#endif


#if (defined(ENABLE_BOILER_HUB) || defined(ALLOW_STATS_RX)) && defined(USE_MODULE_FHT8VSIMPLE) // Listen for calls for heat from remote valves...
#define LISTEN_FOR_FTp2_FS20_native
static void decodeAndHandleFTp2_FS20_native(Print *p, const bool secure, uint8_t * const msg, const uint8_t msglen)
{
  fht8v_msg_t command;
  uint8_t const *lastByte = msg+msglen-1;
  uint8_t const *trailer = FHT8VDecodeBitStream(msg, lastByte, &command);
  if(NULL != trailer)
    {
#if 0 && defined(DEBUG)
p->print("FS20 msg HC "); p->print(command.hc1); p->print(' '); p->println(command.hc2);
#endif
#if defined(ALLOW_STATS_RX) // Only look for the trailer if supported.
    // If whole FHT8V frame was OK then check if there is a valid stats trailer.

    // Check for 'core' stats trailer.
    if((trailer + FullStatsMessageCore_MAX_BYTES_ON_WIRE <= lastByte) && // Enough space for minimum-stats trailer.
       (MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS == (trailer[0] & MESSAGING_FULL_STATS_FLAGS_HEADER_MASK)))
      {
      FullStatsMessageCore_t content;
      const uint8_t *tail = decodeFullStatsMessageCore(trailer, lastByte-trailer+1, stTXalwaysAll, false, &content);
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
if(allGood) { p->println("FS20 ts"); }
#endif
        // If frame looks good then capture it.
        if(allGood) { recordCoreStats(false, &content); }
//            else { setLastRXErr(FHT8VRXErr_BAD_RX_SUBFRAME); }
        // TODO: record error with mismatched ID.
        }
      }
#if defined(ALLOW_MINIMAL_STATS_TXRX)
    // Check for minimum stats trailer.
    else if((trailer + MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES <= lastByte) && // Enough space for minimum-stats trailer.
       (MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS == (trailer[0] & MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MASK)))
      {
      if(verifyHeaderAndCRCForTrailingMinimalStatsPayload(trailer)) // Valid header and CRC.
        {
#if 0 && defined(DEBUG)
      p->println("FS20 MS RX"); // Just notes that a 'valve %' FS20 command has been overheard.
#endif
        trailingMinimalStatsPayload_t payload;
        extractTrailingMinimalStatsPayload(trailer, &payload);
        recordMinimalStats(true, command.hc1, command.hc2, &payload); // Record stats; local loopback is secure.
        }
      }
#endif
#endif

#if defined(ENABLE_BOILER_HUB)
    // Potentially accept as call for heat only if command is 0x26 (38).
    // Later filter on the valve being open enough for some water flow to be likely
    // (for individual valves, and in aggregate)
    // and the housecode being accepted.
    if(0x26 == command.command)
      {
      const uint16_t compoundHC = (command.hc1 << 8) | command.hc2;
#if 0 && defined(DEBUG)
      p->println("FS20 0x26 RX"); // Just notes that a 'valve %' FS20 command has been overheard.
#endif
      remoteCallForHeatRX(compoundHC, (0 == command.extension) ? 0 : (uint8_t) ((command.extension * 100) / 255));
      }
#endif
    }
  return;
  }
#endif




// Decode and handle inbound raw message.
// A message may contain trailing garbage at the end; the decoder/router should cope.
// The buffer may be reused when this returns,
// so a copy should be taken of anything that needs to be retained.
// If secure is true then this message arrived over a secure channel.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This routine is allowed to alter the contents of the buffer passed
// to help avoid extra copies, etc.
static void decodeAndHandleRawRXedMessage(Print *p, const bool secure, uint8_t * const msg, const uint8_t msglen)
  {
  // TODO: consider extracting hash of all message data (good/bad) and injecting into entropy pool.
#if 0 && defined(DEBUG)
  OTRadioLink::printRXMsg(p, msg, msglen);
#endif
  if(msglen < 2) { return; } // Too short to be useful, so ignore.
  switch(msg[0])
    {
    default:
    case OTRadioLink::FTp2_NONE:
      {
#if 0 && defined(DEBUG)
      p->print(F("!RX bad msg ")); OTRadioLink::printRXMsg(p, msg, min(msglen, 8));
#endif
      return;
      }

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
          const bool sy = NominalRadValve.isRecalibrating();
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
          if(RFM23B.sendRaw(txbuf, buflen)) // Send at default volume...  One going missing won't hurt that much.
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

#ifdef ALLOW_STATS_RX
    // Stand-alone stats message.
    case OTRadioLink::FTp2_FullStatsIDL: case OTRadioLink::FTp2_FullStatsIDH:
      {
#if 0 && defined(DEBUG)
DEBUG_SERIAL_PRINTLN_FLASHSTRING("Stats IDx");
#endif
      // May be binary stats frame, so attempt to decode...
      FullStatsMessageCore_t content;
      // (TODO: should reject non-secure messages when expecting secure ones...)
      const uint8_t *tail = decodeFullStatsMessageCore(msg, msglen, stTXalwaysAll, false, &content);
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
           recordCoreStats(false, &content);
           }
         }
      return;
      }
#endif

#if defined(LISTEN_FOR_FTp2_FS20_native) // Listen for calls for heat from remote valves...
    case OTRadioLink::FTp2_FS20_native:
      {
      decodeAndHandleFTp2_FS20_native(p, secure, msg, msglen);
      return;
      }
#endif

#ifdef ALLOW_STATS_RX
    case OTRadioLink::FTp2_JSONRaw:
      {
      if(-1 != adjustJSONMsgForRXAndCheckCRC((char *)msg, msglen))
        { recordJSONStats(secure, (const char *)msg); }
      return;
      }
#endif
    }
  }

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
  if(0 != rl->getRXMsgsQueued())
    {
    if(!neededWaking && wakeSerialIfNeeded && powerUpSerialIfDisabled()) { neededWaking = true; }
    uint8_t buf[64]; // FIXME: get correct size of buffer. // FIXME: move this large stack burden elsewhere?
    const uint8_t msglen = rl->getRXMsg(buf, sizeof(buf));
    // Don't currently regard anything arriving over the air as 'secure'.
    decodeAndHandleRawRXedMessage(p, false, buf, msglen);
    // Note that some work has been done.
    workDone = true;
    }

#ifdef ALLOW_STATS_RX
  // Look for binary-format message.
  FullStatsMessageCore_t stats;
  getLastCoreStats(&stats);
  if(stats.containsID)
    {
    if(!neededWaking && wakeSerialIfNeeded && powerUpSerialIfDisabled()) { neededWaking = true; }
    // Dump (remote) stats field '@<hexnodeID>;TnnCh[P;]'
    // where the T field shows temperature in C with a hex digit after the binary point indicated by C
    // and the optional P field indicates low power.
    p->print(LINE_START_CHAR_RSTATS);
    p->print((((uint16_t)stats.id0) << 8) | stats.id1, HEX);
    if(stats.containsTempAndPower)
      {
      p->print(F(";T"));
      p->print(stats.tempAndPower.tempC16 >> 4, DEC);
      p->print('C');
      p->print(stats.tempAndPower.tempC16 & 0xf, HEX);
      if(stats.tempAndPower.powerLow) { p->print(F(";P")); } // Insert power-low field if needed.
      }
    if(stats.containsAmbL)
      {
      p->print(F(";L"));
      p->print(stats.ambL);
      }
    if(0 != stats.occ)
      {
      p->print(F(";O"));
      p->print(stats.occ);
      }
    p->println();

    // Note that some work has been done.
    workDone = true;
    }

  // Check for JSON/text-format message if no binary message waiting.
  char buf[MSG_JSON_MAX_LENGTH+1]; // FIXME: move this large stack burden elsewhere?
  getLastJSONStats(buf);
  if('\0' != *buf)
    {
    if(!neededWaking && wakeSerialIfNeeded && powerUpSerialIfDisabled()) { neededWaking = true; }
    // Dump contained JSON message as-is at start of line.
    p->println(buf);
    // Note that some work has been done.
    workDone = true;
    }
#endif

  // Turn off serial at end, if this routine woke it.
  if(neededWaking) { flushSerialProductive(); powerDownSerial(); }
  return(workDone);
  }

