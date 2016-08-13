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

Author(s) / Copyright (s): Damon Hart-Davis 2014
*/

/*
 Generic messaging support for OpenTRV.
 */

#include <util/atomic.h>

#include "Messaging.h"
#include "Ambient_Light_Sensor.h"
#include "EEPROM_Utils.h"
#include "PRNG.h"
#include "Power_Management.h"
#include "Security.h"
#include "Serial_IO.h"
#include "Temperature_Sensor.h"

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

/**Update 7-bit CRC with next byte; result always has top bit zero.
 * Polynomial 0x5B (1011011, Koopman) = (x+1)(x^6 + x^5 + x^3 + x^2 + 1) = 0x37 (0110111, Normal)
 * <p>
 * Should maybe initialise with 0x7f.
 * <p>
 * See: http://users.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
 * <p>
 * Should detect all 3-bit errors in up to 7 bytes of payload,
 * see: http://users.ece.cmu.edu/~koopman/crc/0x5b.txt
 * <p>
 * For 2 or 3 byte payloads this should have a Hamming distance of 4 and be within a factor of 2 of optimal error detection.
 * <p>
 * TODO: provide table-driven optimised alternative,
 *     eg see http://www.tty1.net/pycrc/index_en.html
 */
uint8_t crc7_5B_update(uint8_t crc, const uint8_t datum)
    {
    for(uint8_t i = 0x80; i != 0; i >>= 1)
        {
        bool bit = (0 != (crc & 0x40));
        if(0 != (datum & i)) { bit = !bit; }
        crc <<= 1;
        if(bit) { crc ^= 0x37; }
        }
    return(crc & 0x7f);
    }





// Return true if header/structure and CRC looks valid for (3-byte) buffered stats payload.
bool verifyHeaderAndCRCForTrailingMinimalStatsPayload(uint8_t const *const buf)
  {
  return((MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS == ((buf[0]) & MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MASK)) && // Plausible header.
         (0 == (buf[1] & 0x80)) && // Top bit is clear on this byte also.
         (buf[2] == crc7_5B_update(buf[0], buf[1]))); // CRC validates, top bit implicitly zero.
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
  buf[2] = crc7_5B_update(buf[0], buf[1]);
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


// Last core stats record received, or with no ID set if none.
// Should only be accessed under a lock for thread safety.
static /* volatile */ FullStatsMessageCore_t coreStats; // Start up showing no record set.

// Record minimal incoming stats from given ID (if each byte < 100, then may be FHT8V-compatible house code).
// Is thread/ISR-safe and fast.
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
void recordMinimalStats(const bool secure, const uint8_t id0, const uint8_t id1, const trailingMinimalStatsPayload_t * const payload)
  {
#if 0 && defined(DEBUG)
  if(NULL == payload) { panic(); }
#endif  // TODO
   ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    clearFullStatsMessageCore(&coreStats);
    coreStats.id0 = id0;
    coreStats.id1 = id1;
    coreStats.containsID = true;
    memcpy((void *)&coreStats.tempAndPower, payload, sizeof(coreStats.tempAndPower));
    coreStats.containsTempAndPower = true;
    }
  }

// Record core incoming stats; ID must be set as a minimum.
// Is thread/ISR-safe and fast.
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
void recordCoreStats(const bool secure, const FullStatsMessageCore_t * const stats)
  {
#if 0 && defined(DEBUG)
  if(NULL == payload) { panic(); }
#endif  // TODO
   if(!stats->containsID) { return; } // Ignore if no ID.
   ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    memcpy((void *)&coreStats, stats, sizeof(coreStats));
    }
  }

// Gets (and clears) the last core stats record received, if any, returning true and filling in the stats struct.
// If no minimal stats record has been received since the last call then the ID will be absent and the rest undefined.
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



#if defined(SUPPORT_TEMP_TX)
#if !defined(enableTrailingMinimalStatsPayload)
// Returns true if an unencrypted minimal trailing static payload and similar (eg bare stats transmission) is permitted.
// True if the TX_ENABLE value is no higher than stTXmostUnsec.
// Some filtering may be required even if this is true.
// TODO: allow cacheing in RAM for speed.
bool enableTrailingMinimalStatsPayload() { return(eeprom_read_byte((uint8_t *)EE_START_STATS_TX_ENABLE) <= stTXmostUnsec); }
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
        serialPrintAndFlush(F(" ... "));
        for( ; ; )
          {
          // Try to make decently-randomised 'unique-ish' ID with mixture of sources.
          // Is not confidential, and will be transmitted in the clear.
          // System will typically not have been running long when this is invoked.
          const uint8_t envNoise = ((i & 1) ? readTemperatureC16() : readAmbientLight());
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
        serialPrintAndFlush(F("Invalid ID byte "));
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
  for(const uint8_t *p = buf; p < b; ) { crc = crc7_5B_update(crc, *p++); }
  *b++ = crc;
  *b = 0xff;
#if 1 && defined(DEBUG)
  if(b - buf != payloadLength + 1) { panic(F("msg gen err")); }
#endif
  return(b);
  }

// Decode core/common 'full' stats message.
//   * content will contains data decoded from the message; must be non-null
// If successful returns pointer to nexte byte of message, ie just after full stats message decoded.
// Returns null if failed (eg because of corrupt message data) and state of 'content' result is undefined.
const uint8_t *decodeFullStatsMessageCore(const uint8_t * const buf, const uint8_t buflen, const stats_TX_level secLevel, const bool secureChannel,
    FullStatsMessageCore_t * const content)
  {
  if(NULL == buf) { return(NULL); } // Could be an assert/panic instead at a pinch.
  if(NULL == content) { return(NULL); } // Could be an assert/panic instead at a pinch.
  if(buflen < FullStatsMessageCore_MIN_BYTES_ON_WIRE) { return(NULL); } // Must be at least minimal message.

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
//#define MESSAGING_FULL_STATS_HEADER_MASK 0xf0
//#define MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT 4
//#define MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH 2
//#define MESSAGING_FULL_STATS_HEADER_BITS_ID_SECURE 1
  if(MESSAGING_FULL_STATS_HEADER_MSBS != (header & MESSAGING_FULL_STATS_HEADER_MASK)) { return(NULL); } // Bad header.
  if(0 != (header & MESSAGING_FULL_STATS_HEADER_BITS_ID_SECURE)) { return(NULL); } // TODO: cannot do secure messages yet.
  // Extract ID if present.
  const bool containsID = (0 != (header & MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT));
  if(containsID)
    {
    content->containsID = true;
    const uint8_t idHigh = ((0 != (header & MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH)) ? 0x80 : 0);
    content->id0 = *b++ | idHigh;
    content->id1 = *b++ | idHigh;
    }

  // If next header is temp/power then extract it, else must be the flags header.
  if(MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS == (*b & MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MASK))
    {
    if(0 != (0x80 & b[1])) { return(NULL); } // Following byte does not have msb correctly cleared.
    extractTrailingMinimalStatsPayload(b, &(content->tempAndPower));
    b += 2;
    content->containsTempAndPower = true;
    }

  // If next header is flags then extract it.
  // FIXME: risk of misinterpretting CRC.
  if(MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS != (*b & MESSAGING_FULL_STATS_FLAGS_HEADER_MASK)) { return(NULL); } // Corrupt message.
  const uint8_t flagsHeader = *b++;
  content->occ = flagsHeader & 3;
  const bool containsAmbL = (0 != (flagsHeader & MESSAGING_FULL_STATS_FLAGS_HEADER_AMBL));
  if(containsAmbL)
    {
    const uint8_t ambL = *b++;
    if((0 == ambL) || (ambL == (uint8_t)0xff)) { return(NULL); } // Illegal value.
    content->ambL = ambL;
    content->containsAmbL = true;
    }

  // Finish off by computing and checking the CRC (and return pointer to just after CRC).
  // Assumes that b now points just beyond the end of the payload.
  uint8_t crc = MESSAGING_FULL_STATS_CRC_INIT; // Initialisation.
  for(const uint8_t *p = buf; p < b; ) { crc = crc7_5B_update(crc, *p++); }
  if(crc != *b++) { return(NULL); } // Bad CRC.

  return(b); // Point to just after CRC.
  }


