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

#ifndef MESSAGING_H
#define MESSAGING_H

#include "V0p2_Main.h"

#include "Security.h"


// Note on CRCs
// ============
// See http://users.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
// Also: http://users.ece.cmu.edu/~koopman/crc/
// Also: http://www.ross.net/crc/crcpaper.html
// Also: http://en.wikipedia.org/wiki/Cyclic_redundancy_check
// 8-bit CRCs available in AVR (HD = Hamming distance):
//     Nickname | Within 1% of bound | Within 2x of bound | Same HD, but more than 2x bound | Worse HD than bound
// _crc8_ccitt_update() polynomial x^8 + x^2 + x + 1     (0x07/0xE0/0x83) aka ATM-8  | 53-119 | 18-52 | 10-17; 248-2048 | 8-9; 120-247 (not in Arduino 1.0.5)
// _crc_ibutton_update() polynomial: x^8 + x^5 + x^4 + 1 (0x31/0x8C/0x98) aka DOWCRC | 43-119 | 19-42 | 10-18; 248-2048 | 8-9; 120-247
// Provided:
// crc8_C2_update()                                      (..../..../0x97) aka C2     | 27-50; 52; 56-119 | 18-26; 51; 53-55 | 10-17; 248-2048 | 8-9; 120-247

// An implication is that for a 2-byte or 3-byte (16/24bit) message body
// either _crc8_ccitt_update() or _crc_ibutton_update() is as good as can be done
// which means that the supplied optimised implementations are probably good choices.

//// Update 'C2' 8-bit CRC polynomial with next byte.
//// Usually initialised with 0xff.
//// Should work well from 10--119 bits (2--~14 bytes); best 27-50, 52, 56-119 bits.
//// See: http://users.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
//// Also: http://en.wikipedia.org/wiki/Cyclic_redundancy_check
//uint8_t crc8_C2_update(uint8_t crc, uint8_t datum);

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
 */
uint8_t crc7_5B_update(uint8_t crc, uint8_t datum);


// Returns teo iff valid ID byte: must have the top bit set and not be 0xff.
static inline bool validIDByte(const uint8_t v) { return((0 != (0x80 & v)) && (0xff != v)); }

// Coerce any ID bytes to valid values if unset (0xff) or if forced,
// by filling with valid values (0x80--0xfe) from decent entropy.
// Will moan about invalid values and return false but not attempt to reset,
// eg in case underlying EEPROM cell is worn/failing.
// Returns true if all values good.
bool ensureIDCreated(const bool force = false);


// Minimal stats trailer
// =====================
// When already sending a message for some other reason
// it may be convenient to add a trailing minimal stats payload
// that will be ignored by the original recipient (eg FHT8V valve).
// Note that this never contains 0xff (would be taken to be a message terminator; one can be appended)
// and is not all zeros to help keep RF sync depending on the carrier.
// The minimal stats trailer payload contains the measured temperature and a power-level indicator.
// That is wrapped in an initial byte which positively indicates its presence
// and is unlikely to be confused with the main frame data or sync even if mis-framed,
// or data from the body of the main frame.
// This may also be nominally suitable for a frame on its own, ie with the main data elided.
// For an FHT8V frame, with sync bytes of 0xcc (and 0xaa before),
// and with the 1100 and 111000 encoding of the FHT8V data bits,
// A leading byte whose top bits are 010 should suffice if itself included in the check value.
// The trailer ends with a 7-bit CRC selected for reasonable performance on an 16-bit payload.
// NOTE: the CRC is calculated in an unusual way for speed
// (AT THE RISK OF BREAKING SOMETHING SUBTLE ABOUT THE EFFICACY OF THE CRC)
// with byte 0 used as the initial value and a single update with byte 1 to compute the final CRC.
// The full format is (MSB bits first):
//          BIT  7     6     5     4     3     2     1     0
//   byte 0 : |  0  |  1  |  0  |  PL |  T3 |  T2 |  T1 |  T0 |    header, power-low flag, temperature lsbits (C/16)
//   byte 1 : |  0  | T10 |  T9 |  T8 |  T7 |  T6 |  T5 |  T4 |    temperature msbits (C)
//   byte 2 : |  0  |  C6 |  C5 |  C5 |  C3 |  C2 |  C1 |  C0 |    7-bit CRC (crc7_5B_update)
// Temperature is in 1/16th of Celsius ranging from approx -20C (the bias value) to ~107C,
// which should cover everything from most external UK temperatures up to very hot DHW.

// Size of trailing minimal stats payload (including check values) on FHT8V frame in bytes.
#define MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES 3
#define MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS 0x40
#define MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MASK 0xe0
#define MESSAGING_TRAILING_MINIMAL_STATS_TEMP_BIAS (-(20<<4)) // C*16 offset bottom of scale / subtracted from 0C.

// Raw (not-as-transmitted) representation of minimal stats payload header.
// Should be compact in memory.
typedef struct trailingMinimalStatsPayload
  {
  int16_t tempC16 : 15; // Signed fixed-point temperature in C with 4 bits after the binary point.
  bool powerLow : 1; // True if power/battery is low.
  } trailingMinimalStatsPayload_t;

// Store minimal stats payload into (2-byte) buffer from payload struct (without CRC); values are coerced to fit as necessary..
//   * payload  must be non-null
// Used for minimal and full packet forms,
void writeTrailingMinimalStatsPayloadBody(uint8_t *buf, const trailingMinimalStatsPayload_t *payload);
// Store minimal stats payload into (3-byte) buffer from payload struct and append CRC; values are coerced to fit as necessary..
//   * payload  must be non-null
void writeTrailingMinimalStatsPayload(uint8_t *buf, const trailingMinimalStatsPayload_t *payload);
// Return true if header/structure and CRC looks valid for (3-byte) buffered stats payload.
bool verifyHeaderAndCRCForTrailingMinimalStatsPayload(uint8_t const *buf);
// Extract payload from valid (3-byte) header+payload+CRC into payload struct; only 2 bytes are actually read.
// Input data must already have been validated.
void extractTrailingMinimalStatsPayload(const uint8_t *buf, trailingMinimalStatsPayload_t *payload);


// Returns true if an unencrypted minimal trailing static payload and similar (eg bare stats transmission) is permitted.
// True if the TX_ENABLE value is no higher than stTXmostUnsec.
// Some filtering may be required even if this is true.
#if defined(SUPPORT_TEMP_TX)
bool enableTrailingMinimalStatsPayload();
#else
#define enableTrailingMinimalStatsPayload() (false)
#endif



// Full Stats Message (short ID)
// =============================
// Can be sent on its own or as a trailer for (say) an FHT8V message.
// Can be recognised by the msbits of the leading (header) byte
// Nominally allows support for security (auth/enc),
// some predefined environmental stats beyond temperature,
// and the ability for an arbitrary ASCII payload.
// Note that the message frame never contains 0xff (would be taken to be a message terminator; one can be appended)
// and is avoids runs of more than about two bytes of all zeros to help keep RF sync depending on the carrier.
// The ID is two bytes (though effectively 15 bits since the top bits of both bytes must match)
// and is never encrypted.
// If IDH is 1, the top bits of both header bytes is 1, else both are 0 and may be FS20-compatible 'house codes'.
// The CRC is computed in a conventional way over the header and all data bytes
// starting with an all-ones initialisation value, and is never encrypted.
// The ID plus the CRC may be used in an ACK from the hub to semi-uniquely identify this frame,
// with additional secure/authed data for secure links to avoid replay attacks/ambiguity.
// (Note that if secure transmission is expected a recipient must generally ignore all frames with SEC==0.)
//
//           BIT  7     6     5     4     3     2     1     0
// * byte 0 :  |  0  |  1  |  1  |  1  |  R0 | IDP | IDH | SEC |   header, 1x reserved 0 bit, ID Present, ID High, SECure
#define MESSAGING_FULL_STATS_HEADER_MSBS 0x70
#define MESSAGING_FULL_STATS_HEADER_MASK 0xf0
#define MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT 4
#define MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH 2
#define MESSAGING_FULL_STATS_HEADER_BITS_ID_SECURE 1

// ?ID: node ID if present (IDP==1)
//             |  0  |            ID0                          |   7 lsbits of first ID byte, unencrypted
//             |  0  |            ID1                          |   7 lsbits of second ID byte, unencrypted

// SECURITY HEADER
// IF SEC BIT IS 1 THEN ONE OR MORE BYTES INSERTED HERE, TBD, EG INCLUDING LENGTH / NONCE.
// IF SEC BIT IS 1 then all bytes between here and the security trailer are encrypted and/or authenticated.

// Temperature and power section, optional, encoded exactly as for minimal stats payload.
//   byte b :  |  0  |  1  |  0  |  PL |  T3 |  T2 |  T1 |  T0 |   header, power-low flag, temperature lsbits (C/16)
//   byte b+1: |  0  | T10 |  T9 |  T8 |  T7 |  T6 |  T5 |  T4 |   temperature msbits (C)

// Flags indicating which optional elements are present:
// AMBient Light, Relative Humidity %.
// OC1/OC2 = Occupancy: 00 not disclosed, 01 not occupied, 10 possibly occupied, 11 probably occupied.
// IF EXT is 1 a futher flags byte follows.
// ALWAYS has to be present and has a distinct header from the preceeding  temp/power header to allow t/p to be omitted unambiguously.
// * byte b+2: |  0  |  1  |  1  | EXT | ABML| RH% | OC1 | OC2 |   EXTension-follows flag, plus optional section flags.
#define MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS 0x60
#define MESSAGING_FULL_STATS_FLAGS_HEADER_MASK 0xe0
#define MESSAGING_FULL_STATS_FLAGS_HEADER_AMBL 8
#define MESSAGING_FULL_STATS_FLAGS_HEADER_RHP 4
// If EXT = 1:
// Call For Heat, RX High (meaning TX hub can probably turn down power), (SenML) ASCII PayLoad
//   byte b+3: |  0  |  R1 |  R0 |  R0 |  R0 | CFH | RXH | APL |   1x reserved 1 bit, 4x reserved 0 bit, plus optional section flags.

// ?CFH: Call For Heat section, if present.
// May be used as a keep-alive and/or to abruptly stop calling for heat.
// Time in seconds + 1 that this node call for heat for (0--253, encoded as 0x01--0xfe to avoid 0 and 0xff).
// If this field is present and zero (encoded as 0x01) it immediately cancels any current call for heat from this node.
//             |  CFH seconds + 1, range [0,253]               |

// ?ABML: AMBient Light section, if present.
// Lighting level dark--bright 0--253, encoded as 0x01--0xfe to avoid 0 and 0xff).
// This may not be linear, and may not achieve full dynamic range.
// This may be adjusted for typical lighting levels encountered by the node over >= 24h.
//             |  Ambient light level range [0,253]            |

// ?RH%: Relative Humidity %, if present.
// Offset by 1 (encoded range [1,101]) so that a zero byte is never sent.
//             |  0  | RH% [0,100] + 1                         |

// SECURITY TRAILER
// IF SEC BIT IS 1 THEN ZERO OR MORE BYTES INSERTED HERE, TBD.

#define MESSAGING_FULL_STATS_CRC_INIT 0x7f // Initialisation value for CRC.
// *           |  0  |  C6 |  C5 |  C5 |  C3 |  C2 |  C1 |  C0 |    7-bit CRC (crc7_5B_update), unencrypted


// Representation of core/common elements of a 'full' stats message.
// Flags indicate which fields are actually present.
// All-zeros initialiation ensures no fields marked as present.
// Designed to be reasonably compact in memory.
typedef struct FullStatsMessageCore
  {
  bool containsID : 1; // Keep as first field.

  bool containsTempAndPower : 1;
  bool containsAmbL : 1;

  // Node ID (mandatory, 2 bytes)
  uint8_t id0, id1; // ID bytes must share msbit value.

  // Temperature and low-power (optional, 2 bytes)
  trailingMinimalStatsPayload_t tempAndPower;

  // Ambient lighting level; zero means absent, ~0 is invalid (Optional, 1 byte)
  uint8_t ambL;

  // Occupancy; 00 not disclosed, 01 probably, 10 possibly, 11 not occupied recently.
  uint8_t occ : 2;
  } FullStatsMessageCore_t;

// Maximum size on wire including trailing CRC of core of FullStatsMessage.  TX message buffer should be one larger for trailing 0xff.
#define FullStatsMessageCore_MAX_BYTES_ON_WIRE 8
// Minimum size on wire including trailing CRC of core of FullStatsMessage.  TX message buffer should be one larger for trailing 0xff.
#define FullStatsMessageCore_MIN_BYTES_ON_WIRE 3

// Clear a FullStatsMessageCore_t, also indicating no optional fields present.
static inline void clearFullStatsMessageCore(FullStatsMessageCore_t *const p) { memset(p, 0, sizeof(FullStatsMessageCore_t)); }


// Send core/common 'full' stats message.
// Note that up to 7 bytes of payload is optimal for the CRC used.
// If successful, returns pointer to terminating 0xff at end of message.
// Returns null if failed (eg because of bad inputs or insufficient buffer space).
// This will omit from transmission data not appropriate given the channel security and the stats_TX_level.
uint8_t *encodeFullStatsMessageCore(uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel,
    const FullStatsMessageCore_t *content);

// Decode core/common 'full' stats message.
// If successful returns pointer to nexte byte of message, ie just after full stats message decoded.
// Returns null if failed (eg because of corrupt message data) and state of 'content' result is undefined.
// This will avoid coping into the result data (possibly tainted) that has arrived at an inappropriate security level.
const uint8_t *decodeFullStatsMessageCore(const uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel,
    FullStatsMessageCore_t *content);


// Record minimal incoming stats from given ID (if each byte < 100, then may be FHT8V-compatible house code).
// Is thread/ISR-safe and fast.
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
void recordMinimalStats(bool secure, uint8_t id0, uint8_t id1, const trailingMinimalStatsPayload_t *payload);

// Record core incoming stats; ID must be set as a minimum.
// Is thread/ISR-safe and fast.
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
void recordCoreStats(bool secure, const FullStatsMessageCore_t *stats);

// Gets (and clears) the last core stats record received, if any, returning true and filling in the stats struct.
// If no minimal stats record has been received since the last call then the ID will be absent and the rest undefined.
void getLastCoreStats(FullStatsMessageCore_t *stats);


#endif


