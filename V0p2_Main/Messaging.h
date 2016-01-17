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

 Messages be be sent in a number of formats,
 and may be sent stand-alone alone
 or piggybacked on (appended to) another message (eg on the end of an FS20 message).

 There may be a number of efficient binary formats,
 and a general limited JSON format.

 The JSON format is limited in length
 because of CPU/memory/radio limitations,
 and is constrained to ASCII-7 printable characters only (in range [32,126]).

 The messages on the wire are protected by a checksum or CRC.

 NOTE: when communicating to a host over serial, leading punctuation characters are significant,
 and output is line-oriented:
 
  '!' introduces an error.
  '?' introduces a warning.
  '=' introduces a local status message.
  '>' is a CLI prompt.
  '@' introduces a translated (to ASCII7) binary status messages.
  '{' introduces a raw JSON (map) message.
  '+<msgtype> ' introduces a relayed/decoded message of the given message type.  Note the space.

 See Serial_LineType_InitChar in the base library.
 */

#ifndef MESSAGING_H
#define MESSAGING_H

#include "V0p2_Main.h"

#include <OTV0p2Base.h>
#include <OTRadioLink.h>


#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Minimal stats trailer (for devices supporting FS20 encoding only)
// =====================
// When already sending an (FS20/FHT8V) message for some other reason
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
//   * payload  must be non-NULL
// Used for minimal and full packet forms,
void writeTrailingMinimalStatsPayloadBody(uint8_t *buf, const trailingMinimalStatsPayload_t *payload);
// Store minimal stats payload into (3-byte) buffer from payload struct and append CRC; values are coerced to fit as necessary..
//   * payload  must be non-NULL
void writeTrailingMinimalStatsPayload(uint8_t *buf, const trailingMinimalStatsPayload_t *payload);
// Return true if header/structure and CRC looks valid for (3-byte) buffered stats payload.
bool verifyHeaderAndCRCForTrailingMinimalStatsPayload(uint8_t const *buf);
// Extract payload from valid (3-byte) header+payload+CRC into payload struct; only 2 bytes are actually read.
// Input data must already have been validated.
void extractTrailingMinimalStatsPayload(const uint8_t *buf, trailingMinimalStatsPayload_t *payload);
#endif // ENABLE_FS20_ENCODING_SUPPORT


#ifdef ENABLE_FS20_ENCODING_SUPPORT
// Full Stats Message (short ID) (for devices supporting FS20 encoding only)
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
// From 2015/07/14 lsb is 0 and msb is SEC for compatibility with other messages on FS20 carrier.
//
//           BIT  7     6     5     4     3     2     1    0
// * byte 0 :  | SEC |  1  |  1  |  1  |  R0 | IDP | IDH | 0 |   SECure, header, 1x reserved 0 bit, ID Present, ID High
// WAS (pre 2015/07/14) * byte 0 :  |  0  |  1  |  1  |  1  |  R0 | IDP | IDH | SEC |   header, 1x reserved 0 bit, ID Present, ID High, SECure
#define MESSAGING_FULL_STATS_HEADER_MSBS 0x70
#define MESSAGING_FULL_STATS_HEADER_MASK 0x70
#define MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT 4
#define MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH 2
#define MESSAGING_FULL_STATS_HEADER_BITS_ID_SECURE 0x80

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
// IF EXT is 1 a further flags byte follows.
// ALWAYS has to be present and has a distinct header from the preceding temp/power header to allow t/p to be omitted unambiguously.
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
// Lighting level dark--bright 1--254, encoded as 0x01--0xfe to avoid 0 and 0xff).
// This may not be linear, and may not achieve full dynamic range.
// This may be adjusted for typical lighting levels encountered by the node over >= 24h.
//             |  Ambient light level range [1,254]            |

// ?RH%: Relative Humidity %, if present.
// Offset by 1 (encoded range [1,101]) so that a zero byte is never sent.
//             |  0  | RH% [0,100] + 1                         |

// SECURITY TRAILER
// IF SEC BIT IS 1 THEN ZERO OR MORE BYTES INSERTED HERE, TBD.

#define MESSAGING_FULL_STATS_CRC_INIT 0x7f // Initialisation value for CRC.
// *           |  0  |  C6 |  C5 |  C5 |  C3 |  C2 |  C1 |  C0 |    7-bit CRC (crc7_5B_update), unencrypted

// Representation of core/common elements of a 'full' stats message.
// Flags indicate which fields are actually present.
// All-zeros initialisation ensures no fields marked as present.
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
// Returns NULL if failed (eg because of bad inputs or insufficient buffer space).
// This will omit from transmission data not appropriate given the channel security and the stats_TX_level.
uint8_t *encodeFullStatsMessageCore(uint8_t *buf, uint8_t buflen, OTV0P2BASE::stats_TX_level secLevel, bool secureChannel,
    const FullStatsMessageCore_t *content);

#if defined(ENABLE_RADIO_RX)
// Decode core/common 'full' stats message.
// If successful returns pointer to next byte of message, ie just after full stats message decoded.
// Returns NULL if failed (eg because of corrupt/insufficient message data) and state of 'content' result is undefined.
// This will avoid copying into the result data (possibly tainted) that has arrived at an inappropriate security level.
//   * content will contain data decoded from the message; must be non-NULL
const uint8_t *decodeFullStatsMessageCore(const uint8_t *buf, uint8_t buflen, OTV0P2BASE::stats_TX_level secLevel, bool secureChannel,
    FullStatsMessageCore_t *content);
#endif

// Send (valid) core binary stats to specified print channel, followed by "\r\n".
// This does NOT attempt to flush output nor wait after writing.
void outputCoreStats(Print *p, bool secure, const FullStatsMessageCore_t *stats);

// Send (valid) minimal binary stats to specified print channel, followed by "\r\n".
// This does NOT attempt to flush output nor wait after writing.
void outputMinimalStats(Print *p, bool secure, uint8_t id0, uint8_t id1, const trailingMinimalStatsPayload_t *stats);
#endif // ENABLE_FS20_ENCODING_SUPPORT



// Returns true if an unencrypted trailing static payload and similar (eg bare stats transmission) is permitted.
// True if the TX_ENABLE value is no higher than stTXmostUnsec.
// Some filtering may be required even if this is true.
#if defined(ALLOW_STATS_TX)
#if !defined(CONFIG_ALWAYS_TX_ALL_STATS)
// TODO: allow cacheing in RAM for speed.
inline bool enableTrailingStatsPayload() { return(OTV0P2BASE::getStatsTXLevel() <= OTV0P2BASE::stTXmostUnsec); }
#else
#define enableTrailingStatsPayload() (true) // Always allow at least some stats to be TXed.
#endif // !defined(CONFIG_ALWAYS_TX_ALL_STATS)
#else
#define enableTrailingStatsPayload() (false)
#endif


#if defined(ENABLE_RADIO_RX)
// Incrementally poll and process I/O and queued messages, including from the radio link.
// Returns true if some work was done.
// This may mean printing them to Serial (which the passed Print object usually is),
// or adjusting system parameters,
// or relaying them elsewhere, for example.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This will attempt to process messages in such a way
// as to avoid internal overflows or other resource exhaustion.
bool handleQueuedMessages(Print *p, bool wakeSerialIfNeeded, OTRadioLink::OTRadioLink *rl);
#else
#define handleQueuedMessages(p, wakeSerialIfNeeded, rl)
#endif


#ifdef ENABLE_BOILER_HUB
// FUNCTIONALITY REQUIRED (NOT SUPPLIED) BY MESSAGING.
// Raw notification of received call for heat from remote (eg FHT8V) unit.
// This form has a 16-bit ID (eg FHT8V housecode) and percent-open value [0,100].
// Note that this may include 0 percent values for a remote unit explicitly confirming
// that is is not, or has stopped, calling for heat (eg instead of replying on a timeout).
// This is not filtered, and can be delivered at any time from RX data, from a non-ISR thread.
// Does not have to be thread-/ISR- safe.
void remoteCallForHeatRX(uint16_t id, uint8_t percentOpen);
#endif




/* DHD20150423: messaging thoughts including integrity, see: http://www.earth.org.uk/note-on-IoT-security.html#app1

The design aim is to allow transmission of (optionally secure) telemetry from low-power sensor nodes
over a number of alternate backhaul media such as one-way packet-based ISM radios.

Assume that the leaf end is a low-powered CPU and so the code interface and implementation has to be simple,
and with minimal support from some hardware for features such as encryption.

Assume that the messaging maximum possible frame size will generally be 64 bytes or less,
and may vary significantly with the options chosen below, especially if encryption is added.
Assume that some of the data carried may be sensitive, eg privacy related or for driving actuators.

Assume that some implementations can/will not run below a specific integrity level, eg with data checksums/CRCs.

Assume that the raw messaging transport is by default:
  * one way
  * lossy
  * noisy
  * bandwidth limited (low bit rate and/or (say) frames/day capped) and/or expensive per bit or frame
  * real-time but possibly with significant latency
  * overhearable, eg over ISM radio or similar.

(Some variants like TinyHAN allow two-way flows, and others may be radically different such as tunneled in HTTPS over a LAN.)

Have one or more backhaul layers available at run-time leaf (with superset at concentrator)
with some constant capabilities, ie that can be checked/selected at pref at compile time, such as:

  * Frame formats that can be carried on this channel (1 or more):
      * JSON object {...} (compact ASCII7 subset, only printable chars 32--126 ie with no linebreaks or other control).
      * Whitened binary (with no 0x00 or 0xff bytes), so limited-length runs of either bit, and both values possible as delimiters.
      * Structured binary (as interpreted by underlying channel eg with TinyHAN).
      * Pure binary.

  * Ability to mark some frames as 'important' (bool), eg containing critical or changed values, with extra delivery effort (eg double TX or FEC).

  * Maximum data integrity protection available from the channel (enum / small int):
      * CHECK: (required) simple frame check value applied and verified, eg typically 7--16 bit check sum or CRC, or in the underlying medium.
      * SEQ: (optional) above plus small frame sequence number.
      * AUTH: (optional) above plus crypto-based authentication.
      * ENC: (optional) above plus encryption (eg AES-GCM or EAX).
      * ENCHIGH: (optional) above with enhanced security (eg longer keys and/or IVs etc) at cost of frame size and CPU.
    (Data receiver should usually check data for semantic/syntactic integrity etc also, especially if a low level is used here.)

    [DHD20150409: note that all current OpenTRV traffic is effectively sent at level CHECK.]
    [DHD20150409: dropped NONE at Jeremy P suggestion to reduce complexity.]

All systems should support at least JSON object and whitened binary formats with a simple (CHECK) integrity check.
(Note that JSON formats are assumed NOT optimal in bandwidth terms,
and should generally not be used for prolonged production deployments (use a binary format),
but the underlying medium may be able to make some optimisations such as simple compression on the wire.)

All systems with privacy-related data must support encryption (ENC),
and/or have the ability selectively not to send sensitive data,
and/or the underlying backhaul must be able to guarantee ENC-level integrity itself (eg tunnelling over HTTPS or VPN).

At run time (and possibly at compile time) it must be possible to discover the maximum data frame size possible
with the selected transmission parameters.

Note that for higher integrity levels suitably-sized keys may have to have been pre-shared for example,
and any modes not supported by the concentrator may have to be removed to the 'available' list.

At run time it should be possible to specify above parameters with each frame to send from leaf,
and those parameters plus some associated values (eg sequence numbers/range) should be recoverable.
Data that fails integrity checks is in normal circumstances not available nor are crypto keys used,
though parameters such as algorithm and strength may be).

Note that key, IV, etc lengths that are acceptable in 2015 may prove inadequate to future;
to some extent that is implicitly dealt with outside this definition by the key-sharing mechanism,
but frame size limits may ultimately limit available security.

See also:
http://blog.cryptographyengineering.com/2011/11/how-not-to-use-symmetric-encryption.html
http://crypto.stackexchange.com/questions/7951/aesctrhmac-encryption-and-authentication-on-an-arduino
http://www.cs.berkeley.edu/~jaein/papers/cs294_9_paper_fec.pdf
http://packetpushers.net/ipsec-bandwidth-overhead-using-aes/
http://nordsecmob.aalto.fi/en/publications/theses_2008/thesis_gabrielalimon_tkk.pdf
http://www.iacr.org/workshops/fse2010/content/slide/Fast%20Software%20AES%20Encryption.pdf
http://tools.ietf.org/html/rfc4106
Public domain uNaCl crypto for AtMega: http://munacl.cryptojedi.org/ and https://cryptojedi.org/papers/avrnacl-20130514.pdf
https://github.com/kokke/tiny-AES128-C (public domain)
http://csrc.nist.gov/publications/nistpubs/800-38a/addendum-to-nist_sp800-38A.pdf
 
 */


#endif
