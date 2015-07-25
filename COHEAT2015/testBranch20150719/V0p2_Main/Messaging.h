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

 Messages be be sent in a number of formats,
 and may be sent stand-alone alone
 or piggybacked on (appended to) another message (eg on the end of an FS20 message).

 There may be a number of efficient binary formats,
 and a general limited JSON format.

 The JSON format is limited in length
 because of CPU/memory/radio limitations,
 and is constrained to ASCII-7 printable characters only (in range [32,126]).

 The messages on the wire are protected by a checksum or CRC.
 */

#ifndef MESSAGING_H
#define MESSAGING_H

#include "V0p2_Main.h"

#include <OTRadioLink.h>

#include "Security.h"
#include "Sensor.h"






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


// Moved to OTRadioLink::crc7_5B_update() in the OTRadioLink library.
///**Update 7-bit CRC with next byte; result always has top bit zero.
// * Polynomial 0x5B (1011011, Koopman) = (x+1)(x^6 + x^5 + x^3 + x^2 + 1) = 0x37 (0110111, Normal)
// * <p>
// * Should maybe initialise with 0x7f.
// * <p>
// * See: http://users.ece.cmu.edu/~koopman/roses/dsn04/koopman04_crc_poly_embedded.pdf
// * <p>
// * Should detect all 3-bit errors in up to 7 bytes of payload,
// * see: http://users.ece.cmu.edu/~koopman/crc/0x5b.txt
// * <p>
// * For 2 or 3 byte payloads this should have a Hamming distance of 4 and be within a factor of 2 of optimal error detection.
// */
//uint8_t crc7_5B_update(uint8_t crc, uint8_t datum);





// Key used for SimpleStatsRotation items.
typedef const char *SimpleStatsKey;

// Returns true iff if a valid key for our subset of JSON.
// Rejects keys containing " or \ or any chars outside the range [32,126]
// to avoid having to escape anything.
bool isValidKey(SimpleStatsKey key);

// Generic stats descripotor.
// Includes last value transmitted (to allow changed items to be sent selectively).
struct GenericStatsDescriptor
  {
    // Create generic (integer) stats instance.
    // The name must be a valid printable ASCII7 char [32,126] name
    // and the pointer too it must remain valid until this instance
    // and all copies have been disposed of (so is probably best a static string).
    // The default sensitivity is set to forbid transmission at all but minimum (0) leaf TX security settings.
    // By default the stat is normal priority.
    GenericStatsDescriptor(const char * const statKey,
                           const uint8_t statSensitivity = 1,
                           const bool statHighPriority = false)
      : key(statKey), sensitivity(statSensitivity), highPriority(statHighPriority)
    { }

    // Null-terminated short stat/key name.
    // Should generally be of form "x" where x is a single letter (case sensitive) for a unitless quantity,
    // or "x|u" where x is the name followed by a vertical bar and the units, eg "t|C" for temperature in Celsius.
    // This pointer must be to static storage, eg does not need lifetime management.
    SimpleStatsKey key;

    // Device sensitivity threshold has to be at or below this for stat to be sent.
    // The default is to allow the stat to be sent unless device is in default maximum privacy mode.
    uint8_t sensitivity;

    // If true, this statistic has high priority/importance and should be sent in all transmissions.
    bool highPriority;
  };


// Print to a bounded buffer.
class BufPrint : public Print
  {
  private:
    char * const b;
    const uint8_t capacity;
    uint8_t size;
    uint8_t mark;
  public:
    // Wrap around a buffer of size bufSize-1 chars and a trailing '\0'.
    // The buffer must be of at least size 1.
    // A buffer of size n can accommodate n-1 characters.
    BufPrint(char *buf, uint8_t bufSize) : b(buf), capacity(bufSize-1), size(0), mark(0) { buf[0] = '\0'; }
    virtual size_t write(uint8_t c);
    // True if buffer is completely full.
    bool isFull() const { return(size == capacity); }
    // Get size/chars already in the buffer, not including trailing '\0'.
    uint8_t getSize() const { return(size); }
    // Set to record good place to rewind to if necessary.
    void setMark() { mark = size; }
    // Rewind to previous good position, clearing newer text.
    void rewind() { size = mark; b[size] = '\0'; }
  };

// Manage sending of stats, possibly by rotation to keep frame sizes small.
// This will try to prioritise sending some key stats and sending of changed values.
// This is primarily expected to support JSON stats,
// but a hook for other formats such as binary may be provided.
// The template parameter is the maximum number of values to be sent in one frame,
// beyond the compulsory (nominally unique) node ID.
// The total number of statistics that can be handled is limited to 255.
// Not thread-/ISR- safe.
class SimpleStatsRotationBase
  {
  public:
    // Create/update value for given stat/key.
    // If properties not already set and not supplied then stat will get defaults.
    // If descriptor is supplied then its key must match (and the descriptor will be copied).
    // True if successful, false otherwise (eg capacity already reached).
    bool put(SimpleStatsKey key, int newValue);

    // Create/update value for the given sensor.
    // True if successful, false otherwise (eg capacity already reached).
    template <class T> bool put(const Sensor<T> &s) { return(put(s.tag(), s.get())); }

    // Create/update stat/key with specified descriptor/properties.
    // The name is taken from the descriptor.
    bool putDescriptor(const GenericStatsDescriptor &descriptor);

    // Remove given stat and properties.
    // True iff the item existed and was removed.
    bool remove(SimpleStatsKey key);

    // Set ID to given value, or null to track system ID; returns false if ID unsafe.
    // If null (the default) then dynamically generate the system ID,
    // eg house code as two bytes of hex if set, else first two bytes of binary ID as hex.
    // The lifetime of the pointed to string must exceed that of this instance.
    bool setID(const char * const _id)
      {
      if(isValidKey(_id)) { id = _id; return(true); }
      return(false); // Unsafe value.
      }

    // Get number of distinct fields/keys held.
    uint8_t size() { return(nStats); }

    // True if no stats items being managed.
    // May usefully inidicate that the structure needs to be populated.
    bool isEmpty() { return(0 == nStats); }

    // True if any changed values are pending (not yet written out).
    bool changedValue();

    // Iff true enable the count ("+") field and display immediately after the "@"/ID field.
    // The unsigned count increments as a successful write() operation completes,
    // and wraps after 63 (to limit space), potentially allowing easy detection of lost stats/transmissions.
    void enableCount(bool enable) { c.enabled = enable; }

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
    uint8_t writeJSON(uint8_t * const buf, const uint8_t bufSize, const uint8_t sensitivity,
                      const bool maximise = false, const bool suppressClearChanged = false);
#endif

  protected:
    struct DescValueTuple
      {
      DescValueTuple() : descriptor(NULL), value(0) { }

      // Descriptor of this stat.
      GenericStatsDescriptor descriptor;

      // Value.
      int value;

      // Various run-time flags.
      struct Flags
        {
        Flags() : changed(false) { }

        // Set true when the value is changed.
        // Set false when the value written out,
        // ie nominally transmitted to a remote listener,
        // to allow priority to be given to sending changed values.
        bool changed : 1;
//
//        // True if included in the current putative JSON output.
//        // Initial state unimportant.
//        bool thisRun : 1;
        } flags;
      };

    // Maximum capacity including overheads.
    const uint8_t capacity;

    // Returns pointer to stat tuple with given key if present, else NULL.
    DescValueTuple *findByKey(SimpleStatsKey key) const;

    // Initialise base with appropriate storage (non-null) and capacity knowledge.
    SimpleStatsRotationBase(DescValueTuple *_stats, const uint8_t _capacity)
      : capacity(_capacity), stats(_stats), nStats(0),
        lastTXed(~0), lastTXedLoPri(~0), lastTXedHiPri(~0), // Show the first item on the first pass...
        id(NULL)
      { }

  private:
    // Stats to be tracked and sent; never null.
    // The initial nStats slots are used.
    DescValueTuple * const stats;

    // Number of stats being managed (packed at the start of the stats[] array).
    uint8_t nStats;

    // Last stat index TXed; used to avoid resending very last item redundantly.
    // Coerced into range if necessary.
    uint8_t lastTXed;

    // Last normal stat index TXed.
    // Coerced into range if necessary.
    uint8_t lastTXedLoPri;

    // Last high-priority/changed stat index TXed.
    // Coerced into range if necessary.
    uint8_t lastTXedHiPri;

    // ID as null terminated string, or null to track system ID.
    // Used as string value of compulsory leading "@" key/field.
    // Can be changed at run-time.
    const char *id;

    // Small write counter (and flag to enable its display).
    // Helps to track lost transmissions of generated stats.
    // Count field increments after a successful write,
    // and wraps back to zero after 7 (to limit space on the wire);
    // is displayed immediately after the @/ID field when enabled,
    // and missing count values suggest a lost transmission somewhere.
    // Takes minimal space (1 byte).
    struct WriteCount
      {
      WriteCount() : enabled(0), count(0) { }
      uint8_t enabled : 1; // 1 if display of counter is enabled, else 0.
      uint8_t count : 3; // Increments on each successful write.
      } c;

#if defined(ALLOW_JSON_OUTPUT)
    // Print an object field "name":value to the given buffer.
    size_t print(BufPrint &bp, const DescValueTuple &dvt, bool &commaPending) const;
#endif
  };

template<uint8_t MaxStats>
class SimpleStatsRotation : public SimpleStatsRotationBase
  {
  private:
    // Stats to be tracked and sent; mandatory/priority items must be first.
    // A copy is taken of the user-supplied set of descriptions, preserving order.
    DescValueTuple stats[MaxStats];

  public:
    SimpleStatsRotation() : SimpleStatsRotationBase(stats, MaxStats) { }

    // Get capacity.
    uint8_t getCapacity() { return(MaxStats); }
  };



// Extract ASCII hex digit in range [0-9][a-f] (ie lowercase) from bottom 4 bits of argument.
// Eg, passing in 0xa (10) returns 'a'.
// The top 4 bits are ignored.
static inline char hexDigit(const uint8_t value) { const uint8_t v = 0xf&value; if(v<10) { return('0'+v); } return('a'+(v-10)); }
//static inline char hexDigit(const uint8_t value) { const uint8_t v = *("0123456789abcdef" + (0xf&value)); }
// Fill in the first two bytes of buf with the ASCII hex digits of the value passed.
// Eg, passing in a value 0x4e sets buf[0] to '4' and buf[1] to 'e'.
static inline void hexDigits(const uint8_t value, char * const buf) { buf[0] = hexDigit(value>>4); buf[1] = hexDigit(value); }


//// Write a simple raw (unencrypted) JSON message in one pass directly
//// using sprintf formatting into a MSG_JSON_MAX_LENGTH+1 buffer.
//// Caller must take care of all syntax, escaping issues, etc.
//// This message is expected to be one object wrapped in '{' and '}'
//// and containing only ASCII printable characters in the range [32,126].
//// Returns length of formatted message excluding trailing \0,
//// else a negative value in case of any error such as an overrun.
//// See: http://playground.arduino.cc/Main/Printf
//#ifdef F // check to see if F() macro is available
//int8_t sprintfRawSimpleJSONMessage(char *buf, const __FlashStringHelper *format, ...);
//#else
//int8_t sprintfRawSimpleJSONMessage(char *buf, const char *format, ...);
//#endif

// Returns true unless the buffer clearly does not contain a possible valid raw JSON message.
// This message is expected to be one object wrapped in '{' and '}'
// and containing only ASCII printable/non-control characters in the range [32,126].
// The message must be no longer than MSG_JSON_MAX_LENGTH excluding trailing null.
// This only does a quick validation for egregious errors.
bool quickValidateRawSimpleJSONMessage(const char *buf);

// Adjusts null-terminated text JSON message up to MSG_JSON_MAX_LENGTH bytes (not counting trailing '\0') for TX.
// Sets high-bit on final '}' to make it unique, checking that all others are clear.
// Computes and returns 0x5B 7-bit CRC in range [0,127]
// or 0xff if the JSON message obviously invalid and should not be TXed.
// The CRC is initialised with the initial '{' character.
// NOTE: adjusts content in place.
uint8_t adjustJSONMsgForTXAndComputeCRC(char *bptr);

// Extract/adjust raw RXed putative JSON message up to MSG_JSON_MAX_LENGTH chars.
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
int8_t adjustJSONMsgForRXAndCheckCRC(char *bptr, uint8_t bufLen);







// Returns true iff valid ID byte: must have the top bit set and not be 0xff.
static inline bool validIDByte(const uint8_t v) { return((0 != (0x80 & v)) && (0xff != v)); }

// Coerce any ID bytes to valid values if unset (0xff) or if forced,
// by filling with valid values (0x80--0xfe) from decent entropy.
// Will moan about invalid values and return false but not attempt to reset,
// eg in case underlying EEPROM cell is worn/failing.
// Returns true if all values good.
bool ensureIDCreated(const bool force = false);


// Minimal stats trailer
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


// Returns true if an unencrypted trailing static payload and similar (eg bare stats transmission) is permitted.
// True if the TX_ENABLE value is no higher than stTXmostUnsec.
// Some filtering may be required even if this is true.
#if defined(ALLOW_STATS_TX)
bool enableTrailingStatsPayload();
#else
#define enableTrailingStatsPayload() (false)
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
// From 2015/07/14 lsb is 0 and msb is SEC for compatibility with other messages on FS20 carrier.
//
//           BIT  7     6     5     4     3     2     1    0
// * byte 0 :  | SEC |  1  |  1  |  1  |  R0 | IDP | IDH | 0 |   SECure, header, 1x reserved 0 bit, ID Present, ID High
// WAS (pre 2015/07/14) * byte 0 :  |  0  |  1  |  1  |  1  |  R0 | IDP | IDH | SEC |   header, 1x reserved 0 bit, ID Present, ID High, SECure
#define MESSAGING_FULL_STATS_HEADER_MSBS 0x70
#define MESSAGING_FULL_STATS_HEADER_MASK 0x70
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
// Returns null if failed (eg because of bad inputs or insufficient buffer space).
// This will omit from transmission data not appropriate given the channel security and the stats_TX_level.
uint8_t *encodeFullStatsMessageCore(uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel,
    const FullStatsMessageCore_t *content);

// Decode core/common 'full' stats message.
// If successful returns pointer to next byte of message, ie just after full stats message decoded.
// Returns null if failed (eg because of corrupt message data) and state of 'content' result is undefined.
// This will avoid copying into the result data (possibly tainted) that has arrived at an inappropriate security level.
//   * content will contain data decoded from the message; must be non-null
const uint8_t *decodeFullStatsMessageCore(const uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel,
    FullStatsMessageCore_t *content);



#if defined(ALLOW_STATS_RX)
// Record core incoming stats; ID must be set as a minimum.
// If secure is true then this message arrived over a secure channel.
// Is thread/ISR-safe and fast.
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
void recordCoreStats(bool secure, const FullStatsMessageCore_t *stats);

// Gets (and clears) the last core stats record received, if any, filling in the stats struct.
// If no minimal stats record has been received since the last call then the ID will be absent and the rest undefined.
void getLastCoreStats(FullStatsMessageCore_t *stats);

// Get count of dropped inbound stats messages due to insufficient queue space.
uint16_t getInboundStatsQueueOverrun();
#else
#define recordCoreStats(secure, stats) {} // Do nothing.
#define getLastCoreStats(stats) {(stats)->containsID = false;} // Nothing to receive.
#define getInboundStatsQueueOverrun() 0 // No queue to overrun.
#endif

#if defined(ALLOW_STATS_RX) && defined(ALLOW_MINIMAL_STATS_TXRX)
// Record minimal incoming stats from given ID (if each byte < 100, then may be FHT8V-compatible house code).
// If secure is true then this message arrived over a secure channel.
// Is thread/ISR-safe and fast.
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
void recordMinimalStats(bool secure, uint8_t id0, uint8_t id1, const trailingMinimalStatsPayload_t *payload);
#else
#define recordMinimalStats(secure, id0, id1, payload) {} // Do nothing.
#endif


// Maximum length of JSON (text) message payload.
// A little bit less than a power of 2
// to enable packing along with other info.
// A little bit smaller than typical radio module frame buffers (eg RFM23B) of 64 bytes
// to allow other explicit preamble and postamble (such as CRC) to be added,
// and to allow time from final byte arriving to collect the data without overrun.
//
// Absolute maximum, eg with RFM23B / FS20 OOK carrier (and interrupt-serviced RX at hub).
#define MSG_JSON_ABS_MAX_LENGTH 55
// Typical/recommended maximum.
#define MSG_JSON_MAX_LENGTH 54
// Maximum for frames in 'secure' format, eg with authentication and encryption wrappers.
#define MSG_JSON_MAX_LENGTH_SECURE 32

#define MSG_JSON_LEADING_CHAR ('{') // This is for a JSON object { ... }.

#if defined(ALLOW_STATS_RX)
// Record stats (local or remote) in JSON (ie non-empty, {}-surrounded, \0-terminated text) format.
// If secure is true then this message arrived over a secure channel.
// The supplied buffer's content is not altered.
// The supplied JSON should already have been somewhat validated.
// Is thread/ISR-safe and moderately fast (though will require a data copy).
// May be backed by a finite-depth queue, even zero-length (ie discarding); usually holds just one item.
void recordJSONStats(bool secure, const char *json);

// Gets (and clears) the last JSON record received, if any,
// filling in the supplied buffer
// else leaving it starting with '\0' if none available.
// The buffer must be at least MSG_JSON_MAX_LENGTH+1 chars.
void getLastJSONStats(char *buf);
#else
#define recordJSONStats(secure, json) {} // Do nothing.
#define getLastJSONStats(buf) {*(buf) = '\0';} // Nothing to receive.
#endif

// Incrementally process I/O and queued messages, including from the radio link.
// This may mean printing them to Serial (which the passed Print object usually is),
// or adjusting system parameters,
// or relaying them elsewhere, for example.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This will attempt to process messages in such a way
// as to avoid internal overflows or other resource exhaustion.
void handleQueuedMessages(Print *p, bool wakeSerialIfNeeded, OTRadioLink::OTRadioLink *rl);

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
