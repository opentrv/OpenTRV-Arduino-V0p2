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
                           Deniz Erbilgin 2015
*/

/*
 Generic messaging and radio/comms support for OpenTRV.

 Security, integrity, etc, see: http://www.earth.org.uk/note-on-IoT-security.html#app1

 Messages be be sent in a number of formats,
 and may be sent stand-alone alone
 or piggybacked on (appended to) another message (eg on the end of an FS20 message).

 There may be a number of efficient binary formats,
 and a general limited/compact JSON format.
 */

#ifndef MESSAGING_H
#define MESSAGING_H

#include "V0p2_Main.h"

#include <OTV0p2Base.h>
#include <OTRadioLink.h>
#include <OTRFM23BLink.h>
//#include <OTNullRadioLink.h>
#include <OTSIM900Link.h>
#include <OTRN2483Link.h>



#ifdef ENABLE_RADIO_PRIMARY_MODULE
extern OTRadioLink::OTRadioLink &PrimaryRadio;
#endif // ENABLE_RADIO_PRIMARY_MODULE

#ifdef ENABLE_RADIO_SECONDARY_MODULE
extern OTRadioLink::OTRadioLink &SecondaryRadio;
#endif // RADIO_SECONDARY_MODULE_TYPE


//#if defined(ENABLE_RADIO_RFM23B) && defined(PIN_RFM_NIRQ) && defined(DEBUG) // Expose for debugging...
//extern OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS, PIN_RFM_NIRQ> RFM23B;
//#endif

#ifdef ENABLE_RADIO_SIM900
//For EEPROM:
//- Set the first field of SIM900LinkConfig to true.
//- The configs are stored as \0 terminated strings starting at 0x300.
//- You can program the eeprom using ./OTRadioLink/dev/utils/sim900eepromWrite.ino

extern const OTSIM900Link::OTSIM900LinkConfig_t SIM900Config;
#endif // ENABLE_RADIO_SIM900


#define RFM22_PREAMBLE_BYTE 0xaa // Preamble byte for RFM22/23 reception.
#define RFM22_PREAMBLE_MIN_BYTES 4 // Minimum number of preamble bytes for reception.
#define RFM22_PREAMBLE_BYTES 5 // Recommended number of preamble bytes for reliable reception.
#define RFM22_SYNC_BYTE 0xcc // Sync-word trailing byte (with FHT8V primarily).
#define RFM22_SYNC_MIN_BYTES 3 // Minimum number of sync bytes.

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
#define STATS_MSG_START_OFFSET (RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES)
#define STATS_MSG_MAX_LEN (64 - STATS_MSG_START_OFFSET)
#if defined(ENABLE_RFM23B_FS20_RAW_PREAMBLE)
void RFM22RawStatsTXFFTerminated(uint8_t *buf, bool doubleTX, bool RFM23BFramed = true);
#endif
#if defined(ENABLE_RFM23B_FS20_RAW_PREAMBLE)
// Adds the STATS_MSG_START_OFFSET preamble to enable reception by a remote RFM22B/RFM23B.
// Returns the first free byte after the preamble.
inline uint8_t *RFM22RXPreambleAdd(uint8_t *buf)
  {
  // Start with RFM23-friendly preamble which ends with with the aacccccc sync word.
  memset(buf, RFM22_PREAMBLE_BYTE, RFM22_PREAMBLE_BYTES);
  buf += RFM22_PREAMBLE_BYTES;
  // Send the sync bytes.
  memset(buf, RFM22_SYNC_BYTE, RFM22_SYNC_MIN_BYTES);
  buf += RFM22_SYNC_MIN_BYTES;
  // Return the adjusted pointer.
  return(buf);
  }
#endif


#ifdef ALLOW_CC1_SUPPORT_RELAY
// Send a CC1 Alert message with this unit's house code via the RFM23B.
bool sendCC1AlertByRFM23B();
#endif


// Returns true if an unencrypted trailing static payload and similar (eg bare stats transmission) is permitted.
// True if the TX_ENABLE value is no higher than stTXmostUnsec.
// Some filtering may be required even if this is true.
#if defined(ENABLE_STATS_TX)
#if !defined(ENABLE_ALWAYS_TX_ALL_STATS)
// TODO: allow cacheing in RAM for speed.
inline bool enableTrailingStatsPayload() { return(OTV0P2BASE::getStatsTXLevel() <= OTV0P2BASE::stTXmostUnsec); }
#else
#define enableTrailingStatsPayload() (true) // Always allow at least some stats to be TXed.
#endif // !defined(ENABLE_ALWAYS_TX_ALL_STATS)
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
// as to avoid internal overflows or other resource exhaustion,
// which may mean deferring work at certain times
// such as the end of minor cycle.
// The Print object pointer must not be NULL.
bool handleQueuedMessages(Print *p, bool wakeSerialIfNeeded, OTRadioLink::OTRadioLink *rl);
#else
#define handleQueuedMessages(p, wakeSerialIfNeeded, rl)
#endif


#endif
