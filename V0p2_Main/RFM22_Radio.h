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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2015
						   Deniz Erbilgin 2015
*/

/*
 RFM22/RFM23 wireless transceiver module support.
 */

#ifndef RFM22_H
#define RFM22_H

#include "V0p2_Main.h"

#include <OTRadioLink.h>
#include <OTRFM23BLink.h>
//#include <OTNullRadioLink.h>
#include <OTSIM900Link.h>

#ifdef USE_NULLRADIO
extern OTRadioLink::OTNullRadioLink RFM23B;
#elif defined(USE_MODULE_SIM900)
extern OTSIM900Link::OTSIM900Link RFM23B;
#elif defined(PIN_RFM_NIRQ)
extern OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS, PIN_RFM_NIRQ> RFM23B;
#else
extern OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS, -1> RFM23B;
#endif

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
void RFM22RawStatsTXFFTerminated(uint8_t *buf, bool doubleTX, bool RFM23BFramed = true);

// Adds the STATS_MSG_START_OFFSET preamble to enable reception by a remote RFM22B/RFM23B.
// Returns the first free byte after the preamble.
static inline uint8_t *RFM22RXPreambleAdd(uint8_t *buf)
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


#ifdef ALLOW_CC1_SUPPORT_RELAY
// Send a CC1 Alert message with this unit's house code via the RFM23B.
bool sendCC1AlertByRFM23B();
#endif

#endif

