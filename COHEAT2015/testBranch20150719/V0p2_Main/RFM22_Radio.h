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
*/

/*
 RFM22/RFM23 wireless transceiver module support.
 */

#ifndef RFM22_H
#define RFM22_H

#include "V0p2_Main.h"

#include <OTRadioLink.h>
#include <OTRFM23BLink.h>

extern OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS> RFM23B;

//// Minimal set-up of I/O (etc) after system power-up.
//// Performs a software reset and leaves the radio deselected and in a low-power and safe state.
//void RFM22PowerOnInit();

// Enter standby mode (consume least possible power but retain register contents).
// FIFO state and pending interrupts are cleared.
// Typical consumption in standby 450nA (cf 15nA when shut down, 8.5mA TUNE, 18--80mA RX/TX).
void RFM22ModeStandbyAndClearState();

//// Returns true iff RFM22 (or RFM23) appears to be correctly connected.
//bool RFM22CheckConnected();

//// Configure the radio from a list of register/value pairs in readonly PROGMEM/Flash, terminating with an 0xff register value.
//// NOTE: argument is not a pointer into SRAM, it is into PROGMEM!
//void RFM22RegisterBlockSetup(const uint8_t registerValues[][2]);

// Transmit contents of on-chip TX FIFO: caller should revert to low-power standby mode (etc) if required.
// Returns true if packet apparently sent correctly/fully.
// Does not clear TX FIFO (so possible to re-send immediately).
// Note: Reliability possibly helped by early move to 'tune' mode to work other than with default (4MHz) lowish PICAXE clock speeds.
bool RFM22TXFIFO();

// Clears the RFM22 TX FIFO and queues up ready to send via the TXFIFO the 0xff-terminated bytes starting at bptr.
// This routine does not change the command area.
void RFM22QueueCmdToFF(uint8_t *bptr);

// Put RFM22 into RX mode with given RX FIFO 'nearly-full' threshold and optional interrupts enabled.
void RFM22SetUpRX(uint8_t nearlyFullThreshold, bool syncInt, bool dataInt);

// Put RFM22 into standby, attempt to read specified number of bytes from FIFO to buffer.
// Leaves RFM22 in low-power standby mode.
// Trailing bytes (more than were actually sent) may be garbage.
void RFM22RXFIFO(uint8_t *buf, uint8_t bufSize);

// Get current RSSI.
// Only valid when in RX mode.
uint8_t RFM22RSSI();

// Read status (both registers) and clear interrupts.
// Status register 1 is returned in the top 8 bits, register 2 in the bottom 8 bits.
// Zero indicates no pending interrupts or other status flags set.
uint16_t RFM22ReadStatusBoth();

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
//   * isBinary  message type; if true then is nominally binary else text (JSON)
//   * doubleTX  double TX to increase chance of successful reception
// This will use whichever transmission medium/carrier/etc is available.
#define STATS_MSG_START_OFFSET (RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES)
#define STATS_MSG_MAX_LEN (64 - STATS_MSG_START_OFFSET)
void RFM22RawStatsTX(const bool isBinary, uint8_t * const buf, const bool doubleTX);


#endif

