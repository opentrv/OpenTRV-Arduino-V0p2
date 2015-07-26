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
                           Gary Gladman 2015
                           Mike Stirling 2013
*/

/*
 RFM22/RFM23 wireless transceiver module support.
 */

#include "RFM22_Radio.h"

#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "Power_Management.h"
#include "Serial_IO.h"

#ifdef PIN_RFM_NIRQ
OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS, PIN_RFM_NIRQ> RFM23B;
#else
OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS, -1> RFM23B;
#endif

// RFM22 is apparently SPI mode 0 for Arduino library pov.


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
//#define STATS_MSG_START_OFFSET (RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES)
//#define STATS_MSG_MAX_LEN (64 - STATS_MSG_START_OFFSET)
void RFM22RawStatsTX(const bool isBinary, uint8_t * const buf, const bool doubleTX)
  {
  // Write in the preamble/sync bytes.
  uint8_t *bptr = buf;
  // Start with RFM23-friendly preamble which ends with with the aacccccc sync word.
  memset(bptr, RFM22_PREAMBLE_BYTE, RFM22_PREAMBLE_BYTES);
  bptr += RFM22_PREAMBLE_BYTES;
  memset(bptr, RFM22_SYNC_BYTE, RFM22_SYNC_MIN_BYTES);
  bptr += RFM22_SYNC_MIN_BYTES;

//  // TODO: put in listen before TX to reduce collisions (CSMA).
//  // Send message starting with preamble.
//  // Assume RFM22/23 support for now.
//  RFM22QueueCmdToFF(buf);
//  RFM22TXFIFO(); // Send it!  Approx 1.6ms/byte.
//  if(doubleTX)
//    {
//    nap(WDTO_15MS);
//    RFM22TXFIFO(); // Re-send it!
//    }
  const uint8_t buflen = OTRadioLink::frameLenFFTerminated(buf);
  RFM23B.sendRaw(buf, buflen);
  //DEBUG_SERIAL_PRINTLN_FLASHSTRING("RS");
  }

