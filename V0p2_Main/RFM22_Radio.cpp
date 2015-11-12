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


#ifdef USE_NULLRADIO
OTRadioLink::OTNullRadioLink RFM23B;
#elif defined(USE_MODULE_SIM900)
OTSIM900Link::OTSIM900Link RFM23B(A3, (uint8_t)A2, (uint8_t)8, (uint8_t)5); // FIXME change pins around from (A3, 6, 9, 8)
#elif defined(PIN_RFM_NIRQ)
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
//   * doubleTX  double TX to increase chance of successful reception
//   * RFM23BfriendlyPremable  if true then add an extra preamble
//     to allow RFM23B-based receiver to RX this
// This will use whichever transmission medium/carrier/etc is available.
//#define STATS_MSG_START_OFFSET (RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES)
//#define STATS_MSG_MAX_LEN (64 - STATS_MSG_START_OFFSET)
void RFM22RawStatsTXFFTerminated(uint8_t * const buf, const bool doubleTX, bool RFM23BFramed)
  {
  if(RFM23BFramed) RFM22RXPreambleAdd(buf);	// Only needed for RFM23B. This should be made more clear when refactoring
  const uint8_t buflen = OTRadioLink::frameLenFFTerminated(buf);
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("buflen=");
    DEBUG_SERIAL_PRINT(buflen);
    DEBUG_SERIAL_PRINTLN();
#endif // DEBUG
  //if(!RFM23B.sendRaw(buf, buflen, 0, (doubleTX ? OTRadioLink::OTRadioLink::TXmax : OTRadioLink::OTRadioLink::TXnormal)))
  if(!RFM23B.queueToSend(buf, buflen, 0, (doubleTX ? OTRadioLink::OTRadioLink::TXmax : OTRadioLink::OTRadioLink::TXnormal)))
    {
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("!TX failed");
#endif
    } // DEBUG
  //DEBUG_SERIAL_PRINTLN_FLASHSTRING("RS");
  }



#ifdef ALLOW_CC1_SUPPORT_RELAY
#include <OTProtocolCC.h>
#include "FHT8V_Wireless_Rad_Valve.h"
// Send a CC1 Alert message with this unit's house code via the RFM23B.
bool sendCC1AlertByRFM23B()
  {
  OTProtocolCC::CC1Alert a = OTProtocolCC::CC1Alert::make(FHT8VGetHC1(), FHT8VGetHC2());
  if(a.isValid()) // Might be invalid if house codes are, eg if house codes not set.
    {
    uint8_t txbuf[STATS_MSG_START_OFFSET + OTProtocolCC::CC1Alert::primary_frame_bytes+1]; // More than large enough for preamble + sync + alert message.
    uint8_t *const bptr = RFM22RXPreambleAdd(txbuf);
    const uint8_t bodylen = a.encodeSimple(bptr, sizeof(txbuf) - STATS_MSG_START_OFFSET, true);
    const uint8_t buflen = STATS_MSG_START_OFFSET + bodylen;
#if 0 && defined(DEBUG)
OTRadioLink::printRXMsg(p, txbuf, buflen);
#endif
    // Send loud since the hub may be relatively far away,
    // there is no 'ACK', and these messages should not be sent very often.
    // Should be consistent with automatically-generated alerts to help with diagnosis.
    return(RFM23B.sendRaw(txbuf, buflen, 0, OTRadioLink::OTRadioLink::TXmax));
    }
  return(false); // Failed.
  }
#endif
