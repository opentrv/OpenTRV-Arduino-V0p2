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


OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS> RFM23B;

// RFM22 is apparently SPI mode 0 for Arduino library pov.


#define RFM22REG_INT_STATUS1 3 // Interrupt status register 1.
#define RFM22REG_INT_STATUS2 4 // Interrupt status register 2.
#define RFM22REG_INT_ENABLE1 5 // Interrupt enable register 1.
#define RFM22REG_INT_ENABLE2 6 // Interrupt enable register 2.
#define RFM22REG_OP_CTRL1 7 // Operation and control register 1.
#define RFM22REG_OP_CTRL1_SWRES 0x80 // Software reset (at write) in OP_CTRL1.
#define RFM22REG_OP_CTRL2 8 // Operation and control register 2.
#define RFM22REG_RSSI 0x26 // RSSI.
#define RFM22REG_RSSI1 0x28 // Antenna 1 diversity / RSSI.
#define RFM22REG_RSSI2 0x29 // Antenna 2 diversity / RSSI.
#define RFM22REG_TX_POWER $0x6d // Transmit power.
#define RFM22REG_RX_FIFO_CTRL 0x7e // RX FIFO control.
#define RFM22REG_FIFO  0x7f // TX FIFO on write, RX FIFO on read.


// Allow validation of RFM22/RFM23 device and SPI connection to it.
#define RFM22_SUPPORTED_DEVICE_TYPE 0x08 // Read from register 0.
#define RFM22_SUPPORTED_DEVICE_VERSION 0x06 // Read from register 1.

// Internal routines to enable/disable RFM22 on the the SPI bus.
#define _RFM22_SELECT() fastDigitalWrite(PIN_SPI_nSS, LOW) // Select/enable RFM22.
#define _RFM22_DESELECT() fastDigitalWrite(PIN_SPI_nSS, HIGH) // Deselect/disable RFM22.

// Write/read one byte over SPI...
// SPI must already be configured and running.
static uint8_t _RFM22_io(const uint8_t data)
  {
  SPDR = data;
  // TODO: convert from busy-wait to sleep, at least in a standby mode, if likely longer than 10s of uS.
  // At lowest SPI clock prescale (x2) this is likely to spin for ~16 CPU cycles (8 bits each taking 2 cycles).
  while (!(SPSR & _BV(SPIF))) { }
  return(SPDR);
  }

// Write one byte over SPI (ignoring the value read back).
// SPI must already be configured and running.
static void _RFM22_wr(const uint8_t data)
  {
  SPDR = data;
  // TODO: convert from busy-wait to sleep, at least in a standby mode, if likely longer than 10s of uS.
  // At lowest SPI clock prescale (x2) this is likely to spin for ~16 CPU cycles (8 bits each taking 2 cycles).
  while (!(SPSR & _BV(SPIF))) { }
  }

// Write to 8-bit register on RFM22.
// SPI must already be configured and running.
static void _RFM22WriteReg8Bit(const uint8_t addr, const uint8_t val)
  {
  _RFM22_SELECT();
  _RFM22_wr(addr | 0x80); // Force to write.
  _RFM22_wr(val);
  _RFM22_DESELECT();
  }

// Write 0 to 16-bit register on RFM22 as burst.
// SPI must already be configured and running.
static void _RFM22WriteReg16Bit0(const uint8_t addr)
  {
  _RFM22_SELECT();
  _RFM22_wr(addr | 0x80); // Force to write.
  _RFM22_wr(0);
  _RFM22_wr(0);
  _RFM22_DESELECT();
  }

// Read from 8-bit register on RFM22.
// SPI must already be configured and running.
static uint8_t _RFM22ReadReg8Bit(const uint8_t addr)
  {
  _RFM22_SELECT();
  _RFM22_io(addr & 0x7f); // Force to read.
  const uint8_t result = _RFM22_io(0); // Dummy value...
  _RFM22_DESELECT();
  return(result);
  }

// Read from 16-bit big-endian register pair.
// The result has the first (lower-numbered) register in the most significant byte.
static uint16_t _RFM22ReadReg16Bit(const uint8_t addr)
{
  _RFM22_SELECT();
  _RFM22_io(addr & 0x7f); // Force to read.
  uint16_t result = ((uint16_t)_RFM22_io(0)) << 8;
  result |= ((uint16_t)_RFM22_io(0));
  _RFM22_DESELECT();
  return(result);
}

// Enter standby mode.
// SPI must already be configured and running.
static void _RFM22ModeStandby()
  {
  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL1, 0);
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Sb");
#endif
  }

// Enter transmit mode (and send any packet queued up in the TX FIFO).
// SPI must already be configured and running.
static void _RFM22ModeTX()
  {
  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL1, 9);
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Tx");
#endif
  } // TXON | XTON

// Enter receive mode.
// SPI must already be configured and running.
static void _RFM22ModeRX()
  {
  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL1, 5);
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Rx");
#endif
  } // RXON | XTON

// Read/discard status (both registers) to clear interrupts.
// SPI must already be configured and running.
static void _RFM22ClearInterrupts()
  {
//  _RFM22WriteReg8Bit(RFM22REG_INT_STATUS1, 0);
//  _RFM22WriteReg8Bit(RFM22REG_INT_STATUS2, 0); // TODO: combine in burst write with previous...
  _RFM22WriteReg16Bit0(RFM22REG_INT_STATUS1);
  }

// Enter standby mode (consume least possible power but retain register contents).
// FIFO state and pending interrupts are cleared.
// Typical consumption in standby 450nA (cf 15nA when shut down, 8.5mA TUNE, 18--80mA RX/TX).
void RFM22ModeStandbyAndClearState()
  {
  const bool neededEnable = OTV0P2BASE::powerUpSPIIfDisabled();
  _RFM22ModeStandby();
  // Clear RX and TX FIFOs simultaneously.
  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL2, 3); // FFCLRRX | FFCLRTX
  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL2, 0); // Needs both writes to clear.
  // Disable all interrupts.
//  _RFM22WriteReg8Bit(RFM22REG_INT_ENABLE1, 0);
//  _RFM22WriteReg8Bit(RFM22REG_INT_ENABLE2, 0); // TODO: combine in burst write with previous...
  _RFM22WriteReg16Bit0(RFM22REG_INT_ENABLE1);
  // Clear any interrupts already/still pending...
  _RFM22ClearInterrupts();
  //_RFM22WriteReg8Bit(0xd, 0x1f); // Drive GPIO2 to ground as output... (Move to general register settings.)
  if(neededEnable) { OTV0P2BASE::powerDownSPI(); }
  // DEBUG_SERIAL_PRINTLN_FLASHSTRING("SCS");
}

// Read status (both registers) and clear interrupts.
// Status register 1 is returned in the top 8 bits, register 2 in the bottom 8 bits.
// Zero indicates no pending interrupts or other status flags set.
uint16_t RFM22ReadStatusBoth()
  {
  const bool neededEnable = OTV0P2BASE::powerUpSPIIfDisabled();
  const uint16_t result = _RFM22ReadReg16Bit(RFM22REG_INT_STATUS1);
  if(neededEnable) { OTV0P2BASE::powerDownSPI(); }
  return(result);
  }

// Get current RSSI.
// Only valid when in RX mode.
uint8_t RFM22RSSI()
  {
  const bool neededEnable = OTV0P2BASE::powerUpSPIIfDisabled();
  const uint8_t rssi = _RFM22ReadReg8Bit(RFM22REG_RSSI);
  if(neededEnable) { OTV0P2BASE::powerDownSPI(); }
  return(rssi);
  }

// MOVED TO OTRFM23BLink
//// Minimal set-up of I/O (etc) after system power-up.
//// Performs a software reset and leaves the radio deselected and in a low-power and safe state.
//// Will power up SPI if needed.
//void RFM22PowerOnInit()
//  {
//#if 0 && defined(DEBUG)
//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("RFM22 reset...");
//#endif
//  const bool neededEnable = powerUpSPIIfDisabled();
//  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL1, RFM22REG_OP_CTRL1_SWRES);
//  _RFM22ModeStandby();
//  if(neededEnable) { powerDownSPI(); }
//  }

//// Returns true iff RFM22 (or RFM23) appears to be correctly connected.
//bool RFM22CheckConnected()
//  {
//  const bool neededEnable = OTV0P2BASE::powerUpSPIIfDisabled();
//  bool isOK = false;
//  const uint8_t rType = _RFM22ReadReg8Bit(0); // May read as 0 if not connected at all.
//  if(RFM22_SUPPORTED_DEVICE_TYPE == rType)
//    {
//    const uint8_t rVersion = _RFM22ReadReg8Bit(1);
//    if(RFM22_SUPPORTED_DEVICE_VERSION == rVersion)
//      { isOK = true; }
//#if 0 && defined(DEBUG)
//    else
//      {
//      DEBUG_SERIAL_PRINT_FLASHSTRING("RFM22 bad version: ");
//      DEBUG_SERIAL_PRINTFMT(rVersion, HEX);
//      DEBUG_SERIAL_PRINTLN();
//      }
//#endif
//    }
//#if 0 && defined(DEBUG)
//  else
//    {
//    DEBUG_SERIAL_PRINT_FLASHSTRING("RFM22 bad type: ");
//    DEBUG_SERIAL_PRINTFMT(rType, HEX);
//    DEBUG_SERIAL_PRINTLN();
//    }
//#endif
//#if 1 && defined(DEBUG)
//  if(!isOK) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("RFM22 bad"); }
//#endif
//  if(neededEnable) { OTV0P2BASE::powerDownSPI(); }
//  return(isOK);
//  }


//// Configure the radio from a list of register/value pairs in readonly PROGMEM/Flash, terminating with an 0xff register value.
//// NOTE: argument is not a pointer into SRAM, it is into PROGMEM!
//// Could optimise case where multiple values are for successive RFM22 registers by using burst write.
//void RFM22RegisterBlockSetup(const uint8_t registerValues[][2])
//  {
//  const bool neededEnable = OTV0P2BASE::powerUpSPIIfDisabled();
//  for( ; ; )
//    {
//    const uint8_t reg = pgm_read_byte(&(registerValues[0][0]));
//    const uint8_t val = pgm_read_byte(&(registerValues[0][1]));
//    if(0xff == reg) { break; }
//#if 0 && defined(DEBUG)
//    DEBUG_SERIAL_PRINT_FLASHSTRING("RFM22 reg 0x");
//    DEBUG_SERIAL_PRINTFMT(reg, HEX);
//    DEBUG_SERIAL_PRINT_FLASHSTRING(" = 0x");
//    DEBUG_SERIAL_PRINTFMT(val, HEX);
//    DEBUG_SERIAL_PRINTLN();
//#endif
//    _RFM22WriteReg8Bit(reg, val);
//    ++registerValues;
//    }
//  if(neededEnable) { OTV0P2BASE::powerDownSPI(); }
//  }

//// Transmit contents of on-chip TX FIFO: caller should revert to low-power standby mode (etc) if required.
//// Returns true if packet apparently sent correctly/fully.
//// Does not clear TX FIFO (so possible to re-send immediately).
//// Note: Reliability possibly helped by early move to 'tune' mode to work other than with default (4MHz) lowish PICAXE clock speeds.
//bool RFM22TXFIFO()
//  {
//  const bool neededEnable = OTV0P2BASE::powerUpSPIIfDisabled();
//  //gosub RFM22ModeTune ; Warm up the PLL for quick transition to TX below (and ensure NOT in TX mode).
//  // Enable interrupt on packet send ONLY.
//  _RFM22WriteReg8Bit(RFM22REG_INT_ENABLE1, 4);
//  _RFM22WriteReg8Bit(RFM22REG_INT_ENABLE2, 0);
//  _RFM22ClearInterrupts();
//  _RFM22ModeTX(); // Enable TX mode and transmit TX FIFO contents.
//
//  // Repeatedly nap until packet sent, with upper bound of ~120ms on TX time in case there is a problem.
//  // TX time is ~1.6ms per byte at 5000bps.
//  bool result = false; // Usual case is success.
//  for(int8_t i = 8; --i >= 0; )
//    {
//    nap(WDTO_15MS); // Sleep in low power mode for a short time waiting for bits to be sent...
//    const uint8_t status = _RFM22ReadReg8Bit(RFM22REG_INT_STATUS1); // TODO: could use nIRQ instead if available.
//    if(status & 4) { result = true; break; } // Packet sent!
//    }
//
//  if(neededEnable) { OTV0P2BASE::powerDownSPI(); }
//  return(result);
//  }

//// Clear TX FIFO.
//// SPI must already be configured and running.
//static void _RFM22ClearTXFIFO()
//  {
//  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL2, 1); // FFCLRTX
//  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL2, 0);
//  }

//// Clears the RFM22 TX FIFO and queues up ready to send via the TXFIFO the 0xff-terminated bytes starting at bptr.
//// This routine does not change the command area.
//// This uses an efficient burst write.
//// For DEBUG can abort after (over-)filling the 64-byte FIFO at no extra cost with a check before spinning waiting for SPI byte to be sent.
//void RFM22QueueCmdToFF(uint8_t *bptr)
//  {
//#if 0 && defined(DEBUG)
//  if(0 == *bptr) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("RFM22QueueCmdToFF: buffer uninitialised"); panic(); }
//#endif
//  const bool neededEnable = OTV0P2BASE::powerUpSPIIfDisabled();
//  // Clear the TX FIFO.
//  _RFM22ClearTXFIFO();
//  _RFM22_SELECT();
//  _RFM22_wr(RFM22REG_FIFO | 0x80); // Start burst write to TX FIFO.
//  uint8_t val;
//#if 0 && defined(DEBUG)
//  for(int8_t i = 64; ((uint8_t)0xff) != (val = *bptr++); )
//    {
//    // DEBUG_SERIAL_PRINTFMT(val, HEX); DEBUG_SERIAL_PRINTLN();
//    SPDR = val;
//    if(--i < 0) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("RFM22QueueCmdToFF: buffer unterminated"); panic(); }
//    while (!(SPSR & _BV(SPIF))) { }
//    }
//#else
//  while((uint8_t)0xff != (val = *bptr++)) { _RFM22_wr(val); }
//#endif
//  _RFM22_DESELECT();
//  if(neededEnable) { OTV0P2BASE::powerDownSPI(); }
//  }

// Put RFM22 into RX mode with given RX FIFO 'nearly-full' threshold and optional interrupts enabled.
void RFM22SetUpRX(const uint8_t nearlyFullThreshold, const bool syncInt, const bool dataInt)
  {
  const bool neededEnable = OTV0P2BASE::powerUpSPIIfDisabled();

  // Clear RX and TX FIFOs.
  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL2, 3); // FFCLRTX | FFCLRTX
  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL2, 0);

  // Set FIFO RX almost-full threshold as specified.
  _RFM22WriteReg8Bit(RFM22REG_RX_FIFO_CTRL, min(nearlyFullThreshold, 63));

  // Enable requested RX-related interrupts.
  _RFM22WriteReg8Bit(RFM22REG_INT_ENABLE1, (dataInt?0x10:0)); // enrxffafull: Enable RX FIFO Almost Full.
  _RFM22WriteReg8Bit(RFM22REG_INT_ENABLE2, (syncInt?0x80:0)); // enswdet: Enable Sync Word Detected.

  // Clear any current interrupt/status.
  _RFM22ClearInterrupts();

  // Start listening.
  _RFM22ModeRX();

  if(neededEnable) { OTV0P2BASE::powerDownSPI(); }
  }


// Put RFM22 into standby, attempt to read specified number of bytes from FIFO to buffer.
// Leaves RFM22 in low-power standby mode.
// Trailing bytes (more than were actually sent) undefined.
void RFM22RXFIFO(uint8_t *buf, const uint8_t bufSize)
  {
  const bool neededEnable = OTV0P2BASE::powerUpSPIIfDisabled();

  _RFM22ModeStandby();

  _RFM22_SELECT();
  _RFM22_io(RFM22REG_FIFO & 0x7F); // Start burst read from RX FIFO.
//  uint8_t val;
  for(uint8_t i = 0; i < bufSize; ++i)
    { buf[i] = _RFM22_io(0);  }
  _RFM22_DESELECT();

  // Clear RX and TX FIFOs simultaneously.
  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL2, 3); // FFCLRRX | FFCLRTX
  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL2, 0); // Needs both writes to clear.
  // Disable all interrupts.
//  _RFM22WriteReg8Bit(RFM22REG_INT_ENABLE1, 0);
//  _RFM22WriteReg8Bit(RFM22REG_INT_ENABLE2, 0); // TODO: combine in burst write with previous...
  _RFM22WriteReg16Bit0(RFM22REG_INT_ENABLE1);
  // Clear any interrupts already/still pending...
  _RFM22ClearInterrupts();
  //_RFM22WriteReg8Bit(0xd, 0x1f); // Drive GPIO2 to ground as output... (Move to general register settings.)

  if(neededEnable) { OTV0P2BASE::powerDownSPI(); }
  }






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

