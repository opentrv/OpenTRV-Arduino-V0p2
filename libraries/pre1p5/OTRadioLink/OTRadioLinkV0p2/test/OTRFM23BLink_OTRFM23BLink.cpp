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

Author(s) / Copyright (s): Damon Hart-Davis 2015
*/

#include "OTRFM23BLink_OTRFM23BLink.h"

/**TEMPORARILY IN TEST SKETCH BEFORE BEING MOVED TO OWN LIBRARY. */

namespace OTRFM23BLink
    {
// RFM23B is apparently SPI mode 0 for Arduino library pov.

//#define RFM22REG_INT_STATUS1 3 // Interrupt status register 1.
//#define RFM22REG_INT_STATUS2 4 // Interrupt status register 2.
//#define RFM22REG_INT_ENABLE1 5 // Interrupt enable register 1.
//#define RFM22REG_INT_ENABLE2 6 // Interrupt enable register 2.
//#define RFM22REG_OP_CTRL1 7 // Operation and control register 1.
//#define RFM22REG_OP_CTRL1_SWRES 0x80 // Software reset (at write) in OP_CTRL1.
//#define RFM22REG_OP_CTRL2 8 // Operation and control register 2.
//#define RFM22REG_RSSI 0x26 // RSSI.
//#define RFM22REG_RSSI1 0x28 // Antenna 1 diversity / RSSI.
//#define RFM22REG_RSSI2 0x29 // Antenna 2 diversity / RSSI.
//#define RFM22REG_TX_POWER $0x6d // Transmit power.
//#define RFM22REG_RX_FIFO_CTRL 0x7e // RX FIFO control.
//#define RFM22REG_FIFO  0x7f // TX FIFO on write, RX FIFO on read.
//
//// Allow validation of RFM22/RFM23 device and SPI connection to it.
//#define RFM22_SUPPORTED_DEVICE_TYPE 0x08 // Read from register 0.
//#define RFM22_SUPPORTED_DEVICE_VERSION 0x06 // Read from register 1.

//// Write/read one byte over SPI...
//// SPI must already be configured and running.
//uint8_t OTRFM23BLinkBase::_io(const uint8_t data)
//  {
//  SPDR = data;
//  // TODO: convert from busy-wait to sleep, at least in a standby mode, if likely longer than 10s of uS.
//  // At lowest SPI clock prescale (x2) this is likely to spin for ~16 CPU cycles (8 bits each taking 2 cycles).
//  while (!(SPSR & _BV(SPIF))) { }
//  return(SPDR);
//  }

//// Write one byte over SPI (ignoring the value read back).
//// SPI must already be configured and running.
//void OTRFM23BLinkBase::_wr(const uint8_t data)
//  {
//  SPDR = data;
//  // TODO: convert from busy-wait to sleep, at least in a standby mode, if likely longer than 10s of uS.
//  // At lowest SPI clock prescale (x2) this is likely to spin for ~16 CPU cycles (8 bits each taking 2 cycles).
//  while (!(SPSR & _BV(SPIF))) { }
//  }

//// Write to 8-bit register on RFM22.
//// SPI must already be configured and running.
//static void _RFM22WriteReg8Bit(const uint8_t addr, const uint8_t val)
//  {
//  _RFM22_SELECT();
//  _RFM22_wr(addr | 0x80); // Force to write.
//  _RFM22_wr(val);
//  _RFM22_DESELECT();
//  }
//
//// Write 0 to 16-bit register on RFM22 as burst.
//// SPI must already be configured and running.
//static void _RFM22WriteReg16Bit0(const uint8_t addr)
//  {
//  _RFM22_SELECT();
//  _RFM22_wr(addr | 0x80); // Force to write.
//  _RFM22_wr(0);
//  _RFM22_wr(0);
//  _RFM22_DESELECT();
//  }
//
//// Read from 8-bit register on RFM22.
//// SPI must already be configured and running.
//static uint8_t _RFM22ReadReg8Bit(const uint8_t addr)
//  {
//  _RFM22_SELECT();
//  _RFM22_io(addr & 0x7f); // Force to read.
//  const uint8_t result = _RFM22_io(0); // Dummy value...
//  _RFM22_DESELECT();
//  return(result);
//  }
//
//// Read from 16-bit big-endian register pair.
//// The result has the first (lower-numbered) register in the most significant byte.
//static uint16_t _RFM22ReadReg16Bit(const uint8_t addr)
//{
//  _RFM22_SELECT();
//  _RFM22_io(addr & 0x7f); // Force to read.
//  uint16_t result = ((uint16_t)_RFM22_io(0)) << 8;
//  result |= ((uint16_t)_RFM22_io(0));
//  _RFM22_DESELECT();
//  return(result);
//}
//
//// Enter standby mode.
//// SPI must already be configured and running.
//static void _RFM22ModeStandby()
//  {
//  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL1, 0);
//#if 0 && defined(DEBUG)
//  DEBUG_SERIAL_PRINT_FLASHSTRING("Sb");
//#endif
//  }
//
//// Enter transmit mode (and send any packet queued up in the TX FIFO).
//// SPI must already be configured and running.
//static void _RFM22ModeTX()
//  {
//  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL1, 9);
//#if 0 && defined(DEBUG)
//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Tx");
//#endif
//  } // TXON | XTON
//
//// Enter receive mode.
//// SPI must already be configured and running.
//static void _RFM22ModeRX()
//  {
//  _RFM22WriteReg8Bit(RFM22REG_OP_CTRL1, 5);
//#if 0 && defined(DEBUG)
//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Rx");
//#endif
//  } // RXON | XTON
//
//// Read/discard status (both registers) to clear interrupts.
//// SPI must already be configured and running.
//static void _RFM22ClearInterrupts()
//  {
////  _RFM22WriteReg8Bit(RFM22REG_INT_STATUS1, 0);
////  _RFM22WriteReg8Bit(RFM22REG_INT_STATUS2, 0); // TODO: combine in burst write with previous...
//  _RFM22WriteReg16Bit0(RFM22REG_INT_STATUS1);
//  }

    }
