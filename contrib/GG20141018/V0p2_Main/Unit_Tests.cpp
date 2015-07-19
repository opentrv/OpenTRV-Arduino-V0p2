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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2014
                           Gary Gladman 2014
*/

/*
 Development-time unit tests (NOT part of production code).

 Tests code and some I/O and sensors.

 It should be possible to auto-detect success by looking for a line starting "%%%".

 It should be possible to auto-detect failure by looking for a line starting "***Test FAILED".

 Soak testing is possible by simply letting the tests repeat as is the default;
 the first failure will stop the tests and continue reporting in a loop.

 None of these tests should write to EEPROM or FLASH
 (or perform any other unbounded life-limited operation)
 to avoid wear during soak testing, and thus allow soak testing to run without concern.
 */

#include <util/atomic.h>

#include "Unit_Tests.h"

#ifdef UNIT_TESTS // Exclude this code from production systems.

#include "Control.h"
#include "EEPROM_Utils.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "Messaging.h"
#include "Power_Management.h"
#include "PRNG.h"
#include "RFM22_Radio.h"
#include "RTC_Support.h"
#include "Security.h"
#include "Serial_IO.h"
#include "Temperature_Sensor.h"


// Error exit from failed unit test, one int parameter and the failing line number to print...
// Expects to terminate like panic() with flashing light can be detected by eye or in hardware if required.
static void error(int err, int line)
  {
  for( ; ; )
    {
    serialPrintAndFlush(F("***Test FAILED.*** val="));
    serialPrintAndFlush(err, DEC);
    serialPrintAndFlush(F(" =0x"));
    serialPrintAndFlush(err, HEX);
    if(0 != line)
      {
      serialPrintAndFlush(F(" at line "));
      serialPrintAndFlush(line);
      }
    serialPrintlnAndFlush();
    LED_HEATCALL_ON();
    tinyPause();
    LED_HEATCALL_OFF();
    sleepLowPowerMs(1000);
    }
  }

// Test expression and bucket out with error if false, else continue, including line number.
// Macros to get __LINE__ working correctly.
#define AssertIsTrueWithErr(x, err) { if(!(x)) { error((err), __LINE__); } }
#define AssertIsTrue(x) AssertIsTrueWithErr(x, 0)



// Self-test of EEPROM functioning (and smart/split erase/write).
// Will not usually perform any wear-inducing activity (is idempotent).
// Aborts with panic() upon failure.
static void testEEPROM()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testEEPROM");

  if((uint8_t) 0xff != eeprom_read_byte((uint8_t*)EE_START_TEST_LOC))
    {
    if(!eeprom_smart_erase_byte((uint8_t*)EE_START_TEST_LOC)) { panic(); } // Should have attempted erase.
    if((uint8_t) 0xff != eeprom_read_byte((uint8_t*)EE_START_TEST_LOC)) { panic(); } // Should have erased.
    }
  if(eeprom_smart_erase_byte((uint8_t*)EE_START_TEST_LOC)) { panic(); } // Should not need erase nor attempt one.

  const uint8_t eaTestPattern = 0xa5; // Test pattern for masking (selective bit clearing).
  if(0 != ((~eaTestPattern) & eeprom_read_byte((uint8_t*)EE_START_TEST_LOC2))) // Will need to clear some bits.
    {
      if(!eeprom_smart_clear_bits((uint8_t*)EE_START_TEST_LOC2, eaTestPattern)) { panic(); } // Should have attempted write.
      if(0 != ((~eaTestPattern) & eeprom_read_byte((uint8_t*)EE_START_TEST_LOC2))) { panic(); } // Should have written.
    }
  if(eeprom_smart_clear_bits((uint8_t*)EE_START_TEST_LOC2, eaTestPattern)) { panic(); } // Should not need write nor attempt one.
  }

// Do some basic testing of CRC routines.
static void testCRC()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testCRC");
  // Test the 7-bit CRC (0x5b) routine at a few points.
  const uint8_t crc0 = crc7_5B_update(0, 0); // Minimal stats payload with normal power and minimum temperature.
  AssertIsTrueWithErr((0 == crc0), crc0);
  const uint8_t crc1 = crc7_5B_update(0x40, 0); // Minimal stats payload with normal power and minimum temperature.
  AssertIsTrueWithErr((0x1a == crc1), crc1);
  const uint8_t crc2 = crc7_5B_update(0x50, 40); // Minimal stats payload with low power and 20C temperature.
  AssertIsTrueWithErr((0x7b == crc2), crc2);
  }

// Test of FHT8V bitstream encoding and decoding.
static void testFHTEncoding()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testFHTEncoding");

  uint8_t buf[MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE];
  fht8v_msg_t command; // For encoding.
  fht8v_msg_t commandDecoded; // For decoding.

  // Encode an example message for a real house code and command (close valve).
  command.hc1 = 13;
  command.hc2 = 73;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  command.command = 0x26;
  command.extension = 0;
  memset(buf, 0xff, sizeof(buf));
  uint8_t *result1 = FHT8VCreate200usBitStreamBptr(buf, &command);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == 38), result1-buf); // Check correct length.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xe3) == buf[6], buf[6]); // Check end of preamble.
  AssertIsTrueWithErr(((uint8_t)0xce) == buf[34], buf[34]); // Check part of checksum.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(13 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(73 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);

  // Encode shortest-possible (all-zero-bits) FHT8V command as 200us-bit-stream...
  command.hc1 = 0;
  command.hc2 = 0;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  command.command = 0;
  command.extension = 0;
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreate200usBitStreamBptr(buf, &command);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == 35), result1-buf); // Check correct length.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(0 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(0 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);

  // Encode longest-possible (as many 1-bits as possible) FHT8V command as 200us-bit-stream...
  command.hc1 = 0xff;
  command.hc2 = 0xff;
#ifdef FHT8V_ADR_USED
  address = 0xff;
#endif
  command.command = 0xff;
  command.extension = 0xff;
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreate200usBitStreamBptr(buf, &command);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  AssertIsTrueWithErr((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(0xff == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(0xff == commandDecoded.hc2, commandDecoded.hc2);
#ifdef FHT8V_ADR_USED
  AssertIsTrueWithErr(0xff == commandDecoded.address, commandDecoded.address);
#endif
  AssertIsTrueWithErr(0xff == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0xff == commandDecoded.extension, commandDecoded.extension);
  }

// Test of heat and tail of FHT8V bitstream encoding and decoding.
static void testFHTEncodingHeadAndTail()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testFHTEncodingHeadAndTail");

//// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) at bptr with optional headers and trailers.
////   * TRVPercentOpen value is used to generate the frame
////   * doHeader  if true then an extra RFM22/23-friendly 0xaaaaaaaa sync header is preprended
////   * trailer  if not null then a (3-byte) trailer is appented, build from that info plus a CRC
////   * command  on entry hc1, hc2 (and addresss if used) must be set correctly, this sets the command and extension; never NULL
//// The generated command frame can be resent indefinitely.
//// The output buffer used must be (at least) FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE bytes.
//// Returns pointer to the terminating 0xff on exit.
//uint8_t *FHT8VCreateValveSetCmdFrameHT_r(uint8_t *const bptrInitial, const bool doHeader, fht8v_msg_t *const command, const uint8_t TRVPercentOpen, const trailingMinimalStatsPayload_t *trailer)

  uint8_t buf[MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE];
  fht8v_msg_t command; // For encoding.
  fht8v_msg_t commandDecoded; // For decoding.

  // Encode a basic message to set a valve to 0%, without headers or trailers.
  command.hc1 = 13;
  command.hc2 = 73;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  memset(buf, 0xff, sizeof(buf));
  uint8_t *result1 = FHT8VCreateValveSetCmdFrameHT_r(buf, false, &command, 0, NULL);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == 38), result1-buf); // Check correct length: 38-byte body.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xe3) == buf[6], buf[6]); // Check end of preamble.
  AssertIsTrueWithErr(((uint8_t)0xce) == buf[34], buf[34]); // Check part of checksum.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(13 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(73 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);
  // Verify that trailer NOT present.
  AssertIsTrue(!verifyHeaderAndCRCForTrailingMinimalStatsPayload(result1));

 // Encode a basic message to set a valve to 0%, with header but without trailer.
  command.hc1 = 13;
  command.hc2 = 73;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreateValveSetCmdFrameHT_r(buf, true, &command, 0, NULL);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == RFM22_PREAMBLE_BYTES + 38), result1-buf); // Check correct length: preamble + 38-byte body.
  AssertIsTrueWithErr(((uint8_t)0xaa) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[RFM22_PREAMBLE_BYTES], buf[RFM22_PREAMBLE_BYTES]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xe3) == buf[6+RFM22_PREAMBLE_BYTES], buf[6+RFM22_PREAMBLE_BYTES]); // Check end of preamble.
  AssertIsTrueWithErr(((uint8_t)0xce) == buf[34+RFM22_PREAMBLE_BYTES], buf[34+RFM22_PREAMBLE_BYTES]); // Check part of checksum.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf + RFM22_PREAMBLE_BYTES, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(13 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(73 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);
  // Verify that trailer NOT present.
  AssertIsTrue(!verifyHeaderAndCRCForTrailingMinimalStatsPayload(result1));

  // Encode a basic message to set a valve to 0%, with header and trailer.
  command.hc1 = 13;
  command.hc2 = 73;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  FullStatsMessageCore_t fullStats;
  clearFullStatsMessageCore(&fullStats);
  captureEntropy1(); // Try stir a little noise into the PRNG before using it.
  const bool powerLow = !(randRNG8() & 0x40); // Random value.
  fullStats.containsTempAndPower = true;
  fullStats.tempAndPower.powerLow = powerLow;
  const int tempC16 = (randRNG8()&0xff) + (10 << 16); // Random value in range [10C, 25C[.
  fullStats.tempAndPower.tempC16 = tempC16;
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreateValveSetCmdFrameHT_r(buf, true, &command, 0, &fullStats);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == 41 + RFM22_PREAMBLE_BYTES), result1-buf); // Check correct length:prepamble + 38-byte body + 3 byte trailer.
  AssertIsTrueWithErr(((uint8_t)0xaa) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[RFM22_PREAMBLE_BYTES], buf[RFM22_PREAMBLE_BYTES]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xe3) == buf[6+RFM22_PREAMBLE_BYTES], buf[6+RFM22_PREAMBLE_BYTES]); // Check end of preamble.
  AssertIsTrueWithErr(((uint8_t)0xce) == buf[34+RFM22_PREAMBLE_BYTES], buf[34+RFM22_PREAMBLE_BYTES]); // Check part of checksum.
  // Attempt to decode.
  const uint8_t *afterBody = FHT8VDecodeBitStream(buf + RFM22_PREAMBLE_BYTES, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded);
  AssertIsTrue(afterBody);
  AssertIsTrueWithErr(13 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(73 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);
#if 0
  serialPrintAndFlush(F("  Trailer bytes: "));
  serialPrintAndFlush(afterBody[0], HEX);
  serialPrintAndFlush(' ');
  serialPrintAndFlush(afterBody[1], HEX);
  serialPrintAndFlush(' ');
  serialPrintAndFlush(afterBody[2], HEX);
  serialPrintlnAndFlush();
#endif
  // Verify trailer is OK.
  for(uint8_t i = 0; i < 3; ++i)
    {
    AssertIsTrueWithErr(0xff != afterBody[i], i); // No trailer byte should be 0xff (so 0xff can be terminator).
    AssertIsTrueWithErr(0 == (0x80 & afterBody[i]), i); // No trailer byte should have its high bit set.
    }
  AssertIsTrueWithErr(verifyHeaderAndCRCForTrailingMinimalStatsPayload(afterBody), *afterBody);
  // Decode values...
  trailingMinimalStatsPayload_t statsDecoded;
  extractTrailingMinimalStatsPayload(afterBody, &statsDecoded);
  AssertIsTrue(powerLow == statsDecoded.powerLow);
  AssertIsTrue(tempC16 == statsDecoded.tempC16);

  // Encode a basic message to set a a different valve to 0%, with header and trailer.
  // This one was apparently impossible to TX or RX...
  command.hc1 = 65;
  command.hc2 = 74;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreateValveSetCmdFrameHT_r(buf, true, &command, 0, &fullStats);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == 42 + RFM22_PREAMBLE_BYTES), result1-buf); // Check correct length.
  AssertIsTrueWithErr(((uint8_t)0xaa) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[RFM22_PREAMBLE_BYTES], buf[RFM22_PREAMBLE_BYTES]); // Check that result starts with FHT8V 0xcc preamble.
  // Attempt to decode.
  afterBody = FHT8VDecodeBitStream(buf + RFM22_PREAMBLE_BYTES, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded);
  AssertIsTrue(0 != afterBody);
  AssertIsTrueWithErr(65 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(74 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);
  // Verify trailer is OK.
  for(uint8_t i = 0; i < 3; ++i)
    {
    AssertIsTrueWithErr(0xff != afterBody[i], i); // No trailer byte should be 0xff (so 0xff can be terminator).
    AssertIsTrueWithErr(0 == (0x80 & afterBody[i]), i); // No trailer byte should have its high bit set.
    }
  AssertIsTrueWithErr(verifyHeaderAndCRCForTrailingMinimalStatsPayload(afterBody), *afterBody);
  // Decode values...
  memset(&statsDecoded, 0xff, sizeof(statsDecoded)); // Clear structure...
  extractTrailingMinimalStatsPayload(afterBody, &statsDecoded);
  AssertIsTrue(powerLow == statsDecoded.powerLow);
  AssertIsTrue(tempC16 == statsDecoded.tempC16);
  }


// Test elements of encoding and decoding FullStatsMessageCore_t.
// These are the routines primarily under test:
//     uint8_t *encodeFullStatsMessageCore(uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel, const FullStatsMessageCore_t *content)
//     const uint8_t *decodeFullStatsMessageCore(const uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel, FullStatsMessageCore_t *content)
static void testFullStatsMessageCoreEncDec()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testFullStatsMessageCoreEncDec");

  // Ensure that with null buffer/content encode and decode fail regardless of other arguments.
  uint8_t buf[FullStatsMessageCore_MAX_BYTES_ON_WIRE + 1];
  FullStatsMessageCore_t content;
  AssertIsTrue(NULL == encodeFullStatsMessageCore(NULL, randRNG8(), stTXalwaysAll, randRNG8NextBoolean(), NULL));
  AssertIsTrue(NULL == decodeFullStatsMessageCore(NULL, randRNG8(), stTXalwaysAll, randRNG8NextBoolean(), NULL));
  AssertIsTrue(NULL == encodeFullStatsMessageCore(NULL, FullStatsMessageCore_MAX_BYTES_ON_WIRE+1, stTXalwaysAll, randRNG8NextBoolean(), &content));
  AssertIsTrue(NULL == encodeFullStatsMessageCore(buf, FullStatsMessageCore_MAX_BYTES_ON_WIRE+1, stTXalwaysAll, randRNG8NextBoolean(), NULL));
  AssertIsTrue(NULL == decodeFullStatsMessageCore(NULL, FullStatsMessageCore_MIN_BYTES_ON_WIRE+1, stTXalwaysAll, randRNG8NextBoolean(), &content));
  AssertIsTrue(NULL == decodeFullStatsMessageCore(buf, FullStatsMessageCore_MIN_BYTES_ON_WIRE+1, stTXalwaysAll, randRNG8NextBoolean(), NULL));

  // Prepare a minimal (empty) non-secure message.
  memset(buf, 0, sizeof(buf));
  clearFullStatsMessageCore(&content);
  const uint8_t *emptyMsg = encodeFullStatsMessageCore(buf, sizeof(buf), stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != emptyMsg); // Must succeed.
  AssertIsTrueWithErr(emptyMsg - buf == FullStatsMessageCore_MIN_BYTES_ON_WIRE, emptyMsg - buf); // Must correspond to minimum size.
  AssertIsTrueWithErr(MESSAGING_FULL_STATS_HEADER_MSBS == buf[0], buf[0]); // Header byte.
  AssertIsTrueWithErr(MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS == buf[1], buf[1]); // Flags header byte.
  AssertIsTrueWithErr(0x65 == buf[2], buf[2]); // CRC.
  AssertIsTrue(((uint8_t)0xff) == *emptyMsg); // Must be correctly terminated.
  // Decode the message just generated into a freshly-scrubbed content structure.
  clearFullStatsMessageCore(&content);
  const uint8_t *emptyMsgDE = decodeFullStatsMessageCore(buf, emptyMsg-buf, stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != emptyMsgDE); // Must succeed.
  AssertIsTrue(emptyMsg == emptyMsgDE); // Must return correct end of message.
  // Verify that there is no content.
  AssertIsTrue(!content.containsID);
  AssertIsTrue(!content.containsTempAndPower);
  AssertIsTrue(!content.containsAmbL);

  // Prepare a non-secure message with ID.
  memset(buf, 0, sizeof(buf));
  clearFullStatsMessageCore(&content);
  content.id0 = 0x80;
  content.id1 = 0x00;
  content.containsID = true;
  AssertIsTrue(NULL == encodeFullStatsMessageCore(buf, sizeof(buf), stTXalwaysAll, false, &content)); // Should reject ID bytes with differring msbits.
  content.id1 = 0x81;
  const uint8_t *onlyIDMsg = encodeFullStatsMessageCore(buf, sizeof(buf), stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != onlyIDMsg); // Must succeed.
  AssertIsTrueWithErr(onlyIDMsg - buf == FullStatsMessageCore_MIN_BYTES_ON_WIRE + 2, onlyIDMsg - buf); // Must correspond to minimum size + 2 ID bytes.
  AssertIsTrueWithErr((MESSAGING_FULL_STATS_HEADER_MSBS | MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT | MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH) == buf[0], buf[0]); // Header byte.
  AssertIsTrueWithErr(0x00 == buf[1], buf[1]); // ID0 without msbit.
  AssertIsTrueWithErr(0x01 == buf[2], buf[2]); // ID1 without msbit.
  AssertIsTrueWithErr(MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS == buf[3], buf[3]); // Flags header byte.
  AssertIsTrueWithErr(0x01 == buf[4], buf[4]); // CRC.
  AssertIsTrue(((uint8_t)0xff) == *onlyIDMsg); // Must be correctly terminated.
  // Decode the message just generated into a freshly-scrubbed content structure.
  clearFullStatsMessageCore(&content);
  const uint8_t *onlyIDMsgDE = decodeFullStatsMessageCore(buf, onlyIDMsgDE-buf, stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != onlyIDMsgDE); // Must succeed.
  AssertIsTrue(onlyIDMsg == onlyIDMsgDE); // Must return correct end of message.
  // Verify that there is only ID.
  AssertIsTrue(content.containsID);
  AssertIsTrueWithErr(content.id0 == (uint8_t)0x80, content.id0);
  AssertIsTrueWithErr(content.id1 == (uint8_t)0x81, content.id1);
  AssertIsTrue(!content.containsTempAndPower);
  AssertIsTrue(!content.containsAmbL);

  // Prepare a non-secure message with ID, temp/power, ambient light level and occupancy.
  memset(buf, 0, sizeof(buf));
  clearFullStatsMessageCore(&content);
  content.id0 = 0x83;
  content.id1 = 0x98;
  content.containsID = true;
  content.tempAndPower.tempC16 = (19 << 4) + 1; // (19 + 1/16)C.
  content.tempAndPower.powerLow = false; // Normal power.
  content.containsTempAndPower = true;
  content.ambL = 42; // Allowed value in range [1,254].
  content.containsAmbL = true;
  content.occ = 3; // Not occupied recently.
  const uint8_t *msg1 = encodeFullStatsMessageCore(buf, sizeof(buf), stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != msg1); // Must succeed.
  AssertIsTrueWithErr(msg1 - buf == FullStatsMessageCore_MAX_BYTES_ON_WIRE, msg1 - buf); // Must correspond to minimum size + 2 ID bytes.
  AssertIsTrueWithErr((MESSAGING_FULL_STATS_HEADER_MSBS | MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT | MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH) == buf[0], buf[0]); // Header byte.
  AssertIsTrueWithErr(0x03 == buf[1], buf[1]); // ID0 without msbit.
  AssertIsTrueWithErr(0x18 == buf[2], buf[2]); // ID1 without msbit.
  AssertIsTrueWithErr((MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS + 1) == buf[3], buf[3]); // Temp/power first byte.
  AssertIsTrueWithErr((19 + 20) == buf[4], buf[4]); // Temp second byte.
  AssertIsTrueWithErr((MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS | MESSAGING_FULL_STATS_FLAGS_HEADER_AMBL | 3) == buf[5], buf[5]); // Flags header (no extension byte follows).
  AssertIsTrueWithErr(42 == buf[6], buf[6]); // Ambient light.
  AssertIsTrueWithErr(0x44 == buf[7], buf[7]); // CRC.
  AssertIsTrue(((uint8_t)0xff) == *msg1); // Must be correctly terminated.
  // Decode the message just generated into a freshly-scrubbed content structure.
  clearFullStatsMessageCore(&content);
  const uint8_t *msg1DE = decodeFullStatsMessageCore(buf, msg1DE-buf, stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != msg1DE); // Must succeed.
  AssertIsTrue(msg1 == msg1DE); // Must return correct end of message.
  AssertIsTrue(content.containsID);
  AssertIsTrueWithErr(content.id0 == (uint8_t)0x83, content.id0);
  AssertIsTrueWithErr(content.id1 == (uint8_t)0x98, content.id1);
  AssertIsTrue(content.containsTempAndPower);
  AssertIsTrue(!content.tempAndPower.powerLow);
  AssertIsTrue((19 << 4) + 1 == content.tempAndPower.tempC16);
  AssertIsTrue(content.containsAmbL);
  AssertIsTrue(42 == content.ambL);
  }






// Test elements of RTC time persist/restore (without causing more EEPROM wear, if working correctly).
static void testRTCPersist()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testRTCPersist");
  // Perform with interrupts shut out to avoid RTC ISR interferring.
  // This will effectively stall the RTC.
  bool minutesPersistOK;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    const uint_least16_t mb = RTC::getMinutesSinceMidnightLT();
    RTC::persistRTC();
    RTC::restoreRTC();
    const uint_least16_t ma = RTC::getMinutesSinceMidnightLT();
    // Check that persist/restore did not change live minutes value at least, within the 15-minute quantum used.
    minutesPersistOK = (mb/15 == ma/15);
    }
    AssertIsTrue(minutesPersistOK);
  }


// Tests of entropy gathering routines.
//
// Maximum number of identical nominally random bits (or values with approx one bit of entropy) in a row tolerated.
// Set large enough that even soak testing for many hours should not trigger a failure if behaviour is plausibly correct.
#define MAX_IDENTICAL_BITS_SEQUENTIALLY 32
void testEntropyGathering()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testEntropyGathering");

  // Test WDT jitter: assumed about 1 bit of entropy per call/result.
  //DEBUG_SERIAL_PRINT_FLASHSTRING("jWDT... ");
  const uint8_t jWDT = clockJitterWDT();
  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY; --i >= 0; )
    {
    if(jWDT != clockJitterWDT()) { break; } // Stop as soon as a different value is obtained.
    AssertIsTrueWithErr(0 != i, i); // Generated too many identical values in a row.
    }
  //DEBUG_SERIAL_PRINT_FLASHSTRING(" 1st=");
  //DEBUG_SERIAL_PRINTFMT(jWDT, BIN);
  //DEBUG_SERIAL_PRINTLN();

  // Test RTC jitter: assumed about 1 bit of entropy per call/result.
  //DEBUG_SERIAL_PRINT_FLASHSTRING("jRTC... ");
  for(const uint8_t t0 = getSubCycleTime(); t0 == getSubCycleTime(); ) { } // Wait for sub-cycle time to roll to toughen test.
  const uint8_t jRTC = clockJitterRTC();
  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY; --i >= 0; )
    {
    if(jRTC != clockJitterRTC()) { break; } // Stop as soon as a different value is obtained.
    AssertIsTrue(0 != i); // Generated too many identical values in a row.
    }
  //DEBUG_SERIAL_PRINT_FLASHSTRING(" 1st=");
  //DEBUG_SERIAL_PRINTFMT(jRTC, BIN);
  //DEBUG_SERIAL_PRINTLN();

  // Test full-byte jitter: assumed about 8 bits of entropy per call/result.
  //DEBUG_SERIAL_PRINT_FLASHSTRING("jByte... ");
  const uint8_t t0j = getSubCycleTime();
  while(t0j == getSubCycleTime()) { } // Wait for sub-cycle time to roll to toughen test.
  const uint8_t jByte = clockJitterEntropyByte();

  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY/8; --i >= 0; )
    {
    if(jByte != clockJitterEntropyByte()) { break; } // Stop as soon as a different value is obtained.
    AssertIsTrue(0 != i); // Generated too many identical values in a row.
    }
  //DEBUG_SERIAL_PRINT_FLASHSTRING(" 1st=");
  //DEBUG_SERIAL_PRINTFMT(jByte, BIN);
  //DEBUG_SERIAL_PRINT_FLASHSTRING(", ticks=");
  //DEBUG_SERIAL_PRINT((uint8_t)(t1j - t0j - 1));
  //DEBUG_SERIAL_PRINTLN();

  // Test noisy ADC read: assumed at least one bit of noise per call/result.
  const uint8_t nar1 = noisyADCRead();
#if 0
  DEBUG_SERIAL_PRINT_FLASHSTRING("nar1 ");
  DEBUG_SERIAL_PRINTFMT(nar1, BIN);
  DEBUG_SERIAL_PRINTLN();
#endif
  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY; --i >= 0; )
    {
    const uint8_t nar = noisyADCRead();
    if(nar1 != nar) { break; } // Stop as soon as a different value is obtained.
#if 1
    DEBUG_SERIAL_PRINT_FLASHSTRING("repeat nar ");
    DEBUG_SERIAL_PRINTFMT(nar, BIN);
    DEBUG_SERIAL_PRINTLN();
#endif
    AssertIsTrue(0 != i); // Generated too many identical values in a row.
    }

  // Test secure random byte generation.
  const uint8_t srb1 = getSecureRandomByte();
#if 0
  DEBUG_SERIAL_PRINT_FLASHSTRING("srb1 ");
  DEBUG_SERIAL_PRINTFMT(srb1, BIN);
  DEBUG_SERIAL_PRINTLN();
#endif
  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY/8; --i >= 0; )
    {
    if(srb1 != getSecureRandomByte()) { break; } // Stop as soon as a different value is obtained.
    AssertIsTrue(0 != i); // Generated too many identical values in a row.
    }
  }

// Test sleepUntilSubCycleTime() routine.
void testSleepUntilSubCycleTime()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testSleepUntilSubCycleTime");

  const uint8_t start = getSubCycleTime();

  // Check that this correctly notices/vetoes attempt to sleep until time already past.
  if(start > 0) { AssertIsTrue(!sleepUntilSubCycleTime(start-1)); }

  // Don't attempt rest of test if near the end of the current minor cycle...
  if(start > (GSCT_MAX/2)) { return; }

  // Set a random target significantly before the end of the current minor cycle.
#if 0x3f > GSCT_MAX/4
#error
#endif
  const uint8_t sleepTicks = 2 + (randRNG8() & 0x3f);
  const uint8_t target = start + sleepTicks;
  AssertIsTrue(target > start);
  AssertIsTrue(target < GSCT_MAX);

  // Call should succeed.
  AssertIsTrue(sleepUntilSubCycleTime(target));

  // Call should return with some of specified target tick still to run...
  const uint8_t end = getSubCycleTime();
  AssertIsTrueWithErr(target == end, end); // FIXME: DHD2014020: getting occasional failures.

#if 0
  DEBUG_SERIAL_PRINT_FLASHSTRING("Sleep ticks: ");
  DEBUG_SERIAL_PRINT(sleepTicks);
  DEBUG_SERIAL_PRINTLN();
#endif
  }

// Test that the simple smoothing function never generates an out of range value.
// In particular, with a legitimate value range of [0,254]
// smoothStatsValue() must never generate 255 (0xff) which looks like an uninitialised EEPROM value,
// nor wrap around in either direction.
static void testSmoothStatsValue()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testSmoothStatsValue");
  // Covers the key cases 0 and 254 in particular.
  for(int i = 256; --i >= 0; ) { AssertIsTrue((uint8_t)i == smoothStatsValue((uint8_t)i, (uint8_t)i)); }
  }

// Test for expected behaviour of RNG8 PRNG starting from a known state.
static void testRNG8()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testRNG8");
  // Reset to known state; API not normally exposed and only exists for unit tests.
  resetRNG8();
  // Extract and check a few initial values.
  const uint8_t v1 = randRNG8();
  const uint8_t v2 = randRNG8();
  const uint8_t v3 = randRNG8();
  const uint8_t v4 = randRNG8();
  //DEBUG_SERIAL_PRINT(v1); DEBUG_SERIAL_PRINTLN();
  //DEBUG_SERIAL_PRINT(v2); DEBUG_SERIAL_PRINTLN();
  //DEBUG_SERIAL_PRINT(v3); DEBUG_SERIAL_PRINTLN();
  //DEBUG_SERIAL_PRINT(v4); DEBUG_SERIAL_PRINTLN();
  AssertIsTrue(1 == v1);
  AssertIsTrue(0 == v2);
  AssertIsTrue(3 == v3);
  AssertIsTrue(14 == v4);
  }


// Test temperature companding.
static void testTempCompand()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testTempCompand");
  // Ensure that all (whole) temperatures from 0C to 100C are correctly compressed and expanded.
  for(int i = 0; i <= 100; ++i)
    {
    //DEBUG_SERIAL_PRINT(i<<4); DEBUG_SERIAL_PRINT(" => "); DEBUG_SERIAL_PRINT(compressTempC16(i<<4)); DEBUG_SERIAL_PRINT(" => "); DEBUG_SERIAL_PRINT(expandTempC16(compressTempC16(i<<4))); DEBUG_SERIAL_PRINTLN();
    AssertIsTrueWithErr(i<<4 == expandTempC16(compressTempC16(i<<4)), i);
    }
  // Ensure that out-of-range inputs are coerced to the limits.
  AssertIsTrueWithErr(0 == expandTempC16(compressTempC16(-1)), -1);
  AssertIsTrueWithErr((100<<4) == expandTempC16(compressTempC16(101<<4)), 101);
  AssertIsTrueWithErr(COMPRESSION_C16_CEIL_VAL_AFTER == compressTempC16(102<<4), COMPRESSION_C16_CEIL_VAL_AFTER); // Verify ceiling.
  AssertIsTrue(COMPRESSION_C16_CEIL_VAL_AFTER < 0xff);
  // Ensure that 'unset' compressed value expands to 'unset' uncompressed value.
  AssertIsTrue(STATS_UNSET_INT == expandTempC16(STATS_UNSET_BYTE));
  }








#if !defined(DISABLE_SENSOR_UNIT_TESTS)
// Test temperature sensor returns value in reasonable bounds for a test environment.
// Attempts to test that the sensor is actually present.
static void testTempSensor()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testTempSensor");
  const tempC16_t temp = Temp::readTemperatureC16();
#if 0
  serialPrintAndFlush("  temp: ");
  serialPrintAndFlush(temp >> 4, DEC);
  serialPrintAndFlush('C');
  serialPrintAndFlush(temp & 0xf, HEX);
  serialPrintlnAndFlush();
#endif
  // During testing temp should be above 0C (0C might indicate a missing/broken sensor) and below 50C.
  AssertIsTrueWithErr((temp > 0) && (temp < (50 << 4)), temp);
  }
#endif

#if !defined(DISABLE_SENSOR_UNIT_TESTS)
// Test that on-chip temperature sensor returns value in half-reasonable bounds for a test environment.
// Internal sensor may be +/- 10C out.
static void testInternalTempSensor()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testInternalTempSensor");
  const int temp = readInternalTemperatureC16();
#if 0
  serialPrintAndFlush("  int temp: ");
  serialPrintAndFlush(temp >> 4, DEC);
  serialPrintAndFlush('C');
  serialPrintAndFlush(temp & 0xf, HEX);
  serialPrintlnAndFlush();
#endif
  // During testing temp should be above 0C (0C might indicate a missing/broken sensor) and below 50C.
  // Internal sensor may be +/- 10C out.
  AssertIsTrueWithErr((temp > -10) && (temp < (60 << 4)), temp);
  }
#endif

#if !defined(DISABLE_SENSOR_UNIT_TESTS)
static void testSupplyVoltageMonitor()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testSupplyVoltageMonitor");
  const int mv = readBatterymV();
#if 0
  serialPrintAndFlush("  Battery mv: ");
  serialPrintAndFlush(mv, DEC);
  serialPrintlnAndFlush();
#endif
  // During testing power supply voltage should be above ~1.7V BOD limit,
  // and below 3.6V for V0p2 boards which is RFM22 Vss limit.
  AssertIsTrueWithErr((mv >= 1700) && (mv <= 3600), mv);
  }
#endif



// To be called from loop() instead of main code when running unit tests.
// Tests generally flag an error and stop the test cycle with a call to panic() or error().
void unitTestLoop()
  {
  static int loopCount = 0;

  // Allow the terminal console to be brought up.
  for(int i = 3; i > 0; --i)
    {
    serialPrintAndFlush(F("Tests starting shortly... "));
    serialPrintAndFlush(i);
    serialPrintlnAndFlush();
    sleepLowPowerMs(1000);
    }
  serialPrintlnAndFlush();


  // Run the tests, fastest / newest / most-fragile / most-interesting first...
  testFullStatsMessageCoreEncDec();
  testCRC();
  testTempCompand();
  testRNG8();
  testEntropyGathering();
  testRTCPersist();
  testEEPROM();
  testSmoothStatsValue();
  testSleepUntilSubCycleTime();
  testFHTEncoding();
  testFHTEncodingHeadAndTail();

  // Sensor tests.
  // May need to be disabled if, for example, running in a simulator or on a partial board.
  // Should not involve anything too complex from the normal run-time, such as interrupts.
#if !defined(DISABLE_SENSOR_UNIT_TESTS)
  testTempSensor();
  testInternalTempSensor();
  testSupplyVoltageMonitor();
#endif


  // Announce successful loop count.
  ++loopCount;
  serialPrintlnAndFlush();
  serialPrintAndFlush(F("%%% All tests completed OK, round "));
  serialPrintAndFlush(loopCount);
  serialPrintlnAndFlush();
  serialPrintlnAndFlush();
  serialPrintlnAndFlush();
  // Help avoid tests spinning too fast even to see!
  sleepLowPowerMs(1000);
  }


#endif



