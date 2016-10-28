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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2016
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


#include "V0p2_Main.h"


#ifdef UNIT_TESTS // Exclude unit test code from production systems.

#include <util/atomic.h>

#include "Control.h"

#include "FHT8V_Wireless_Rad_Valve.h"
#include "Messaging.h"
#include "RFM22_Radio.h"
#include "Schedule.h"
#include "Security.h"
#include "UI_Minimal.h"


// Error exit from failed unit test, one int parameter and the failing line number to print...
// Expects to terminate like panic() with flashing light can be detected by eye or in hardware if required.
static void error(int err, int line)
  {
  for( ; ; )
    {
    OTV0P2BASE::serialPrintAndFlush(F("***Test FAILED*** val="));
    OTV0P2BASE::serialPrintAndFlush(err, DEC);
    OTV0P2BASE::serialPrintAndFlush(F(" =0x"));
    OTV0P2BASE::serialPrintAndFlush(err, HEX);
    if(0 != line)
      {
      OTV0P2BASE::serialPrintAndFlush(F(" at line "));
      OTV0P2BASE::serialPrintAndFlush(line);
      }
    OTV0P2BASE::serialPrintlnAndFlush();
    LED_HEATCALL_ON();
    tinyPause();
    LED_HEATCALL_OFF();
    OTV0P2BASE::sleepLowPowerMs(1000);
    }
  }

// Deal with common equality test.
static inline void errorIfNotEqual(int expected, int actual, int line) { if(expected != actual) { error(actual, line); } }
// Allowing a delta.
static inline void errorIfNotEqual(int expected, int actual, int delta, int line) { if(abs(expected - actual) > delta) { error(actual, line); } }

// Test expression and bucket out with error if false, else continue, including line number.
// Macros allow __LINE__ to work correctly.
#define AssertIsTrueWithErr(x, err) { if(!(x)) { error((err), __LINE__); } }
#define AssertIsTrue(x) AssertIsTrueWithErr((x), 0)
#define AssertIsEqual(expected, x) { errorIfNotEqual((expected), (x), __LINE__); }
#define AssertIsEqualWithDelta(expected, x, delta) { errorIfNotEqual((expected), (x), (delta), __LINE__); }



// Check that correct versions of underlying libraries are in use.
static void testLibVersions()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("LibVersions");
#if !(0 == ARDUINO_LIB_OTV0P2BASE_VERSION_MAJOR) || !(8 <= ARDUINO_LIB_OTV0P2BASE_VERSION_MINOR)
#error Wrong OTV0p2Base library version!
#endif
#if !(0 == ARDUINO_LIB_OTRADIOLINK_VERSION_MAJOR) || !(9 <= ARDUINO_LIB_OTRADIOLINK_VERSION_MINOR)
#error Wrong OTRadioLink library version!
#endif
//  AssertIsEqual(0, ARDUINO_LIB_OTRADIOLINK_VERSION_MAJOR);
//  AssertIsTrue(1 <= ARDUINO_LIB_OTRADIOLINK_VERSION_MINOR); // Minimum acceptable minor version.
#if !(0 == ARDUINO_LIB_OTRFM23BLINK_VERSION_MAJOR) || !(9 <= ARDUINO_LIB_OTRFM23BLINK_VERSION_MINOR)
#error Wrong OTRFM23BLink library version!
#endif
  }


// Test handling of JSON stats.
static void testJSONStats()
  {
#if defined(ENABLE_JSON_OUTPUT)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("JSONStats");
  SimpleStatsRotation<2> ss1;
  ss1.setID("1234");
  AssertIsEqual(0, ss1.size());
  //AssertIsTrue(0 == ss1.writeJSON(NULL, randRNG8(), randRNG8(), randRNG8NextBoolean()));
  char buf[MSG_JSON_MAX_LENGTH + 2]; // Allow for trailing '\0' and spare byte.
  // Create minimal JSON message with no data content. just the (supplied) ID.
  const uint8_t l1 = ss1.writeJSON((uint8_t*)buf, sizeof(buf), OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean());
//OTV0P2BASE::serialPrintAndFlush(buf); OTV0P2BASE::serialPrintlnAndFlush();
  AssertIsEqual(12, l1);
  const char PROGMEM *t1 = (const char PROGMEM *)F("{\"@\":\"1234\"}");
  AssertIsTrue(0 == strcmp_P(buf, t1));
  ss1.enableCount(false);
  AssertIsEqual(12, ss1.writeJSON((uint8_t*)buf, sizeof(buf), OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean()));
  AssertIsTrue(0 == strcmp_P(buf, t1));
  // Check that count works.
  ss1.enableCount(true);
  AssertIsEqual(0, ss1.size());
  AssertIsEqual(18, ss1.writeJSON((uint8_t*)buf, sizeof(buf), OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean()));
//OTV0P2BASE::serialPrintAndFlush(buf); OTV0P2BASE::serialPrintlnAndFlush();
  AssertIsTrue(0 == strcmp_P(buf, (const char PROGMEM *)F("{\"@\":\"1234\",\"+\":2}")));
  // Turn count off for rest of tests.
  ss1.enableCount(false);
  AssertIsEqual(12, ss1.writeJSON((uint8_t*)buf, sizeof(buf), OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean()));
  // Check that removal of absent entry does nothing.
  AssertIsTrue(!ss1.remove("bogus"));
  AssertIsEqual(0, ss1.size());
  // Check that new item can be added/put (with no/default properties).
  ss1.put("f1", 42);
  AssertIsEqual(1, ss1.size());
  AssertIsEqual(20, ss1.writeJSON((uint8_t*)buf, sizeof(buf), 0, OTV0P2BASE::randRNG8NextBoolean()));
#if 0 // Short of Flash space!
//OTV0P2BASE::serialPrintAndFlush(buf); OTV0P2BASE::serialPrintlnAndFlush();
  AssertIsTrue(0 == strcmp_P(buf, (const char PROGMEM *)F("{\"@\":\"1234\",\"f1\":42}")));
#endif
  ss1.put("f1", -111);
  AssertIsEqual(1, ss1.size());
  AssertIsEqual(22, ss1.writeJSON((uint8_t*)buf, sizeof(buf), 0, OTV0P2BASE::randRNG8NextBoolean()));
  AssertIsTrue(0 == strcmp_P(buf, (const char PROGMEM *)F("{\"@\":\"1234\",\"f1\":-111}")));
#endif
  }

// Test handling of JSON messages for transmission and reception.
// Includes bit-twiddling, CRC computation, and other error checking.
static void testJSONForTX()
  {
#if defined(ENABLE_JSON_OUTPUT)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("JSONForTX");
  char buf[MSG_JSON_MAX_LENGTH + 2]; // Allow for trailing '\0' or CRC + 0xff terminator.
  // Clear the buffer.
  memset(buf, 0, sizeof(buf));
  // Fail sanity check on a completely empty buffer (zero-length string).
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // Fail sanity check on a few initially-plausible length-1 values.
  buf[0] = '{';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  buf[0] = '}';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  buf[0] = '[';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  buf[0] = ']';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  buf[0] = ' ';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // Fail sanity check with already-adjusted (minimal) nessage.
  buf[0] = '{';
  buf[1] = ('}' | 0x80);
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // Minimal correct messaage should pass.
  buf[0] = '{';
  buf[1] = '}';
  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
  // Try a longer valid trivial message.
  strcpy_P(buf, (const char PROGMEM *)F("{  }"));
  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
  // Invalidate it with a non-printable char and check that it is rejected.
  buf[2] = '\1';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // Try a longer valid non-trivial message.
  __FlashStringHelper const* longJSONMsg1 = F("{\"@\":\"cdfb\",\"T|C16\":299,\"H|%\":83,\"L\":255,\"B|cV\":256}");
  memset(buf, 0, sizeof(buf));
  strcpy_P(buf, (const char PROGMEM *)longJSONMsg1);
  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
  // Invalidate it with a high-bit set and check that it is rejected.
  buf[5] |= 0x80;
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // CRC fun!  
  memset(buf, 0, sizeof(buf));
  buf[0] = '{';
  buf[1] = '}';
  const uint8_t crc1 = adjustJSONMsgForTXAndComputeCRC(buf);
  // Check that top bit is not set (ie CRC was computed OK).
  AssertIsTrueWithErr(!(crc1 & 0x80), crc1);
  // Check for expected CRC value.
  AssertIsTrueWithErr((0x38 == crc1), crc1);
  // Check that initial part unaltered.
  AssertIsTrueWithErr(('{' == buf[0]), buf[0]);
  // Check that top bit has been set in trailing brace.
  AssertIsTrueWithErr(((char)('}' | 0x80) == buf[1]), buf[1]);
  // Check that trailing '\0' still present.
  AssertIsTrueWithErr((0 == buf[2]), buf[2]);
  // Check that TX-format can be converted for RX.
  buf[2] = crc1;
  buf[3] = 0xff; // As for normal TX...
// FIXME
//  const int8_t l1 = adjustJSONMsgForRXAndCheckCRC(buf, sizeof(buf));
//  AssertIsTrueWithErr(2 == l1, l1);
//  AssertIsTrueWithErr(2 == strlen(buf), strlen(buf));
//  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
  // Now a longer message...
  memset(buf, 0, sizeof(buf));
  strcpy_P(buf, (const char PROGMEM *)longJSONMsg1);
  const int8_t l2o = strlen(buf);
  const uint8_t crc2 = adjustJSONMsgForTXAndComputeCRC(buf);
  // Check that top bit is not set (ie CRC was computed OK).
  AssertIsTrueWithErr(!(crc2 & 0x80), crc2);
  // Check for expected CRC value.
  AssertIsTrueWithErr((0x77 == crc2), crc2);
// FIXME
//  // Check that TX-format can be converted for RX.
//  buf[l2o] = crc2;
//  buf[l2o+1] = 0xff;
// FIXME
//  const int8_t l2 = adjustJSONMsgForRXAndCheckCRC(buf, sizeof(buf));
//  AssertIsTrueWithErr(l2o == l2, l2);
//  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
#endif
  }


//// Self-test of EEPROM functioning (and smart/split erase/write).
//// Will not usually perform any wear-inducing activity (is idempotent).
//// Aborts with panic() upon failure.
//static void testEEPROM()
//  {
//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("EEPROM");
//
//  if((uint8_t) 0xff != eeprom_read_byte((uint8_t*)EE_START_TEST_LOC))
//    {
//    if(!eeprom_smart_erase_byte((uint8_t*)EE_START_TEST_LOC)) { panic(); } // Should have attempted erase.
//    if((uint8_t) 0xff != eeprom_read_byte((uint8_t*)EE_START_TEST_LOC)) { panic(); } // Should have erased.
//    }
//  if(eeprom_smart_erase_byte((uint8_t*)EE_START_TEST_LOC)) { panic(); } // Should not need erase nor attempt one.
//
//  const uint8_t eaTestPattern = 0xa5; // Test pattern for masking (selective bit clearing).
//  if(0 != ((~eaTestPattern) & eeprom_read_byte((uint8_t*)EE_START_TEST_LOC2))) // Will need to clear some bits.
//    {
//      if(!eeprom_smart_clear_bits((uint8_t*)EE_START_TEST_LOC2, eaTestPattern)) { panic(); } // Should have attempted write.
//      if(0 != ((~eaTestPattern) & eeprom_read_byte((uint8_t*)EE_START_TEST_LOC2))) { panic(); } // Should have written.
//    }
//  if(eeprom_smart_clear_bits((uint8_t*)EE_START_TEST_LOC2, eaTestPattern)) { panic(); } // Should not need write nor attempt one.
//  }


// Test elements of encoding and decoding FullStatsMessageCore_t.
// These are the routines primarily under test:
//     uint8_t *encodeFullStatsMessageCore(uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel, const FullStatsMessageCore_t *content)
//     const uint8_t *decodeFullStatsMessageCore(const uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel, FullStatsMessageCore_t *content)
static void testFullStatsMessageCoreEncDec()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("FullStatsMessageCoreEncDec");

  // Ensure that with null buffer/content encode and decode fail regardless of other arguments.
  uint8_t buf[FullStatsMessageCore_MAX_BYTES_ON_WIRE + 1];
  FullStatsMessageCore_t content;
  AssertIsTrue(NULL == encodeFullStatsMessageCore(NULL, OTV0P2BASE::randRNG8(), stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), NULL));
  AssertIsTrue(NULL == decodeFullStatsMessageCore(NULL, OTV0P2BASE::randRNG8(), stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), NULL));
  AssertIsTrue(NULL == encodeFullStatsMessageCore(NULL, FullStatsMessageCore_MAX_BYTES_ON_WIRE+1, stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), &content));
  AssertIsTrue(NULL == encodeFullStatsMessageCore(buf, FullStatsMessageCore_MAX_BYTES_ON_WIRE+1, stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), NULL));
  AssertIsTrue(NULL == decodeFullStatsMessageCore(NULL, FullStatsMessageCore_MIN_BYTES_ON_WIRE+1, stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), &content));
  AssertIsTrue(NULL == decodeFullStatsMessageCore(buf, FullStatsMessageCore_MIN_BYTES_ON_WIRE+1, stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), NULL));

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
  const uint8_t *onlyIDMsgDE = decodeFullStatsMessageCore(buf, onlyIDMsg-buf, stTXalwaysAll, false, &content);
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
  const uint8_t *msg1DE = decodeFullStatsMessageCore(buf, msg1-buf, stTXalwaysAll, false, &content);
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



// Test sleepUntilSubCycleTime() routine.
void testSleepUntilSubCycleTime()
  {
#ifdef ENABLE_WAKEUP_32768HZ_XTAL
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("SleepUntilSubCycleTime");

  const uint8_t start = OTV0P2BASE::getSubCycleTime();

  // Check that this correctly notices/vetoes attempt to sleep until time already past.
  if(start > 0) { AssertIsTrue(!sleepUntilSubCycleTime(start-1)); }

  // Don't attempt rest of test if near the end of the current minor cycle...
  if(start > (OTV0P2BASE::GSCT_MAX/2)) { return; }

  // Set a random target significantly before the end of the current minor cycle.
//#if 0x3f > GSCT_MAX/4
//#error
//#endif
  AssertIsTrue(0x3f <= OTV0P2BASE::GSCT_MAX/4);
  const uint8_t sleepTicks = 2 + (OTV0P2BASE::randRNG8() & 0x3f);
  const uint8_t target = start + sleepTicks;
  AssertIsTrue(target > start);
  AssertIsTrue(target < OTV0P2BASE::GSCT_MAX);

  // Call should succeed.
  AssertIsTrue(sleepUntilSubCycleTime(target));

  // Call should return with some of specified target tick still to run...
  const uint8_t end = OTV0P2BASE::getSubCycleTime();
  AssertIsTrueWithErr(target == end, end); // FIXME: DHD2014020: getting occasional failures.

#if 0
  DEBUG_SERIAL_PRINT_FLASHSTRING("Sleep ticks: ");
  DEBUG_SERIAL_PRINT(sleepTicks);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif
  }

// Test that the simple smoothing function never generates an out of range value.
// In particular, with a legitimate value range of [0,254]
// smoothStatsValue() must never generate 255 (0xff) which looks like an uninitialised EEPROM value,
// nor wrap around in either direction.
static void testSmoothStatsValue()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("SmoothStatsValue");
  // Covers the key cases 0 and 254 in particular.
  for(int i = 256; --i >= 0; ) { AssertIsTrue((uint8_t)i == smoothStatsValue((uint8_t)i, (uint8_t)i)); }
  }


// Test some of the mask/port calculations.
static void testFastDigitalIOCalcs()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("FastDigitalIOCalcs");
  AssertIsEqual(digitalPinToBitMask(0), fastDigitalMask(0));
  AssertIsEqual(digitalPinToBitMask(2), fastDigitalMask(2));
  AssertIsEqual(digitalPinToBitMask(13), fastDigitalMask(13));
  AssertIsEqual(digitalPinToBitMask(19), fastDigitalMask(19));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(0)), (intptr_t)fastDigitalInputRegister(0));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(2)), (intptr_t)fastDigitalInputRegister(2));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(7)), (intptr_t)fastDigitalInputRegister(7));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(8)), (intptr_t)fastDigitalInputRegister(8));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(14)), (intptr_t)fastDigitalInputRegister(14));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(19)), (intptr_t)fastDigitalInputRegister(19));
  }







#if !defined(DISABLE_SENSOR_UNIT_TESTS)
// Test temperature sensor returns value in reasonable bounds for a test environment.
// Attempts to test that the sensor is actually present.
static void testTempSensor()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("TempSensor");
  const int temp = TemperatureC16.read();
#if 0
  OTV0P2BASE::serialPrintAndFlush("  temp: ");
  OTV0P2BASE::serialPrintAndFlush(temp >> 4, DEC);
  OTV0P2BASE::serialPrintAndFlush('C');
  OTV0P2BASE::serialPrintAndFlush(temp & 0xf, HEX);
  OTV0P2BASE::serialPrintlnAndFlush();
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
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("InternalTempSensor");
  const int temp = readInternalTemperatureC16();
#if 0
  OTV0P2BASE::serialPrintAndFlush("  int temp: ");
  OTV0P2BASE::serialPrintAndFlush(temp >> 4, DEC);
  OTV0P2BASE::serialPrintAndFlush('C');
  OTV0P2BASE::serialPrintAndFlush(temp & 0xf, HEX);
  OTV0P2BASE::serialPrintlnAndFlush();
#endif
  // During testing temp should be above 0C (0C might indicate a missing/broken sensor) and below 50C.
  // Internal sensor may be +/- 10C out.
  // DHD20141223: Just has a reading of ~17C from an otherwise-OK AVR with room temp ~20C.
  AssertIsTrueWithErr((temp > (-20 << 4)) && (temp < (60 << 4)), temp);
  }
#endif

#if !defined(DISABLE_SENSOR_UNIT_TESTS)
static void testSupplyVoltageMonitor()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("SupplyVoltageMonitor");

  //  const int mv = Supply_mV.read();
//#if 0
//  OTV0P2BASE::serialPrintAndFlush("  Battery mv: ");
//  OTV0P2BASE::serialPrintAndFlush(mv, DEC);
//  OTV0P2BASE::serialPrintlnAndFlush();
//#endif
//  // During testing power supply voltage should be above ~1.7V BOD limit,
//  // and no higher than 3.6V for V0p2 boards which is RFM22 Vss limit.
//  // Note that REV9 first boards are running at 3.6V nominal!
//  AssertIsTrueWithErr((mv >= 1700) && (mv < 3700), mv);

  const bool neededPowerUp = OTV0P2BASE::powerUpADCIfDisabled();
  const uint8_t cV = Supply_cV.read();
  const uint8_t ri = Supply_cV.getRawInv();
#if 1
  OTV0P2BASE::serialPrintAndFlush("  Battery cV: ");
  OTV0P2BASE::serialPrintAndFlush(cV);
  OTV0P2BASE::serialPrintlnAndFlush();
  OTV0P2BASE::serialPrintAndFlush("  Raw inverse: ");
  OTV0P2BASE::serialPrintAndFlush(ri);
  OTV0P2BASE::serialPrintlnAndFlush();
#endif
  // During testing power supply voltage should be above ~1.7V BOD limit,
  // and no higher than 3.6V for V0p2 boards which is RFM22 Vss limit.
  // Note that REV9 first boards are running at 3.6V nominal!
  // Also, this test may get run on UNO/5V hardware.
  AssertIsTrueWithErr((cV >= 170) && (cV < 510), cV);
  // Would expect raw inverse to be <= 1023 for Vcc >= 1.1V.
  // Should be ~512 at 2.2V, ~310 at 3.3V.
  AssertIsTrueWithErr((ri >= 200) && (ri < 1023), ri);
  if(neededPowerUp) { OTV0P2BASE::powerDownADC(); }
  }
#endif




// To be called from loop() instead of main code when running unit tests.
// Tests generally flag an error and stop the test cycle with a call to panic() or error().
void loopUnitTest()
  {
  static int loopCount = 0;

  // Allow the terminal console to be brought up.
  for(int i = 3; i > 0; --i)
    {
    OTV0P2BASE::serialPrintAndFlush(F("Tests starting... "));
    OTV0P2BASE::serialPrintAndFlush(i);
    OTV0P2BASE::serialPrintlnAndFlush();
    OTV0P2BASE::sleepLowPowerMs(1000);
    }
  OTV0P2BASE::serialPrintlnAndFlush();


  // Run the tests, fastest / newest / most-fragile / most-interesting first...
  testLibVersions();
  testFastDigitalIOCalcs();
  testJSONStats(); // FIXME: move to portable unit tests.
  testJSONForTX(); // FIXME: move to portable unit tests.
  testFullStatsMessageCoreEncDec(); // FIXME: move to portable unit tests.
  testSmoothStatsValue(); // FIXME: move to portable unit tests.
  testSleepUntilSubCycleTime();


  // Sensor tests.
  // May need to be disabled if, for example, running in a simulator or on a partial board.
  // Should not involve anything too complex from the normal run-time, such as interrupts.
#if !defined(DISABLE_SENSOR_UNIT_TESTS)
  testTempSensor();
  testInternalTempSensor();
  testSupplyVoltageMonitor();
#endif


  // Announce successful loop completion and count.
  ++loopCount;
  OTV0P2BASE::serialPrintlnAndFlush();
  OTV0P2BASE::serialPrintAndFlush(F("%%% All tests completed OK, round "));
  OTV0P2BASE::serialPrintAndFlush(loopCount);
  OTV0P2BASE::serialPrintlnAndFlush();
  OTV0P2BASE::serialPrintlnAndFlush();
  OTV0P2BASE::serialPrintlnAndFlush();
  // Briefly flash the LED once to indicate successful completion of the tests.
  // (Panic/failure causes repeated rapid flash by contrast, and a hang may result in no flashes.)
  LED_HEATCALL_ON();
  tinyPause();
  LED_HEATCALL_OFF();
  // Help avoid tests spinning too fast even to see!
  // Also make panic() state flash clearly different to (faster than) this loop success/repeat.
  OTV0P2BASE::sleepLowPowerMs(2000);
  }

#endif



