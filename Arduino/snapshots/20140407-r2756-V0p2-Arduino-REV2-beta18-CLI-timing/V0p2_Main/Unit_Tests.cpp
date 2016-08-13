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
*/

/*
 Development-time unit tests (NOT part of production code).
 */

#include <util/atomic.h>

#include "Unit_Tests.h"

#ifdef UNIT_TESTS // Exclude this code from production systems.

#include "Control.h"
#include "EEPROM_Utils.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "Power_Management.h"
#include "PRNG.h"
#include "RTC_Support.h"
#include "Serial_Debug.h"


// Error exit from failed unit test, one int parameter to print...
static void error(int err, int line)
  {
  for( ; ; )
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("***Test FAILED.*** val=");
    DEBUG_SERIAL_PRINT(err);
    if(0 != line)
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING(" at line ");
      DEBUG_SERIAL_PRINT(line);
      }
    DEBUG_SERIAL_PRINTLN();
    sleepLowPowerMs(1000);
    }
  }

// Test expression and bucket out with error if false, else continue.
static void AssertIsTrue(bool x) { if(!(x)) { error(0, 0); } }
static void AssertIsTrue(bool x, int err) { if(!(x)) { error(err, 0); } }
static void AssertIsTrue(bool x, int err, int line) { if(!(x)) { error(err, line); } }



// Self-test of EEPROM functioning (and smart/split erase/write).
// Will not usually perform any wear-inducing activity.
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

// Test of FHT8V bitstream encoding and decoding.
// Aborts with panic() upon failure.
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
  uint8_t *result1 = FHT8VCreate200usBitStreamBptr(buf, &command);
  AssertIsTrue(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrue((result1 - buf == 38), result1-buf); // Check correct length.
  AssertIsTrue(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrue(((uint8_t)0xe3) == buf[6], buf[6]); // Check end of preamble.
  AssertIsTrue(((uint8_t)0xce) == buf[34], buf[34]); // Check part of checksum.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded), 0, __LINE__);
  AssertIsTrue(13 == commandDecoded.hc1, commandDecoded.hc1, __LINE__);
  AssertIsTrue(73 == commandDecoded.hc2, commandDecoded.hc2, __LINE__);
  AssertIsTrue(0x26 == commandDecoded.command, commandDecoded.command, __LINE__);
  AssertIsTrue(0 == commandDecoded.extension, commandDecoded.extension, __LINE__);

  // Encode shortest-possible (all-zero-bits) FHT8V command as 200us-bit-stream...
  command.hc1 = 0;
  command.hc2 = 0;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  command.command = 0;
  command.extension = 0;
  result1 = FHT8VCreate200usBitStreamBptr(buf, &command);
  AssertIsTrue(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrue((result1 - buf == 35), result1-buf); // Check correct length.
  AssertIsTrue(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded), 0, __LINE__);
  AssertIsTrue(0 == commandDecoded.hc1, commandDecoded.hc1, __LINE__);
  AssertIsTrue(0 == commandDecoded.hc2, commandDecoded.hc2, __LINE__);
  AssertIsTrue(0 == commandDecoded.command, commandDecoded.command, __LINE__);
  AssertIsTrue(0 == commandDecoded.extension, commandDecoded.extension, __LINE__);

  // Encode longest-possible (as many 1-bits as possible) FHT8V command as 200us-bit-stream...
  command.hc1 = 0xff;
  command.hc2 = 0xff;
#ifdef FHT8V_ADR_USED
  address = 0xff;
#endif
  command.command = 0xff;
  command.extension = 0xff;
  result1 = FHT8VCreate200usBitStreamBptr(buf, &command);
  AssertIsTrue(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrue(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded), 0, __LINE__);
  AssertIsTrue(0xff == commandDecoded.hc1, commandDecoded.hc1, __LINE__);
  AssertIsTrue(0xff == commandDecoded.hc2, commandDecoded.hc2, __LINE__);
#ifdef FHT8V_ADR_USED
  AssertIsTrue(0xff == commandDecoded.address, commandDecoded.address, __LINE__);
#endif  AssertIsTrue(0xff == commandDecoded.command, commandDecoded.command, __LINE__);
  AssertIsTrue(0xff == commandDecoded.extension, commandDecoded.extension, __LINE__);
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
    const uint_least16_t mb = getMinutesSinceMidnightLT();
    persistRTC();
    restoreRTC();
    const uint_least16_t ma = getMinutesSinceMidnightLT();
    // Check that persist/restore did not change live minutes value at least, within the 15-minute quantum used.
    minutesPersistOK = (mb/15 == ma/15);
    }
    AssertIsTrue(minutesPersistOK, 1);
  }


// Maximum number of identical nominally random bits (or values with approx one bit of entropy) in a row tolerated.
// Set large enough that even soak testing for many hours should not trigger a failure if behaviour is plausibly correct.
#define MAX_IDENTICAL_BITS_SEQUENTIALLY 32
// Tests of entropy gathering routines.
void testEntropyGathering()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testEntropyGathering");

  // Test WDT jitter: assumed about 1 bit of entropy per call/result.
  //DEBUG_SERIAL_PRINT_FLASHSTRING("jWDT... ");
  const uint8_t jWDT = clockJitterWDT();
  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY; --i >= 0; )
    {
    if(jWDT != clockJitterWDT()) { /* DEBUG_SERIAL_PRINT(MAX_IDENTICAL_BITS_SEQUENTIALLY - i); */ break; } // Indicate how many goes it took to get a new value.
    AssertIsTrue(0 != i, i, __LINE__); // Generated too many identical values in a row. 
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
    if(jRTC != clockJitterRTC()) { /* DEBUG_SERIAL_PRINT(MAX_IDENTICAL_BITS_SEQUENTIALLY - i); */ break; } // Indicate how many goes it took to get a new value.
    AssertIsTrue(0 != i, i, __LINE__); // Generated too many identical values in a row. 
    }
  //DEBUG_SERIAL_PRINT_FLASHSTRING(" 1st=");
  //DEBUG_SERIAL_PRINTFMT(jRTC, BIN);
  //DEBUG_SERIAL_PRINTLN();

  // Test full-byte jitter: assumed about 8 bits of entropy per call/result.
  //DEBUG_SERIAL_PRINT_FLASHSTRING("jByte... ");
  const uint8_t t0j = getSubCycleTime();
  while(t0j == getSubCycleTime()) { } // Wait for sub-cycle time to roll to toughen test.
  const uint8_t jByte = clockJitterEntropyByte();
  const uint8_t t1j = getSubCycleTime();
  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY/8; --i >= 0; )
    {
    if(jByte != clockJitterEntropyByte()) { /* DEBUG_SERIAL_PRINT(MAX_IDENTICAL_BITS_SEQUENTIALLY/8 - i); */ break; } // Indicate how many goes it took to get a new value.
    AssertIsTrue(0 != i, i, __LINE__); // Generated too many identical values in a row. 
    }
  //DEBUG_SERIAL_PRINT_FLASHSTRING(" 1st=");
  //DEBUG_SERIAL_PRINTFMT(jByte, BIN);
  //DEBUG_SERIAL_PRINT_FLASHSTRING(", ticks=");
  //DEBUG_SERIAL_PRINT((uint8_t)(t1j - t0j - 1));
  //DEBUG_SERIAL_PRINTLN();
  }

// Test sleepUntilSubCycleTime() routine.
void testSleepUntilSubCycleTime()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("testSleepUntilSubCycleTime");

  const uint8_t start = getSubCycleTime();

  // Check that this correctly notices/vetoes attempt to sleep until time already past.
  if(start > 0) { AssertIsTrue(!sleepUntilSubCycleTime(start-1), 1); }

  // Don't attempt rest of test if near the end of the current minor cycle...
  if(start > (GSCT_MAX/2)) { return; }

  // Set a random target significantly before the end of the current minor cycle.
  const uint8_t sleepTicks = 2 + (random() % (GSCT_MAX-start-2));
  const uint8_t target = start + sleepTicks;
  AssertIsTrue(target > start);
  AssertIsTrue(target < GSCT_MAX);

  // Call should succeed.
  AssertIsTrue(sleepUntilSubCycleTime(target), 2);

  // Call should return with some of specified target tick still to run...
  const uint8_t end = getSubCycleTime();
  AssertIsTrue(target == end, end);

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
  // Reset to known state; not normally permistted and only exists for unit tests.
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
    AssertIsTrue(i<<4 == expandTempC16(compressTempC16(i<<4)), i);
    }
  // Ensure that out-of-range inputs are coerced to the limits.
  AssertIsTrue(0 == expandTempC16(compressTempC16(-1)), -1);
  AssertIsTrue((100<<4) == expandTempC16(compressTempC16(101<<4)), 101);
  AssertIsTrue(COMPRESSION_C16_CEIL_VAL_AFTER == compressTempC16(102<<4), COMPRESSION_C16_CEIL_VAL_AFTER); // Verify ceiling.
  AssertIsTrue(COMPRESSION_C16_CEIL_VAL_AFTER < 0xff);
  // Ensure that 'unset' compressed value expands to 'unset' uncompressed value.
  AssertIsTrue(STATS_UNSET_INT == expandTempC16(STATS_UNSET_BYTE));
  }




// To be called from loop() instead of main code when running unit tests.
// Tests generally flag an error and stop the test cycle with a call to panic() or error().
void unitTestLoop()
  {
  // Allow the terminal console to be brought up.
  for(int i = 3; i > 0; --i)
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("Tests starting shortly... ");
    DEBUG_SERIAL_PRINT(i);
    DEBUG_SERIAL_PRINTLN();
    sleepLowPowerMs(1000);
    }
  DEBUG_SERIAL_PRINTLN();


  // Run the tests, fastest / newest / most-fragile / most-interesting first...
  testTempCompand();
  testRNG8();
  testEntropyGathering();
  testRTCPersist();
  testFHTEncoding();
  testEEPROM();
  testSmoothStatsValue();
  testSleepUntilSubCycleTime();


  DEBUG_SERIAL_PRINTLN_FLASHSTRING("All tests completed OK!");
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("");
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("");
  }


#endif



