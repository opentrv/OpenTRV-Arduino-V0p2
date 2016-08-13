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

/*Unit test routines for library code.
 */

// Include the library under test.
#include <AESGCM.h>


void setup()
  {
  // initialize serial communications at 9600 bps.
  Serial.begin(9600); 
  }



// Error exit from failed unit test, one int parameter and the failing line number to print...
// Expects to terminate like panic() with flashing light can be detected by eye or in hardware if required.
static void error(int err, int line)
  {
  for( ; ; )
    {
    Serial.print(F("***Test FAILED*** val="));
    Serial.print(err, DEC);
    Serial.print(F(" =0x"));
    Serial.print(err, HEX);
    if(0 != line)
      {
      Serial.print(F(" at line "));
      Serial.print(line);
      }
    Serial.println();
//    LED_HEATCALL_ON();
//    tinyPause();
//    LED_HEATCALL_OFF();
//    sleepLowPowerMs(1000);
    delay(1000);
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


// Check that correct version of library is under test.
static void testLibVersion()
  {
  Serial.println("LibVersion");
  AssertIsEqual(0, ARDUINO_LIB_AESGCM_VERSION_MAJOR);
  AssertIsEqual(1, ARDUINO_LIB_AESGCM_VERSION_MINOR);
  }

static const int AES_KEY_SIZE = 128; // in bits
static const int GCM_NONCE_LENGTH = 12; // in bytes
static const int GCM_TAG_LENGTH = 16; // in bytes (default 16, 12 possible)

//    /**Test on specific simple plaintext/ADATA.key value.
//     * Can be used to test MCU-based implementations.
//     */
//    @Test
//    public void testAESGCMAll0() throws Exception
//        {
//        final byte[] input = new byte[30]; // All-zeros input.
//
//        // All-zeros key.
//        final SecretKey key = new SecretKeySpec(new byte[AES_KEY_SIZE/8], 0, AES_KEY_SIZE/8, "AES");
//        final byte[] nonce = new byte[GCM_NONCE_LENGTH]; // All-zeros nonce.
//        final byte[] aad = new byte[4]; // All-zeros ADATA.
//
//        // Encrypt...
//        final Cipher cipherE = Cipher.getInstance("AES/GCM/NoPadding", "SunJCE"); // JDK 7 breaks here..
//        final GCMParameterSpec spec = new GCMParameterSpec(GCM_TAG_LENGTH * 8, nonce);
//        cipherE.init(Cipher.ENCRYPT_MODE, key, spec);
//        cipherE.updateAAD(aad);
//        final byte[] cipherText = cipherE.doFinal(input);
//        assertEquals(input.length + GCM_TAG_LENGTH, cipherText.length);
//        System.out.println(DatatypeConverter.printHexBinary(cipherText));
//        assertEquals((16 == GCM_TAG_LENGTH) ?
//            "0388DACE60B6A392F328C2B971B2FE78F795AAAB494B5923F7FD89FF948B614772C7929CD0DD681BD8A37A656F33" :
//            "0388DACE60B6A392F328C2B971B2FE78F795AAAB494B5923F7FD89FF948B614772C7929CD0DD681BD8A3",
//            DatatypeConverter.printHexBinary(cipherText));
//
//        // Decrypt...
//        final Cipher cipherD = Cipher.getInstance("AES/GCM/NoPadding", "SunJCE"); // JDK 7 breaks here..
//        cipherD.init(Cipher.DECRYPT_MODE, key, spec);
//        cipherD.updateAAD(aad);
//        final byte[] plainText = cipherD.doFinal(cipherText);
//        // Check that the decryption result matches.
//        assertTrue((Arrays.equals(input, plainText)));
//        }

// Check that all zeros key, plaintext and ADATA gives the correct result.
static void testAESGCMAll0()
  {
  Serial.println("AESGCMAll0");
  // Inputs to encryption.
  uint8_t input[30]; // All-zeros input, typical input size.
  memset(input, 0, sizeof(input));
  uint8_t key[AES_KEY_SIZE/8];
  memset(key, 0, sizeof(key)); // All-zeros key.
  uint8_t nonce[GCM_NONCE_LENGTH];
  memset(nonce, 0, sizeof(nonce)); // All-zeros nonce.
  uint8_t aad[4];
  memset(aad, 0, sizeof(aad)); // All-zeros ADATA.
  // Space for outputs from encryption.
  uint8_t tag[GCM_TAG_LENGTH]; // Space for tag.
  uint8_t cipherText[sizeof(input)]; // Space for encrypted text.
  // Instance to perform enc/dec.
  OpenTRV::AESGCM::AES128GCM16small eo;
  // Do encryption.
  AssertIsTrue(eo.encrypt(
                key, (size_t) sizeof(key),
                nonce, sizeof(nonce),
                input, sizeof(input),
                aad, sizeof(aad),
                cipherText, tag));
  // Check some of the cipher text and tag.
//            "0388DACE60B6A392F328C2B971B2FE78F795AAAB494B5923F7FD89FF948B614772C7929CD0DD681BD8A37A656F33" :
  AssertIsEqual(0x03, cipherText[0]);
  AssertIsEqual(0x88, cipherText[1]);
  AssertIsEqual(0x8b, cipherText[sizeof(cipherText)-1]);
  AssertIsEqual(0x61, tag[0]);
  AssertIsEqual(0x33, tag[15]);

  // Decrypt...
  uint8_t plain[sizeof(cipherText)]; // Space for decrypted text.
  AssertIsTrue(eo.decrypt(
                key, (size_t) sizeof(key),
                nonce, sizeof(nonce),
		cipherText, sizeof(cipherText),
		aad, sizeof(aad), tag,
                plain));
  AssertIsEqual(0, memcmp(input, plain, sizeof(input))); // 0 indicates plain text recovered correctly.
  }





// To be called from loop() instead of main code when running unit tests.
// Tests generally flag an error and stop the test cycle with a call to panic() or error().
void loop()
  {
  static int loopCount = 0;

  // Allow the terminal console to be brought up.
  for(int i = 3; i > 0; --i)
    {
    Serial.print(F("Tests starting... "));
    Serial.print(i);
    Serial.println();
    delay(1000);
    }
  Serial.println();


  // Run the tests, fastest / newest / most-fragile / most-interesting first...
  testLibVersion();
  testAESGCMAll0();




  // Announce successful loop completion and count.
  ++loopCount;
  Serial.println();
  Serial.print(F("%%% All tests completed OK, round "));
  Serial.print(loopCount);
  Serial.println();
  Serial.println();
  Serial.println();
  delay(2000);
  }
