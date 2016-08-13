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

#ifndef ARDUINO_LIB_AESGCM_H
#define ARDUINO_LIB_AESGCM_H

#define ARDUINO_LIB_AESGCM_VERSION_MAJOR 0
#define ARDUINO_LIB_AESGCM_VERSION_MINOR 1

#include <stdint.h>
#include <string.h>


namespace OpenTRV { namespace AESGCM {

// Class to support small code AES-128 GCM 16-byte-tag encryption and decryption.
// ***DO NOT DERIVE FROM THIS CLASS.
// Note: this class instance is large (~256 bytes) so stack allocation may not be a good idea.
// TODO: carries all necessary state without needing (eg) malloc.
class AES128GCM16small
  {
  public:
     // Block size for AES operations (bytes).
     static const int AES_block_size = 16;

     // Working space needed for encryption (bytes).
     static const int AES_context_size = (4 * 4 * 15 + 4);

    /*AES-GCM encrypt - GCM-AE_K(IV, P, A).
      16-byte (128-bit) input key.
      12-byte (96-bit) input nonce/IV.
      16-byte (128-bit) output tag.
      ??? Does data need to be multiple of block size (16-bytes) ???
      Returns true in case of success.
    */
    bool encrypt(const uint8_t *key, size_t key_len,
                 const uint8_t *iv, size_t iv_len,
                 const uint8_t *plain, size_t plain_len,
                 const uint8_t *aad, size_t aad_len,
                 uint8_t *crypt,
                 uint8_t *tag);

    /*AES-GCM decrypt - GCM-AD_K(IV, C, A, T).
      16-byte (128-bit) input key.
      12-byte (96-bit) input nonce/IV.
      16-byte (128-bit) input tag.
      Output plain-text size same as input cipher-text.
      ??? Does data need to be multiple of block size (16-bytes) ???
      Returns true in case of success.
      */
    bool decrypt(const uint8_t *key, size_t key_len,
                 const uint8_t *iv, size_t iv_len,
                 const uint8_t *crypt, size_t crypt_len,
                 const uint8_t *aad, size_t aad_len, const uint8_t *tag,
                 uint8_t *plain);

//    // Erase any sensitive state, eg once an encryption/decryption is done.
//    // Will be called during object destruction.
//    void cleanup() { memset(context, 0, sizeof(context)); }
//
//    // Non-virtual destructor to clear up private state
//    // and free resources if any.
//    ~AES128GCM16small() { cleanup(); }

  private:
    // Private context; should be cleared when finished with.
    uint8_t context[AES_context_size];
  };

} }


#endif
