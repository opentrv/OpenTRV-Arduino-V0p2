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

#include "AESGCM.h"

extern "C" {
//#include "utility/includes.h"
//#include "utility/aes.h"
//#include "utility/aes_i.h"
#include "utility/aes_gcm.h"
};

bool OpenTRV::AESGCM::AES128GCM16small::encrypt(
                 const uint8_t *const key, const size_t key_len,
                 const uint8_t *const iv, const size_t iv_len,
                 const uint8_t *const plain, const size_t plain_len,
                 const uint8_t *const aad, const size_t aad_len,
                 uint8_t *const crypt,
                 uint8_t *const tag)
  {
  // Some parameter validation.
  // Only expecting 16-byte (128-bit) key for AES-128.
  if((NULL == key) || (16 != key_len)) { return(false); }
  // Only expecting 12-byte (96-bit) IV/nonce for AES-128.
  if((NULL == iv) || (12 != iv_len)) { return(false); }
  // For now insist on both plai and ADATA parts, possibly length zero.
  if(NULL == plain) { return(false); }
  if(NULL == aad) { return(false); }
  // Must have targets to write results to.
  if(NULL == crypt) { return(false); }
  if(NULL == tag) { return(false); }

  return(0 == aes_gcm_ae(context,
                         key, key_len,
                         iv, iv_len,
                         plain, plain_len,
                         aad, aad_len,
                         crypt, tag));
  }

bool OpenTRV::AESGCM::AES128GCM16small::decrypt(
                const uint8_t *const key, const size_t key_len,
                const uint8_t *const iv, const size_t iv_len,
                const uint8_t *const crypt, const size_t crypt_len,
                const uint8_t *const aad, const size_t aad_len, const uint8_t *const tag,
                uint8_t *const plain)
  {
  // Some parameter validation.
  // Only expecting 16-byte (128-bit) key for AES-128.
  if((NULL == key) || (16 != key_len)) { return(false); }
  // Only expecting 12-byte (96-bit) IV/nonce for AES-128.
  if((NULL == iv) || (12 != iv_len)) { return(false); }
  // For now insist on both plain and ADATA parts, possibly length zero.
  if(NULL == crypt) { return(false); }
  if(NULL == aad) { return(false); }
  if(NULL == tag) { return(false); }
  // Must have targets to write results to.
  if(NULL == plain) { return(false); }

  return(0 == aes_gcm_ad(context,
                         key, key_len,
                         iv, iv_len,
                         crypt, crypt_len,
                         aad, aad_len, tag,
                         plain));
  }
