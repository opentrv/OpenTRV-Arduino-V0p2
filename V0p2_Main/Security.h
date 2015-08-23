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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2015
*/

/*
 Security support for OpenTRV.
 */

#ifndef SECURITY_H
#define SECURITY_H

// How much info does a leaf node transmit about stats such as temperature and occupancy?
// This is a privacy level: the greater the value the less data is sent, eg over an insecure channel.
// Excess unencrypted stats may, for example, allow a clever burglar to work out when no one is home.
// Note that even in the 'always' setting,
// some TXes may be selectively skipped or censored for energy saving and security reasons
// eg an additional 'never transmit occupancy' flag may be set locally.
// The values correspond to levels and intermediate values not explicitly enumerated are allowed.
// Lower values mean less that security is required.
enum stats_TX_level
  {
    stTXalwaysAll = 0, // Always be prepared to transmit all stats.
    stTXmostUnsec = 0x80, // Allow TX of all but most security-sensitive stats in plaintext, eg occupancy status.
    stTXsecOnly = 0xfe, // Only transmit if the stats TX can be kept secure/encrypted.
    stTXnever = 0xff, // Never transmit status info above the minimum necessary.
  };

#include "V0p2_Main.h"

// Get the current stats transmission level (for data outbound from this node).
// May not exactly match enumerated levels; use inequalities.
// Not thread-/ISR- safe.
stats_TX_level getStatsTXLevel();


// Generate 'secure' new random byte.
// This should be essentially all entropy and unguessable.
// Likely to be slow and may force some I/O.
// Not thread-/ISR- safe.
//  * whiten  if true whiten the output a little more, but little or no extra entropy is added;
//      if false then it is easier to test if the underlying source provides new entropy reliably
uint8_t getSecureRandomByte(bool whiten = true);

// Add entropy to the pool, if any, along with an estimate of how many bits of real entropy are present.
//   * data   byte containing 'random' bits.
//   * estBits estimated number of truely securely random bits in range [0,8].
// Not thread-/ISR- safe.
void addEntropyToPool(uint8_t data, uint8_t estBits);



#if 0 // Pairing API outline.
struct pairInfo { bool successfullyPaired; };
bool startPairing(bool primary, &pairInfo);
bool continuePairing(bool primary, &pairInfo); // Incremental work.
void clearPairing(bool primary, &pairInfo);
#endif

#endif


