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

Author(s) / Copyright (s): Damon Hart-Davis 2014
                           Gary Gladman 2014
*/

/*
 Security support for OpenTRV.
 */

#ifndef SECURITY_H
#define SECURITY_H

#include "V0p2_Main.h"

// How much info does a leaf node transmit about stats such as temperature and occupancy?
// Excess unencrypted stats may, for example, allow a clever burglar to work out when no one is home.
// Note that even in the 'always' setting,
// some TXes may be selectively skipped or censored for energy saving and security reasons
// eg an additional 'never transmit occupancy' flag may be set locally.
// The values correspond to levels and intermediate values not explicitly enumerated may be allowed.
// Lower values mean less security is required.
enum stats_TX_level
  {
    stTXalwaysAll = 0, // Always be prepared to transmit all stats.
    stTXmostUnsec, // Allow TX of all but most security-sensitive stats in plaintext, eg occupancy status.
    stTXsecOnly, // Only transmit if the stats TX can be kept secure/encrypted.
    stTXnever = 0xff // Never transmit status info above the minimum necessary.
  };

// Get the current stats transmission level (for data outbound from this node).
// May not exactly match enumerated levels; use inequalities.
stats_TX_level getStatsTXLevel();


// Generate 'secure' new random byte.
// This should be essentially all entropy and unguessable.
// Likely to be slow and may force some I/O.
uint8_t getSecureRandomByte();

// Add entropy to the pool, if any, along with an estimate of how many bits of real entropy are present.
//   * data   byte containing 'random' bits.
//   * estBits estimated number of truely securely random bits in range [0,8].
void addEntropyToPool(uint8_t data, uint8_t estBits);

#endif


