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
 Simple/small/fast Pseudo-Random Number Generator support.
 
 For when rand()/random() are too big/slow/etc.
 */

#ifndef PRNG_H
#define PRNG_H

#include <stdint.h>

#include "V0p2_Main.h"

// "RNG8" 8-bit 'ultra fast' PRNG suitable for 8-bit microcontrollers: low bits probably least good.
// NOT in any way suitable for crypto, but may be good to help avoid TX collisions, etc.
// C/o: http://www.electro-tech-online.com/general-electronics-chat/124249-ultra-fast-pseudorandom-number-generator-8-bit.html
// Reseed with 3 bytes of state.
void seedRNG8(uint8_t s1, uint8_t s2, uint8_t s3); // Originally called 'init_rnd()'.
// Get 1 byte of uniformly-distributed unsigned values.
uint8_t randRNG8(); // Originally called 'randomize()'.
#ifdef UNIT_TESTS
// Reset to known state; only use in unit testing as this destroys any residual entropy.
void resetRNG8();
#endif

#endif


