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
Note: original XABC code from "EternityForest" appears to be in the public domain.
*/

/*
 Simple/small/fast Pseudo-Random Number Generator support.
 
 For when rand()/random() are too big/slow/etc.
 */

#include "PRNG.h"


// Other potental fast/OK PRNGs...
// DHD201305022: consider use of built-in optimised CRC routines as PRNG (with constant data input).


// "RNG8" 8-bit 'ultra fast' PRNG suitable for 8-bit microcontrollers: low bits probably least good.
// NOT in any way suitable for crypto, but may be good to help avoid TX collisions, etc.
// C/o: http://www.electro-tech-online.com/general-electronics-chat/124249-ultra-fast-pseudorandom-number-generator-8-bit.html
// User "EternityForest": http://eternityforest.wordpress.com/
// Reseed with 3 bytes of state.
void seedRNG8(uint8_t s1, uint8_t s2, uint8_t s3); // Originally called 'init_rnd()'.
// Get 1 byte of uniformly-distributed unsigned values.
uint8_t randRNG8(); // Originally called 'randomize()'.
// RNG8 working state.
static uint8_t a, b, c;
// DHD20130603: avoid the hidden counter always starting at zero c/o some per-build state.
// Derived from linker-driven pointer base plus hash of compilation timestamp, with little run-time cost.
static uint8_t x = (uint8_t)(intptr_t) ((&c) + (uint8_t)((__TIME__[7] * 17) ^ (__TIME__[6]) ^ (__TIME__[4] << 11)));

// Original code as provided, with minor renaming/reformatting/editing as required.

//X ABC Algorithm Random Number Generator for 8-Bit Devices:
//This is a small PRNG, experimentally verified to have at least a 50 million byte period
//by generating 50 million bytes and observing that there were no overlapping sequences and repeats.
//This generator passes serial correlation, entropy , Monte Carlo Pi value, arithmetic mean,
//And many other statistical tests. This generator may have a period of up to 2^32, but this has
//not been verified.
//
// By XORing 3 bytes into the a,b, and c registers, you can add in entropy from 
//an external source easily.
//
//This generator is free to use, but is not suitable for cryptography due to its short period(by
//cryptographic standards) and simple construction. No attempt was made to make this generator 
// suitable for cryptographic use.
//
//Due to the use of a constant counter, the generator should be resistant to latching up.
//A significant performance gain is had in that the x variable is only ever incremented.
//
//Only 4 bytes of ram are needed for the internal state, and generating a byte requires 3 XORs ,
//2 ADDs, one bit shift right , and one increment. Difficult or slow operations like multiply, etc 
//were avoided for maximum speed on ultra low power devices.
void seedRNG8(uint8_t s1, uint8_t s2, uint8_t s3) // Originally init_rng(s1,s2,s3) //Can also be used to seed the rng with more entropy during use.
  {
  //XOR new entropy into key state
  a ^=s1;
  b ^=s2;
  c ^=s3;
  x++;
  a = (a^c^x);
  b = (b+a);
  c = ((c+(b>>1))^a);
  }
//
uint8_t randRNG8() // Originally unsigned char randomize().
  {
  x++;               //x is incremented every round and is not affected by any other variable
  a = (a^c^x);       //note the mix of addition and XOR
  b = (b+a);         //And the use of very few instructions
  c = ((c+(b>>1))^a);  //the right shift is to ensure that high-order bits from b can affect  
  return(c);         //low order bits of other variables
  }
//
#ifdef UNIT_TESTS
// Reset to known state; only allow in unit testing as this destroys any residual entropy.
void resetRNG8()
  {
  x = 0;
  a = 0;
  b = 0;
  c = 0;
  };
#endif


