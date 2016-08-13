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

Author(s) / Copyright (s): Damon Hart-Davis 2013
                           Mike Stirling 2013
*/

/*
 FTH8V wireless radiator valve support.

 For details of protocol including sync between this and FHT8V see https://sourceforge.net/p/opentrv/wiki/FHT%20Protocol/
 */

#ifndef FHT8V_WIRELESS_RAD_VALVE_H
#define FHT8V_WIRELESS_RAD_VALVE_H

#include "V0p2_Main.h"

// Type for information content of FHT8V message.
// Omits the address field unless it is actually used.
typedef struct
  {
  uint8_t hc1;
  uint8_t hc2;
#ifdef FHT8V_ADR_USED
  uint8_t address;
#endif
  uint8_t command;
  uint8_t extension;
  } fht8v_msg_t;


// Create stream of bytes to be transmitted to FHT80V at 200us per bit, msbit of each byte first.
// Byte stream is terminated by 0xff byte which is not a possible valid encoded byte.
// On entry the populated FHT8V command struct is passed by pointer.
// On exit, the memory block starting at buffer contains the low-byte, msbit-first bit, 0xff terminated TX sequence.
// The maximum and minimum possible encoded message sizes are 35 (all zero bytes) and 45 (all 0xff bytes) bytes long.
// Note that a buffer space of at least 46 bytes is needed to accommodate the longest-possible encoded message and terminator.
// Returns pointer to the terminating 0xff on exit.
uint8_t *FHT8VCreate200usBitStreamBptr(uint8_t *bptr, const fht8v_msg_t *command);
#define MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE 46 // For longest-possible encoded command plus terminating 0xff.


#ifdef USE_MODULE_RFM22RADIOSIMPLE
// Provide RFM22/RFM23 register settings for use with FHT8V, stored in (read-only) program/Flash memory.
// Consists of a sequence of (reg#,value) pairs terminated with a $ff register.  The reg#s are <128, ie top bit clear.
// Magic numbers c/o Mike Stirling!
extern const uint8_t FHT8V_RFM22_Reg_Values[][2] PROGMEM;

// IF DEFINED: only use RFM22 RX sync to indicate call for heat from boiler rather than reading the FHT8V frame content.
// This can remain true on the transmission side even for more sophisticated receivers.
// Very simple devices such as PICAXE need not actually decode frame,
// though this is insufficient with (for example) neighbouring houses both using such simple boiler-controllers,
// as any TRV opening in either house would turn on both boilers...
#define RFM22_SYNC_ONLY_BCFH
#endif


// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) at bptr.
// The TRVPercentOpen value is used to generate the frame.
// On entry hc1, hc2 (and addresss if used) must be set correctly; this sets command and extension.
// The generated command frame can be resent indefinitely.
// The command buffer used must be (at least) FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE bytes.
// Returns pointer to the terminating 0xff on exit.
uint8_t *FHT8VCreateValveSetCmdFrame_r(uint8_t *bptr, fht8v_msg_t *command, const uint8_t TRVPercentOpen);
#ifdef RFM22_SYNC_ONLY_BCFH
#define FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE (4+MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE) // Buffer needed with RFM22-friendly extra header.
#else
#define FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE // Buffer needed without RFM22-friendly extra header.
#endif
// Approximate maximum transmission (TX) time for FHT8V command frame in ms; strictly positive.
#define FHT8V_APPROX_MAX_TX_MS ((((FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE-1)*8) + 4) / 5)

// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) in the shared TX buffer.
// The getTRVPercentOpen() result is used to generate the frame.
// HC1 and HC2 are fetched with the FHT8VGetHC1() and FHT8VGetHC2() calls, and address is always 0.
// The generated command frame can be resent indefinitely.
void FHT8VCreateValveSetCmdFrame();

// Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
void FHT8VSetHC1(uint8_t hc);
void FHT8VSetHC2(uint8_t hc);
// Get (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control (will be 0xff until set).
uint8_t FHT8VGetHC1();
uint8_t FHT8VGetHC2();

// True once/while this node is synced with and controlling the target FHT8V valve; initially false.
bool isSyncedWithFHT8V();

// True if FHT8V valve is believed to be open under instruction from this system; undefined if not in sync.
bool getFHT8V_isValveOpen();

// Call to reset comms with FHT8V valve and force resync.
// Resets values to power-on state so need not be called in program preamble if variables not tinkered with.
void FHT8VSyncAndTXReset();

// Call at start of minor cycle to manage initial sync and subsequent comms with FHT8V valve.
// Conveys this system's TRVPercentOpen value to the FHT8V value periodically,
// setting FHT8V_isValveOpen true when the valve will be open/opening provided it received the latest TX from this system.
//
//   * allowDoubleTX  if true then a double TX is allowed for better resilience, but at cost of extra time and energy
//
// Uses its static/internal transmission buffer, and always leaves it in valid date.
//
// Iff this returns true then call FHT8VPollSyncAndTX_Next() at or before each 0.5s from the cycle start
// to allow for possible transmissions.
//
// See https://sourceforge.net/p/opentrv/wiki/FHT%20Protocol/ for the underlying protocol.
bool FHT8VPollSyncAndTX_First(bool allowDoubleTX = false);

// If FHT8VPollSyncAndTX_First() returned true then call this each 0.5s from the start of the cycle, as nearly as possible.
// This allows for possible transmission slots on each half second.
//
//   * allowDoubleTX  if true then a double TX is allowed for better resilience, but at cost of extra time and energy
//
// This will sleep (at reasonably low power) as necessary to the start of its TX slot,
// else will return immediately if no TX needed in this slot.
//
// Iff this returns false then no further TX slots will be needed
// (and thus this routine need not be called again) on this minor cycle
bool FHT8VPollSyncAndTX_Next(bool allowDoubleTX = false);

#endif
