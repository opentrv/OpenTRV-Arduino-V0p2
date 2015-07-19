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
                           Mike Stirling 2013
*/

/*
 FTH8V wireless radiator valve support.

 For details of protocol including sync between this and FHT8V see https://sourceforge.net/p/opentrv/wiki/FHT%20Protocol/
 */

#ifndef FHT8V_WIRELESS_RAD_VALVE_H
#define FHT8V_WIRELESS_RAD_VALVE_H

#include "V0p2_Main.h"

// Minimum and maximum FHT8V TX cycle times in half seconds: [115.0,118.5].
// Fits in an 8-bit unsigned value.
#define MIN_FHT8V_TX_CYCLE_HS (115*2)
#define MAX_FHT8V_TX_CYCLE_HS (118*2+1)

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
// Very simple devices such as PICAXE need not actually decode the complete frame,
// though this is insufficient with (for example) neighbouring houses both using such simple boiler-controllers,
// as any TRV opening in either house would turn on both boilers...
//#define RFM22_SYNC_ONLY_BCFH
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

// Decode raw bitstream into non-null command structure passed in; returns true if successful.
// Will return false if anything obviously invalid is detected such as failing parity or checksum.
// Finds and discards leading encoded 1 and trailing 0.
bool FHT8VDecodeBitStream(uint8_t const *bitStream, uint8_t const *lastByte, fht8v_msg_t *command);


// Clear both housecode parts (and thus disable local valve).
void FHT8VClearHC();
// Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
void FHT8VSetHC1(uint8_t hc);
void FHT8VSetHC2(uint8_t hc);
// Get (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control (will be 0xff until set).
uint8_t FHT8VGetHC1();
uint8_t FHT8VGetHC2();

// True once/while this node is synced with and controlling the target FHT8V valve; initially false.
bool isSyncedWithFHT8V();


// IF DEFINED: this unit may act as a thermostat controlling a local TRV (and calling for heat from the boiler).
#ifdef LOCAL_TRV
// Returns TRV if valve/radiator is to be controlled by this unit.
// Usually the case, but may not be for (a) a hub or (b) a not-yet-configured unit.
// Returns false if house code parts are set to invalid or uninitialised values (>99).
#define localFHT8VTRVEnabled() ((FHT8VGetHC1() <= 99) && (FHT8VGetHC2() <= 99))
#else
#define localFHT8VTRVEnabled() (false) // Local FHT8V TRV disabled.
#endif


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
//
// ALSO MANAGES RX FROM OTHER NODES WHEN ENABLED IN HUB MODE.
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
//
// ALSO MANAGES RX FROM OTHER NODES WHEN ENABLED IN HUB MODE.
bool FHT8VPollSyncAndTX_Next(bool allowDoubleTX = false);

// Does an extra (single) TX if safe to help ensure that the hub hears, eg in case of poor comms.
// Safe means when in sync with the valve,
// and well away from the normal transmission windows to avoid confusing the valve.
// Returns true iff a TX was done.
// This may also be omitted if the TX would not be heard by the hub anyway.
bool FHT8VDoSafeExtraTXToHub();


// Set up radio to listen for remote TRV nodes calling for heat.
// Only done if in central hub mode.
void SetupToEavesdropOnFHT8V(bool force = false);

// Stop listening out for remote TRVs calling for heat.
// Puts radio in standby mode.
// DOES NOT clear flags that indicate call for heat has been heard.
void StopEavesdropOnFHT8V(bool force = false);

// Polls radio for OpenTRV calls for heat once/if SetupToEavesdropOnFHT8V() is in effect.
// Does not misbehave (eg return false positives) even if SetupToEavesdropOnFHT8V() not set, eg has been in standby.
// If used instead of an interrupt then should probably called at least about once every 100ms.
// Returns true if any activity was detected by this call (not necessarily a full valid call),
// and comes out of eavesdropping mode to save energy.
// Does not block nor take significant time.
bool FHT8VCallForHeatPoll();

// Returns true if there is a pending accepted call for heat.
// If so a non-~0 housecode will be returned by FHT8VCallForHeatHeardGetAndClear().
bool FHT8VCallForHeatHeard();

// Atomically returns one housecode calling for heat heard since last call and clears, or ~0 if none.
uint16_t FHT8VCallForHeatHeardGetAndClear();


#ifdef ENABLE_BOILER_HUB
// Maximum number of housecodes that can be remembered and filtered for in hub selective-response mode.
// Strictly positive if compiled in.
// Limited in size partly by memory and partly to limit filtering time at RX.
//#define FHT8V_MAX_HUB_REMEMBERED_HOUSECODES 0
#define FHT8V_MAX_HUB_REMEMBERED_HOUSECODES EE_HUB_HC_FILTER_COUNT

// Count of house codes selectively listened for at hub.
// If zero then calls for heat are not filtered by house code.
uint8_t FHT8VHubListenCount();

// Get remembered house code N where N < FHT8V_MAX_HUB_REMEMBERED_HOUSECODES.
// Returns hc1:hc2 packed into a 16-bit value, with hc1 in most-significant byte.
// Returns 0xffff if requested house code index not in use.
uint16_t FHT8CHubListenHouseCodeAtIndex(uint8_t index);

// Remember and respond to calls for heat from hc1:hc2 when a hub.
// Returns true if successfully remembered (or already present), else false if cannot be remembered.
bool FHT8VHubListenForHouseCode(uint8_t hc1, uint8_t hc2);

// Forget and no longer respond to calls for heat from hc1:hc2 when a hub.
void FHT8VHubUnlistenForHouseCode(uint8_t hc1, uint8_t hc2);

// Returns true if given house code is a remembered one to accept calls for heat from, or if no filtering is being done.
// Fast, and safe to call from an interrupt routine.
bool FHT8VHubAcceptedHouseCode(uint8_t hc1, uint8_t hc2);

#else
#define FHT8V_MAX_HUB_REMEMBERED_HOUSECODES 0
#define FHT8VHubListenCount() (0)
#define FHT8CHubListenHouseCodeAtIndex(index) ((uint16_t)~0)
#define FHT8VHubListenForHouseCode(hc1, hc2) (false)
#define FHT8VHubUnlistenForHouseCode(hc1, hc2) {}
#define FHT8VHubAcceptedHouseCode(uint8_t hc1, uint8_t hc2) (true)
#endif



#endif

