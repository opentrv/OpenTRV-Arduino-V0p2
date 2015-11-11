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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2015
                           Mike Stirling 2013
*/

/*
 FTH8V wireless radiator valve support.

 For details of protocol including sync between this and FHT8V see https://sourceforge.net/p/opentrv/wiki/FHT%20Protocol/
 */

#ifndef FHT8V_WIRELESS_RAD_VALVE_H
#define FHT8V_WIRELESS_RAD_VALVE_H

#include "V0p2_Main.h"

#include "Messaging.h"


// FHT8V radio-controlled radiator valve, using FS20 protocol.
// DHD2151111: Work-In-Progress!
class FHT8VRadValve : public OTRadValve::AbstractRadValve
  {
  public:
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

    // Minimum and maximum FHT8V TX cycle times in half seconds: [115.0,118.5].
    // Fits in an 8-bit unsigned value.
    static const uint8_t MIN_FHT8V_TX_CYCLE_HS = (115*2);
    static const uint8_t MAX_FHT8V_TX_CYCLE_HS = (118*2+1);

    // Compute interval (in half seconds) between TXes for FHT8V given house code 2.
    // (In seconds, the formula is t = 115 + 0.5 * (HC2 & 7) seconds, in range [115.0,118.5].)
    static inline uint8_t FHT8VTXGapHalfSeconds(const uint8_t hc2) { return((hc2 & 7) + 230); }

    // For longest-possible encoded FHT8V/FS20 command in bytes plus terminating 0xff.
    static const uint8_t MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE = 46;
    // Create stream of bytes to be transmitted to FHT80V at 200us per bit, msbit of each byte first.
    // Byte stream is terminated by 0xff byte which is not a possible valid encoded byte.
    // On entry the populated FHT8V command struct is passed by pointer.
    // On exit, the memory block starting at buffer contains the low-byte, msbit-first bit, 0xff terminated TX sequence.
    // The maximum and minimum possible encoded message sizes are 35 (all zero bytes) and 45 (all 0xff bytes) bytes long.
    // Note that a buffer space of at least 46 bytes is needed to accommodate the longest-possible encoded message plus terminator.
    // This FHT8V messages is encoded with the FS20 protocol.
    // Returns pointer to the terminating 0xff on exit.
    static uint8_t *FHT8VCreate200usBitStreamBptr(uint8_t *bptr, const FHT8VRadValve::fht8v_msg_t *command);

    // Typical FHT8V 'open' percentage, though partly depends on valve tails, etc.
    // This is set to err on the side of slightly open to allow
    // the 'linger' feature to work to help boilers dump heat with pump over-run
    // when the the boiler is turned off.
    // Actual values observed by DHD range from 6% to 25%.
    static const uint8_t TYPICAL_MIN_PERCENT_OPEN = 10;

    // Get estimated minimum percentage open for significant flow for this device; strictly positive in range [1,99].
    // Defaults to typical value from observation.
    virtual uint8_t getMinPercentOpen() const { return(TYPICAL_MIN_PERCENT_OPEN); }
  };



#ifdef USE_MODULE_FHT8VSIMPLE




#ifdef USE_MODULE_RFM22RADIOSIMPLE
// Provide RFM22/RFM23 register settings for use with FHT8V, stored in (read-only) program/Flash memory.
// Consists of a sequence of (reg#,value) pairs terminated with a $ff register.  The reg#s are <128, ie top bit clear.
// Magic numbers c/o Mike Stirling!
extern const uint8_t FHT8V_RFM22_Reg_Values[][2] PROGMEM;
// IF DEFINED: use RFM22 RX sync to indicate something for the hubs to listen to, including but not only a call for heat.
// (Older receivers relied on just this RFM22/23 sync which is no longer enough.
// Very simple devices such as PICAXE did not actually decode the complete frame,
// and this is ultimately insufficient with (for example) neighbouring houses both using such simple boiler-controllers,
// as any TRV opening in either house would turn on both boilers...)
#define RFM22_SYNC_BCFH
#endif


// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) at bptr.
// The TRVPercentOpen value is used to generate the frame.
// On entry hc1, hc2 (and addresss if used) must be set correctly; this sets command and extension.
// The generated command frame can be resent indefinitely.
// The command buffer used must be (at least) FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE bytes plus extra preamble and trailers.
// Returns pointer to the terminating 0xff on exit.
uint8_t *FHT8VCreateValveSetCmdFrame_r(uint8_t *bptr, FHT8VRadValve::fht8v_msg_t *command, const uint8_t TRVPercentOpen);

#ifdef RFM22_SYNC_BCFH
#define FHT8V_MAX_EXTRA_PREAMBLE_BYTES RFM22_PREAMBLE_BYTES
#else
#define FHT8V_MAX_EXTRA_PREAMBLE_BYTES 0
#endif
#define FHT8V_MAX_EXTRA_TRAILER_BYTES (1+max(MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES,FullStatsMessageCore_MAX_BYTES_ON_WIRE))
#define FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE (FHT8V_MAX_EXTRA_PREAMBLE_BYTES + (FHT8VRadValve::MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE) + FHT8V_MAX_EXTRA_TRAILER_BYTES) // Buffer space needed.


// Approximate maximum transmission (TX) time for FHT8V command frame in ms; strictly positive.
// ~80ms upwards.
#define FHT8V_APPROX_MAX_TX_MS ((((FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE-1)*8) + 4) / 5)

// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) at bptr with optional headers and trailers.
//   * TRVPercentOpen value is used to generate the frame
//   * doHeader  if true then an extra RFM22/23-friendly 0xaaaaaaaa sync header is preprended
//   * trailer  if not null then a stats trailer is appended, built from that info plus a CRC
//   * command  on entry hc1, hc2 (and addresss if used) must be set correctly, this sets the command and extension; never NULL
// The generated command frame can be resent indefinitely.
// The output buffer used must be (at least) FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE bytes.
// Returns pointer to the terminating 0xff on exit.
uint8_t *FHT8VCreateValveSetCmdFrameHT_r(uint8_t *bptr, bool doHeader, FHT8VRadValve::fht8v_msg_t *command, uint8_t TRVPercentOpen, const FullStatsMessageCore_t *trailer);

// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) in the shared TX buffer.
// The getTRVPercentOpen() result is used to generate the frame.
// HC1 and HC2 are fetched with the FHT8VGetHC1() and FHT8VGetHC2() calls, and address is always 0.
// The generated command frame can be resent indefinitely.
// If no valve is set up then this may simply terminate an empty buffer with 0xff.
void FHT8VCreateValveSetCmdFrame(const OTRadValve::AbstractRadValve &valve);

// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) in the shared TX buffer.
//   * valvePC  the percentage open to set the valve [0,100]
// HC1 and HC2 are fetched with the FHT8VGetHC1() and FHT8VGetHC2() calls, and address is always 0.
// The generated command frame can be resent indefinitely.
// If no valve is set up then this may simply terminate an empty buffer with 0xff.
void FHT8VCreateValveSetCmdFrame(const uint8_t valvePC);

// Decode raw bitstream into non-null command structure passed in; returns true if successful.
// Will return non-null if OK, else NULL if anything obviously invalid is detected such as failing parity or checksum.
// Finds and discards leading encoded 1 and trailing 0.
// Returns NULL on failure, else pointer to next full byte after last decoded.
uint8_t const *FHT8VDecodeBitStream(uint8_t const *bitStream, uint8_t const *lastByte, FHT8VRadValve::fht8v_msg_t *command);


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


// This unit may control a local TRV.
#if defined(LOCAL_TRV) || defined(SLAVE_TRV)
// Returns TRV if valve/radiator is to be controlled by this unit.
// Usually the case, but may not be for (a) a hub or (b) a not-yet-configured unit.
// Returns false if house code parts are set to invalid or uninitialised values (>99).
bool localFHT8VTRVEnabled();
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


// True iff the FHT8V valve(s) (if any) controlled by this unit are really open.
// This waits until, for example, an ACK where appropriate, or at least the command has been sent.
// This also implies open to OTRadValve::DEFAULT_VALVE_PC_MIN_REALLY_OPEN or equivalent.
// If more than one valve is being controlled by this unit,
// then this should return true if all of the valves are (significantly) open.
bool FHT8VisControlledValveOpen();


//#ifdef ENABLE_BOILER_HUB
//// Maximum number of housecodes that can be remembered and filtered for in hub selective-response mode.
//// Strictly positive if compiled in.
//// Limited in size partly by memory and partly to limit filtering time at RX.
////#define FHT8V_MAX_HUB_REMEMBERED_HOUSECODES 0
//#define FHT8V_MAX_HUB_REMEMBERED_HOUSECODES EE_HUB_HC_FILTER_COUNT
//
//// Count of house codes selectively listened for at hub.
//// If zero then calls for heat are not filtered by house code.
//uint8_t FHT8VHubListenCount();
//
//// Get remembered house code N where N < FHT8V_MAX_HUB_REMEMBERED_HOUSECODES.
//// Returns hc1:hc2 packed into a 16-bit value, with hc1 in most-significant byte.
//// Returns 0xffff if requested house code index not in use.
//uint16_t FHT8CHubListenHouseCodeAtIndex(uint8_t index);
//
//// Remember and respond to calls for heat from hc1:hc2 when a hub.
//// Returns true if successfully remembered (or already present), else false if cannot be remembered.
//bool FHT8VHubListenForHouseCode(uint8_t hc1, uint8_t hc2);
//
//// Forget and no longer respond to calls for heat from hc1:hc2 when a hub.
//void FHT8VHubUnlistenForHouseCode(uint8_t hc1, uint8_t hc2);
//
//// Returns true if given house code is a remembered one to accept calls for heat from, or if no filtering is being done.
//// Fast, and safe to call from an interrupt routine.
//bool FHT8VHubAcceptedHouseCode(uint8_t hc1, uint8_t hc2);
//
//#else
//#define FHT8V_MAX_HUB_REMEMBERED_HOUSECODES 0
//#define FHT8VHubListenCount() (0)
//#define FHT8CHubListenHouseCodeAtIndex(index) ((uint16_t)~0)
//#define FHT8VHubListenForHouseCode(hc1, hc2) (false)
//#define FHT8VHubUnlistenForHouseCode(hc1, hc2) {}
//#define FHT8VHubAcceptedHouseCode(hc1, hc2) (true)
//#endif


#endif

#endif

