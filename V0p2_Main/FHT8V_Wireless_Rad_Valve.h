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

 This implementation expects to work with an RFM23B (or RFM22B) radio as of 2015/11/11;
 this may be generalised.

 For details of protocol including sync between this and FHT8V see https://sourceforge.net/p/opentrv/wiki/FHT%20Protocol/
 */

#ifndef FHT8V_WIRELESS_RAD_VALVE_H
#define FHT8V_WIRELESS_RAD_VALVE_H

#include "V0p2_Main.h"

#include <OTRadioLink.h>

#include "RFM22_Radio.h"
#include "Messaging.h"


// FHT8V radio-controlled radiator valve, using FS20 protocol.
class FHT8VRadValveBase : public OTRadValve::AbstractRadValve
  {
  public:
    // Type of function to extend the TX buffer, returns pointer to 0xff just beyond last content byte appended.
    // In case of failure this returns NULL.
    typedef uint8_t *appendToTXBufferFF_t(uint8_t *buf, uint8_t bufSize);

    // Type for information content of FHT8V message.
    // Omits the address field unless it is actually used.
    typedef struct
      {
      uint8_t hc1;
      uint8_t hc2;
#ifdef OTV0P2BASE_FHT8V_ADR_USED
      uint8_t address;
#endif
      uint8_t command;
      uint8_t extension;
      } fht8v_msg_t;

  protected:
    // Radio link usually expected to be RFM23B; non-NULL when available.
    OTRadioLink::OTRadioLink *radio;

    // TX buffer (non-null) and size (non-zero).
    uint8_t * const buf;
    const uint8_t bufSize;

    // Function to append (stats) trailer(s) to TX buffer (and add trailing 0xff if anything added); NULL if not needed.
    // Pointer set at construction.
    appendToTXBufferFF_t const *trailerFn;

    // Construct an instance, providing TX buffer details.
    FHT8VRadValveBase(uint8_t *_buf, uint8_t _bufSize, appendToTXBufferFF_t *trailerFnPtr)
      : radio(NULL),
        buf(_buf), bufSize(_bufSize),
        trailerFn(trailerFnPtr),
        halfSecondCount(0)
      {
      // Cleared housecodes will prevent any immediate attempt to sync with FTH8V.
      // This also sets state to force resync afterwards.
      clearHC(); 
      }

    // Sync status and down counter for FHT8V, initially zero; value not important once in sync.
    // If syncedWithFHT8V = 0 then resyncing, AND
    //     if syncStateFHT8V is zero then cycle is starting
    //     if syncStateFHT8V in range [241,3] (inclusive) then sending sync command 12 messages.
    uint8_t syncStateFHT8V;

    // Count-down in half-second units until next transmission to FHT8V valve.
    uint8_t halfSecondsToNextFHT8VTX;

    // Half second count within current minor cycle for FHT8VPollSyncAndTX_XXX().
    uint8_t halfSecondCount;

    // True once/while this node is synced with and controlling the target FHT8V valve; initially false.
    bool syncedWithFHT8V;

    // True if FHT8V valve is believed to be open under instruction from this system; false if not in sync.
    bool FHT8V_isValveOpen;

    // Send current (assumed valve-setting) command and adjust FHT8V_isValveOpen as appropriate.
    // Only appropriate when the command is going to be heard by the FHT8V valve itself, not just the hub.
    void valveSettingTX(bool allowDoubleTX);

    // Run the algorithm to get in sync with the receiver.
    // Uses halfSecondCount.
    // Iff this returns true then a(nother) call FHT8VPollSyncAndTX_Next() at or before each 0.5s from the cycle start should be made.
    bool doSync(const bool allowDoubleTX);

    #if defined(V0P2BASE_TWO_S_TICK_RTC_SUPPORT)
    static const uint8_t MAX_HSC = 3; // Max allowed value of halfSecondCount.
    #else
    static const uint8_t MAX_HSC = 1; // Max allowed value of halfSecondCount.
    #endif

    // Sleep in reasonably low-power mode until specified target subcycle time, optionally listening (RX) for calls-for-heat.
    // Returns true if OK, false if specified time already passed or significantly missed (eg by more than one tick).
    // May use a combination of techniques to hit the required time.
    // Requesting a sleep until at or near the end of the cycle risks overrun and may be unwise.
    // Using this to sleep less then 2 ticks may prove unreliable as the RTC rolls on underneath...
    // This is NOT intended to be used to sleep over the end of a minor cycle.
    // FIXME: be passed a function (such as pollIIO) to call while waiting.
    void sleepUntilSubCycleTimeOptionalRX(uint8_t sleepUntil);

    // Sends to FHT8V in FIFO mode command bitstream from buffer starting at bptr up until terminating 0xff,
    // then reverts to low-power standby mode if not in hub mode, RX for OpenTRV FHT8V if in hub mode.
    // The trailing 0xff is not sent.
    //
    // Returns immediately without transmitting if the command buffer starts with 0xff (ie is empty).
    // (If doubleTX is true, sends the bitstream twice, with a short (~8ms) pause between transmissions, to help ensure reliable delivery.)
    //
    // Note: single transmission time is up to about 80ms (without extra trailers), double up to about 170ms.
    void FHT8VTXFHTQueueAndSendCmd(uint8_t *bptr, const bool doubleTX);

    // Call just after TX of valve-setting command which is assumed to reflect current TRVPercentOpen state.
    // This helps avoiding calling for heat from a central boiler until the valve is really open,
    // eg to avoid excess load on (or energy wasting by) the circulation pump.
    // FIXME: compare against own threshold and have NominalRadValve look at least open of all vs minPercentOpen.
    void setFHT8V_isValveOpen();

    // House codes part 1 and 2 (must each be <= 99 to be valid).
    // Starts as '0xff' as unset EEPROM values would be to indicate 'unset'.
    uint8_t hc1, hc2;

  public:
    // Returns true if the supplied house code part is valid for an FHT8V valve.
    static inline bool isValidFHTV8HouseCode(const uint8_t hc) { return(hc <= 99); }

    // Clear both housecode parts (and thus disable use of FHT8V valve).
    void clearHC() { hc1 = ~0, hc2 = ~0; resyncWithValve(); }
    // Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
    // Both parts must be <= 99 for the house code to be valid and the valve used.
    // Forces resync with remote valve if house code changed.
    void setHC1(uint8_t hc) { if(hc != hc1) { hc1 = hc; resyncWithValve(); } }
    void setHC2(uint8_t hc) { if(hc != hc2) { hc2 = hc; resyncWithValve(); } }
    // Get (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control (will be 0xff until set).
    // Both parts must be <= 99 for the house code to be valid and the valve used.
    uint8_t getHC1() const { return(hc1); }
    uint8_t getHC2() const { return(hc2); }
    // Check if housecode is valid for controlling an FHT8V.
    bool isValidHC() const { return(isValidFHTV8HouseCode(hc1) && isValidFHTV8HouseCode(hc2)); }

    // Set radio to use (if non-NULL) or clear access to radio (if NULL).
    void setRadio(OTRadioLink::OTRadioLink *r) { radio = r; }

    // Decode raw bitstream into non-null command structure passed in; returns true if successful.
    // Will return non-null if OK, else NULL if anything obviously invalid is detected such as failing parity or checksum.
    // Finds and discards leading encoded 1 and trailing 0.
    // Returns NULL on failure, else pointer to next full byte after last decoded.
    static uint8_t const *FHT8VDecodeBitStream(uint8_t const *bitStream, uint8_t const *lastByte, fht8v_msg_t *command);

    // Minimum and maximum FHT8V TX cycle times in half seconds: [115.0,118.5].
    // Fits in an 8-bit unsigned value.
    static const uint8_t MIN_FHT8V_TX_CYCLE_HS = (115*2);
    static const uint8_t MAX_FHT8V_TX_CYCLE_HS = (118*2+1);

    // Compute interval (in half seconds) between TXes for FHT8V given house code 2 (HC2).
    // (In seconds, the formula is t = 115 + 0.5 * (HC2 & 7) seconds, in range [115.0,118.5].)
    inline uint8_t FHT8VTXGapHalfSeconds(const uint8_t hc2) { return((hc2 & 7) + 230); }

    // Compute interval (in half seconds) between TXes for FHT8V given house code 2
    // given current halfSecondCountInMinorCycle assuming all remaining tick calls to _Next
    // will be foregone in this minor cycle,
    inline uint8_t FHT8VTXGapHalfSeconds(const uint8_t hc2, const uint8_t halfSecondCountInMinorCycle)
      { return(FHT8VTXGapHalfSeconds(hc2) - (MAX_HSC - halfSecondCountInMinorCycle)); }

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
    static uint8_t *FHT8VCreate200usBitStreamBptr(uint8_t *bptr, const fht8v_msg_t *command);

    // Approximate maximum transmission (TX) time for bare FHT8V command frame in ms; strictly positive.
    // This ignores any prefix needed for particular radios such as the RFM23B.
    // ~80ms upwards.
    static const uint8_t FHT8V_APPROX_MAX_RAW_TX_MS = ((((MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE-1)*8) + 4) / 5);

    // Typical FHT8V 'open' percentage, though partly depends on valve tails, etc.
    // This is set to err on the side of slightly open to allow
    // the 'linger' feature to work to help boilers dump heat with pump over-run
    // when the the boiler is turned off.
    // Actual values observed by DHD range from 6% to 25%.
    static const uint8_t TYPICAL_MIN_PERCENT_OPEN = 10;

    // Returns true if radio or house codes not set.
    // Remains false while syncing as that is only temporary unavailability.
    virtual bool isUnavailable() const { return((NULL == radio) || !isValidHC()); }

    // Get estimated minimum percentage open for significant flow for this device; strictly positive in range [1,99].
    // Defaults to typical value from observation.
    virtual uint8_t getMinPercentOpen() const { return(TYPICAL_MIN_PERCENT_OPEN); }

    // Call to reset comms with FHT8V valve and force (re)sync.
    // Resets values to power-on state so need not be called in program preamble if variables not tinkered with.
    // Requires globals defined that this maintains:
    //   syncedWithFHT8V (bit, true once synced)
    //   FHT8V_isValveOpen (bit, true if this node has last sent command to open valve)
    //   syncStateFHT8V (byte, internal)
    //   halfSecondsToNextFHT8VTX (byte).
    void resyncWithValve()
      {
      syncedWithFHT8V = false;
      syncStateFHT8V = 0;
      halfSecondsToNextFHT8VTX = 0;
      FHT8V_isValveOpen = false;
      }

//    //#ifndef IGNORE_FHT_SYNC
//    // True once/while this node is synced with and controlling the target FHT8V valve; initially false.
//    // FIXME: fit into standard RadValve API.
//    bool isSyncedWithFHT8V() { return(syncedWithFHT8V); }
//    //#else
//    //bool isSyncedWithFHT8V() { return(true); } // Lie and claim always synced.
//    //#endif

    // Returns true iff not in error state and not (re)calibrating/(re)initialising/(re)syncing.
    // By default there is no recalibration step.
    virtual bool isInNormalRunState() const { return(syncedWithFHT8V); }

    // True if the controlled physical valve is thought to be at least partially open right now.
    // If multiple valves are controlled then is this true only if all are at least partially open.
    // Used to help avoid running boiler pump against closed valves.
    // Must not be true while (re)calibrating.
    // Returns try if in sync AND current position AND last command sent to valve indicate open.
    virtual bool isControlledValveReallyOpen() const { return(syncedWithFHT8V && FHT8V_isValveOpen && (value >= getMinPercentOpen())); }

    // A set of RFM22/RFM23 register settings for use with FHT8V, stored in (read-only) program/Flash memory.
    // Consists of a sequence of (reg#,value) pairs terminated with a 0xff register.
    // The (valid) reg#s are <128, ie top bit clear.
    // Magic numbers c/o Mike Stirling!
    // Should not be linked into code image unless explicitly referred to.
    static const uint8_t FHT8V_RFM23_Reg_Values[][2] PROGMEM;

    // Values designed to work with FHT8V_RFM23_Reg_Values register settings.
    static const uint8_t RFM23_PREAMBLE_BYTE = 0xaa; // Preamble byte for RFM23 reception.
    static const uint8_t RFM23_PREAMBLE_MIN_BYTES = 4; // Minimum number of preamble bytes for reception.
    static const uint8_t RFM23_PREAMBLE_BYTES = 5; // Recommended number of preamble bytes for reliable reception.
    static const uint8_t RFM23_SYNC_BYTE = 0xcc; // Sync-word trailing byte (with FHT8V primarily).
    static const uint8_t RFM23_SYNC_MIN_BYTES = 3; // Minimum number of sync bytes.

    // Does nothing for now; different timing/driver routines are used.
    virtual uint8_t read() { return(value); }

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
  };


// maxTrailerBytes specifies the maximum number of bytes of trailer that can be added.
// preambleBytes specifies the space to leave for preamble bytes for remote receiver sync (defaults to RFM23-suitable value).
// preambleByte specifies the (default) preamble byte value to use (defaults to RFM23-suitable value).
template <uint8_t maxTrailerBytes, uint8_t preambleBytes = FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, uint8_t preambleByte = FHT8VRadValveBase::RFM23_PREAMBLE_BYTE>
class FHT8VRadValve : public FHT8VRadValveBase
  {
  public:
    static const uint8_t FHT8V_MAX_EXTRA_PREAMBLE_BYTES = preambleBytes; // RFM22_PREAMBLE_BYTES
    static const uint8_t FHT8V_MAX_EXTRA_TRAILER_BYTES = maxTrailerBytes; // (1+max(MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES,FullStatsMessageCore_MAX_BYTES_ON_WIRE));
    static const uint8_t FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE = (FHT8V_MAX_EXTRA_PREAMBLE_BYTES + (FHT8VRadValve::MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE) + FHT8V_MAX_EXTRA_TRAILER_BYTES); // Buffer space needed.

    // Approximate maximum transmission (TX) time for FHT8V command frame in ms; strictly positive.
    // ~80ms upwards.
    static const uint8_t FHT8V_APPROX_MAX_TX_MS = ((((FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE-1)*8) + 4) / 5);

  private:
    // Shared command buffer for TX to FHT8V.
    uint8_t FHT8VTXCommandArea[FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE];

  public:
    // Construct an instance.
    // Optional function to add a trailer, eg a stats trailer, to each TX buffer.
    FHT8VRadValve(appendToTXBufferFF_t *trailerFnPtr)
      : FHT8VRadValveBase(FHT8VTXCommandArea, FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE, trailerFnPtr)
      { }

    // Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) in the shared TX buffer.
    //   * valvePC  the percentage open to set the valve [0,100]
    //   * forceExtraPreamble  if true then force insertion of an extra preamble
    //         to make it possible for an OpenTRV hub to receive the frame,
    //         typically when calling for heat or when there is a stats trailer;
    //         note that a preamble will be forced if a trailer is being added
    //         and FHT8Vs can hear without the preamble.
    // The generated command frame can be resent indefinitely.
    // If no valve is set up then this may simply terminate an empty buffer with 0xff.
    void FHT8VCreateValveSetCmdFrame(const uint8_t valvePC, const bool forceExtraPreamble = false)
      {
      FHT8VRadValveBase::fht8v_msg_t command;
      command.hc1 = getHC1();
      command.hc2 = getHC2();
#ifdef OTV0P2BASE_FHT8V_ADR_USED
      command.address = 0;
#endif
      command.command = 0x26;
      //  command.extension = (valvePC * 255) / 100; // needlessly expensive division.
      // Optimised for speed and to avoid pulling in a division subroutine.
      // Guaranteed to be 255 when valvePC is 100 (max), and 0 when TRVPercentOpen is 0,
      // and a decent approximation of (valvePC * 255) / 100 in between.
      // The approximation is (valvePC * 250) / 100, ie *2.5, as *(2+0.5).
      command.extension = (valvePC >= 100) ? 255 :
        ((valvePC<<1) + ((1+valvePC)>>1));

      // Work out if a trailer is allowed (by security level) and is possible to encode.
      appendToTXBufferFF_t const *tfp = *trailerFn;
      const bool doTrailer = (NULL != tfp) && (OTV0P2BASE::getStatsTXLevel() <= OTV0P2BASE::stTXmostUnsec);

      // Usually add RFM23-friendly preamble (0xaaaaaaaa sync header) only
      // IF calling for heat from the boiler (TRV actually open)
      // OR if adding a (stats) trailer that the hub should see.
      const bool doHeader = forceExtraPreamble || doTrailer;

      uint8_t * const bptrInitial = FHT8VTXCommandArea;
      const uint8_t bufSize = sizeof(FHT8VTXCommandArea);
      uint8_t *bptr = bptrInitial;

      // Start with RFM23-friendly preamble if requested.
      if(doHeader)
        {
        memset(bptr, preambleByte, preambleBytes);
        bptr += preambleBytes;
        }

      // Encode and append FHT8V FS20 command.
      // ASSUMES sufficient buffer space.
      bptr = FHT8VRadValveBase::FHT8VCreate200usBitStreamBptr(bptr, &command);

      // Append trailer if allowed/possible.
      if(doTrailer)
        {
        uint8_t * const tail = tfp(bptr, bufSize - (bptr - bptrInitial));
        // If appending stats failed, write in a terminating 0xff explicitly.
        if(NULL == tail) { *bptr = 0xff; }
        //if(NULL != tail) { bptr = tail; } // Encoding should not actually fail, but this copes gracefully if so!
        }

#if 0 && defined(DEBUG)
// Check that the buffer end was not overrun.
if(bptr - bptrInitial >= bufSize) { panic(F("FHT8V frame too big")); }
#endif
      }
  };



#ifdef USE_MODULE_FHT8VSIMPLE
// Singleton FHT8V valve instance (to control remote FHT8V valve by radio).
static const uint8_t _FHT8V_MAX_EXTRA_TRAILER_BYTES = (1+max(MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES,FullStatsMessageCore_MAX_BYTES_ON_WIRE));
//extern OTRadValve::FHT8VRadValve<RFM22_PREAMBLE_BYTES, _FHT8V_MAX_EXTRA_TRAILER_BYTES> FHT8V;
extern FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V;

// This unit may control a local TRV.
#if defined(LOCAL_TRV) || defined(SLAVE_TRV)
// Returns TRV if valve/radiator is to be controlled by this unit.
// Usually the case, but may not be for (a) a hub or (b) a not-yet-configured unit.
// Returns false if house code parts are set to invalid or uninitialised values (>99).
inline bool localFHT8VTRVEnabled() { return(!FHT8V.isUnavailable()); }
#else
#define localFHT8VTRVEnabled() (false) // Local FHT8V TRV disabled.
#endif

// Clear both housecode parts (and thus disable local valve).
void FHT8VClearHC();
// Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
// Will cache in FHT8V instance for speed.
void FHT8VSetHC1(uint8_t hc);
void FHT8VSetHC2(uint8_t hc);
// Get (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control (will be 0xff until set).
// Used FHT8V instance as a transparent cache of the values for speed.
uint8_t FHT8VGetHC1();
uint8_t FHT8VGetHC2();
// Load EEPROM house codes into primary FHT8V instance at start-up or once cleared in FHT8V instance.
void FHT8VLoadHCFromEEPROM();
#endif // USE_MODULE_FHT8VSIMPLE


#endif

