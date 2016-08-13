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

#include <util/parity.h>

#include "FHT8V_Wireless_Rad_Valve.h"

#include "Control.h"
#include "EEPROM_Utils.h"
#include "RFM22_Radio.h"
#include "Serial_Debug.h"
#include "Power_Management.h"

// Minimum valve percentage open to be considered actually open; [1,100].
// Setting this above 0 delays calling for heat from a central boiler until water is likely able to flow.
// (It may however be possible to scavenge some heat if a particular valve opens below this and the circulation pump is already running, for example.)
// DHD20130522: FHT8V + valve heads I have been using are not typically upen until around 6%.
#define FHT8V_MIN_VALVE_PC_REALLY_OPEN 10


#if defined(USE_MODULE_RFM22RADIOSIMPLE)
// Provide RFM22/RFM23 register settings for use with FHT8V in Flash memory.
// Consists of a sequence of (reg#,value) pairs terminated with a 0xff register number.  The reg#s are <128, ie top bit clear.
// Magic numbers c/o Mike Stirling!
const uint8_t FHT8V_RFM22_Reg_Values[][2] PROGMEM =
  {
    {6,0}, // Disable default chiprdy and por interrupts.
    {8,0}, // RFM22REG_OP_CTRL2: ANTDIVxxx, RXMPK, AUTOTX, ENLDM

#ifndef RFM22_IS_ACTUALLY_RFM23
// For RFM22 with RXANT tied to GPIO0, and TXANT tied to GPIO1...
    {0xb,0x15}, {0xc,0x12}, // Can be omitted FOR RFM23.
#endif

// 0x30 = 0x00 - turn off packet handling
// 0x33 = 0x06 - set 4 byte sync
// 0x34 = 0x08 - set 4 byte preamble
// 0x35 = 0x10 - set preamble threshold (RX) 2 nybbles / 1 bytes of preamble.
// 0x36-0x39 = 0xaacccccc - set sync word, using end of RFM22-pre-preamble and start of FHT8V preamble.
    {0x30,0}, {0x33,6}, {0x34,8}, {0x35,0x10}, {0x36,0xaa}, {0x37,0xcc}, {0x38,0xcc}, {0x39,0xcc},

// From AN440: The output power is configurable from +13 dBm to –8 dBm (Si4430/31), and from +20 dBM to –1 dBM (Si4432) in ~3 dB steps. txpow[2:0]=000 corresponds to min output power, while txpow[2:0]=111 corresponds to max output power.
// The maximum legal ERP (not TX output power) on 868.35 MHz is 25 mW with a 1% duty cycle (see IR2030/1/16).
//EEPROM ($6d,%00001111) ; RFM22REG_TX_POWER: Maximum TX power: 100mW for RFM22; not legal in UK/EU on RFM22 for this band.
//EEPROM ($6d,%00001000) ; RFM22REG_TX_POWER: Minimum TX power (-1dBm).
#ifndef RFM22_IS_ACTUALLY_RFM23
    #ifndef RFM22_GOOD_RF_ENV
    {0x6d,0xd}, // RFM22REG_TX_POWER: RFM22 +14dBm ~25mW ERP with 1/4-wave antenna.
    #else // Tone down for good RF backplane, etc.
    {0x6d,0x9},
    #endif
#else
    #ifndef RFM22_GOOD_RF_ENV
    {0x6d,0xf}, // RFM22REG_TX_POWER: RFM23 max power (+13dBm) for ERP ~25mW with 1/4-wave antenna.
    #else // Tone down for good RF backplane, etc.
    {0x6d,0xb},
    #endif
#endif

    {0x6e,40}, {0x6f,245}, // 5000bps, ie 200us/bit for FHT (6 for 1, 4 for 0).  10485 split across the registers, MSB first.
    {0x70,0x20}, // MOD CTRL 1: low bit rate (<30kbps), no Manchester encoding, no whitening.
    {0x71,0x21}, // MOD CTRL 2: OOK modulation.
    {0x72,0x20}, // Deviation GFSK. ; WAS EEPROM ($72,8) ; Deviation 5 kHz GFSK.
    {0x73,0}, {0x74,0}, // Frequency offset
// Channel 0 frequency = 868 MHz, 10 kHz channel steps, high band.
    {0x75,0x73}, {0x76,100}, {0x77,0}, // BAND_SELECT,FB(hz), CARRIER_FREQ0&CARRIER_FREQ1,FC(hz) where hz=868MHz
    {0x79,35}, // 868.35 MHz - FHT
    {0x7a,1}, // One 10kHz channel step.

// RX-only
#ifdef USE_MODULE_FHT8VSIMPLE_RX // RX-specific settings, again c/o Mike S.
    {0x1c,0xc1}, {0x1d,0x40}, {0x1e,0xa}, {0x1f,3}, {0x20,0x96}, {0x21,0}, {0x22,0xda}, {0x23,0x74}, {0x24,0}, {0x25,0xdc},
    {0x2a,0x24},
    {0x2c,0x28}, {0x2d,0xfa}, {0x2e,0x29},
    {0x69,0x60}, // AGC enable: SGIN | AGCEN
#endif

    { 0xff, 0xff } // End of settings.
  };
#endif // defined(USE_MODULE_RFM22RADIOSIMPLE)

#if 0 // DHD20130226 dump
     00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F
00 : 08 06 20 20 00 00 00 00 00 7F 06 15 12 00 00 00
01 : 00 00 20 00 03 00 01 00 00 01 14 00 C1 40 0A 03
02 : 96 00 DA 74 00 DC 00 1E 00 00 24 00 28 FA 29 08
03 : 00 00 0C 06 08 10 AA CC CC CC 00 00 00 00 00 00
04 : 00 00 00 FF FF FF FF 00 00 00 00 FF 08 08 08 10
05 : 00 00 DF 52 20 64 00 01 87 00 01 00 0E 00 00 00
06 : A0 00 24 00 00 81 02 1F 03 60 9D 00 01 0B 28 F5
07 : 20 21 20 00 00 73 64 00 19 23 01 03 37 04 37
#endif


// Appends encoded 200us-bit representation of logical bit (true for 1, false for 0).
// If the most significant bit is 0 this appends 1100 else this appends 111000
// msb-first to the byte stream being created by FHT8VCreate200usBitStreamBptr.
// bptr must be pointing at the current byte to update on entry which must start off as 0xff;
// this will write the byte and increment bptr (and write 0xff to the new location) if one is filled up.
// Partial byte can only have even number of bits present, ie be in one of 4 states.
// Two least significant bits used to indicate how many bit pairs are still to be filled,
// so initial 0xff value (which is never a valid complete filled byte) indicates 'empty'.
static uint8_t *_FHT8VCreate200usAppendEncBit(uint8_t *bptr, bool is1)
  {
  const uint8_t bitPairsLeft = (*bptr) & 3; // Find out how many bit pairs are left to fill in the current byte.
  if(!is1) // Appending 1100.
    {
    switch(bitPairsLeft)
      {
      case 3: // Empty target byte (should be 0xff currently).
        *bptr = 0xcd; // %11001101 Write back partial byte (msbits now 1100 and two bit pairs remain free).
        break;
      case 2: // Top bit pair already filled.
        *bptr = (*bptr & 0xc0) | 0x30; // Preserve existing ms bit-pair, set middle four bits 1100, one bit pair remains free.
        break;
      case 1: // Top two bit pairs already filled.
        *bptr = (*bptr & 0xf0) | 0xc; // Preserve existing ms (2) bit-pairs, set bottom four bits 1100, write back full byte.
        *++bptr = (uint8_t) ~0U; // Initialise next byte for next incremental update.
        break;
      default: // Top three bit pairs already filled.
        *bptr |= 3; // Preserve existing ms (3) bit-pairs, OR in leading 11 bits, write back full byte.
        *++bptr = 0x3e; // Write trailing 00 bits to next byte and indicate 3 bit-pairs free for next incremental update.
        break;
      }
    }
  else // Appending 111000.
    {
    switch(bitPairsLeft)
      {
      case 3: // Empty target byte (should be 0xff currently).
        *bptr = 0xe0; // %11100000 Write back partial byte (msbits now 111000 and one bit pair remains free).
        break;
      case 2: // Top bit pair already filled.
        *bptr = (*bptr & 0xc0) | 0x38; // Preserve existing ms bit-pair, set lsbits to 111000, write back full byte.
        *++bptr = (uint8_t) ~0U; // Initialise next byte for next incremental update.
        break;
      case 1: // Top two bit pairs already filled.
        *bptr = (*bptr & 0xf0) | 0xe; // Preserve existing (2) ms bit-pairs, set bottom four bits to 1110, write back full byte.
        *++bptr = 0x3e; // %00111110 Write trailing 00 bits to next byte and indicate 3 bit-pairs free for next incremental update.
        break;
      default: // Top three bit pairs already filled.
        *bptr |= 3; // Preserve existing ms (3) bit-pairs, OR in leading 11 bits, write back full byte.
        *++bptr = 0x8d; // Write trailing 1000 bits to next byte and indicate 2 bit-pairs free for next incremental update.
        break;
      }
    }
  return(bptr);
  }

// Appends encoded byte in b msbit first plus trailing even parity bit (9 bits total)
// to the byte stream being created by FHT8VCreate200usBitStreamBptr.
static uint8_t *_FHT8VCreate200usAppendByteEP(uint8_t *bptr, const uint8_t b)
  {
  for(uint8_t mask = 0x80; mask != 0; mask >>= 1)
    { bptr = _FHT8VCreate200usAppendEncBit(bptr, 0 != (b & mask)); }
  return(_FHT8VCreate200usAppendEncBit(bptr, (bool) parity_even_bit(b))); // Append even parity bit.
  }

// Create stream of bytes to be transmitted to FHT80V at 200us per bit, msbit of each byte first.
// Byte stream is terminated by 0xff byte which is not a possible valid encoded byte.
// On entry the populated FHT8V command struct is passed by pointer.
// On exit, the memory block starting at buffer contains the low-byte, msbit-first bit, 0xff-terminated TX sequence.
// The maximum and minimum possible encoded message sizes are 35 (all zero bytes) and 45 (all 0xff bytes) bytes long.
// Note that a buffer space of at least 46 bytes is needed to accommodate the longest-possible encoded message and terminator.
// Returns pointer to the terminating 0xff on exit.
uint8_t *FHT8VCreate200usBitStreamBptr(uint8_t *bptr, const fht8v_msg_t *command)
  {
  // Generate FHT8V preamble.
  // First 12 x 0 bits of preamble, pre-encoded as 6 x 0xcc bytes.
  *bptr++ = 0xcc;
  *bptr++ = 0xcc;
  *bptr++ = 0xcc;
  *bptr++ = 0xcc;
  *bptr++ = 0xcc;
  *bptr++ = 0xcc;
  *bptr = (uint8_t) ~0U; // Initialise for _FHT8VCreate200usAppendEncBit routine.
  // Push remaining 1 of preamble.
  bptr = _FHT8VCreate200usAppendEncBit(bptr, true); // Encode 1.

  // Generate body.
  bptr = _FHT8VCreate200usAppendByteEP(bptr, command->hc1);
  bptr = _FHT8VCreate200usAppendByteEP(bptr, command->hc2);
#ifdef FHT8V_ADR_USED
  bptr = _FHT8VCreate200usAppendByteEP(bptr, command->address);
#else
  bptr = _FHT8VCreate200usAppendByteEP(bptr, 0); // Default/broadcast.  TODO: could possibly be further optimised to send 0 value more efficiently.
#endif
  bptr = _FHT8VCreate200usAppendByteEP(bptr, command->command);
  bptr = _FHT8VCreate200usAppendByteEP(bptr, command->extension);
  // Generate checksum.
#ifdef FHT8V_ADR_USED
  const uint8_t checksum = 0xc + command->hc1 + command->hc2 + command->address + command->command + command->extension;
#else
  const uint8_t checksum = 0xc + command->hc1 + command->hc2 + command->command + command->extension;
#endif
  bptr = _FHT8VCreate200usAppendByteEP(bptr, checksum);

  // Generate trailer.
  // Append 0 bit for trailer.
  bptr = _FHT8VCreate200usAppendEncBit(bptr, false);
  // Append extra 0 bit to ensure that final required bits are flushed out.
  bptr = _FHT8VCreate200usAppendEncBit(bptr, false);
  *bptr = (uint8_t)0xff; // Terminate TX bytes.
  return(bptr);
  }

// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) at bptr.
// The TRVPercentOpen value is used to generate the frame.
// On entry hc1, hc2 (and addresss if used) must be set correctly; this sets command and extension.
// The generated command frame can be resent indefinitely.
// The command buffer used must be (at least) FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE bytes.
// Returns pointer to the terminating 0xff on exit.
uint8_t *FHT8VCreateValveSetCmdFrame_r(uint8_t *bptr, fht8v_msg_t *command, const uint8_t TRVPercentOpen)
  {
  command->command = 0x26;
  command->extension = (TRVPercentOpen * 255) / 100;
#ifdef RFM22_SYNC_ONLY_BCFH
    // Huge cheat: only add RFM22-friendly pre-preamble if calling for heat from the boiler (TRV not closed).
    // NOTE: this requires more buffer space.
    // NOTE: the percentage-open threshold to call for heat from the boiler could be raised above 0 to allow the valve to open significantly, etc.
    if(TRVPercentOpen != 0)
      {
      *bptr++ = 0xaa;
      *bptr++ = 0xaa;
      *bptr++ = 0xaa;
      *bptr++ = 0xaa;
      }
#endif
  return(FHT8VCreate200usBitStreamBptr(bptr, command));
  }

// Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
void FHT8VSetHC1(uint8_t hc) { eeprom_smart_update_byte((uint8_t*)EE_START_FHT8V_HC1, hc); }
void FHT8VSetHC2(uint8_t hc) { eeprom_smart_update_byte((uint8_t*)EE_START_FHT8V_HC2, hc); }

// Get (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control (will be 0xff until set).
uint8_t FHT8VGetHC1() { return(eeprom_read_byte((uint8_t*)EE_START_FHT8V_HC1)); }
uint8_t FHT8VGetHC2() { return(eeprom_read_byte((uint8_t*)EE_START_FHT8V_HC2)); }

// Shared command buffer for TX to FHT8V.
static uint8_t FHT8VTXCommandArea[FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE];

// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) in the shared TX buffer.
// The getTRVPercentOpen() result is used to generate the frame.
// HC1 and HC2 are fetched with the FHT8VGetHC1() and FHT8VGetHC2() calls, and address is always 0.
// The generated command frame can be resent indefinitely.
void FHT8VCreateValveSetCmdFrame()
  {
  fht8v_msg_t command;
  command.hc1 = FHT8VGetHC1();
  command.hc2 = FHT8VGetHC2();
#ifdef FHT8V_ADR_USED
  command.address = 0;
#endif
  FHT8VCreateValveSetCmdFrame_r(FHT8VTXCommandArea, &command, getTRVPercentOpen());
  }

// True once/while this node is synced with and controlling the target FHT8V valve; initially false.
static bool syncedWithFHT8V;
#ifndef IGNORE_FHT_SYNC
bool isSyncedWithFHT8V() { return(syncedWithFHT8V); }
#else
bool isSyncedWithFHT8V() { return(true); } // Lie and claim always synced.
#endif

// True if FHT8V valve is believed to be open under instruction from this system; undefined if not in sync.
static bool FHT8V_isValveOpen;
bool getFHT8V_isValveOpen() { return(FHT8V_isValveOpen); }

// Sync status and down counter for FHT8V, initially zero; value not important once in sync.
// If syncedWithFHT8V = 0 then resyncing, AND
//     if syncStateFHT8V is zero then cycle is starting
//     if syncStateFHT8V in range [241,3] (inclusive) then sending sync command 12 messages.
static uint8_t syncStateFHT8V;

// Count-down in half-second units until next transmission to FHT8V valve.
static uint8_t halfSecondsToNextFHT8VTX;



// Call to reset comms with FHT8V valve and force resync.
// Resets values to power-on state so need not be called in program preamble if variables not tinkered with.
// Requires globals defined that this maintains:
//   syncedWithFHT8V (bit, true once synced)
//   FHT8V_isValveOpen (bit, true if this node has last sent command to open valve)
//   syncStateFHT8V (byte, internal)
//   halfSecondsToNextFHT8VTX (byte).
void FHT8VSyncAndTXReset()
  {
  syncedWithFHT8V = false;
  syncStateFHT8V = 0;
  halfSecondsToNextFHT8VTX = 0;
  FHT8V_isValveOpen = false;
  }

#if 0
; Call once per second to manage initial sync and subsequent comms with FHT8V valve.
; Requires globals defined that this maintains:
;     syncedWithFHT8V (bit, true once synced)
;     syncStateFHT8V (byte, internal)
;     halfSecondsToNextFHT8VTX (byte)
; Use globals maintained/set elsewhere / shared:
;     TRVPercentOpen (byte, in range 0 to 100) valve open percentage to convey to FHT8V
;     slowOpDone (bit) set to 1 if this routine takes significant time
;     FHT8VTXCommandArea (byte block) for FHT8V outgoing commands
; Can be VERY CPU AND I/O INTENSIVE so running at high clock speed may be necessary.
; If on exit the first byte of the command buffer has been set to $ff
; then the command buffer should immediately (or within a few seconds)
; have a valid outgoing command put in it for the next scheduled transmission to the TRV.
; The command buffer can then be updated whenever required for subsequent async transmissions.
FHT8VPollSyncAndTX:
    if syncedWithFHT8V = 0 then
        ; Give priority to getting in sync over all other tasks, though pass control to them afterwards...
        ; NOTE: startup state, or state to force resync is: syncedWithFHT8V = 0 AND syncStateFHT8V = 0
        if syncStateFHT8V = 0 then
            ; Starting sync process.
            syncStateFHT8V = 241
        endif

        if syncStateFHT8V >= 3 then
            ; Generate and send sync (command 12) message immediately.
            FHT8V_CMD = $2c ; Command 12, extension byte present.
            FHT8V_EXT = syncStateFHT8V
            syncStateFHT8V = syncStateFHT8V - 2
            bptr = FHT8VTXCommandArea
            gosub FHT8VCreate200usBitStreamBptr
            bptr = FHT8VTXCommandArea
            gosub FHT8VTXFHTQueueAndTwiceSendCmd ; SEND SYNC
            ; On final tick set up time to sending of final sync command.
            if syncStateFHT8V = 1 then
                ; Set up timer to sent sync final (0) command
                ; with formula: t = 0.5 * (HC2 & 7) + 4 seconds.
                halfSecondsToNextFHT8VTX = FHT8V_HC2 & 7 + 8 ; Note units of half-seconds for this counter.
            endif

            slowOpDone = 1 ; Will have eaten up lots of time...
        else ; < 3 so waiting to send sync final (0) command...

            if halfSecondsToNextFHT8VTX >= 2 then
                halfSecondsToNextFHT8VTX = halfSecondsToNextFHT8VTX - 2
            endif

            if halfSecondsToNextFHT8VTX < 2 then

                ; Set up correct delay to this TX and next TX dealing with half seconds if need be.
                gosub FHT8VTXGapHalfSeconds
                if halfSecondsToNextFHT8VTX = 1 then ; Need to pause an extra half-second.
                    pause 500
                endif
                halfSecondsToNextFHT8VTX = halfSecondsToNextFHT8VTX + tempB0

                ; Send sync final command.
                FHT8V_CMD = $20 ; Command 0, extension byte present.
                FHT8V_EXT = 0 ; DHD20130324: could set to TRVPercentOpen, but anything other than zero seems to lock up FHT8V-3 units.
                bptr = FHT8VTXCommandArea
                gosub FHT8VCreate200usBitStreamBptr
                bptr = FHT8VTXCommandArea
                gosub FHT8VTXFHTQueueAndTwiceSendCmd ; SEND SYNC FINAL

                ; Assume now in sync...
                syncedWithFHT8V = 1

                ; Mark buffer as empty to get it filled with the real TRV valve-setting command immediately.
                poke FHT8VTXCommandArea, $ff

                slowOpDone = 1 ; Will have eaten up lots of time already...
            endif
        endif

    else ; In sync: count down and send command as required.
            if halfSecondsToNextFHT8VTX >= 2 then
                halfSecondsToNextFHT8VTX = halfSecondsToNextFHT8VTX - 2
            endif

            if halfSecondsToNextFHT8VTX < 2 then

                ; Set up correct delay to this TX and next TX dealing with half seconds if need be.
                gosub FHT8VTXGapHalfSeconds
                if halfSecondsToNextFHT8VTX = 1 then ; Need to pause an extra half-second.
                    pause 500
                endif
                halfSecondsToNextFHT8VTX = halfSecondsToNextFHT8VTX + tempB0

                ; Send already-computed command to TRV.
                ; Queue and send the command.
                bptr = FHT8VTXCommandArea
                gosub FHT8VTXFHTQueueAndTwiceSendCmd

                ; Assume that command just sent reflects the current TRV inetrnal model state.
                ; If TRVPercentOpen is not zero assume that remote valve is now open(ing).
                if TRVPercentOpen = 0 then : FHT8V_isValveOpen = 0 : else : FHT8V_isValveOpen = 1 : endif

                slowOpDone = 1 ; Will have eaten up lots of time...
            endif
    endif
    return
#endif


// Sends to FHT8V in FIFO mode command bitstream from buffer starting at bptr up until terminating 0xff, then reverts to low-power standby mode.
// The trailing 0xff is not sent.
// Returns immediately without transmitting if the command buffer starts with 0xff (ie is empty).
// (If doubleTX is true, sends the bitstream twice, with a short (~8ms) pause between transmissions, to help ensure reliable delivery.)
static void FHT8VTXFHTQueueAndSendCmd(const uint8_t *bptr, bool doubleTX)
  {
  if(((uint8_t)0xff) == *bptr) { return; }
#ifdef DEBUG
  if(0 == *bptr) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("FHT8V frame not initialised"); panic(); }
#endif

  RFM22QueueCmdToFF(bptr);
  RFM22TXFIFO(); // Send it!

  if(doubleTX)
    {
    // Should nominally pause about 8--9ms or similar before retransmission...
    sleepLowPowerMs(8);
    RFM22TXFIFO(); // Re-send it!
    }

  RFM22ModeStandbyAndClearState();
  }

// Call just after TX of valve-setting command which is assumed to reflect current TRVPercentOpen state.
// This helps avoiding calling for heat from a central boiler until the valve is really open,
// eg to avoid excess load on (or power wasting in) the circulation pump.
static void setFHT8V_isValveOpen()
  { FHT8V_isValveOpen = (getTRVPercentOpen() >= FHT8V_MIN_VALVE_PC_REALLY_OPEN); }

// Send current (assumed valve-setting) command and adjust FHT8V_isValveOpen as appropriate.
static void valveSettingTX(bool allowDoubleTX)
  {
  // Transmit correct valve-setting command that should already be in the buffer...
  FHT8VTXFHTQueueAndSendCmd(FHT8VTXCommandArea, allowDoubleTX);
  // Indicate state valve should be in...
  setFHT8V_isValveOpen();
  }

// Half second count within current minor cycle for FHT8VPollSyncAndTX_XXX().
static uint8_t halfSecondCount;
#if defined(TWO_S_TICK_RTC_SUPPORT)
#define MAX_HSC 3 // Max allowed value of halfSecondCount.
#else
#define MAX_HSC 1 // Max allowed value of halfSecondCount.
#endif


// Compute interval (in half seconds) between TXes for FHT8V given house code 2.
// (In seconds, the formula is t = 115 + 0.5 * (HC2 & 7) seconds.)
static uint8_t FHT8VTXGapHalfSeconds(uint8_t hc2) { return((hc2 & 7) + 230); }

// Compute interval (in half seconds) between TXes for FHT8V given house code 2
// given current halfSecondCountInMinorCycle assuming all remaining tick calls to _Next
// will be foregone in this minor cycle,
static uint8_t FHT8VTXGapHalfSeconds(uint8_t hc2, uint8_t halfSecondCountInMinorCycle)
  { return(FHT8VTXGapHalfSeconds(hc2) - (MAX_HSC - halfSecondCountInMinorCycle)); }

// Run the algorithm to get in sync with the receiver.
// Uses halfSecondCount.
// Iff this returns true then a(nother) call FHT8VPollSyncAndTX_Next() at or before each 0.5s from the cycle start should be made.
static bool doSync(bool allowDoubleTX)
  {
  if(0 == syncStateFHT8V)
    {
    // Starting sync process.
    syncStateFHT8V = 241;
#if 1 && defined(DEBUG)
    DEBUG_SERIAL_TIMESTAMP();
    DEBUG_SERIAL_PRINT_FLASHSTRING(" FHT8V syncing...");
    DEBUG_SERIAL_PRINTLN();
#endif
    }

  if(syncStateFHT8V >= 2)
    {
    // Generate and send sync (command 12) message immediately for odd-numbered ticks, ie once per second.
    if(syncStateFHT8V & 1)
      {
      fht8v_msg_t command;
      command.hc1 = FHT8VGetHC1();
      command.hc2 = FHT8VGetHC2();
      command.command = 0x2c; // Command 12, extension byte present.
      command.extension = syncStateFHT8V;
      FHT8VCreate200usBitStreamBptr(FHT8VTXCommandArea, &command);
      if(halfSecondCount > 0) { sleepUntilSubCycleTime((SUB_CYCLE_TICKS_PER_S/2) * halfSecondCount); }
      FHT8VTXFHTQueueAndSendCmd(FHT8VTXCommandArea, allowDoubleTX); // SEND SYNC
      // Note that FHT8VTXCommandArea now does not contain a valid valve-setting command...
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_TIMESTAMP();
    DEBUG_SERIAL_PRINT_FLASHSTRING(" FHT8V SYNC ");
    DEBUG_SERIAL_PRINT(syncStateFHT8V);
    DEBUG_SERIAL_PRINTLN();
#endif
      }

    // After penultimate sync TX set up time to sending of final sync command.
    if(1 == --syncStateFHT8V)
      {
      // Set up timer to sent sync final (0) command
      // with formula: t = 0.5 * (HC2 & 7) + 4 seconds.
      halfSecondsToNextFHT8VTX = (FHT8VGetHC2() & 7) + 8; // Note units of half-seconds for this counter.
      halfSecondsToNextFHT8VTX -= (MAX_HSC - halfSecondCount);
      return(false); // No more TX this minor cycle.
      }
    }

  else // syncStateFHT8V == 1 so waiting to send sync final (0) command...
    {
    if(--halfSecondsToNextFHT8VTX == 0)
      {
      // Send sync final command.
      fht8v_msg_t command;
      command.hc1 = FHT8VGetHC1();
      command.hc2 = FHT8VGetHC2();
      command.command = 0x20; // Command 0, extension byte present.
      command.extension = 0; // DHD20130324: could set to TRVPercentOpen, but anything other than zero seems to lock up FHT8V-3 units.
      FHT8VCreate200usBitStreamBptr(FHT8VTXCommandArea, &command);      
      if(halfSecondCount > 0) { sleepUntilSubCycleTime((SUB_CYCLE_TICKS_PER_S/2) * halfSecondCount); }
      FHT8VTXFHTQueueAndSendCmd(FHT8VTXCommandArea, allowDoubleTX); // SEND SYNC FINAL
    // Note that FHT8VTXCommandArea now does not contain a valid valve-setting command...
#if 1 && defined(DEBUG)
     DEBUG_SERIAL_TIMESTAMP();
     DEBUG_SERIAL_PRINTLN_FLASHSTRING(" FHT8V SYNC FINAL");
#endif

      // Assume now in sync...
      syncedWithFHT8V = true;

      // On PICAXE there was no time to recompute valve-setting command immediately after SYNC FINAL SEND...
      // Mark buffer as empty to get it filled with the real TRV valve-setting command ASAP.
      //*FHT8VTXCommandArea = 0xff;

      // On ATmega there is plenty of CPU heft to fill command buffer immediately with valve-setting command.
      FHT8VCreateValveSetCmdFrame();

      // Set up correct delay to next TX; no more this minor cycle...
      halfSecondsToNextFHT8VTX = FHT8VTXGapHalfSeconds(command.hc2, halfSecondCount);
      return(false);
      }
    }

  // For simplicity, insist on being called every half-second during sync.
  // TODO: avoid forcing most of these calls to save some CPU/energy and improve responsiveness.
  return(true);
  }

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
bool FHT8VPollSyncAndTX_First(bool allowDoubleTX)
  {
  halfSecondCount = 0;

#ifdef IGNORE_FHT_SYNC // Will TX on 0 and 2 half second offsets.
  // Transmit correct valve-setting command that should already be in the buffer...
  valveSettingTX(allowDoubleTX);
  return(true); // Will need anther TX in slot 2.
#else

  // Give priority to getting in sync over all other tasks, though pass control to them afterwards...
  // NOTE: startup state, or state to force resync is: syncedWithFHT8V = 0 AND syncStateFHT8V = 0
  if(!syncedWithFHT8V) { return(doSync(allowDoubleTX)); }

#ifdef DEBUG
   if(0 == halfSecondsToNextFHT8VTX) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("FHT8V hs count 0 too soon"); panic(); }
#endif

  // If no TX required in this minor cycle then can return false quickly (having decremented ticks-to-next-TX value suitably).
  if(halfSecondsToNextFHT8VTX > MAX_HSC+1)
    {
    halfSecondsToNextFHT8VTX -= (MAX_HSC+1);
    return(false); // No TX this minor cycle.
    }

  // TX is due this slot so do it (and no more will be needed this minor cycle).
  if(0 == --halfSecondsToNextFHT8VTX)
    {
    FHT8VTXFHTQueueAndSendCmd(FHT8VTXCommandArea, allowDoubleTX);
#if 1 && defined(DEBUG)
     DEBUG_SERIAL_TIMESTAMP();
     DEBUG_SERIAL_PRINTLN_FLASHSTRING(" FHT8V TX");
#endif
    // Set up correct delay to next TX.
    halfSecondsToNextFHT8VTX = FHT8VTXGapHalfSeconds(FHT8VGetHC2(), 0);
    return(false);
    }

  // Will need to TX in a following slot in this minor cycle...
  return(true);
#endif
  }

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
bool FHT8VPollSyncAndTX_Next(bool allowDoubleTX)
  {
  ++halfSecondCount; // Reflects count of calls since _First(), ie how many 
#ifdef DEBUG
    if(halfSecondCount > MAX_HSC) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("FHT8VPollSyncAndTX_Next() called too often"); panic(); }
#endif

#ifdef IGNORE_FHT_SYNC // Will TX on 0 and 2 half second offsets.
  if(2 == halfSecondCount)
      {
      // Sleep until 1s from start of cycle.
      sleepUntilSubCycleTime(SUB_CYCLE_TICKS_PER_S);
      // Transmit correct valve-setting command that should already be in the buffer...
      valveSettingTX(allowDoubleTX);
      return(false); // Don't need any slots after this.
      }
  return(true); // Need to do further TXes this minor cycle.
#else

  // Give priority to getting in sync over all other tasks, though pass control to them afterwards...
  // NOTE: startup state, or state to force resync is: syncedWithFHT8V = 0 AND syncStateFHT8V = 0
  if(!syncedWithFHT8V) { return(doSync(allowDoubleTX)); }

  // TX is due this slot so do it (and no more will be needed this minor cycle).
  if(0 == --halfSecondsToNextFHT8VTX)
    {
    sleepUntilSubCycleTime((SUB_CYCLE_TICKS_PER_S/2) * halfSecondCount);
    FHT8VTXFHTQueueAndSendCmd(FHT8VTXCommandArea, allowDoubleTX);
#if 1 && defined(DEBUG)
     DEBUG_SERIAL_TIMESTAMP();
     DEBUG_SERIAL_PRINTLN_FLASHSTRING(" FHT8V TX");
#endif
    // Set up correct delay to next TX.
    halfSecondsToNextFHT8VTX = FHT8VTXGapHalfSeconds(FHT8VGetHC2(), halfSecondCount);
    return(false);
    }

  // Will need to TX in a following slot in this minor cycle...
  return(true);
#endif
  }
