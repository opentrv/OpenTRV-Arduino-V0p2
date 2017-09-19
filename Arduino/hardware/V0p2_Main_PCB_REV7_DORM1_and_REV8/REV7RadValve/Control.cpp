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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2017
                           Deniz Erbilgin 2015
*/

/*
 Control/model for TRV and boiler.
 */

#include "V0p2_Main.h"
#include <OTAESGCM.h>

static OTV0P2BASE::EEPROMByHourByteStats ebhs;
// Create setback lockout if needed.
  typedef bool(*setbackLockout_t)();
  // If allowing setback lockout, eg for testing, then inject suitable lambda.
  static bool setbackLockout() { return(0 != OTRadValve::getSetbackLockout()); }

// Algorithm for computing target temperature.
constexpr OTRadValve::ModelledRadValveComputeTargetTempBasic<
  PARAMS,
  &valveMode,
  decltype(TemperatureC16),   &TemperatureC16,
  decltype(tempControl),      &tempControl,
  decltype(Occupancy),        &Occupancy,
  decltype(AmbLight),         &AmbLight,
  decltype(valveUI),          &valveUI,
  decltype(Scheduler),        &Scheduler,
  decltype(ebhs),             &ebhs,
  decltype(RelHumidity),      &RelHumidity,
  setbackLockout
  >
  cttBasic;
// Internal model of controlled radiator valve position.
OTRadValve::ModelledRadValve NominalRadValve(
  &cttBasic,
  &valveMode,
  &tempControl,
  &ValveDirect,
  #ifdef TRV_SLEW_GLACIAL
    true,
  #else
    false,
  #endif
  #ifdef TRV_MAX_PC_OPEN
    TRV_MAX_PC_OPEN
  #else
    100
  #endif
  );


// Call this to do an I/O poll if needed; returns true if something useful definitely happened.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// Should also do nothing that interacts with Serial.
// Limits actual poll rate to something like once every 8ms, unless force is true.
//   * force if true then force full poll on every call (ie do not internally rate-limit)
// Note that radio poll() can be for TX as well as RX activity.
// Not thread-safe, eg not to be called from within an ISR.
// FIXME trying to move into utils (for the time being.)
bool pollIO(const bool force)
  {
  static volatile uint8_t _pO_lastPoll;
  // Poll RX at most about every ~8ms.
  const uint8_t sct = OTV0P2BASE::getSubCycleTime();
  if(force || (sct != _pO_lastPoll))
    {
    _pO_lastPoll = sct;
    // Poll for inbound frames.
    // If RX is not interrupt-driven then
    // there will usually be little time to do this
    // before getting an RX overrun or dropped frame.
    PrimaryRadio.poll();
    }
  return(false);
  }


// Managed JSON stats.
static OTV0P2BASE::SimpleStatsRotation<12> ss1; // Configured for maximum different stats.	// FIXME increased for voice & for setback lockout
// Do bare stats transmission.
// Output should be filtered for items appropriate
// to current channel security and sensitivity level.
// This may be binary or JSON format.
//   * allowDoubleTX  allow double TX to increase chance of successful reception
//   * doBinary  send binary form if supported, else JSON form if supported
// Sends stats on primary radio channel 0 with possible duplicate to secondary channel.
// If sending encrypted then ID/counter fields (eg @ and + for JSON) are omitted
// as assumed supplied by security layer to remote recipent.
void bareStatsTX(const bool doBinary)
  {
  // Capture heavy stack usage from local allocations here.
  OTV0P2BASE::MemoryChecks::recordIfMinSP(2);

  // Note if radio/comms channel is itself framed.
  const bool framed = !PrimaryRadio.getChannelConfig()->isUnframed;
  constexpr bool RFM23BFramed = false; // Never use this raw framing unless enabled explicitly.

  constexpr bool doEnc = true;

  const bool neededWaking = OTV0P2BASE::powerUpSerialIfDisabled<>();
  
static_assert(OTV0P2BASE::FullStatsMessageCore_MAX_BYTES_ON_WIRE <= STATS_MSG_MAX_LEN, "FullStatsMessageCore_MAX_BYTES_ON_WIRE too big");
static_assert(OTV0P2BASE::MSG_JSON_MAX_LENGTH+1 <= STATS_MSG_MAX_LEN, "MSG_JSON_MAX_LENGTH too big"); // Allow 1 for trailing CRC.

    // Create scratch space for secure stats TX // FIXME
    // Buffer need be no larger than leading length byte + typical 64-byte radio module TX buffer limit + optional terminator.
    constexpr uint8_t MSG_BUF_SIZE = 1 + 64 + 1;
    constexpr uint8_t bufEncJSONlen = OTRadioLink::ENC_BODY_SMALL_FIXED_PTEXT_MAX_SIZE + 1;  // 3 = '}' + 0x0 + ? FIXME whuut?
    constexpr uint8_t ptextBuflen = bufEncJSONlen + 2;  // 2 = valvePC + hasStats
    static_assert(ptextBuflen == 34, "ptextBuflen wrong");  // TODO make sure this is correct!
    constexpr uint8_t scratchSpaceNeeded = MSG_BUF_SIZE + ptextBuflen;
    constexpr size_t WorkspaceSize = OTRadioLink::SimpleSecureFrame32or0BodyTXBase::generateSecureOFrameRawForTX_total_scratch_usage_OTAESGCM_2p0 + scratchSpaceNeeded;
    uint8_t workspace[WorkspaceSize];
    OTV0P2BASE::ScratchSpaceL sW(workspace, sizeof(workspace));

  // Allow space in buffer for:
  //   * buffer offset/preamble
  //   * max binary length, or max JSON length + 1 for CRC + 1 to allow detection of oversize message
  //   * terminating 0xff
  uint8_t * const buf = sW.buf;

  if(doBinary && !doEnc) // Note that binary form is not secure, so not permitted for secure systems.
    {} else // Send binary *or* JSON on each attempt so as not to overwhelm the receiver.
    {
    // Send JSON message.
    bool sendingJSONFailed = false; // Set true and stop attempting JSON send in case of error.

    // Set pointer location based on whether start of message will have preamble TODO move to OTRFM23BLink queueToSend?
    uint8_t *bptr = buf;
    if(RFM23BFramed) { bptr += STATS_MSG_START_OFFSET; }
    // Leave space for possible leading frame-length byte, eg for encrypted frame.
    else { ++bptr; }
    // Where to write the real frame content.
    uint8_t *const realTXFrameStart = bptr;

    // If forcing encryption or if unconditionally suppressed
    // then suppress the "@" ID field entirely,
    // assuming that the encrypted commands will carry the ID, ie in the 'envelope'.
    if(doEnc)
        { ss1.setID(V0p2_SENSOR_TAG_F("")); }

    // Managed JSON stats.
    // Make best use of available bandwidth...
    constexpr bool maximise = true;
    // Enable "+" count field for diagnostic purposes, eg while TX is lossy,
    // if the primary radio channel does not include a sequence number itself.
    // Assume that an encrypted channel will provide its own (visible) sequence counter.
    ss1.enableCount(!doEnc); 
//    if(ss1.isEmpty())
//      {
//      // Perform run-once operations...
//      }
#ifdef OTV0P2BASE_ErrorReport_DEFINED
    ss1.putOrRemove(OTV0P2BASE::ErrorReporter);
#endif
    ss1.put(TemperatureC16);
    ss1.put(RelHumidity);
    ss1.put(Occupancy.twoBitTag(), Occupancy.twoBitOccupancyValue()); // Reduce spurious TX cf percentage.
    ss1.put(Occupancy.vacHSubSensor);
    // OPTIONAL items
    // Only TX supply voltage for units apparently not mains powered, and TX with low priority as slow changing.
    if(!Supply_cV.isMains()) { ss1.put(Supply_cV, true); } else { ss1.remove(Supply_cV.tag()); }
    ss1.put(AmbLight); // Always send ambient light level (assuming sensor is present).
    // Show TRV-related stats since enabled.
    ss1.put(NominalRadValve); // Show modelled value to be able to deduce call-for-heat.
    ss1.put(NominalRadValve.targetTemperatureSubSensor);
    ss1.put(NominalRadValve.setbackSubSensor);
    ss1.put(NominalRadValve.cumulativeMovementSubSensor);
    // Show state of setback lockout.
    ss1.put(V0p2_SENSOR_TAG_F("gE"), OTRadValve::getSetbackLockout(), true);
    const uint8_t privacyLevel = OTV0P2BASE::stTXalwaysAll;

    // Redirect JSON output appropriately.
    // Part of sW, directly after the message buffer.
    // |    0    |     1    | 2 |  3:n | n+1 | n+2 | n is the end of the stats message. n+2 <= 34
    // | valvePC | hasStats | { | json | '}' | 0x0 |
    // ptextBuf is the entire frame
    uint8_t * const ptextBuf = sW.buf + MSG_BUF_SIZE;

    // Allow for a cap on JSON TX size, eg where TX is lossy for near-maximum sizes.
    // This can only reduce the maximum size, and it should not try to make it silly small.
    constexpr uint8_t max_plaintext_JSON_len = OTV0P2BASE::MSG_JSON_MAX_LENGTH;

    // Redirect JSON output appropriately.
    uint8_t *const bufJSON = doEnc ? (ptextBuf + 2) : bptr;
    const uint8_t bufJSONlen = doEnc ? bufEncJSONlen : min(max_plaintext_JSON_len+2, MSG_BUF_SIZE - (bptr-buf));  // XXX

    // Number of bytes written for body.
    // For non-secure, this is the size of the JSON text.
    // For secure this is overridden with the secure frame size.
    int8_t wrote = 0;

    // Generate JSON text.
    if(!sendingJSONFailed)
      {
      // Generate JSON and write to appropriate buffer:
      // direct to TX buffer if not encrypting, else to separate buffer.
      wrote = ss1.writeJSON(bufJSON, bufJSONlen, privacyLevel, maximise); //!allowDoubleTX && randRNG8NextBoolean());
      if(0 == wrote)
        {
#if 0 && defined(DEBUG)
DEBUG_SERIAL_PRINTLN_FLASHSTRING("JSON gen err!");
#endif
        sendingJSONFailed = true;
        }
      }

    // Push the JSON output to Serial.
    if(!sendingJSONFailed)
      {
      if(doEnc)
        {
        // Insert synthetic full ID/@ field for local stats, but no sequence number for now.
        Serial.print(F("{\"@\":\""));
        for(int i = 0; i < OTV0P2BASE::OpenTRV_Node_ID_Bytes; ++i) { Serial.print(eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_ID+i), HEX); }
        Serial.print(F("\","));
        Serial.write(bufJSON+1, wrote-1);
        Serial.println();
        }
      else
        { OTV0P2BASE::outputJSONStats(&Serial, true, bufJSON, bufJSONlen); } // Serial must already be running!
      OTV0P2BASE::flushSerialSCTSensitive(); // Ensure all flushed since system clock may be messed with...
      }

    // Get the 'building' key for stats sending.
    uint8_t key[16];
    if(!sendingJSONFailed && doEnc)
      {
      if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key))
        {
        sendingJSONFailed = true;
        OTV0P2BASE::serialPrintlnAndFlush(F("!TX key")); // Know why TX failed.
        }
      }

    // If doing encryption
    // then build encrypted frame from raw JSON.
    if(!sendingJSONFailed && doEnc)
      {
      // TODO Fold JSON
      // Explicit-workspace version of encryption.
      const OTRadioLink::SimpleSecureFrame32or0BodyTXBase::fixed32BTextSize12BNonce16BTagSimpleEncWithLWorkspace_ptr_t eW = OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleEnc_DEFAULT_WITH_LWORKSPACE;

      // Create subscratch space for encryption functions
      OTV0P2BASE::ScratchSpaceL subScratch(sW, scratchSpaceNeeded);
      constexpr uint8_t txIDLen = OTRadioLink::ENC_BODY_DEFAULT_ID_BYTES;
      // When sending on a channel with framing, do not explicitly send the frame length byte.
      const uint8_t offset = framed ? 1 : 0;
      // Assumed to be at least one free writeable byte ahead of bptr.
      // Get current modelled valve position.
      const uint8_t valvePC = NominalRadValve.get();
      const uint8_t bodylen = OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::getInstance().generateSecureOFrameRawForTX(
            (realTXFrameStart - offset), (MSG_BUF_SIZE - (realTXFrameStart-buf) + offset),
            txIDLen, valvePC, ptextBuf, eW, subScratch, key);
      sendingJSONFailed = (0 == bodylen);
      wrote = bodylen - offset;
      }

#if 0 && defined(DEBUG)
    if(sendingJSONFailed) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("!failed JSON enc"); }
#endif
    if(!sendingJSONFailed)
      {
      // If not encrypting, adjust the JSON for transmission and add a CRC.
      // (Set high-bit on final closing brace to make it unique, and compute (non-0xff) CRC.)
      if(!doEnc)
          {
          const uint8_t crc = OTV0P2BASE::adjustJSONMsgForTXAndComputeCRC((char *)bptr);
          if(0xff == crc) { sendingJSONFailed = true; }
          else
            {
            bptr += wrote;
            *bptr++ = crc; // Add 7-bit CRC for on-the-wire check.
            ++wrote;
            }
          }
        {
        // Send directly to the primary radio...
        if(!PrimaryRadio.queueToSend(realTXFrameStart, wrote)) { sendingJSONFailed = true; }
        }
      }

#if 1 && defined(DEBUG)
    if(sendingJSONFailed) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("!failed JSON TX"); }
#endif
    }

//DEBUG_SERIAL_PRINTLN_FLASHSTRING("Stats TX");
  if(neededWaking) { OTV0P2BASE::flushSerialProductive(); OTV0P2BASE::powerDownSerial(); }
  }

// Wire components together, eg for occupancy sensing.
static void wireComponentsTogether()
  {

  AmbLight.setOccCallbackOpt([](bool prob){if(prob){Occupancy.markAsPossiblyOccupied();}else{Occupancy.markAsJustPossiblyOccupied();}});

#if defined(TEMP_POT_AVAILABLE) && defined(valveUI_DEFINED)
  // Callbacks to set various mode combinations.
  // Typically at most one call would be made on any appropriate pot adjustment.
  TempPot.setWFBCallbacks([](bool x){valveUI.setWarmModeFromManualUI(x);},
                          [](bool x){valveUI.setBakeModeFromManualUI(x);});
#endif // TEMP_POT_AVAILABLE
  }


// Update sensors with historic/trailing statistics information where needed.
// Should be called at least hourly after all stats have been updated,
// but can also be called whenever the user adjusts settings for example.
static void updateSensorsFromStats()
  {
  // Update with rolling stats to adapt to sensors and local environment...
  // ...and prevailing bias, so may take a while to adjust.
  AmbLight.setTypMinMax(
          eeStats.getByHourStatRTC(OTV0P2BASE::NVByHourByteStatsBase::STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED),
          eeStats.getMinByHourStat(OTV0P2BASE::NVByHourByteStatsBase::STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED),
          eeStats.getMaxByHourStat(OTV0P2BASE::NVByHourByteStatsBase::STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED),
          !tempControl.hasEcoBias());
  }

// Run tasks needed at the end of each hour.
// Should be run once at a fixed slot in the last minute of each hour.
// Will be run after all stats for the current hour have been updated.
static void endOfHourTasks()
  {
  }

// Run tasks needed at the end of each day (nominal midnight).
// Should be run once at a fixed slot in the last minute of the last hour of each day.
// Will be run after all stats for the current hour have been updated.
static void endOfDayTasks()
  {
    // Count down the setback lockout if not finished...  (TODO-786, TODO-906)
    OTRadValve::countDownSetbackLockout();
  }


// Controller's view of Least Significant Digits of the current (local) time, in this case whole seconds.
// TIME_LSD ranges from 0 to TIME_CYCLE_S-1, also major cycle length.
static constexpr uint_fast8_t TIME_CYCLE_S = 60;
// Controller's notion/cache of seconds within major cycle.
static uint_fast8_t TIME_LSD;

// 'Elapsed minutes' count of minute/major cycles; cheaper than accessing RTC and not tied to real time.
// Starts at or just above zero (within the first 4-minute cycle) to help avoid collisions between units after mass power-up.
// Wraps at its maximum (0xff) value.
uint8_t minuteCount; // XXX

// Mask for Port B input change interrupts.
#define MASK_PB_BASIC 0b00000000 // Nothing.
#if defined(PIN_RFM_NIRQ) && defined(ENABLE_RADIO_RX) // RFM23B IRQ only used for RX.
  #if (PIN_RFM_NIRQ < 8) || (PIN_RFM_NIRQ > 15)
    #error PIN_RFM_NIRQ expected to be on port B
  #endif
  #define RFM23B_INT_MASK (1 << (PIN_RFM_NIRQ&7))
  #define MASK_PB (MASK_PB_BASIC | RFM23B_INT_MASK)
#else
  #define MASK_PB MASK_PB_BASIC
#endif

// Mask for Port C input change interrupts.
#define MASK_PC_BASIC 0b00000000 // Nothing.

// Mask for Port D input change interrupts.
#define SERIALRX_INT_MASK 0b00000001 // Serial RX
#define MASK_PD_BASIC SERIALRX_INT_MASK // Serial RX by default.
  #define MASK_PD1 MASK_PD_BASIC // Just serial RX, no voice.
#if BUTTON_MODE_L > 7
  #error BUTTON_MODE_L expected to be on port D
#endif
  #define MODE_INT_MASK (1 << (BUTTON_MODE_L&7))
  #define MASK_PD (MASK_PD1 | MODE_INT_MASK) // MODE button interrupt (et al).

void setupOpenTRV()
  {
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Entering setup...");
#endif

  // Radio not listening to start with.
  // Ignore any initial spurious RX interrupts for example.
  PrimaryRadio.listen(false);

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("PrimaryRadio.listen(false);");
#endif

  // Set up async edge interrupts.
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    //PCMSK0 = PB; PCINT  0--7    (LEARN1 and Radio)
    //PCMSK1 = PC; PCINT  8--15
    //PCMSK2 = PD; PCINT 16--24   (Serial RX and LEARN2 and MODE and Voice)

    PCICR =
#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
        1 | // 0x1 enables PB/PCMSK0.
#endif
#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
        2 | // 0x2 enables PC/PCMSK1.
#endif
#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
        4 | // 0x4 enables PD/PCMSK2.
#endif
        0;

#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
    PCMSK0 = MASK_PB;
#endif
#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
    PCMSK1 = MASK_PC;
#endif
#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
    PCMSK2 = MASK_PD;
#endif
    }

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("ints set up");
#endif

  // Wire components directly together, eg for occupancy sensing.
  wireComponentsTogether();

  // Initialise sensors with stats info where needed.
  updateSensorsFromStats();

  // Do early 'wake-up' stats transmission if possible
  // when everything else is set up and ready and allowed (TODO-636)
  // including all set-up and inter-wiring of sensors/actuators.
  if(enableTrailingStatsPayload())
    {
    // Attempt to maximise chance of reception with a double TX.
    // Assume not in hub mode (yet).
    // Send all possible formats, binary first (assumed complete in one message).
    bareStatsTX(true);  // XXX
    // Send JSON stats repeatedly (typically once or twice)
    // until all values pushed out (no 'changed' values unsent)
    // or limit reached.
    for(uint8_t i = 5; --i > 0; )
      {
      ::OTV0P2BASE::nap(WDTO_120MS, false); // Sleep long enough for receiver to have a chance to process previous TX.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING(" TX...");
#endif
      bareStatsTX(false);  // XXX
      if(!ss1.changedValue()) { break; }
      }
    }

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("setup stats sent");
#endif

#if !defined(DONT_RANDOMISE_MINUTE_CYCLE)
  // Start local counters in randomised positions to help avoid inter-unit collisions,
  // eg for mains-powered units starting up together after a power cut,
  // but without (eg) breaking any of the logic about what order things will be run first time through.
  // Uses some decent noise to try to start the units separated.
  const uint8_t b = OTV0P2BASE::getSecureRandomByte(); // randRNG8();
  // Start within bottom half of minute (or close to); sensor readings happen in second half.
  OTV0P2BASE::setSeconds(b >> 2);
  // Start anywhere in first 4 minute cycle.
  minuteCount = b & 3;
#endif

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Finishing setup...");
#endif

#if 0
  // Provide feedback to user that UI is coming to life (if any).
  userOpFeedback();
#endif

  // Set appropriate loop() values just before entering it.
  TIME_LSD = OTV0P2BASE::getSecondsLT();
  }

#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
//// Interrupt count.  Marked volatile so safe to read without a lock as is a single byte.
//static volatile uint8_t intCountPB;
// Previous state of port B pins to help detect changes.
static volatile uint8_t prevStatePB;
// Interrupt service routine for PB I/O port transition changes.
ISR(PCINT0_vect)
  {
//  ++intCountPB;
  const uint8_t pins = PINB;
  const uint8_t changes = pins ^ prevStatePB;
  prevStatePB = pins;

#if defined(RFM23B_INT_MASK)
  // RFM23B nIRQ falling edge is of interest.
  // Handler routine not required/expected to 'clear' this interrupt.
  // TODO: try to ensure that OTRFM23BLink.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
  if((changes & RFM23B_INT_MASK) && !(pins & RFM23B_INT_MASK))
    { PrimaryRadio.handleInterruptSimple(); }
#endif
  }
#endif

#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
// Previous state of port C pins to help detect changes.
static volatile uint8_t prevStatePC;
// Interrupt service routine for PC I/O port transition changes.
ISR(PCINT1_vect)
  {
//  const uint8_t pins = PINC;
//  const uint8_t changes = pins ^ prevStatePC;
//  prevStatePC = pins;
//
// ...
  }
#endif

#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
// Previous state of port D pins to help detect changes.
static volatile uint8_t prevStatePD;
// Interrupt service routine for PD I/O port transition changes (including RX).
ISR(PCINT2_vect)
  {
  const uint8_t pins = PIND;
  const uint8_t changes = pins ^ prevStatePD;
  prevStatePD = pins;

  // Mode button detection is on the falling edge (button pressed).
  if((changes & MODE_INT_MASK) && !(pins & MODE_INT_MASK))
    { valveUI.startBakeFromInt(); }

  // If an interrupt arrived from the serial RX then wake up the CLI.
  // Use a nominally rising edge to avoid spurious trigger when other interrupts are handled.
  // The will ensure that it is possible to wake the CLI subsystem with an extra CR or LF.
  // It is OK to trigger this from other things such as button presses.
  // TODO: ensure that resetCLIActiveTimer() is inlineable to minimise ISR prologue/epilogue time and space.
  if((changes & SERIALRX_INT_MASK) && !(pins & SERIALRX_INT_MASK))
    { OTV0P2BASE::CLI::resetCLIActiveTimer(); }
  }
#endif


// Main loop for OpenTRV radiator control.
// Note: exiting and re-entering can take a little while, handling Arduino background tasks such as serial.
void loopOpenTRV()
  {
#if 0 && defined(DEBUG) // Indicate loop start.
  DEBUG_SERIAL_PRINT('L');
  DEBUG_SERIAL_PRINT(TIME_LSD);
  DEBUG_SERIAL_PRINTLN();
#endif

  // Set up some variables before sleeping to minimise delay/jitter after the RTC tick.
  bool showStatus = false; // Show status at end of loop?

  // Sensor readings are taken late in each minute (where they are taken)
  // and if possible noise and heat and light should be minimised in this part of each minute to improve readings.
//  const bool sensorReading30s = (TIME_LSD >= 30);
  // Sensor readings and (stats transmissions) are nominally on a 4-minute cycle.
  const uint8_t minuteFrom4 = (minuteCount & 3);
  // The 0th minute in each group of four is always used for measuring where possible (possibly amongst others)
  // and where possible locally-generated noise and heat and light should be minimised in this minute
  // to give the best possible readings.
  // True if this is the first (0th) minute in each group of four.
  const bool minute0From4ForSensors = (0 == minuteFrom4);

  // Note last-measured battery status.
  const bool batteryLow = Supply_cV.isSupplyVoltageLow();

  // Run some tasks less often when not demanding heat (at the valve or boiler), so as to conserve battery/energy.
  // Spare the batteries if they are low, or the unit is in FROST mode, or if the room/area appears to be vacant.
  // Stay responsive if the valve is open and/or we are otherwise calling for heat.
  const bool conserveBattery =
    (batteryLow || !valveMode.inWarmMode() || Occupancy.longVacant()) &&
//    (!NominalRadValve.isControlledValveReallyOpen()); // &&  // Run at full speed until valve(s) should actually have shut and the boiler gone off.
    (!NominalRadValve.isCallingForHeat()); // Run at full speed until not nominally demanding heat, eg even during FROST mode or pre-heating.

  // Try if very near to end of cycle and thus causing an overrun.
  // Conversely, if not true, should have time to safely log outputs, etc.
  const uint8_t nearOverrunThreshold = OTV0P2BASE::GSCT_MAX - 8; // ~64ms/~32 serial TX chars of grace time...
//  bool tooNearOverrun = false; // Set flag that can be checked later.

//  if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }

  // Sleep in low-power mode (waiting for interrupts) until seconds roll.
  // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
  // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*E"); // End-of-cycle sleep.
#endif
//  // Ensure that serial I/O is off while sleeping, unless listening with radio.
//  if(!needsToListen) { powerDownSerial(); } else { powerUpSerialIfDisabled<V0P2_UART_BAUD>(); }
  // Ensure that serial I/O is off while sleeping.
  OTV0P2BASE::powerDownSerial();
  // Power down most stuff (except radio for hub RX).
  OTV0P2BASE::minimisePowerWithoutSleep();
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = OTV0P2BASE::getSecondsLT()))
    {

// If missing h/w interrupts for anything that needs rapid response
// then AVOID the lowest-power long sleep.
    if(false)
      {
      // If there is not hardware interrupt wakeup on receipt of a frame,
      // then this can only sleep for a short time between explicit poll()s,
      // though in any case allow wake on interrupt to minimise loop timing jitter
      // when the slow RTC 'end of sleep' tick arrives.
      OTV0P2BASE::nap(WDTO_15MS, true);
      }
    else
      {
      // Normal long minimal-power sleep until wake-up interrupt.
      // Rely on interrupt to force quick loop round to I/O poll.
      OTV0P2BASE::sleepUntilInt();
      }
//    DEBUG_SERIAL_PRINTLN_FLASHSTRING("w"); // Wakeup.
    }
  TIME_LSD = newTLSD;
  // Reset and immediately re-prime the RTC-based watchdog.
  OTV0P2BASE::resetRTCWatchDog();
  OTV0P2BASE::enableRTCWatchdog(true);
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif

//#if defined(ENABLE_BOILER_HUB) && defined(ENABLE_FHT8VSIMPLE) // Deal with FHT8V eavesdropping if needed.
//  // Check RSSI...
//  if(needsToListen)
//    {
//    const uint8_t rssi = RFM23.getRSSI();
//    static uint8_t lastRSSI;
//    if((rssi > 0) && (lastRSSI != rssi))
//      {
//      lastRSSI = rssi;
//      addEntropyToPool(rssi, 0); // Probably some real entropy but don't assume it.
//#if 0 && defined(DEBUG)
//      DEBUG_SERIAL_PRINT_FLASHSTRING("RSSI=");
//      DEBUG_SERIAL_PRINT(rssi);
//      DEBUG_SERIAL_PRINTLN();
//#endif
//      }
//    }
//#endif

#if 0 && defined(DEBUG) // Show CPU cycles.
  DEBUG_SERIAL_PRINT('C');
  DEBUG_SERIAL_PRINT(cycleCountCPU());
  DEBUG_SERIAL_PRINTLN();
#endif


  // START LOOP BODY
  // ===============


//  // Warn if too near overrun before.
//  if(tooNearOverrun) { OTV0P2BASE::serialPrintlnAndFlush(F("?near overrun")); }


  // Get current power supply voltage.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Vcc: ");
  DEBUG_SERIAL_PRINT(Supply_mV.read());
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("mV");
#endif

  // High-priority UI handing, every other/even second.
  // Show status if the user changed something significant.
  // Must take ~300ms or less so as not to run over into next half second if two TXs are done.
  bool recompute = false; // Set true if an extra recompute of target temperature should be done.
#if !defined(V0P2BASE_TWO_S_TICK_RTC_SUPPORT)
  if(0 == (TIME_LSD & 1))
#endif
    {
    // Run the OpenTRV button/LED UI if required.
    if(0 != valveUI.read()) // if(tickUI(TIME_LSD))
      {
      showStatus = true;
      recompute = true;
      }
    }

  // Handling the UI may have taken a little while, so process I/O a little.
  messageQueue.handle(true, PrimaryRadio); // Deal with any pending I/O.


  if(recompute || valveUI.veryRecentUIControlUse())
    {
    // Force immediate recompute of target temperature for (UI) responsiveness.
    NominalRadValve.computeTargetTemperature();
    // Keep dynamic adjustment of sensors up to date.
    updateSensorsFromStats();
    }


  // DO SCHEDULING

  // Once-per-minute tasks: all must take << 0.3s unless particular care is taken.
  // Run tasks spread throughout the minute to be as kind to batteries (etc) as possible.
  // Only when runAll is true run less-critical tasks that be skipped sometimes when particularly conserving energy.
  // Run all for first full 4-minute cycle, eg because unit may start anywhere in it.
  // Note: ensure only take ambient light reading at times when all LEDs are off (or turn them off).
  // TODO: coordinate temperature reading with time when radio and other heat-generating items are off for more accurate readings.
  const bool runAll = (!conserveBattery) || minute0From4ForSensors || (minuteCount < 4);

  switch(TIME_LSD) // With V0P2BASE_TWO_S_TICK_RTC_SUPPORT only even seconds are available.
    {
    case 0:
      {
      // Tasks that must be run every minute.
      ++minuteCount; // Note simple roll-over to 0 at max value.
      // Force to user's programmed schedule(s), if any, at the correct time.
      Scheduler.applyUserSchedule(&valveMode, OTV0P2BASE::getMinutesSinceMidnightLT());
      // Ensure that the RTC has been persisted promptly when necessary.
      OTV0P2BASE::persistRTC();
      // Run hourly tasks at the end of the hour.
      if(59 == OTV0P2BASE::getMinutesLT())
          {
          endOfHourTasks();
          if(23 == OTV0P2BASE::getHoursLT())
              { endOfDayTasks(); }
          }
      break;
      }

    // Churn/reseed PRNG(s) a little to improve unpredictability in use: should be lightweight.
    case 2: { if(runAll) { OTV0P2BASE::seedRNG8(minuteCount ^ OTV0P2BASE::getCPUCycleCount() ^ (uint8_t)Supply_cV.get(), OTV0P2BASE::_getSubCycleTime() ^ AmbLight.get(), (uint8_t)TemperatureC16.get()); } break; }
    // Force read of supply/battery voltage; measure and recompute status (etc) less often when already thought to be low, eg when conserving.
    case 4: { if(runAll) { Supply_cV.read(); } break; }

    // Periodic transmission of stats if NOT driving a local valve (else stats can be piggybacked onto that).
    // Randomised somewhat between slots and also within the slot to help avoid collisions.
    static uint8_t txTick;
    case 6: { txTick = OTV0P2BASE::randRNG8() & 7; break; } // Pick which of the 8 slots to use.
    case 8: case 10: case 12: case 14: case 16: case 18: case 20: case 22:
      {
      // Only the slot where txTick is zero is used.
      if(0 != txTick--) { break; }

#if !defined(ENABLE_FREQUENT_STATS_TX) // If ENABLE_FREQUENT_STATS_TX then send every minute regardless.
      // Stats TX in the minute (#1) after all sensors should have been polled
      // (so that readings are fresh) and evenly between.
      // Usually send one frame every 4 minutes, 2 if this is a valve.
      // No extra stats TX for changed data to reduce information/activity leakage.
      // Note that all O frames contain the current valve percentage,
      // which implies that any extra stats TX also speeds response to call-for-heat changes.
      // DHD20170113: was once every 4 minutes, but can make boiler response too slow.
      if(0 == (minuteFrom4 & 1)) { break; }
#endif

      // Abort if not allowed to send stats at all.
      // FIXME: fix this to send bare calls for heat / valve % instead from valves for secure non-FHT8V comms.
      if(!enableTrailingStatsPayload()) { break; }

      // Sleep randomly up to ~25% of the minor cycle
      // to spread transmissions and thus help avoid collisions.
      // (Longer than 25%/0.5s could interfere with other ops such as FHT8V TXes.)
      const uint8_t stopBy = 1 + (((OTV0P2BASE::GSCT_MAX >> 2) | 7) & OTV0P2BASE::randRNG8());
      while(OTV0P2BASE::getSubCycleTime() <= stopBy)
        {
        // Handle any pending I/O while waiting.
        if(messageQueue.handle(true, PrimaryRadio)) { continue; }
        // Sleep a little.
        OTV0P2BASE::nap(WDTO_15MS, true);
        }

      // Send stats!
      // Try for double TX for extra robustness unless:
      //   * this is a speculative 'extra' TX
      //   * battery is low
      //   * this node is a hub so needs to listen as much as possible
      // This doesn't generally/always need to send binary/both formats
      // if this is controlling a local FHT8V on which the binary stats can be piggybacked.
      // Ie, if doesn't have a local TRV then it must send binary some of the time.
      // Any recently-changed stats value is a hint that a strong transmission might be a good idea.
      const bool doBinary = false;
      bareStatsTX(doBinary);
      break;
      }

// SENSOR READ AND STATS
//
// All external sensor reads should be in the second half of the minute (>32) if possible.
// This is to have them as close to stats collection at the end of the minute as possible,
// and to allow randomisation of the start-up cycle position in the first 32s to help avoid inter-unit collisions.
// Also all sources of noise, self-heating, etc, may be turned off for the 'sensor read minute'
// and thus will have diminished by this point.

#ifdef TEMP_POT_AVAILABLE
    // Sample the user-selected WARM temperature target at a fixed rate.
    // This allows the unit to stay reasonably responsive to adjusting the temperature dial.
    case 48: { TempPot.read(); break; }
#endif

    // Read all environmental inputs, late in the cycle.
    // Sample humidity.
    case 50: { if(runAll) { RelHumidity.read(); } break; }

    // Poll ambient light level at a fixed rate.
    // This allows the unit to respond consistently to (eg) switching lights on (eg TODO-388).
    case 52:
      {
      // Force all UI lights off before sampling ambient light level.
      OTV0P2BASE::LED_HEATCALL_OFF();
      AmbLight.read();
      break;
      }

    // At a hub, sample temperature regularly as late as possible in the minute just before recomputing valve position.
    // Force a regular read to make stats such as rate-of-change simple and to minimise lag.
    // TODO: optimise to reduce power consumption when not calling for heat.
    // TODO: optimise to reduce self-heating jitter when in hub/listen/RX mode.
    case 54: { TemperatureC16.read(); break; }

    // Compute targets and heat demand based on environmental inputs and occupancy.
    // This should happen as soon after the latest readings as possible (temperature especially).
    case 56:
      {
#if defined(OTV0P2BASE_ErrorReport_DEFINED)
      // Age errors/warnings.
      OTV0P2BASE::ErrorReporter.read();
#endif

      // Update occupancy status (fresh for target recomputation) at a fixed rate.
      Occupancy.read();

      // Recompute target, valve position and call for heat, etc.
      // Should be called once per minute to work correctly.
      NominalRadValve.read();

      // Show current status if appropriate.
      if(runAll) { showStatus = true; }
      break;
      }

    // Stats samples; should never be missed.
    case 58:
      {
      // Update non-volatile stats.
      // Make the final update as near the end of the hour as possible to reduce glitches (TODO-1086),
      // and with other optional non-full samples evenly spaced throughout the hour.
      // Race-free.
      const uint_least16_t msm = OTV0P2BASE::getMinutesSinceMidnightLT();
      const uint8_t mm = msm % 60;
      if(59 == mm) { statsU.sampleStats(true, uint8_t(msm / 60)); }
      else if((statsU.maxSamplesPerHour > 1) && (29 == mm)) { statsU.sampleStats(false, uint8_t(msm / 60)); }
      break;
      }
    }

  // Generate periodic status reports.
  if(showStatus) { serialStatusReport(); }

  // End-of-loop processing, that may be slow.
  // Ensure progress on queued messages ahead of slow work.  (TODO-867)
  messageQueue.handle(true, PrimaryRadio); // Deal with any pending I/O.

  // Handle local direct-drive valve, eg DORM1.
  // If waiting for for verification that the valve has been fitted
  // then accept any manual interaction with controls as that signal.
  // Also have a backup timeout of at least ~10m from startup
  // for automatic recovery after a crash and restart,
  // or where fitter simply forgets to initiate cablibration.
  if(ValveDirect.isWaitingForValveToBeFitted())
      {
      // Defer automatic recovery when battery low or in dark in case crashing/restarting
      // to try to avoid disturbing/waking occupants and/or entering battery death spiral.  (TODO-1037, TODO-963)
      // The initial minuteCount value can be anywhere in the range [0,3];
      // pick threshold to give user at least a couple of minutes to fit the device
      // if they do so with the battery in place.
      const bool delayRecalibration = batteryLow || AmbLight.isRoomDark();
      if(valveUI.veryRecentUIControlUse() || (minuteCount >= (delayRecalibration ? 240 : 5)))
          { ValveDirect.signalValveFitted(); }
      }
  // Provide regular poll to motor driver.
  // May take significant time to run
  // so don't call when timing is critical
  // nor when not much time left this cycle,
  // nor some of the time during startup if possible,
  // so as (for example) to allow the CLI to be operable.
  // Only calling this after most other heavy-lifting work is likely done.
  // Note that FHT8V sync will take up at least the first 1s of a 2s subcycle.
  if(!showStatus &&
     (OTV0P2BASE::getSubCycleTime() < ((OTV0P2BASE::GSCT_MAX/4)*3)))
    { ValveDirect.read(); }

  // Command-Line Interface (CLI) polling.
  // If a reasonable chunk of the minor cycle remains after all other work is done
  // AND the CLI is / should be active OR a status line has just been output
  // then poll/prompt the user for input
  // using a timeout which should safely avoid overrun, ie missing the next basic tick,
  // and which should also allow some energy-saving sleep.
  if(OTV0P2BASE::CLI::isCLIActive())
    {
    const uint8_t stopBy = nearOverrunThreshold - 1;
    char buf[BUFSIZ_pollUI];
    OTV0P2BASE::ScratchSpace s((uint8_t*)buf, sizeof(buf));
    pollCLI(stopBy, 0 == TIME_LSD, s);
    }


#if 0 && defined(DEBUG)
  const int tDone = getSubCycleTime();
  if(tDone > 1) // Ignore for trivial 1-click time.
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("done in "); // Indicates what fraction of available loop time was used / 256.
    DEBUG_SERIAL_PRINT(tDone);
    DEBUG_SERIAL_PRINT_FLASHSTRING(" @ ");
    DEBUG_SERIAL_TIMESTAMP();
    DEBUG_SERIAL_PRINTLN();
    }
#endif
  }
