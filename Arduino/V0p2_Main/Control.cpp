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
#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT) || defined(ENABLE_SECURE_RADIO_BEACON)
#include <OTAESGCM.h>
#endif

#ifdef ENABLE_MODELLED_RAD_VALVE
static OTV0P2BASE::EEPROMByHourByteStats ebhs;
// Create setback lockout if needed.
  typedef bool(*setbackLockout_t)();
#if defined(ENABLE_SETBACK_LOCKOUT_COUNTDOWN) && defined(ARDUINO_ARCH_AVR)
  // If allowing setback lockout, eg for testing, then inject suitable lambda.
  static bool setbackLockout() { return(0 != OTRadValve::getSetbackLockout()); }
#else
  static constexpr setbackLockout_t setbackLockout = NULL;
#endif
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
#ifdef HAS_DORM1_VALVE_DRIVE
  &ValveDirect,
#else
  NULL,
#endif
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
#endif // ENABLE_MODELLED_RAD_VALVE


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
#ifdef ENABLE_RADIO_PRIMARY_MODULE
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
  #ifdef ENABLE_RADIO_SECONDARY_MODULE
    SecondaryRadio.poll();
  #endif
    }
#endif
  return(false);
  }


#if defined(ENABLE_RFM23B_FS20_RAW_PREAMBLE)
// Send the underlying stats binary/text 'whitened' message.
// This must be terminated with an 0xff (which is not sent),
// and no longer than STATS_MSG_MAX_LEN bytes long in total (excluding the terminating 0xff).
// This must not contain any 0xff and should not contain long runs of 0x00 bytes.
// The message to be sent must be written at an offset of STATS_MSG_START_OFFSET from the start of the buffer.
// This routine will alter the content of the buffer for transmission,
// and the buffer should not be re-used as is.
//   * doubleTX  double TX to increase chance of successful reception
//   * RFM23BfriendlyPremable  if true then add an extra preamble
//     to allow RFM23B-based receiver to RX this
// This will use whichever transmission medium/carrier/etc is available.
static void RFM22RawStatsTXFFTerminated(uint8_t * const buf, const bool doubleTX, bool RFM23BFramed = true)
  {
  if(RFM23BFramed) RFM22RXPreambleAdd(buf);     // Only needed for RFM23B. This should be made more clear when refactoring
  const uint8_t buflen = OTRadioLink::frameLenFFTerminated(buf);
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("buflen=");
    DEBUG_SERIAL_PRINT(buflen);
    DEBUG_SERIAL_PRINTLN();
#endif // DEBUG
  if(!PrimaryRadio.queueToSend(buf, buflen, 0, (doubleTX ? OTRadioLink::OTRadioLink::TXmax : OTRadioLink::OTRadioLink::TXnormal)))
    {
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("!TX failed");
#endif
    } // DEBUG
  //DEBUG_SERIAL_PRINTLN_FLASHSTRING("RS");
  }
#endif // defined(ENABLE_RFM23B_FS20_RAW_PREAMBLE)


#ifdef ENABLE_STATS_TX
#if defined(ENABLE_JSON_OUTPUT)
// Managed JSON stats.
static OTV0P2BASE::SimpleStatsRotation<12> ss1; // Configured for maximum different stats.	// FIXME increased for voice & for setback lockout
#endif // ENABLE_JSON_OUTPUT
// Do bare stats transmission.
// Output should be filtered for items appropriate
// to current channel security and sensitivity level.
// This may be binary or JSON format.
//   * allowDoubleTX  allow double TX to increase chance of successful reception
//   * doBinary  send binary form if supported, else JSON form if supported
// Sends stats on primary radio channel 0 with possible duplicate to secondary channel.
// If sending encrypted then ID/counter fields (eg @ and + for JSON) are omitted
// as assumed supplied by security layer to remote recipent.
void bareStatsTX(const bool allowDoubleTX, const bool doBinary)
  {
  // Capture heavy stack usage from local allocations here.
  OTV0P2BASE::MemoryChecks::recordIfMinSP(2);

  // Note if radio/comms channel is itself framed.
  const bool framed = !PrimaryRadio.getChannelConfig()->isUnframed;
#if defined(ENABLE_RFM23B_FS20_RAW_PREAMBLE)
  // Add RFM23B preamble and a trailing CRC to the frame IFF channel is unframed.
  const bool RFM23BFramed = !framed;
#else
  constexpr bool RFM23BFramed = false; // Never use this raw framing unless enabled explicitly.
#endif

#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
  constexpr bool doEnc = true;
#else
  constexpr bool doEnc = false;
#endif

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

#if defined(ENABLE_JSON_OUTPUT)
  if(doBinary && !doEnc) // Note that binary form is not secure, so not permitted for secure systems.
#endif // ENABLE_JSON_OUTPUT
    {
#if defined(ENABLE_BINARY_STATS_TX) && defined(ENABLE_FS20_ENCODING_SUPPORT) && !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
    // Send binary message first (insecure, FS20-piggyback format).
    // Gather core stats.
    OTV0P2BASE::FullStatsMessageCore_t content;
    OTRadValve::populateCoreStats(&content,
                    (localFHT8VTRVEnabled() ? &FHT8V : NULL),
                    TemperatureC16.get(),
                    Supply_cV.isSupplyVoltageLow(),
                    AmbLight.get(),
                    Occupancy.twoBitOccupancyValue());
    const uint8_t *msg1 = OTV0P2BASE::encodeFullStatsMessageCore(buf + STATS_MSG_START_OFFSET, sizeof(buf) - STATS_MSG_START_OFFSET, OTV0P2BASE::getStatsTXLevel(), false, &content);
    if(NULL == msg1)
      {
#if 0
DEBUG_SERIAL_PRINTLN_FLASHSTRING("Bin gen err!");
#endif
      return;
      }
    // Send it!
    RFM22RawStatsTXFFTerminated(buf, allowDoubleTX);
    // Record stats as if remote, and treat channel as secure.
    outputCoreStats(&Serial, true, &content);
#endif // defined(ENABLE_BINARY_STATS_TX) ...
    }

#if defined(ENABLE_JSON_OUTPUT)
  else // Send binary *or* JSON on each attempt so as not to overwhelm the receiver.
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
#if defined(ENABLE_JSON_SUPPRESSED_ID)
    if(true)
#else
    if(doEnc)
#endif // defined(ENABLE_JSON_SUPPRESSED_ID)
        { ss1.setID(V0p2_SENSOR_TAG_F("")); }
    else
      {
#if defined(ENABLE_FHT8VSIMPLE) && 0 // FIXME: find alternative
      // Insert FHT8V-style ID in stats messages if appropriate.
      // Will not be appropriate if primary channel provides ID itself.
      static char idBuf[5]; // Static so as to have lifetime no shorter than ss1.
      if(localFHT8VTRVEnabled())
        {
        const uint8_t hc1 = FHT8V.nvGetHC1();
        const uint8_t hc2 = FHT8V.nvGetHC2();
        idBuf[0] = OTV0P2BASE::hexDigit(hc1 >> 4);
        idBuf[1] = OTV0P2BASE::hexDigit(hc1);
        idBuf[2] = OTV0P2BASE::hexDigit(hc2 >> 4);
        idBuf[3] = OTV0P2BASE::hexDigit(hc2);
        idBuf[4] = '\0';
        ss1.setID(idBuf);
        }
      else { ss1.setID(NULL); } // Use built-in ID.
#endif
      }

    // Managed JSON stats.
#if defined(ENABLE_JSON_FRAME_MINIMISED)
    // Minimise frame size (eg for noisy radio links)...
    constexpr bool maximise = false;
    // Suppress "+" count field, accepting loss of diagnostics.
    ss1.enableCount(false); 
#else
    // Make best use of available bandwidth...
    constexpr bool maximise = true;
    // Enable "+" count field for diagnostic purposes, eg while TX is lossy,
    // if the primary radio channel does not include a sequence number itself.
    // Assume that an encrypted channel will provide its own (visible) sequence counter.
    ss1.enableCount(!doEnc); 
#endif // defined(ENABLE_JSON_FRAME_MINIMISED)
//    if(ss1.isEmpty())
//      {
//      // Perform run-once operations...
//      }
#ifdef OTV0P2BASE_ErrorReport_DEFINED
    ss1.putOrRemove(OTV0P2BASE::ErrorReporter);
#endif
    ss1.put(TemperatureC16);
#if defined(HUMIDITY_SENSOR_SUPPORT)
    ss1.put(RelHumidity);
#endif // defined(HUMIDITY_SENSOR_SUPPORT)
#if defined(ENABLE_OCCUPANCY_SUPPORT)
    ss1.put(Occupancy.twoBitTag(), Occupancy.twoBitOccupancyValue()); // Reduce spurious TX cf percentage.
#if !defined(ENABLE_TRIMMED_BANDWIDTH)
    ss1.put(Occupancy.vacHSubSensor);
#endif // !defined(ENABLE_TRIMMED_BANDWIDTH)
#endif // defined(ENABLE_OCCUPANCY_SUPPORT)
    // OPTIONAL items
    // Only TX supply voltage for units apparently not mains powered, and TX with low priority as slow changing.
    if(!Supply_cV.isMains()) { ss1.put(Supply_cV, true); } else { ss1.remove(Supply_cV.tag()); }
#ifdef ENABLE_BOILER_HUB
    // Show boiler state for boiler hubs.
    ss1.put(V0p2_SENSOR_TAG_F("b"), (int) BoilerHub.isBoilerOn());
#endif // ENABLE_BOILER_HUB
#ifdef ENABLE_AMBLIGHT_SENSOR
    ss1.put(AmbLight); // Always send ambient light level (assuming sensor is present).
#endif // ENABLE_AMBLIGHT_SENSOR
#ifdef ENABLE_VOICE_STATS
    ss1.put(Voice);
#endif // ENABLE_VOICE_STATS
#if defined(ENABLE_LOCAL_TRV)
    // Show TRV-related stats since enabled.
    ss1.put(NominalRadValve); // Show modelled value to be able to deduce call-for-heat.
    ss1.put(NominalRadValve.targetTemperatureSubSensor);
    ss1.put(NominalRadValve.setbackSubSensor);
#if !defined(ENABLE_TRIMMED_BANDWIDTH)
    ss1.put(NominalRadValve.cumulativeMovementSubSensor);
#endif // !defined(ENABLE_TRIMMED_BANDWIDTH)
#endif // defined(ENABLE_LOCAL_TRV)
#ifdef ENABLE_SETBACK_LOCKOUT_COUNTDOWN
    // Show state of setback lockout.
    ss1.put(V0p2_SENSOR_TAG_F("gE"), OTRadValve::getSetbackLockout(), true);
#endif // ENABLE_SETBACK_LOCKOUT_COUNTDOWN
#if defined(ENABLE_ALWAYS_TX_ALL_STATS)
    const uint8_t privacyLevel = OTV0P2BASE::stTXalwaysAll;
#else
    const uint8_t privacyLevel = OTV0P2BASE::getStatsTXLevel();
#endif

    // Redirect JSON output appropriately.
    // Part of sW, directly after the message buffer.
    // |    0    |     1    | 2 |  3:n | n+1 | n+2 | n is the end of the stats message. n+2 <= 34
    // | valvePC | hasStats | { | json | '}' | 0x0 |
    // ptextBuf is the entire frame
    uint8_t * const ptextBuf = sW.buf + MSG_BUF_SIZE;

    // Allow for a cap on JSON TX size, eg where TX is lossy for near-maximum sizes.
    // This can only reduce the maximum size, and it should not try to make it silly small.
#if defined(ENABLE_JSON_STATS_LEN_CAP)
    constexpr uint8_t max_plaintext_JSON_len = min(OTV0P2BASE::MSG_JSON_MAX_LENGTH, ENABLE_JSON_STATS_LEN_CAP);
#else
    constexpr uint8_t max_plaintext_JSON_len = OTV0P2BASE::MSG_JSON_MAX_LENGTH;
#endif

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
 #if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
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
#endif // defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
        { OTV0P2BASE::outputJSONStats(&Serial, true, bufJSON, bufJSONlen); } // Serial must already be running!
      OTV0P2BASE::flushSerialSCTSensitive(); // Ensure all flushed since system clock may be messed with...
      }

    // Get the 'building' key for stats sending.
    uint8_t key[16];
    if(!sendingJSONFailed && doEnc)
      {
#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key))
        {
        sendingJSONFailed = true;
        OTV0P2BASE::serialPrintlnAndFlush(F("!TX key")); // Know why TX failed.
        }
#else
      sendingJSONFailed = true; // Crypto support may not be available.
#endif // defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      }

    // If doing encryption
    // then build encrypted frame from raw JSON.
    if(!sendingJSONFailed && doEnc)
      {
#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      // TODO Fold JSON
      // Explicit-workspace version of encryption.
      const OTRadioLink::SimpleSecureFrame32or0BodyTXBase::fixed32BTextSize12BNonce16BTagSimpleEncWithLWorkspace_ptr_t eW = OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleEnc_DEFAULT_WITH_LWORKSPACE;

      // Create subscratch space for encryption functions
      OTV0P2BASE::ScratchSpaceL subScratch(sW, scratchSpaceNeeded);
      constexpr uint8_t txIDLen = OTRadioLink::ENC_BODY_DEFAULT_ID_BYTES;
      // When sending on a channel with framing, do not explicitly send the frame length byte.
      const uint8_t offset = framed ? 1 : 0;
      // Assumed to be at least one free writeable byte ahead of bptr.
#if defined(ENABLE_NOMINAL_RAD_VALVE)
      // Get current modelled valve position.
      const uint8_t valvePC = NominalRadValve.get();
#else  // defined(ENABLE_NOMINAL_RAD_VALVE)
      // Distinguished 'invalid' valve position; never mistaken for a real valve.
      constexpr uint8_t valvePC = 0x7f;
#endif // defined(ENABLE_NOMINAL_RAD_VALVE)
      const uint8_t bodylen = OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::getInstance().generateSecureOFrameRawForTX(
            (realTXFrameStart - offset), (MSG_BUF_SIZE - (realTXFrameStart-buf) + offset),
            txIDLen, valvePC, ptextBuf, eW, subScratch, key);
      sendingJSONFailed = (0 == bodylen);
      wrote = bodylen - offset;
#else  // defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      sendingJSONFailed = true; // Crypto support may not be available.
#endif // defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      }

#if 0 && defined(DEBUG)
    if(sendingJSONFailed) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("!failed JSON enc"); }
#endif

#ifdef ENABLE_RADIO_SECONDARY_MODULE
    if(!sendingJSONFailed)
      {
      // Write out unadjusted JSON or encrypted frame on secondary radio.
//      SecondaryRadio.queueToSend(realTXFrameStart, doEnc ? (bptr - realTXFrameStart) : wrote);
      // Assumes that framing (or not) of primary and secondary radios is the same (usually: both framed).
      SecondaryRadio.queueToSend(realTXFrameStart, wrote);
      }
#endif // ENABLE_RADIO_SECONDARY_MODULE

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

#if defined(ENABLE_RFM23B_FS20_RAW_PREAMBLE)
      // Use ugly 0xff-terminated RFM23B send.
      if(RFM23BFramed)
        {
        *bptr = 0xff; // Terminate message for TX.
        RFM22RawStatsTXFFTerminated(buf, allowDoubleTX, RFM23BFramed);
        }
      else
#endif
        {
        // Send directly to the primary radio...
        if(!PrimaryRadio.queueToSend(realTXFrameStart, wrote)) { sendingJSONFailed = true; }
        }
      }

#if 1 && defined(DEBUG)
    if(sendingJSONFailed) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("!failed JSON TX"); }
#endif
    }
#endif // defined(ENABLE_JSON_OUTPUT)

//DEBUG_SERIAL_PRINTLN_FLASHSTRING("Stats TX");
  if(neededWaking) { OTV0P2BASE::flushSerialProductive(); OTV0P2BASE::powerDownSerial(); }
  }
#endif // defined(ENABLE_STATS_TX)


// Wire components together, eg for occupancy sensing.
static void wireComponentsTogether()
  {
#ifdef ENABLE_FHT8VSIMPLE
  // Set up radio with FHT8V.
  FHT8V.setRadio(&PrimaryRadio);
  // Load EEPROM house codes into primary FHT8V instance at start.
  FHT8V.nvLoadHC();
#endif // ENABLE_FHT8VSIMPLE

#if defined(ENABLE_OCCUPANCY_SUPPORT) && defined(ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT)
  AmbLight.setOccCallbackOpt([](bool prob){if(prob){Occupancy.markAsPossiblyOccupied();}else{Occupancy.markAsJustPossiblyOccupied();}});
#endif // ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT

#if defined(ENABLE_OCCUPANCY_SUPPORT) && defined(ENABLE_OCCUPANCY_DETECTION_FROM_VOICE)
  Voice.setPossOccCallback([]{Occupancy.markAsPossiblyOccupied();});
#endif // ENABLE_OCCUPANCY_DETECTION_FROM_VOICE

#if defined(TEMP_POT_AVAILABLE) && defined(valveUI_DEFINED)
  // Callbacks to set various mode combinations.
  // Typically at most one call would be made on any appropriate pot adjustment.
  TempPot.setWFBCallbacks([](bool x){valveUI.setWarmModeFromManualUI(x);},
                          [](bool x){valveUI.setBakeModeFromManualUI(x);});
#endif // TEMP_POT_AVAILABLE

#if V0p2_REV == 14
  pinMode(REGULATOR_POWERUP, OUTPUT);
#ifdef ENABLE_VOICE_SENSOR
  fastDigitalWrite(REGULATOR_POWERUP, HIGH);
#else
  fastDigitalWrite(REGULATOR_POWERUP, LOW);
#endif // ENABLE_VOICE_SENSOR
#endif // V0p2_REV == 14
  }


// Update sensors with historic/trailing statistics information where needed.
// Should be called at least hourly after all stats have been updated,
// but can also be called whenever the user adjusts settings for example.
static void updateSensorsFromStats()
  {
#if defined(ENABLE_AMBLIGHT_SENSOR) && defined(ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT)
  // Update with rolling stats to adapt to sensors and local environment...
  // ...and prevailing bias, so may take a while to adjust.
  AmbLight.setTypMinMax(
          eeStats.getByHourStatRTC(OTV0P2BASE::NVByHourByteStatsBase::STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED),
          eeStats.getMinByHourStat(OTV0P2BASE::NVByHourByteStatsBase::STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED),
          eeStats.getMaxByHourStat(OTV0P2BASE::NVByHourByteStatsBase::STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED),
          !tempControl.hasEcoBias());
#endif // ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT
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
#if defined(ENABLE_SETBACK_LOCKOUT_COUNTDOWN)
    // Count down the setback lockout if not finished...  (TODO-786, TODO-906)
    OTRadValve::countDownSetbackLockout();
#endif
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
#if defined(ENABLE_VOICE_SENSOR)
  #if VOICE_NIRQ > 7
    #error VOICE_NIRQ expected to be on port D
  #endif
  #define VOICE_INT_MASK (1 << (VOICE_NIRQ&7))
  #define MASK_PD1 (MASK_PD_BASIC | VOICE_INT_MASK)
#else
  #define MASK_PD1 MASK_PD_BASIC // Just serial RX, no voice.
#endif
#if defined(ENABLE_SIMPLIFIED_MODE_BAKE)
#if BUTTON_MODE_L > 7
  #error BUTTON_MODE_L expected to be on port D
#endif
  #define MODE_INT_MASK (1 << (BUTTON_MODE_L&7))
  #define MASK_PD (MASK_PD1 | MODE_INT_MASK) // MODE button interrupt (et al).
#else
  #define MASK_PD MASK_PD1 // No MODE button interrupt.
#endif

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

#ifdef ENABLE_STATS_TX
  // Do early 'wake-up' stats transmission if possible
  // when everything else is set up and ready and allowed (TODO-636)
  // including all set-up and inter-wiring of sensors/actuators.
  if(enableTrailingStatsPayload())
    {
    // Attempt to maximise chance of reception with a double TX.
    // Assume not in hub mode (yet).
    // Send all possible formats, binary first (assumed complete in one message).
    bareStatsTX(true, true);  // XXX
    // Send JSON stats repeatedly (typically once or twice)
    // until all values pushed out (no 'changed' values unsent)
    // or limit reached.
    for(uint8_t i = 5; --i > 0; )
      {
      ::OTV0P2BASE::nap(WDTO_120MS, false); // Sleep long enough for receiver to have a chance to process previous TX.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING(" TX...");
#endif
      bareStatsTX(true, false);  // XXX
      if(!ss1.changedValue()) { break; }
      }
    }
#endif

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

#if !defined(ALT_MAIN_LOOP) // Do not define handlers here when alt main is in use.

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

#if defined(ENABLE_SIMPLIFIED_MODE_BAKE)
  // Mode button detection is on the falling edge (button pressed).
  if((changes & MODE_INT_MASK) && !(pins & MODE_INT_MASK))
    { valveUI.startBakeFromInt(); }
#endif // defined(ENABLE_SIMPLIFIED_MODE_BAKE)

#if defined(ENABLE_VOICE_SENSOR)
//  // Voice detection is a falling edge.
//  // Handler routine not required/expected to 'clear' this interrupt.
//  // FIXME: ensure that Voice.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
//  if((changes & VOICE_INT_MASK) && !(pins & VOICE_INT_MASK))
  // Voice detection is a RISING edge.
  // Handler routine not required/expected to 'clear' this interrupt.
  // FIXME: ensure that Voice.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
  if((changes & VOICE_INT_MASK) && (pins & VOICE_INT_MASK))
    { Voice.handleInterruptSimple(); }
#endif // defined(ENABLE_VOICE_SENSOR)

  // If an interrupt arrived from the serial RX then wake up the CLI.
  // Use a nominally rising edge to avoid spurious trigger when other interrupts are handled.
  // The will ensure that it is possible to wake the CLI subsystem with an extra CR or LF.
  // It is OK to trigger this from other things such as button presses.
  // TODO: ensure that resetCLIActiveTimer() is inlineable to minimise ISR prologue/epilogue time and space.
  if((changes & SERIALRX_INT_MASK) && !(pins & SERIALRX_INT_MASK))
    { OTV0P2BASE::CLI::resetCLIActiveTimer(); }
  }
#endif

#endif // !defined(ALT_MAIN_LOOP) // Do not define handlers here when alt main is in use.

#if defined(ENABLE_RADIO_RX)
// Returns true if continuous background RX has been set up.
static bool setUpContinuousRX()
  {
  // Possible paranoia...
  // Periodically (every few hours) force radio off or at least to be not listening.
  if((30 == TIME_LSD) && (128 == minuteCount)) { PrimaryRadio.listen(false); }

#if defined(ENABLE_CONTINUOUS_RX)
  // IF IN CENTRAL HUB MODE: listen out for OpenTRV units calling for heat.
  // Power optimisation 1: when >> 1 TX cycle (of ~2mins) need not listen, ie can avoid enabling receiver.
  // Power optimisation 2: TODO: when (say) >>30m since last call for heat then only sample listen for (say) 3 minute in 10 (not at a TX cycle multiple).
  // TODO: These optimisation are more important when hub unit is running a local valve
  // to avoid temperature over-estimates from self-heating,
  // and could be disabled if no local valve is being run to provide better response to remote nodes.
#ifdef ENABLE_DEFAULT_ALWAYS_RX
  const bool needsToListen = true; // By default listen if always doing RX.
#else
  bool needsToListen = hubManager.inHubMode(); // By default assume no need to listen unless in hub mode.
#endif

#if 0 && defined(DEBUG) && defined(ENABLE_DEFAULT_ALWAYS_RX)
  const int8_t listenChannel = PrimaryRadio.getListenChannel();
 if(listenChannel < 0)
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("LISTEN CHANNEL ");
    DEBUG_SERIAL_PRINT(listenChannel);
    DEBUG_SERIAL_PRINTLN();
    }
#if 0 && defined(ENABLE_RADIO_RFM23B) // ONLY IF PrimaryRadio really is RFM23B!
    const uint8_t rmode = RFM23B.getMode();
    DEBUG_SERIAL_PRINT_FLASHSTRING("RFM23B mode ");
    DEBUG_SERIAL_PRINT(rmode);
    DEBUG_SERIAL_PRINTLN();
#endif
#endif

  // Act on eavesdropping need, setting up or clearing down hooks as required.
  PrimaryRadio.listen(needsToListen);

  if(needsToListen)
    {
#if 1 && defined(DEBUG) && defined(ENABLE_RADIO_RX) && !defined(ENABLE_TRIMMED_MEMORY)
    for(uint8_t lastErr; 0 != (lastErr = PrimaryRadio.getRXErr()); )
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING("!RX err ");
      DEBUG_SERIAL_PRINT(lastErr);
      DEBUG_SERIAL_PRINTLN();
      }
    const uint8_t dropped = PrimaryRadio.getRXMsgsDroppedRecent();
    static uint8_t oldDropped;
    if(dropped != oldDropped)
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING("!RX DROP ");
      DEBUG_SERIAL_PRINT(dropped);
      DEBUG_SERIAL_PRINTLN();
      oldDropped = dropped;
      }
#endif
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
    // Filtered out messages are not an error.
    const uint8_t filtered = PrimaryRadio.getRXMsgsFilteredRecent();
    static uint8_t oldFiltered;
    if(filtered != oldFiltered)
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING("RX filtered ");
      DEBUG_SERIAL_PRINT(filtered);
      DEBUG_SERIAL_PRINTLN();
      oldFiltered = filtered;
      }
#endif
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("hub listen, on/cd ");
    DEBUG_SERIAL_PRINT(boilerCountdownTicks);
    DEBUG_SERIAL_PRINT_FLASHSTRING("t quiet ");
    DEBUG_SERIAL_PRINT(boilerNoCallM);
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("m");
#endif
    }
  return(needsToListen);
#else
  return(false);
#endif // defined(ENABLE_CONTINUOUS_RX)
  }
#endif // defined(ENABLE_RADIO_RX)


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

  // Use the zeroth second in each minute to force extra deep device sleeps/resets, etc.
  const bool second0 = (0 == TIME_LSD);
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
  // True if this is the minute after all sensors should have been sampled.
  const bool minute1From4AfterSensors = (1 == minuteFrom4);

  // Note last-measured battery status.
  const bool batteryLow = Supply_cV.isSupplyVoltageLow();

  // Run some tasks less often when not demanding heat (at the valve or boiler), so as to conserve battery/energy.
  // Spare the batteries if they are low, or the unit is in FROST mode, or if the room/area appears to be vacant.
  // Stay responsive if the valve is open and/or we are otherwise calling for heat.
  const bool conserveBattery =
    (batteryLow || !valveMode.inWarmMode() || Occupancy.longVacant()) &&
#if defined(ENABLE_BOILER_HUB)
    (!BoilerHub.isBoilerOn()) && // Unless the boiler is off, stay responsive.
#endif
#if defined(ENABLE_NOMINAL_RAD_VALVE) && defined(LOCAL_VALVE)
//    (!NominalRadValve.isControlledValveReallyOpen()); // &&  // Run at full speed until valve(s) should actually have shut and the boiler gone off.
    (!NominalRadValve.isCallingForHeat()); // Run at full speed until not nominally demanding heat, eg even during FROST mode or pre-heating.
#else
    true; // Allow local power conservation if all other factors are right.
#endif

  // Try if very near to end of cycle and thus causing an overrun.
  // Conversely, if not true, should have time to safely log outputs, etc.
  const uint8_t nearOverrunThreshold = OTV0P2BASE::GSCT_MAX - 8; // ~64ms/~32 serial TX chars of grace time...
//  bool tooNearOverrun = false; // Set flag that can be checked later.

//  if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }

#if defined(ENABLE_CONTINUOUS_RX)
  const bool needsToListen = setUpContinuousRX();
#endif

#if defined(ENABLE_BOILER_HUB)
  // Set BOILER_OUT as appropriate for calls for heat.
  BoilerHub.processCallsForHeat(second0, hubManager.inHubMode());
#endif


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
#ifdef ENABLE_RADIO_RX
    // Poll I/O and process message incrementally (in this otherwise idle time)
    // before sleep and on wakeup in case some IO needs further processing now,
    // eg work was accrued during the previous major slow/outer loop
    // or the in a previous orbit of this loop sleep or nap was terminated by an I/O interrupt.
    // May generate output to host on Serial.
    // Come back and have another go immediately until no work remaining.
    if(messageQueue.handle(true, PrimaryRadio)) { continue; }
#endif

// If missing h/w interrupts for anything that needs rapid response
// then AVOID the lowest-power long sleep.
#if defined(ENABLE_CONTINUOUS_RX) && !defined(PIN_RFM_NIRQ)
    if(needsToListen)
#else
    if(false)
#endif
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
#if defined(ENABLE_WATCHDOG_SLOW)
  // Reset and immediately re-prime the RTC-based watchdog.
  OTV0P2BASE::resetRTCWatchDog();
  OTV0P2BASE::enableRTCWatchdog(true);
#endif
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


#if defined(ENABLE_FHT8VSIMPLE)
  // Try for double TX for more robust conversation with valve unless:
  //   * battery is low
  //   * the valve is not required to be wide open (ie a reasonable temperature is currently being maintained).
  //   * this is a hub and has to listen as much as possible
  // to conserve battery and bandwidth.
  #ifdef ENABLE_NOMINAL_RAD_VALVE
  const bool doubleTXForFTH8V = !conserveBattery && !hubManager.inHubMode() && (NominalRadValve.get() >= 50);
  #else
  const bool doubleTXForFTH8V = false;
  #endif
  // FHT8V is highest priority and runs first.
  // ---------- HALF SECOND #0 -----------
  bool useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8V.FHT8VPollSyncAndTX_First(doubleTXForFTH8V); // Time for extra TX before UI.
//  if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@0"); }
#endif


  // High-priority UI handing, every other/even second.
  // Show status if the user changed something significant.
  // Must take ~300ms or less so as not to run over into next half second if two TXs are done.
  bool recompute = false; // Set true if an extra recompute of target temperature should be done.
#if !defined(V0P2BASE_TWO_S_TICK_RTC_SUPPORT)
  if(0 == (TIME_LSD & 1))
#endif
    {
#if defined(ENABLE_FULL_OT_UI) && defined(valveUI_DEFINED)
    // Run the OpenTRV button/LED UI if required.
    if(0 != valveUI.read()) // if(tickUI(TIME_LSD))
      {
      showStatus = true;
      recompute = true;
      }
#endif
    }

  // Handling the UI may have taken a little while, so process I/O a little.
  messageQueue.handle(true, PrimaryRadio); // Deal with any pending I/O.


#ifdef ENABLE_MODELLED_RAD_VALVE
  if(recompute || valveUI.veryRecentUIControlUse())
    {
    // Force immediate recompute of target temperature for (UI) responsiveness.
    NominalRadValve.computeTargetTemperature();
    // Keep dynamic adjustment of sensors up to date.
    updateSensorsFromStats();
    }
#endif


#if defined(ENABLE_FHT8VSIMPLE)
  if(useExtraFHT8VTXSlots)
    {
    // Time for extra TX before other actions, but don't bother if minimising power in frost mode.
    // ---------- HALF SECOND #1 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8V.FHT8VPollSyncAndTX_Next(doubleTXForFTH8V);
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@1"); }
    // Handling the FHT8V may have taken a little while, so process I/O a little.
    messageQueue.handle(true, PrimaryRadio); // Deal with any pending I/O.
    }
#endif


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

#if defined(ENABLE_STATS_TX)
    // Periodic transmission of stats if NOT driving a local valve (else stats can be piggybacked onto that).
    // Randomised somewhat between slots and also within the slot to help avoid collisions.
    static uint8_t txTick;
    case 6: { txTick = OTV0P2BASE::randRNG8() & 7; break; } // Pick which of the 8 slots to use.
    case 8: case 10: case 12: case 14: case 16: case 18: case 20: case 22:
      {
      // Only the slot where txTick is zero is used.
      if(0 != txTick--) { break; }

#if defined(ENABLE_FHT8VSIMPLE)
      // Avoid transmit conflict with FS20; just drop the slot.
      // We should possibly choose between this and piggybacking stats to avoid busting duty-cycle rules.
      if(useExtraFHT8VTXSlots && localFHT8VTRVEnabled()) { break; }
#endif

#if !defined(ENABLE_FREQUENT_STATS_TX) // If ENABLE_FREQUENT_STATS_TX then send every minute regardless.
      // Stats TX in the minute (#1) after all sensors should have been polled
      // (so that readings are fresh) and evenly between.
      // Usually send one frame every 4 minutes, 2 if this is a valve.
      // No extra stats TX for changed data to reduce information/activity leakage.
      // Note that all O frames contain the current valve percentage,
      // which implies that any extra stats TX also speeds response to call-for-heat changes.
#ifdef ENABLE_NOMINAL_RAD_VALVE
      // DHD20170113: was once every 4 minutes, but can make boiler response too slow.
      if(0 == (minuteFrom4 & 1)) { break; }
#else
      if(!minute1From4AfterSensors) { break; }
#endif
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
#if defined(ENABLE_BINARY_STATS_TX) && defined(ENABLE_FS20_ENCODING_SUPPORT)
      const bool doBinary = !localFHT8VTRVEnabled() && OTV0P2BASE::randRNG8NextBoolean();
#else
      const bool doBinary = false;
#endif
      bareStatsTX(!batteryLow && !hubManager.inHubMode() && ss1.changedValue(), doBinary);
      break;
      }
#endif // defined(ENABLE_STATS_TX)

#if defined(ENABLE_SECURE_RADIO_BEACON)
    // Send a small secure radio beacon "I'm alive!" message regularly if configured.
    case 30:
      {
#if 1 && defined(DEBUG)
      DEBUG_SERIAL_PRINT_FLASHSTRING("Beacon TX... ");
#endif
      // Get the 'building' key for broadcast.
      uint8_t key[16];
      if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key))
        {
#if 1 && defined(DEBUG)
        DEBUG_SERIAL_PRINTLN_FLASHSTRING("!failed (no key)");
#endif
        break;
        }
      const OTRadioLink::SimpleSecureFrame32or0BodyTXBase::fixed32BTextSize12BNonce16BTagSimpleEnc_ptr_t e = OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleEnc_DEFAULT_STATELESS;
      const uint8_t txIDLen = OTRadioLink::ENC_BODY_DEFAULT_ID_BYTES;
      uint8_t buf[OTRadioLink::generateSecureBeaconMaxBufSize];
      const uint8_t bodylen = OTRadioLink::generateSecureBeaconRawForTX(buf, sizeof(buf), txIDLen, e, NULL, key);
      // ASSUME FRAMED CHANNEL 0 (but could check with config isUnframed flag).
      // When sending on a channel with framing, do not explicitly send the frame length byte.
      // DO NOT attempt to send if construction of the secure frame failed;
      // doing so may reuse IVs and destroy the cipher security.
      const bool success = (0 != bodylen) && PrimaryRadio.sendRaw(buf+1, bodylen-1);
#if 1 && defined(DEBUG)
      DEBUG_SERIAL_PRINT(success);
      DEBUG_SERIAL_PRINTLN();
#endif
      break;
      }
#endif // defined(ENABLE_SECURE_RADIO_BEACON)

// SENSOR READ AND STATS
//
// All external sensor reads should be in the second half of the minute (>32) if possible.
// This is to have them as close to stats collection at the end of the minute as possible,
// and to allow randomisation of the start-up cycle position in the first 32s to help avoid inter-unit collisions.
// Also all sources of noise, self-heating, etc, may be turned off for the 'sensor read minute'
// and thus will have diminished by this point.

#ifdef ENABLE_VOICE_SENSOR
    // Poll voice detection sensor at a fixed rate.
    case 46: { Voice.read(); break; }
#endif

#ifdef TEMP_POT_AVAILABLE
    // Sample the user-selected WARM temperature target at a fixed rate.
    // This allows the unit to stay reasonably responsive to adjusting the temperature dial.
    case 48: { TempPot.read(); break; }
#endif

    // Read all environmental inputs, late in the cycle.
#ifdef HUMIDITY_SENSOR_SUPPORT
    // Sample humidity.
    case 50: { if(runAll) { RelHumidity.read(); } break; }
#endif

#if defined(ENABLE_AMBLIGHT_SENSOR)
    // Poll ambient light level at a fixed rate.
    // This allows the unit to respond consistently to (eg) switching lights on (eg TODO-388).
    case 52:
      {
      // Force all UI lights off before sampling ambient light level.
      OTV0P2BASE::LED_HEATCALL_OFF();
#if defined(LED_UI2_EXISTS) && defined(ENABLE_UI_LED_2_IF_AVAILABLE)
      // Turn off second UI LED if available.
      OTV0P2BASE::LED_UI2_OFF();
#endif
      AmbLight.read();
      break;
      }
#endif

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

#if defined(ENABLE_OCCUPANCY_SUPPORT)
      // Update occupancy measures that partially use rolling stats.

      // Update occupancy status (fresh for target recomputation) at a fixed rate.
      Occupancy.read();
#endif // defined(ENABLE_OCCUPANCY_SUPPORT)

#ifdef ENABLE_NOMINAL_RAD_VALVE
      // Recompute target, valve position and call for heat, etc.
      // Should be called once per minute to work correctly.
      NominalRadValve.read();
#endif

#if defined(ENABLE_FHT8VSIMPLE) && defined(ENABLE_LOCAL_TRV) // Only regen when needed.
      // If there was a change in target valve position,
      // or periodically in the minute after all sensors should have been read,
      // precompute some or all of any outgoing frame/stats/etc ready for the next transmission.
      if(NominalRadValve.isValveMoved() ||
         (minute1From4AfterSensors && enableTrailingStatsPayload()))
        {
        if(localFHT8VTRVEnabled()) { FHT8V.set(NominalRadValve.get() /*, NominalRadValve.isCallingForHeat() */); }
        }

#if defined(ENABLE_BOILER_HUB)
      // Feed in the local valve position when calling for heat just as if over the air.
      // (Does not arrive with the normal FHT8V timing of 2-minute gaps so boiler may turn off out of sync.)
      if(FHT8V.isControlledValveReallyOpen()) { BoilerHub.remoteCallForHeatRX(FHT8V.nvGetHC(), FHT8V.get(), minuteCount); }
#endif // defined(ENABLE_BOILER_HUB)
#elif defined(ENABLE_NOMINAL_RAD_VALVE) && defined(ENABLE_LOCAL_TRV) // Other local valve types, simulate a remote call for heat with a fake ID.
#if defined(ENABLE_BOILER_HUB)
      // Feed in the local valve position when calling for heat just as if over the air.
      if(NominalRadValve.isControlledValveReallyOpen()) { BoilerHub.remoteCallForHeatRX(~0, NominalRadValve.get(), minuteCount); }
#endif // defined(ENABLE_BOILER_HUB)
#endif

#if 1 && defined(DEBUG) && defined(ENABLE_BOILER_HUB) && !defined(ENABLE_TRIMMED_MEMORY)
      // Track how long since remote call for heat last heard.
      if(BoilerHub.isBoilerOn())
        {
        DEBUG_SERIAL_PRINT_FLASHSTRING("Boiler on, s: ");
        DEBUG_SERIAL_PRINT(boilerCountdownTicks * OTV0P2BASE::MAIN_TICK_S);
        DEBUG_SERIAL_PRINTLN();
        }
#endif

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

#if defined(ENABLE_FHT8VSIMPLE) && defined(V0P2BASE_TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // ---------- HALF SECOND #2 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8V.FHT8VPollSyncAndTX_Next(doubleTXForFTH8V);
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@2"); }
    // Handling the FHT8V may have taken a little while, so process I/O a little.
    messageQueue.handle(true, PrimaryRadio); // Deal with any pending I/O.
    }
#endif

  // Generate periodic status reports.
  if(showStatus) { serialStatusReport(); }

#if defined(ENABLE_FHT8VSIMPLE) && defined(V0P2BASE_TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // ---------- HALF SECOND #3 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8V.FHT8VPollSyncAndTX_Next(doubleTXForFTH8V);
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@3"); }
    // Handling the FHT8V may have taken a little while, so process I/O a little.
    messageQueue.handle(true, PrimaryRadio); // Deal with any pending I/O.
    }
#endif

  // End-of-loop processing, that may be slow.
  // Ensure progress on queued messages ahead of slow work.  (TODO-867)
  messageQueue.handle(true, PrimaryRadio); // Deal with any pending I/O.

#if defined(HAS_DORM1_VALVE_DRIVE) && defined(ENABLE_LOCAL_TRV)
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
#endif

  // Command-Line Interface (CLI) polling.
  // If a reasonable chunk of the minor cycle remains after all other work is done
  // AND the CLI is / should be active OR a status line has just been output
  // then poll/prompt the user for input
  // using a timeout which should safely avoid overrun, ie missing the next basic tick,
  // and which should also allow some energy-saving sleep.
#if 1 && defined(ENABLE_CLI)
  if(OTV0P2BASE::CLI::isCLIActive())
    {
    const uint8_t stopBy = nearOverrunThreshold - 1;
    char buf[BUFSIZ_pollUI];
    OTV0P2BASE::ScratchSpace s((uint8_t*)buf, sizeof(buf));
    pollCLI(stopBy, 0 == TIME_LSD, s);
    }
#endif


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

// Do explicit overrun detection iff RTC watchdog not enabled (should reset instead).
#if !defined(ENABLE_WATCHDOG_SLOW) // || !defined(ENABLE_TRIMMED_MEMORY) // Could reinstate if not short memory...
  // Detect and handle (actual or near) overrun, if it happens, though it should not.
  if(TIME_LSD != OTV0P2BASE::getSecondsLT())
    {
    // Increment the overrun counter (stored inverted, so 0xff initialised => 0 overruns).
    const uint8_t orc = 1 + ~eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_OVERRUN_COUNTER);
    OTV0P2BASE::eeprom_smart_update_byte((uint8_t *)V0P2BASE_EE_START_OVERRUN_COUNTER, ~orc);
#if 1 && defined(DEBUG)
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("!loop overrun");
#endif
#if defined(ENABLE_FHT8VSIMPLE)
    FHT8V.resyncWithValve(); // Assume that sync with valve may have been lost, so re-sync.
#endif
    TIME_LSD = OTV0P2BASE::getSecondsLT(); // Prepare to sleep until start of next full minor cycle.
    }
#if 0 && defined(DEBUG) // Expect to pick up near overrun at start of next loop.
  else if(getSubCycleTime() >= nearOverrunThreshold)
    {
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("?O"); // Near overrun.  Note 2ms/char to send...
    }
#endif
#endif // !defined(ENABLE_WATCHDOG_SLOW)
  }
