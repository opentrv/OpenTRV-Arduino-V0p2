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
                           Deniz Erbilgin 2015--2017
*/

/******************************************************************************
 * NOTES
 *****************************************************************************/
/*
  V0p2 (V0.2) core.

  DHD20130417: hardware setup on bare board.
    * 1MHz CPU clock (from 8MHz internal RC clock with /8 prescaler) ATmega328P running at 1.8V--5V (typically 2V--3.3V).
    * Fuse set for BOD-managed additional clock settle time, ie as fast a restart from sleep as possible.
    * All unused pins unconnected and nominally floating (though driven low as output where possible).
    * 32768Hz xtal between pins XTAL1 and XTAL2, async timer 2, for accurate timekeeping and low-power sleep.
    * All unused system modules turned off.

  Basic AVR power consumption ticking an (empty) control loop at ~0.5Hz should be ~1uA.
 */


/******************************************************************************
 * INCLUDES
 *****************************************************************************/
////// GLOBAL flags that alter system build and behaviour.

//// If defined, do extra checks and serial logging.  Will take more code space and power.
//#undef DEBUG

// *** Global flag for REVx configuration here *** //
#define V0p2_REV 7

// --------------------------------------------
// I/O pin allocation and setup: include ahead of I/O module headers.
#include <OTV0p2_Board_IO_Config.h>


////// MAIN LIBRARIES

#include <Arduino.h>
#include <OTV0p2Base.h>
#include <OTRadioLink.h>


////// ADDITIONAL LIBRARIES/PERIPHERALS

#include <OTRadValve.h>
#include <OTRFM23BLink.h>
#include <OTAESGCM.h>


////// ADDITIONAL USER HEADERS


/******************************************************************************
 * GENERAL CONSTANTS AND VARIABLES
 * Core constants and variables required for V0p2 loop.
 *****************************************************************************/
// Controller's notion/cache of seconds within major cycle.
static uint_fast8_t TIME_LSD;

// 'Elapsed minutes' count of minute/major cycles; cheaper than accessing RTC and not tied to real time.
// Starts at or just above zero (within the first 4-minute cycle) to help avoid collisions between units after mass power-up.
// Wraps at its maximum (0xff) value.
static uint8_t minuteCount;

// Controller's view of Least Significant Digits of the current (local) time, in this case whole seconds.
// TIME_LSD ranges from 0 to TIME_CYCLE_S-1, also major cycle length.
constexpr uint_fast8_t TIME_CYCLE_S = 60;

// Period in minutes for simple learned on-time; strictly positive (and less than 256).
static constexpr uint8_t LEARNED_ON_PERIOD_M = 60;
// Period in minutes for simple learned on-time with comfort bias; strictly positive (and less than 256).
// Defaults to twice LEARNED_ON_PERIOD_M.
// Should be no shorter/less than LEARNED_ON_PERIOD_M to avoid confusion.
static constexpr uint8_t LEARNED_ON_PERIOD_COMFORT_M = (OTV0P2BASE::fnmin(2*(LEARNED_ON_PERIOD_M),255));

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
static constexpr uint8_t RFM22_PREAMBLE_BYTES = 5; // Recommended number of preamble bytes for reliable reception.
static constexpr uint8_t RFM22_SYNC_MIN_BYTES = 3; // Minimum number of sync bytes.
static constexpr uint8_t STATS_MSG_START_OFFSET = (RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES);
static constexpr uint8_t STATS_MSG_MAX_LEN = (64 - STATS_MSG_START_OFFSET);

// Mask for Port D input change interrupts.
constexpr uint8_t SERIALRX_INT_MASK = 0b00000001; // Serial RX
#if BUTTON_MODE_L > 7
  #error BUTTON_MODE_L expected to be on port D
#endif
constexpr uint8_t MODE_INT_MASK = (1 << (BUTTON_MODE_L&7));
constexpr uint8_t MASK_PD = (SERIALRX_INT_MASK | MODE_INT_MASK); // MODE button interrupt (et al).

/******************************************************************************
 * PERIPHERALS
 * Peripheral drivers and config
 *****************************************************************************/

////// FORWARD DECLARATIONS

// Valve physical UI controller.
typedef OTRadValve::ModeButtonAndPotActuatorPhysicalUI valveUI_t;
extern valveUI_t valveUI;  // Extern as otherwise compiler attempts to instantiate.


////// RADIOS

// Config
// Nodes talking on fast GFSK channel 0.
static constexpr uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] = {
    // GFSK channel 0 full config, RX/TX, not in itself secure.
    OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true), };

// Brings in necessary radio libs.
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTV0P2BASE::fnmax(uint8_t(2), uint8_t(OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY)) - 1;
static constexpr int8_t RFM23B_IRQ_PIN = PIN_RFM_NIRQ;
static constexpr bool RFM23B_allowRX = false;
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> PrimaryRadio;

////// SENSORS

// Sensor for supply (eg battery) voltage in millivolts.
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;

// Support for general timed and multi-input occupancy detection / use.
typedef OTV0P2BASE::PseudoSensorOccupancyTracker OccupancyTracker;
// Singleton implementation for entire node.
OccupancyTracker Occupancy;

// Sensor for temperature potentiometer/dial UI control.
typedef OTV0P2BASE::SensorTemperaturePot
    <
    decltype(Occupancy), &Occupancy,
    48, 296, // Correct for DORM1/TRV1 with embedded REV7.
    false // REV7 does not drive pot from IO_POWER_UP.
    > TempPot_t;
TempPot_t TempPot;

// Sensor for ambient light level; 0 is dark, 255 is bright.
typedef OTV0P2BASE::SensorAmbientLight AmbientLight;
AmbientLight AmbLight;

// Singleton implementation/instance.
typedef OTV0P2BASE::HumiditySensorSHT21 RelHumidity_t;
// Singleton implementation/instance.
OTV0P2BASE::HumiditySensorSHT21 RelHumidity;

// Ambient/room temperature sensor, usually on main board.
OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.


////// ACTUATORS

// DORM1/REV7 direct drive motor actuator.
static constexpr bool binaryOnlyValveControl = false;
static constexpr uint8_t m1 = MOTOR_DRIVE_ML;
static constexpr uint8_t m2 = MOTOR_DRIVE_MR;
typedef OTRadValve::ValveMotorDirectV1<OTRadValve::ValveMotorDirectV1HardwareDriver, m1, m2, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN, OTRadValve::MOTOR_DRIVE_NSLEEP_UNUSED, decltype(Supply_cV), &Supply_cV> ValveDirect_t;
// DORM1/REV7 direct drive actuator.
// Singleton implementation/instance.
// Suppress unnecessary activity when room dark, eg to avoid disturbance if device crashes/restarts,
// unless recent UI use because value is being fitted/adjusted.
ValveDirect_t ValveDirect([](){return((!valveUI.veryRecentUIControlUse()) && AmbLight.isRoomDark());});


////// CONTROL

// Singleton non-volatile stats store instance.
OTV0P2BASE::EEPROMByHourByteStats eeStats;

// Singleton stats-updater object.
typedef
    OTV0P2BASE::ByHourSimpleStatsUpdaterSampleStats <
      decltype(eeStats), &eeStats,
      decltype(Occupancy), &Occupancy,
      decltype(AmbLight), &AmbLight,
      decltype(TemperatureC16), &TemperatureC16,
      decltype(RelHumidity), &RelHumidity,
      2
      > StatsU_t;
// Stats updater singleton.
StatsU_t statsU;

// Singleton scheduler instance.
// Dummy scheduler to simplify coding.
typedef OTRadValve::NULLValveSchedule Scheduler_t;
Scheduler_t Scheduler;

// Radiator valve mode (FROST, WARM, BAKE).
OTRadValve::ValveMode valveMode;

// Settings for room TRV.
typedef OTRadValve::DEFAULT_ValveControlParameters PARAMS;
// Choose which subtype to use depending on enabled settings and board type.
typedef OTRadValve::TempControlTempPot<decltype(TempPot), &TempPot, PARAMS, decltype(RelHumidity), &RelHumidity> TempControl_t;
// Temperature control object.
TempControl_t tempControl;

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
  setbackLockout>
  cttBasic;

// Internal model of controlled radiator valve position.
OTRadValve::ModelledRadValve NominalRadValve(
  &cttBasic,
  &valveMode,
  &tempControl,
  &ValveDirect,
    false,
    100);

// Valve physical UI controller.
valveUI_t valveUI(
  &valveMode,
  &tempControl,
  &NominalRadValve,
  &Occupancy,
  &AmbLight,
  &TempPot,
  &Supply_cV,
  OTV0P2BASE::LED_HEATCALL_ON,
  OTV0P2BASE::LED_HEATCALL_OFF,
  OTV0P2BASE::LED_HEATCALL_ON_ISR_SAFE);

////// MESSAGING

// Managed JSON stats.
OTV0P2BASE::SimpleStatsRotation<12> ss1; // Configured for maximum different stats.  // FIXME REDUCE AS MUCH AS POSSIBLE


/******************************************************************************
 * INTERRUPT SERVICE ROUTINES
 *****************************************************************************/

// Previous state of port D pins to help detect changes.
static volatile uint8_t prevStatePD;
// Interrupt service routine for PD I/O port transition changes (including RX).
ISR(PCINT2_vect)
{
    const uint8_t pins = PIND;
    const uint8_t changes = pins ^ prevStatePD;
    prevStatePD = pins;
    // Mode button detection is on the falling edge (button pressed).
    if((changes & MODE_INT_MASK) && !(pins & MODE_INT_MASK)) { valveUI.startBakeFromInt(); }
    // If an interrupt arrived from the serial RX then wake up the CLI.
    // Use a nominally rising edge to avoid spurious trigger when other interrupts are handled.
    // The will ensure that it is possible to wake the CLI subsystem with an extra CR or LF.
    // It is OK to trigger this from other things such as button presses.
    // TODO: ensure that resetCLIActiveTimer() is inlineable to minimise ISR prologue/epilogue time and space.
    if((changes & SERIALRX_INT_MASK) && !(pins & SERIALRX_INT_MASK)) { OTV0P2BASE::CLI::resetCLIActiveTimer(); }
}


/******************************************************************************
 * LOCAL FUNCTIONS
 *****************************************************************************/

////// PANIC

// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic() {
    // Reset radio and go into low-power mode.
    PrimaryRadio.panicShutdown();
    // Power down almost everything else...
    OTV0P2BASE::minimisePowerWithoutSleep();
    pinMode(OTV0P2BASE::LED_HEATCALL_L, OUTPUT);
    for( ; ; ) {
        OTV0P2BASE::LED_HEATCALL_ON();
        OTV0P2BASE::nap(WDTO_15MS);
        OTV0P2BASE::LED_HEATCALL_OFF();
        OTV0P2BASE::nap(WDTO_120MS);
    }
}

// Panic with fixed message.
void panic(const __FlashStringHelper *s) {
    OTV0P2BASE::serialPrintlnAndFlush(); // Start new line to highlight error.  // May fail.
    OTV0P2BASE::serialPrintAndFlush('!'); // Indicate error with leading '!' // May fail.
    OTV0P2BASE::serialPrintlnAndFlush(s); // Print supplied detail text. // May fail.
    panic();
}


////// DIAGNOSTICS

/**
 * @brief   Force restart if SPAM/heap/stack likely corrupt.
 *          Complain and keep complaining when getting near stack overflow.
 *          TODO: make DEBUG-only when confident all configs OK.
 *          Optionally reports max stack usage and location, per loop.
 */
inline void stackCheck()
{
    const int16_t minsp = OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd();
#if 1 //&& defined(DEBUG)
    const uint8_t location = OTV0P2BASE::MemoryChecks::getLocation();
    const uint16_t progCounter = OTV0P2BASE::MemoryChecks::getPC();  // not isr safe
    OTV0P2BASE::serialPrintAndFlush(F("minsp: "));
    OTV0P2BASE::serialPrintAndFlush(minsp);
    OTV0P2BASE::serialPrintAndFlush(F(" loc: "));
    OTV0P2BASE::serialPrintAndFlush(location);
    OTV0P2BASE::serialPrintAndFlush(F(" prog: "));
    OTV0P2BASE::serialPrintAndFlush(progCounter, HEX);
    OTV0P2BASE::serialPrintlnAndFlush();
//    OTV0P2BASE::MemoryChecks::forceResetIfStackOverflow();  // XXX
    OTV0P2BASE::MemoryChecks::resetMinSP();
#else
    if(128 > minsp) { OTV0P2BASE::serialPrintlnAndFlush(F("!SP")); }
    OTV0P2BASE::MemoryChecks::forceResetIfStackOverflow();
#endif
}

////// SENSORS

// Update sensors with historic/trailing statistics information where needed.
// Should be called at least hourly after all stats have been updated,
// but can also be called whenever the user adjusts settings for example.
void updateSensorsFromStats() {
    // Update with rolling stats to adapt to sensors and local environment...
    // ...and prevailing bias, so may take a while to adjust.
    AmbLight.setTypMinMax(
          eeStats.getByHourStatRTC(OTV0P2BASE::NVByHourByteStatsBase::STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED),
          eeStats.getMinByHourStat(OTV0P2BASE::NVByHourByteStatsBase::STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED),
          eeStats.getMaxByHourStat(OTV0P2BASE::NVByHourByteStatsBase::STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED),
          !tempControl.hasEcoBias());
}


////// CONTROL

// Call this to do an I/O poll if needed; returns true if something useful definitely happened.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// Should also do nothing that interacts with Serial.
// Limits actual poll rate to something like once every 8ms, unless force is true.
//   * force if true then force full poll on every call (ie do not internally rate-limit)
// Note that radio poll() can be for TX as well as RX activity.
// Not thread-safe, eg not to be called from within an ISR.
// FIXME trying to move into utils (for the time being.)
bool pollIO(const bool force = false) {
    static volatile uint8_t _pO_lastPoll;
    // Poll RX at most about every ~8ms.
    const uint8_t sct = OTV0P2BASE::getSubCycleTime();
    if(force || (sct != _pO_lastPoll)) {
        _pO_lastPoll = sct;
        // Poll for inbound frames.
        // If RX is not interrupt-driven then
        // there will usually be little time to do this
        // before getting an RX overrun or dropped frame.
        PrimaryRadio.poll();
    }
    return(false);
}

// Run tasks needed at the end of each day (nominal midnight).
// Should be run once at a fixed slot in the last minute of the last hour of each day.
// Will be run after all stats for the current hour have been updated.
static void endOfDayTasks() {
    // Count down the setback lockout if not finished...  (TODO-786, TODO-906)
//    OTRadValve::countDownSetbackLockout();
}


////// MESSAGING

// Do bare stats transmission.
// Output should be filtered for items appropriate
// to current channel security and sensitivity level.
// This may be binary or JSON format.
//   * allowDoubleTX  allow double TX to increase chance of successful reception
// Sends stats on primary radio channel 0 with possible duplicate to secondary channel.
// If sending encrypted then ID/counter fields (eg @ and + for JSON) are omitted
// as assumed supplied by security layer to remote recipent.
void bareStatsTX() {

    OTV0P2BASE::MemoryChecks::resetMinSP();
//    // Capture heavy stack usage from local allocations here.
//    OTV0P2BASE::MemoryChecks::recordIfMinSP();
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
    // Send binary *or* JSON on each attempt so as not to overwhelm the receiver.
    // Send JSON message.
    bool sendingJSONFailed = false; // Set true and stop attempting JSON send in case of error.
    // Set pointer location based on whether start of message will have preamble TODO move to OTRFM23BLink queueToSend?
    uint8_t *bptr = buf;
    // Leave space for possible leading frame-length byte, eg for encrypted frame.
    ++bptr;
    // Where to write the real frame content.
    uint8_t *const realTXFrameStart = bptr;
    // If forcing encryption or if unconditionally suppressed
    // then suppress the "@" ID field entirely,
    // assuming that the encrypted commands will carry the ID, ie in the 'envelope'.
    ss1.setID(V0p2_SENSOR_TAG_F(""));
    // Managed JSON stats.
    // Enable "+" count field for diagnostic purposes, eg while TX is lossy,
    // if the primary radio channel does not include a sequence number itself.
    // Assume that an encrypted channel will provide its own (visible) sequence counter.
    ss1.enableCount(false);  // Always encrypted
    ss1.put(TemperatureC16);
    ss1.put(RelHumidity);
    ss1.put(Occupancy.twoBitTag(), Occupancy.twoBitOccupancyValue()); // Reduce spurious TX cf percentage.
    ss1.put(Occupancy.vacHSubSensor);
    // OPTIONAL items
    // Only TX supply voltage for units apparently not mains powered, and TX with low priority as slow changing.
    ss1.put(Supply_cV, true);
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
    // Redirect JSON output appropriately.
    uint8_t *const bufJSON = ptextBuf + 2;
    const uint8_t bufJSONlen = bufEncJSONlen;
    // Number of bytes written for body.
    // For non-secure, this is the size of the JSON text.
    // For secure this is overridden with the secure frame size.
    int8_t wrote = 0;
    // Generate JSON text.
    if(!sendingJSONFailed) {
        // Generate JSON and write to appropriate buffer:
        // direct to TX buffer if not encrypting, else to separate buffer.
        wrote = ss1.writeJSON(bufJSON, bufJSONlen, privacyLevel, true); // make best use of JSON stats;
        if(0 == wrote) { sendingJSONFailed = true; }
    }
    // Push the JSON output to Serial.
    if(!sendingJSONFailed) {
        // Insert synthetic full ID/@ field for local stats, but no sequence number for now.
        Serial.print(F("{\"@\":\""));
        for(int i = 0; i < OTV0P2BASE::OpenTRV_Node_ID_Bytes; ++i) { Serial.print(eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_ID+i), HEX); }
        Serial.print(F("\","));
        Serial.write(bufJSON+1, wrote-1);
        Serial.println();
        OTV0P2BASE::flushSerialSCTSensitive(); // Ensure all flushed since system clock may be messed with...
    }
    // Get the 'building' key for stats sending.
    uint8_t key[16];
    if(!sendingJSONFailed) {
        if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key)) {
            sendingJSONFailed = true;
            OTV0P2BASE::serialPrintlnAndFlush(F("!TX key")); // Know why TX failed.
        }
    }
    // If doing encryption
    // then build encrypted frame from raw JSON.
    if(!sendingJSONFailed) {
        // TODO Fold JSON
        // Explicit-workspace version of encryption.
        const OTRadioLink::SimpleSecureFrame32or0BodyTXBase::fixed32BTextSize12BNonce16BTagSimpleEncWithLWorkspace_ptr_t eW = OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleEnc_DEFAULT_WITH_LWORKSPACE;
        // Create subscratch space for encryption functions
        OTV0P2BASE::ScratchSpaceL subScratch(sW, scratchSpaceNeeded);
        constexpr uint8_t txIDLen = OTRadioLink::ENC_BODY_DEFAULT_ID_BYTES;
        // When sending on a channel with framing, do not explicitly send the frame length byte.
        constexpr uint8_t offset = 1;
        // Assumed to be at least one free writeable byte ahead of bptr.
        // Get current modelled valve position.
        const uint8_t valvePC = NominalRadValve.get();
        const uint8_t bodylen = OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::getInstance().generateSecureOFrameRawForTX(
            (realTXFrameStart - offset), (MSG_BUF_SIZE - (realTXFrameStart-buf) + offset),
            txIDLen, valvePC, ptextBuf, eW, subScratch, key);
        sendingJSONFailed = (0 == bodylen);
        wrote = bodylen - offset;
    }
    // Send directly to the primary radio...
    if((!sendingJSONFailed) && (!PrimaryRadio.queueToSend(realTXFrameStart, wrote))) { sendingJSONFailed = true; }
    if(neededWaking) { OTV0P2BASE::flushSerialProductive(); OTV0P2BASE::powerDownSerial(); }
    stackCheck();
}


////// UI

// Dump some brief CLI usage instructions to serial TX, which must be up and running.
// If this gets too big there is a risk of overrunning and missing the next tick...
static void dumpCLIUsage()
{
    OTV0P2BASE::CLI::InvalidIgnored(); // Minimal placeholder.
    Serial.println();
}

// Suggested minimum buffer size for pollUI() to ensure maximum-sized commands can be received.
static constexpr uint8_t MAXIMUM_CLI_RESPONSE_CHARS = 1 + OTV0P2BASE::CLI::MAX_TYPICAL_CLI_BUFFER;
static constexpr uint8_t BUFSIZ_pollUI = 1 + MAXIMUM_CLI_RESPONSE_CHARS;
// Used to poll user side for CLI input until specified sub-cycle time.
// Commands should be sent terminated by CR *or* LF; both may prevent 'E' (exit) from working properly.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// Times itself out after at least a minute or two of inactivity.
// NOT RENTRANT (eg uses static state for speed and code space).

void pollCLI(const uint8_t maxSCT, const bool startOfMinute, const OTV0P2BASE::ScratchSpace &s)
{
    // Perform any once-per-minute operations.
    if(startOfMinute) { OTV0P2BASE::CLI::countDownCLI(); }
    const bool neededWaking = OTV0P2BASE::powerUpSerialIfDisabled<V0P2_UART_BAUD>();
    // Wait for input command line from the user (received characters may already have been queued)...
    // Read a line up to a terminating CR, either on its own or as part of CRLF.
    // (Note that command content and timing may be useful to fold into PRNG entropy pool.)
    // A static buffer generates better code but permanently consumes previous SRAM.
    const uint8_t n = OTV0P2BASE::CLI::promptAndReadCommandLine(maxSCT, s, [](){pollIO();});
    char *buf = (char *)s.buf;
    if(n > 0) {
        // Got plausible input so keep the CLI awake a little longer.
        OTV0P2BASE::CLI::resetCLIActiveTimer();
        // Process the input received, with action based on the first char...
        switch(buf[0]) {
//            // Explicit request for help, or unrecognised first character.
//            // Avoid showing status as may already be rather a lot of output.
//            default: case '?': { dumpCLIUsage(); break; }
//            // Exit/deactivate CLI immediately.
//            // This should be followed by JUST CR ('\r') OR LF ('\n')
//            // else the second will wake the CLI up again.
//            case 'E': { OTV0P2BASE::CLI::makeCLIInactive(); break; }
//            // Show/set generic parameter values (eg "G N [M]").
//            case 'G': { OTV0P2BASE::CLI::GenericParam().doCommand(buf, n); break; }
//            // Reset or display ID.
//            case 'I': { OTV0P2BASE::CLI::NodeIDWithSet().doCommand(buf, n); break; } // XXX
//            // Status line stats print and TX.
//            case 'S': {
//                Serial.print(F("Resets: "));
//                const uint8_t resetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
//                Serial.print(resetCount);
//                Serial.println();
//                // Show stack headroom.
//                OTV0P2BASE::serialPrintAndFlush(F("SH ")); OTV0P2BASE::serialPrintAndFlush(OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd()); OTV0P2BASE::serialPrintlnAndFlush();
//                // Default light-weight print and TX of stats.
////                bareStatsTX();  // FIXME No way this won't overflow!
//                break; // Note that status is by default printed after processing input line.
//            }
//            // Switch to FROST mode OR set FROST/setback temperature (even with temp pot available).
//            // With F! force to frost and holiday (long-vacant) mode.  Useful for testing and for remote CLI use.
//            case 'F': {
//                valveMode.setWarmModeDebounced(false); // No parameter supplied; switch to FROST mode.
//                break;
//            }
//            /**
//            * Set secret key.
//            * @note  The OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond
//            *        function pointer MUST be passed here to ensure safe handling of the key and the Tx message
//            *        counter.
//            */
//            case 'K': { OTV0P2BASE::CLI::SetSecretKey(OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond).doCommand(buf, n); break; }  // XXX
//            // Switch to WARM (not BAKE) mode OR set WARM temperature.
//            case 'W':{
//                valveMode.cancelBakeDebounced(); // Ensure BAKE mode not entered.
//                valveMode.setWarmModeDebounced(true); // No parameter supplied; switch to WARM mode.
//                break;
//            }
//            // Zap/erase learned statistics.
//            case 'Z': { OTV0P2BASE::CLI::ZapStats().doCommand(buf, n); break; } // XXX
        }
        // Else show ack of command received.
        Serial.println(F("OK"));
    } else { Serial.println(); } // Terminate empty/partial CLI input line after timeout.
    // Force any pending output before return / possible UART power-down.
    OTV0P2BASE::flushSerialSCTSensitive();
    if(neededWaking) { OTV0P2BASE::powerDownSerial(); }
}


/******************************************************************************
 * SETUP
 *****************************************************************************/

// Setup routine: runs once after reset.
// Does some limited board self-test and will panic() if anything is obviously broken.
void setup()
{
    // Set appropriate low-power states, interrupts, etc, ASAP.
    OTV0P2BASE::powerSetup();
    // IO setup for safety, and to avoid pins floating.
    OTV0P2BASE::IOSetup();
    // Restore previous RTC state if available.
    OTV0P2BASE::restoreRTC();
    OTV0P2BASE::serialPrintAndFlush(F("\r\nOpenTRV: ")); // Leading CRLF to clear leading junk, eg from bootloader.
    V0p2Base_serialPrintlnBuildVersion();
    // Count resets to detect unexpected crashes/restarts.
    const uint8_t oldResetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
    eeprom_write_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT, 1 + oldResetCount);

    // Have 32678Hz clock at least running before going any further.
    // Check that the slow clock is running reasonably OK, and tune the fast one to it.
    if(!::OTV0P2BASE::HWTEST::calibrateInternalOscWithExtOsc()) { panic(F("Xtal")); } // Async clock not running or can't tune.

    // Signal that xtal is running AND give it time to settle.
    OTV0P2BASE::sleepLowPowerMs(1000);
    OTV0P2BASE::LED_HEATCALL_OFF();
    OTV0P2BASE::sleepLowPowerMs(250); // TODO: use this time to gather entropy.
    OTV0P2BASE::LED_HEATCALL_ON();
    OTV0P2BASE::sleepLowPowerMs(1000); // TODO: use this time to gather entropy.

    // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
    PrimaryRadio.preinit(NULL);
    // Check that the radio is correctly connected; panic if not...
    if(!PrimaryRadio.configure(nPrimaryRadioChannels, RFM23BConfigs) || !PrimaryRadio.begin()) { panic(F("r1")); }

    // Buttons should not be activated DURING boot for user-facing boards; an activated button implies a fault.
    // Check buttons not stuck in the activated position.
    if(fastDigitalRead(BUTTON_MODE_L) == LOW) { panic(F("b")); }

    // Collect full set of environmental values before entering loop() in normal mode.
    // This should also help ensure that sensors are properly initialised.
    TemperatureC16.read();
    AmbLight.read();
    RelHumidity.read();
    TempPot.read();
    Supply_cV.read();
    OTV0P2BASE::seedPRNGs();
    // Update targets, output to TRV and boiler, etc, to be sensible before main loop starts.
    NominalRadValve.read();
    // Ensure that the unique node ID is set up (mainly on first use).
    // Have one attempt (don't want to stress an already failing EEPROM) to force-reset if not good, then panic.
    // Needs to have had entropy gathered, etc.
    if(!OTV0P2BASE::ensureIDCreated()) {
        if(!OTV0P2BASE::ensureIDCreated(true)) { panic(F("ID")); } // Force reset.
    }
    // Initialised: turn main/heatcall UI LED off.
    OTV0P2BASE::LED_HEATCALL_OFF();
    // Do OpenTRV-specific (late) setup.
    // Radio not listening to start with.
    // Ignore any initial spurious RX interrupts for example.
    PrimaryRadio.listen(false);
    // Set up async edge interrupts.
    ATOMIC_BLOCK (ATOMIC_RESTORESTATE) {
        //PCMSK0 = PB; PCINT  0--7    (LEARN1 and Radio)
        //PCMSK1 = PC; PCINT  8--15
        //PCMSK2 = PD; PCINT 16--24   (Serial RX and LEARN2 and MODE and Voice)
        PCICR = 4 | 0; // 0x4 enables PD/PCMSK2.
        PCMSK2 = MASK_PD;
    }

    // Wire components together, eg. for occupancy sensing.
    AmbLight.setOccCallbackOpt([](bool prob){if(prob){Occupancy.markAsPossiblyOccupied();}else{Occupancy.markAsJustPossiblyOccupied();}});
    // Callbacks to set various mode combinations.
    // Typically at most one call would be made on any appropriate pot adjustment.
    TempPot.setWFBCallbacks([](bool x){valveUI.setWarmModeFromManualUI(x);},
                            [](bool x){valveUI.setBakeModeFromManualUI(x);});
    // Initialise sensors with stats info where needed.
    updateSensorsFromStats();

    OTV0P2BASE::MemoryChecks::resetMinSP();
    OTV0P2BASE::MemoryChecks::recordIfMinSP(1);
    stackCheck();

    // Do early 'wake-up' stats transmission if possible // XXX
    // when everything else is set up and ready and allowed (TODO-636)
    // including all set-up and inter-wiring of sensors/actuators.
    // Attempt to maximise chance of reception with a double TX.
    // Assume not in hub mode (yet).
    // Send all possible formats, binary first (assumed complete in one message).
    bareStatsTX();  // XXX
    // Send JSON stats repeatedly (typically once or twice)
    // until all values pushed out (no 'changed' values unsent)
    // or limit reached.
    for(uint8_t i = 5; --i > 0; ) {
        ::OTV0P2BASE::nap(WDTO_120MS, false); // Sleep long enough for receiver to have a chance to process previous TX.
        bareStatsTX();  // XXX
        if(!ss1.changedValue()) { break; }
    }
    // Start local counters in randomised positions to help avoid inter-unit collisions,
    // eg for mains-powered units starting up together after a power cut,
    // but without (eg) breaking any of the logic about what order things will be run first time through.
    // Uses some decent noise to try to start the units separated.
    const uint8_t b = OTV0P2BASE::getSecureRandomByte(); // randRNG8();
    // Start within bottom half of minute (or close to); sensor readings happen in second half.
    OTV0P2BASE::setSeconds(b >> 2);
    // Start anywhere in first 4 minute cycle.
    minuteCount = b & 3;
    // Set appropriate loop() values just before entering it.
    TIME_LSD = OTV0P2BASE::getSecondsLT();
}


/******************************************************************************
 * MAIN LOOP
 *****************************************************************************/

void loop()
{
#if defined(EST_CPU_DUTYCYCLE)
    const unsigned long usStart = micros();
#endif
    OTV0P2BASE::MemoryChecks::resetMinSP();
    OTV0P2BASE::MemoryChecks::recordIfMinSP(2);
    stackCheck();

    // Main loop for OpenTRV radiator control.
    // Note: exiting and re-entering can take a little while, handling Arduino background tasks such as serial.
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
    // Run at full speed until not nominally demanding heat, eg even during FROST mode or pre-heating.
    const bool conserveBattery = (batteryLow || !valveMode.inWarmMode() || Occupancy.longVacant()) && (!NominalRadValve.isCallingForHeat());
    // Try if very near to end of cycle and thus causing an overrun.
    // Conversely, if not true, should have time to safely log outputs, etc.
    const uint8_t nearOverrunThreshold = OTV0P2BASE::GSCT_MAX - 8; // ~64ms/~32 serial TX chars of grace time...
    // Sleep in low-power mode (waiting for interrupts) until seconds roll.
    // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
    // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
    // Ensure that serial I/O is off while sleeping.
    OTV0P2BASE::powerDownSerial();
    // Power down most stuff (except radio for hub RX).
    OTV0P2BASE::minimisePowerWithoutSleep();
    uint_fast8_t newTLSD;
    while(TIME_LSD == (newTLSD = OTV0P2BASE::getSecondsLT())) {
        // If missing h/w interrupts for anything that needs rapid response
        // then AVOID the lowest-power long sleep.
        if(false) {
            // If there is not hardware interrupt wakeup on receipt of a frame,
            // then this can only sleep for a short time between explicit poll()s,
            // though in any case allow wake on interrupt to minimise loop timing jitter
            // when the slow RTC 'end of sleep' tick arrives.
            OTV0P2BASE::nap(WDTO_15MS, true);
        } else {
            // Normal long minimal-power sleep until wake-up interrupt.
            // Rely on interrupt to force quick loop round to I/O poll.
            OTV0P2BASE::sleepUntilInt();
        }
    }
    TIME_LSD = newTLSD;
    // Reset and immediately re-prime the RTC-based watchdog.
    OTV0P2BASE::resetRTCWatchDog();
    OTV0P2BASE::enableRTCWatchdog(false);

    // START LOOP BODY
    // ===============
    // High-priority UI handing, every other/even second.
    // Show status if the user changed something significant.
    // Must take ~300ms or less so as not to run over into next half second if two TXs are done.
    bool recompute = false; // Set true if an extra recompute of target temperature should be done.
    // Run the OpenTRV button/LED UI if required.
    if(0 != valveUI.read()) { recompute = true; }
    // Handling the UI may have taken a little while, so process I/O a little.
    if(recompute || valveUI.veryRecentUIControlUse()) {
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
    switch(TIME_LSD) { // With V0P2BASE_TWO_S_TICK_RTC_SUPPORT only even seconds are available.
        case 0: {
            // Tasks that must be run every minute.
            ++minuteCount; // Note simple roll-over to 0 at max value.
            // Force to user's programmed schedule(s), if any, at the correct time.
//            Scheduler.applyUserSchedule(&valveMode, OTV0P2BASE::getMinutesSinceMidnightLT());  // XXX
            // Ensure that the RTC has been persisted promptly when necessary.
            OTV0P2BASE::persistRTC();
            // Run hourly tasks at the end of the hour.
            if((59 == OTV0P2BASE::getMinutesLT() ) && (23 == OTV0P2BASE::getHoursLT() )) { endOfDayTasks(); }
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
        case 8: case 10: case 12: case 14: case 16: case 18: case 20: case 22: {
            // Only the slot where txTick is zero is used.
            if(0 != txTick--) { break; }
            // Stats TX in the minute (#1) after all sensors should have been polled
            // (so that readings are fresh) and evenly between.
            // Usually send one frame every 4 minutes, 2 if this is a valve.
            // No extra stats TX for changed data to reduce information/activity leakage.
            // Note that all O frames contain the current valve percentage,
            // which implies that any extra stats TX also speeds response to call-for-heat changes.
            // DHD20170113: was once every 4 minutes, but can make boiler response too slow.
            if(0 == (minuteFrom4 & 1)) { break; }
            // Sleep randomly up to ~25% of the minor cycle
            // to spread transmissions and thus help avoid collisions.
            // (Longer than 25%/0.5s could interfere with other ops such as FHT8V TXes.)
            const uint8_t stopBy = 1 + (((OTV0P2BASE::GSCT_MAX >> 2) | 7) & OTV0P2BASE::randRNG8());
            while(OTV0P2BASE::getSubCycleTime() <= stopBy) { OTV0P2BASE::nap(WDTO_15MS, true); } // Sleep a little.
            // Send stats!
            // Try for double TX for extra robustness unless:
            //   * this is a speculative 'extra' TX
            //   * battery is low
            //   * this node is a hub so needs to listen as much as possible
            // This doesn't generally/always need to send binary/both formats
            // if this is controlling a local FHT8V on which the binary stats can be piggybacked.
            // Ie, if doesn't have a local TRV then it must send binary some of the time.
            // Any recently-changed stats value is a hint that a strong transmission might be a good idea.
            bareStatsTX();
            break;
        }
        // SENSOR READ AND STATS
        //
        // All external sensor reads should be in the second half of the minute (>32) if possible.
        // This is to have them as close to stats collection at the end of the minute as possible,
        // and to allow randomisation of the start-up cycle position in the first 32s to help avoid inter-unit collisions.
        // Also all sources of noise, self-heating, etc, may be turned off for the 'sensor read minute'
        // and thus will have diminished by this point.

        // Sample the user-selected WARM temperature target at a fixed rate.
        // This allows the unit to stay reasonably responsive to adjusting the temperature dial.
        case 48: { TempPot.read(); break; }
        // Read all environmental inputs, late in the cycle.
        // Sample humidity.
        case 50: { if(runAll) { RelHumidity.read(); } break; }
        // Poll ambient light level at a fixed rate.
        // This allows the unit to respond consistently to (eg) switching lights on (eg TODO-388).
        // Force all UI lights off before sampling ambient light level.
        case 52: { OTV0P2BASE::LED_HEATCALL_OFF(); AmbLight.read(); break; }
        // At a hub, sample temperature regularly as late as possible in the minute just before recomputing valve position.
        // Force a regular read to make stats such as rate-of-change simple and to minimise lag.
        // TODO: optimise to reduce power consumption when not calling for heat.
        // TODO: optimise to reduce self-heating jitter when in hub/listen/RX mode.
        case 54: { TemperatureC16.read(); break; }
        // Compute targets and heat demand based on environmental inputs and occupancy.
        // This should happen as soon after the latest readings as possible (temperature especially).
        case 56: {
            // Update occupancy status (fresh for target recomputation) at a fixed rate.
            Occupancy.read();
            // Recompute target, valve position and call for heat, etc.
            // Should be called once per minute to work correctly.
            NominalRadValve.read();
            break;
        }
        // Stats samples; should never be missed.
        case 58: {
            // Update non-volatile stats.
            // Make the final update as near the end of the hour as possible to reduce glitches (TODO-1086),
            // and with other optional non-full samples evenly spaced throughout the hour.
            // Race-free.
            const uint_least16_t msm = OTV0P2BASE::getMinutesSinceMidnightLT();
            const uint8_t mm = msm % 60;
//            if(59 == mm) { statsU.sampleStats(true, uint8_t(msm / 60)); }
//            else if((statsU.maxSamplesPerHour > 1) && (29 == mm)) { statsU.sampleStats(false, uint8_t(msm / 60)); }
            break;
        }
    }
    // End-of-loop processing, that may be slow.
    // Handle local direct-drive valve, eg DORM1.
    // If waiting for for verification that the valve has been fitted
    // then accept any manual interaction with controls as that signal.
    // Also have a backup timeout of at least ~10m from startup
    // for automatic recovery after a crash and restart,
    // or where fitter simply forgets to initiate cablibration.
    if(ValveDirect.isWaitingForValveToBeFitted()) {
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
    if(OTV0P2BASE::getSubCycleTime() < ((OTV0P2BASE::GSCT_MAX/4)*3))
        { ValveDirect.read(); }
    // Command-Line Interface (CLI) polling.
    // If a reasonable chunk of the minor cycle remains after all other work is done
    // AND the CLI is / should be active OR a status line has just been output
    // then poll/prompt the user for input
    // using a timeout which should safely avoid overrun, ie missing the next basic tick,
    // and which should also allow some energy-saving sleep.
    if(OTV0P2BASE::CLI::isCLIActive()) {
        const uint8_t stopBy = nearOverrunThreshold - 1;
        char buf[BUFSIZ_pollUI];
        OTV0P2BASE::ScratchSpace s((uint8_t*)buf, sizeof(buf));
        pollCLI(stopBy, 0 == TIME_LSD, s);
    }

    panic();
#if defined(EST_CPU_DUTYCYCLE)
    const unsigned long usEnd = micros();
    // Nominal loop time should be 2s x 1MHz clock, ie 2,000,000 if CPU running all the time.
    // Should generally be <2000 (us) (<0.1%) for leaf, <20000 (<1%) for hub.
    // Example output.
    //us apparent: 4544
    //us apparent: 25280
    //us apparent: 9280
    const unsigned long usApparentTaken = usEnd - usStart;
#if 1 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("us apparent: ");
    DEBUG_SERIAL_PRINT(usApparentTaken);
    DEBUG_SERIAL_PRINTLN();
#endif
#endif
}

