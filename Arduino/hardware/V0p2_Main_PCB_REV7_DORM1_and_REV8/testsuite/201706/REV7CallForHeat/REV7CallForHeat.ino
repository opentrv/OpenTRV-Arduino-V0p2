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

Author(s) / Copyright (s): Deniz Erbilgin 2016
*/
/**
 * Minimal REV7 config for testing power consumption.
 * Aim is to:
 *     - todo init GPIO pins to safe mode.
 *     - todo init peripherals to safe low power mode.
 *     - loop endlessly.
 */

// INCLUDES & DEFINES
// Debug output flag
#define DEBUG
// REV7 / DORM1 all-in-one valve unit, secure TX.
#define CONFIG_DORM1
// Get defaults for valve applications.
#include <OTV0p2_valve_ENABLE_defaults.h>
// All-in-one valve unit (DORM1).
#include <OTV0p2_CONFIG_REV7.h>
// I/O pin allocation and setup: include ahead of I/O module headers.
#include <OTV0p2_Board_IO_Config.h>
// OTV0p2Base Libraries
#include <OTV0p2Base.h>
// Radio Libraries
//#include <OTRadioLink.h>
#include <OTRFM23BLink.h>
// RadValve libraries
#include <OTRadValve.h>
#include <OTAESGCM.h>


// Debugging output
#ifndef DEBUG
#define DEBUG_SERIAL_PRINT(s) // Do nothing.
#define DEBUG_SERIAL_PRINTFMT(s, format) // Do nothing.
#define DEBUG_SERIAL_PRINT_FLASHSTRING(fs) // Do nothing.
#define DEBUG_SERIAL_PRINTLN_FLASHSTRING(fs) // Do nothing.
#define DEBUG_SERIAL_PRINTLN() // Do nothing.
#define DEBUG_SERIAL_TIMESTAMP() // Do nothing.
#else
// Send simple string or numeric to serial port and wait for it to have been sent.
// Make sure that Serial.begin() has been invoked, etc.
#define DEBUG_SERIAL_PRINT(s) { OTV0P2BASE::serialPrintAndFlush(s); }
#define DEBUG_SERIAL_PRINTFMT(s, fmt) { OTV0P2BASE::serialPrintAndFlush((s), (fmt)); }
#define DEBUG_SERIAL_PRINT_FLASHSTRING(fs) { OTV0P2BASE::serialPrintAndFlush(F(fs)); }
#define DEBUG_SERIAL_PRINTLN_FLASHSTRING(fs) { OTV0P2BASE::serialPrintlnAndFlush(F(fs)); }
#define DEBUG_SERIAL_PRINTLN() { OTV0P2BASE::serialPrintlnAndFlush(); }
// Print timestamp with no newline in format: MinutesSinceMidnight:Seconds:SubCycleTime
extern void _debug_serial_timestamp();
#define DEBUG_SERIAL_TIMESTAMP() _debug_serial_timestamp()
#endif // DEBUG

// OBJECTS & VARIABLES
/**
 * Peripherals present on REV7
 * - Pot        Set input to Hi-Z?  +
 * - PHT        IO_POWER_UP LOW
 * - Encoder    IO_POWER_UP LOW
 * - LED        Set pin HIGH?
 * - Button     Set to INPUT
 * - H-Bridge   Control pins HIGH, Current sense set to input.  +
 * - SHT21      read() once.        +
 * - RFM23B     ??                  +
 * - XTAL       Setup and leave running?
 * - UART       Disable
 */

/**
 * Dummy Types
 */
// Placeholder class with dummy static status methods to reduce code complexity.
typedef OTV0P2BASE::DummySensorOccupancyTracker OccupancyTracker;

/**
 * Supply Voltage instance
 */
// Sensor for supply (eg battery) voltage in millivolts.
// Singleton implementation/instance.
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;

/*
 * Radio instance
 */
static constexpr uint8_t RFM22_PREAMBLE_BYTES = 5; // Recommended number of preamble bytes for reliable reception.
static constexpr uint8_t RFM22_SYNC_MIN_BYTES = 3; // Minimum number of sync bytes.
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
static constexpr uint8_t STATS_MSG_START_OFFSET = (RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES);
static constexpr uint8_t STATS_MSG_MAX_LEN = (64 - STATS_MSG_START_OFFSET);
 
static constexpr bool RFM23B_allowRX = false;
 
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
static constexpr int8_t RFM23B_IRQ_PIN = -1;// PIN_RFM_NIRQ;
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> PrimaryRadio;//RFM23B;
// Pick an appropriate radio config for RFM23 (if it is the primary radio).
// Nodes talking on fast GFSK channel 0.
static const uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] = {
    // GFSK channel 0 full config, RX/TX, not in itself secure.
    OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true), };

/*
 * SHT21 instance
 */
OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.

// HUMIDITY_SENSOR_SUPPORT is defined if at least one humidity sensor has support compiled in.
// Simple implementations can assume that the sensor will be present if defined;
// more sophisticated implementations may wish to make run-time checks.
// If SHT21 support is enabled at compile-time then its humidity sensor may be used at run-time.
// Singleton implementation/instance.
typedef OTV0P2BASE::HumiditySensorSHT21 RelHumidity_t;
RelHumidity_t RelHumidity;

/**
 * Temp pot
 */
// Sensor for temperature potentiometer/dial UI control.
// Correct for DORM1/TRV1 with embedded REV7.
// REV7 does not drive pot from IO_POWER_UP.
typedef OTV0P2BASE::SensorTemperaturePot<OccupancyTracker, nullptr, 48, 296, false> TempPot_t;
TempPot_t TempPot;

/**
 * Ambient Light Sensor
 */
typedef OTV0P2BASE::SensorAmbientLight AmbientLight;
// Singleton implementation/instance.
AmbientLight AmbLight;

/**
 * Valve Actuator
 */
// DORM1/REV7 direct drive motor actuator.
static constexpr bool binaryOnlyValveControl = false;
static constexpr uint8_t m1 = MOTOR_DRIVE_ML;
static constexpr uint8_t m2 = MOTOR_DRIVE_MR;
static constexpr uint8_t mSleep = OTRadValve::MOTOR_DRIVE_NSLEEP_UNUSED;
typedef OTRadValve::ValveMotorDirectV1<OTRadValve::ValveMotorDirectV1HardwareDriver, m1, m2, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN, mSleep, decltype(Supply_cV), &Supply_cV> ValveDirect_t;// Singleton implementation/instance.
// Suppress unnecessary activity when room dark, eg to avoid disturbance if device crashes/restarts,
// unless recent UI use because value is being fitted/adjusted.
ValveDirect_t ValveDirect([](){return(AmbLight.isRoomDark());});

// FUNCTIONS

/**
 * Panic Functions
 */
// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic()
{
    // Reset radio and go into low-power mode.
    PrimaryRadio.panicShutdown();
    // Power down almost everything else...
    OTV0P2BASE::minimisePowerWithoutSleep();
    pinMode(OTV0P2BASE::LED_HEATCALL_L, OUTPUT);
    for( ; ; )
    {
        OTV0P2BASE::LED_HEATCALL_ON();
        OTV0P2BASE::nap(WDTO_15MS);
        OTV0P2BASE::LED_HEATCALL_OFF();
        OTV0P2BASE::nap(WDTO_120MS);
    }
}
// Panic with fixed message.
void panic(const __FlashStringHelper *s)
{
    OTV0P2BASE::serialPrintlnAndFlush(); // Start new line to highlight error.  // May fail.
    OTV0P2BASE::serialPrintAndFlush('!'); // Indicate error with leading '!' // May fail.
    OTV0P2BASE::serialPrintlnAndFlush(s); // Print supplied detail text. // May fail.
    panic();
}


/**
 * @brief   Set pins and on-board peripherals to safe low power state.
 */


// Managed JSON stats.
static OTV0P2BASE::SimpleStatsRotation<12> ss1; // Configured for maximum different stats.  // FIXME increased for voice & for setback lockout
// Do bare stats transmission.
// Output should be filtered for items appropriate
// to current channel security and sensitivity level.
// This is JSON format.
// Sends stats on primary radio channel 0 with possible duplicate to secondary channel.
// If sending encrypted then ID/counter fields (eg @ and + for JSON) are omitted
// as assumed supplied by security layer to remote recipent.
void callForHeatTX()
{
    // Note if radio/comms channel is itself framed.
    const bool framed = !PrimaryRadio.getChannelConfig()->isUnframed;
    const bool neededWaking = OTV0P2BASE::powerUpSerialIfDisabled<>();
  
static_assert(OTV0P2BASE::FullStatsMessageCore_MAX_BYTES_ON_WIRE <= STATS_MSG_MAX_LEN, "FullStatsMessageCore_MAX_BYTES_ON_WIRE too big");
static_assert(OTV0P2BASE::MSG_JSON_MAX_LENGTH+1 <= STATS_MSG_MAX_LEN, "MSG_JSON_MAX_LENGTH too big"); // Allow 1 for trailing CRC.

    // Allow space in buffer for:
    //   * buffer offset/preamble
    //   * max binary length, or max JSON length + 1 for CRC + 1 to allow detection of oversize message
    //   * terminating 0xff
    //  uint8_t buf[STATS_MSG_START_OFFSET + max(FullStatsMessageCore_MAX_BYTES_ON_WIRE,  MSG_JSON_MAX_LENGTH+1) + 1];
    // Buffer need be no larger than leading length byte + typical 64-byte radio module TX buffer limit + optional terminator.
    const uint8_t MSG_BUF_SIZE = 1 + 64 + 1;
    uint8_t buf[MSG_BUF_SIZE];
    // Send JSON message.
    bool sendingJSONFailed = false; // Set true and stop attempting JSON send in case of error.

    // Set pointer location based on whether start of message will have preamble
    uint8_t *bptr = buf;
    bptr += STATS_MSG_START_OFFSET;
    // Where to write the real frame content.
    uint8_t *const realTXFrameStart = bptr;

    // If forcing encryption or if unconditionally suppressed
    // then suppress the "@" ID field entirely,
    // assuming that the encrypted commands will carry the ID, ie in the 'envelope'.
    ss1.setID(V0p2_SENSOR_TAG_F(""));

    // Managed JSON stats.
    // Make best use of available bandwidth...
    constexpr bool maximise = true;
    // Enable "+" count field for diagnostic purposes, eg while TX is lossy,
    // if the primary radio channel does not include a sequence number itself.
    // Assume that an encrypted channel will provide its own (visible) sequence counter.
    ss1.enableCount(false); // as encrypted
    ss1.put(TemperatureC16);
    // OPTIONAL items
    // Only TX supply voltage for units apparently not mains powered, and TX with low priority as slow changing.
    constexpr uint8_t privacyLevel = OTV0P2BASE::stTXalwaysAll;

    // Buffer to write JSON to before encryption.
    // Size for JSON in 'O' frame is:
    constexpr uint8_t maxSecureJSONSize = OTRadioLink::ENC_BODY_SMALL_FIXED_PTEXT_MAX_SIZE - 2 + 1;
    // writeJSON() requires two further bytes including one for the trailing '\0'.
    uint8_t ptextBuf[maxSecureJSONSize + 2];

    // Redirect JSON output appropriately.
    uint8_t *const bufJSON = ptextBuf;
    const uint8_t bufJSONlen = sizeof(ptextBuf);

    // Number of bytes written for body.
    // For non-secure, this is the size of the JSON text.
    // For secure this is overridden with the secure frame size.
    int8_t wrote = 0;

    // Generate JSON text.
    if(!sendingJSONFailed)
    {
        // Generate JSON and write to appropriate buffer:
        // direct to TX buffer if not encrypting, else to separate buffer.
        wrote = ss1.writeJSON(bufJSON, bufJSONlen, privacyLevel, maximise);
        if(0 == wrote) { sendingJSONFailed = true; }
    }

    // Push the JSON output to Serial.
    if(!sendingJSONFailed)
    {
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
    if(!sendingJSONFailed)
    {
        if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key))
        {
            sendingJSONFailed = true;
            OTV0P2BASE::serialPrintlnAndFlush(F("!TX key")); // Know why TX failed.
        }
    }

    // If doing encryption
    // then build encrypted frame from raw JSON.
    if(!sendingJSONFailed)
    {
        // Explicit-workspace version of encryption.
        const OTRadioLink::SimpleSecureFrame32or0BodyTXBase::fixed32BTextSize12BNonce16BTagSimpleEncWithWorkspace_ptr_t eW = OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleEnc_DEFAULT_WITH_WORKSPACE;
        constexpr uint8_t workspaceSize = OTRadioLink::SimpleSecureFrame32or0BodyTXBase::generateSecureOFrameRawForTX_total_scratch_usage_OTAESGCM_2p0;
        uint8_t workspace[workspaceSize];
        OTV0P2BASE::ScratchSpace sW(workspace, workspaceSize);
        constexpr uint8_t txIDLen = OTRadioLink::ENC_BODY_DEFAULT_ID_BYTES;
        // When sending on a channel with framing, do not explicitly send the frame length byte.
        const uint8_t offset = framed ? 1 : 0;
        // Assumed to be at least one free writeable byte ahead of bptr.
        // Force the valve to call for heat.
        constexpr uint8_t valvePC = 99;
        const uint8_t bodylen = OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::getInstance().generateSecureOFrameRawForTX(
            realTXFrameStart - offset, sizeof(buf) - (realTXFrameStart-buf) + offset,
            txIDLen, valvePC, (const char *)bufJSON, eW, sW, key);
        sendingJSONFailed = (0 == bodylen);
        wrote = bodylen - offset;
    }

    if(!sendingJSONFailed)
    {
        // Send directly to the primary radio...
        PrimaryRadio.queueToSend(realTXFrameStart, wrote);
    }
    if(neededWaking) { OTV0P2BASE::flushSerialProductive(); OTV0P2BASE::powerDownSerial(); }
}

//========================================
// SETUP
//========================================

// Setup routine: runs once after reset.
// Does some limited board self-test and will panic() if anything is obviously broken.
void setup()
{
    // Set appropriate low-power states, interrupts, etc, ASAP.
    OTV0P2BASE::powerSetup();
    // IO setup for safety, and to avoid pins floating.
    OTV0P2BASE::IOSetup();

    OTV0P2BASE::serialPrintAndFlush(F("\r\nOpenTRV: ")); // Leading CRLF to clear leading junk, eg from bootloader.
    V0p2Base_serialPrintlnBuildVersion();

    OTV0P2BASE::LED_HEATCALL_ON();

    // Give plenty of time for the XTal to settle.
    delay(1000);

    // Have 32678Hz clock at least running before going any further.
    // Check that the slow clock is running reasonably OK, and tune the fast one to it.
    if(!::OTV0P2BASE::HWTEST::calibrateInternalOscWithExtOsc()) { panic(F("Xtal")); } // Async clock not running or can't tune.

    // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
    PrimaryRadio.preinit(NULL);
    // Check that the radio is correctly connected; panic if not...
    if(!PrimaryRadio.configure(nPrimaryRadioChannels, RFM23BConfigs) || !PrimaryRadio.begin()) { panic(F("r1")); }

    // Buttons should not be activated DURING boot for user-facing boards; an activated button implies a fault.
    // Check buttons not stuck in the activated position.
    if(fastDigitalRead(BUTTON_MODE_L) == LOW) { panic(F("b")); }

    // Collect full set of environmental values before entering loop() in normal mode.
    // This should also help ensure that sensors are properly initialised.

    // No external sensors are *assumed* present if running alt main loop
    // This may mean that the alt loop/POST will have to initialise them explicitly,
    // and the initial seed entropy may be marginally reduced also.
    const int cV = Supply_cV.read();
    DEBUG_SERIAL_PRINT_FLASHSTRING("V: ");
    DEBUG_SERIAL_PRINT(cV);
    DEBUG_SERIAL_PRINTLN();
    const int heat = TemperatureC16.read();
    DEBUG_SERIAL_PRINT_FLASHSTRING("T: ");
    DEBUG_SERIAL_PRINT(heat);
    DEBUG_SERIAL_PRINTLN();
    const uint8_t rh = RelHumidity.read();
    DEBUG_SERIAL_PRINT_FLASHSTRING("RH%: ");
    DEBUG_SERIAL_PRINT(rh);
    DEBUG_SERIAL_PRINTLN();
    const int light = AmbLight.read();
    DEBUG_SERIAL_PRINT_FLASHSTRING("L: ");
    DEBUG_SERIAL_PRINT(light);
    DEBUG_SERIAL_PRINTLN();
    const int tempPot = TempPot.read();
    DEBUG_SERIAL_PRINT_FLASHSTRING("temp pot: ");
    DEBUG_SERIAL_PRINT(tempPot);
    DEBUG_SERIAL_PRINTLN();

    ValveDirect.read();

    // Initialised: turn main/heatcall UI LED off.
    OTV0P2BASE::LED_HEATCALL_OFF();

    // Do OpenTRV-specific (late) setup.
    PrimaryRadio.listen(false);

    // Long delay after everything set up to allow a non-sleep power measurement.
    delay(10000);
}

//========================================
// MAIN LOOP
//========================================
/**
 * @brief   Sleep in low power mode if not sleeping.
 */
void loop()
{
    static uint8_t tickCounter = 0;
    // TX every 30 cycles (1 min, assuming 2 second cycles)
    // Too high for normal use, but convenient for testing purposes.
    if(0 == tickCounter) {
        callForHeatTX();
        tickCounter = 30;
    }
    --tickCounter;
    
    // Ensure that serial I/O is off while sleeping.
    OTV0P2BASE::powerDownSerial();
    // Power down most stuff (except radio for hub RX).
    OTV0P2BASE::minimisePowerWithoutSleep();
    // Normal long minimal-power sleep until wake-up interrupt.
    // Rely on interrupt to force quick loop round to I/O poll.
    OTV0P2BASE::sleepUntilInt();
    
}

