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

Author(s) / Copyright (s): Deniz Erbilgin 2017
*/
/**
 * Minimal REV7 config for testing motor optical encoder.
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
static constexpr bool RFM23B_allowRX = false;
 
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
static constexpr int8_t RFM23B_IRQ_PIN = -1;// PIN_RFM_NIRQ;
// XXX Is it really worth having a separate primary radio in this case?
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> PrimaryRadio;//RFM23B;
///* constexpr */ OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B
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
OTRadValve::ValveMotorDirectV1HardwareDriver<m1, m2, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN, OTRadValve::MOTOR_DRIVE_NSLEEP_UNUSED> motorDriver;
OTRadValve::TestValveMotor ValveDirect((OTRadValve::HardwareMotorDriverInterface *)(&motorDriver));

static constexpr uint8_t adcSetting = (1 << 6) | (MOTOR_DRIVE_MC_AIN & 7);

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
//    if(!::OTV0P2BASE::HWTEST::check32768HzOsc()) { panic(F("xtal")); } // Async clock not running correctly.

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

    // Initialised: turn main/heatcall UI LED off.
    OTV0P2BASE::LED_HEATCALL_OFF();

    // Do OpenTRV-specific (late) setup.
    PrimaryRadio.listen(false);

    // Long delay after everything set up to allow a non-sleep power measurement.
    delay(1000);
 
//    OTV0P2BASE::powerUpADCIfDisabled();
//    
//    pinMode(IO_POWER_UP, OUTPUT);
//    fastDigitalWrite(IO_POWER_UP, HIGH);
//    pinMode(m1, OUTPUT);
//    fastDigitalWrite(m1, LOW);
}


//========================================
// MAIN LOOP
//========================================
/**
 * @brief   Sleep in low power mode if not sleeping.
 */
void loop()
{
    ValveDirect.poll();
    // Capture and print encoder state.
//    const uint16_t encoderLevel = (OTV0P2BASE::analogueNoiseReducedRead(MOTOR_DRIVE_MC_AIN, DEFAULT) >> 2);
//    OTV0P2BASE::serialPrintAndFlush(encoderLevel);
//    uint16_t counter = 0;
//    bool oldLevel = false;
//    const int startTime = millis();
//    for(uint8_t i = 0; i < 255; ++i) {
//        const uint8_t encoderLevel = OTV0P2BASE::analogueNoiseReducedRead(MOTOR_DRIVE_MC_AIN, DEFAULT)  & 0xff;
//        bool newLevel = encoderLevel > 100;
//        counter = (newLevel != oldLevel) ? counter + 1 : counter;
//        oldLevel = newLevel;
////        OTV0P2BASE::serialPrintAndFlush(encoderLevel);
////        OTV0P2BASE::serialPrintlnAndFlush();
//    }
//    fastDigitalWrite(m2, HIGH);
//    OTV0P2BASE::serialPrintAndFlush("dT: ");
//    OTV0P2BASE::serialPrintAndFlush(millis() - startTime);
//    OTV0P2BASE::serialPrintAndFlush("\tCount: ");
//    OTV0P2BASE::serialPrintAndFlush(counter);
//    OTV0P2BASE::serialPrintlnAndFlush();
//    while(true) {}
}



/**
 * @brief    Motor encoder test (VCC ref) (DE201702)
 * Vcc: 2.57
 * State      read    Voltage (V)
 * white,     24-28   ~0.06
 * black,     42      ~0.1
 * daylight*, > 600   > 1.5
 * @note  - *: PCB removed from battery compartment + encoder pointed at window.
 *        - When PCB is attached to battery compartment, encoder is unaffected by daylight, even with valve not put together.
 *        - Increase in detected IR is decrease in measured value.
 */
