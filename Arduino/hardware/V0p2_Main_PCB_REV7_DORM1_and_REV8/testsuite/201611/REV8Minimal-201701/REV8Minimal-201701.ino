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

Author(s) / Copyright (s): Deniz Erbilgin 2016--2017
                           Damon Hart-Davis 2017
*/

/**
 * Minimal REV8 config for testing basics such as radio and relay.
 *
 * Aim is to:
 *     - init GPIO pins to safe mode.
 *     - init peripherals to safe low power mode.
 *     - toggle LEDs and relay
 *     - loop endlessly, toggling LED/relay, reading sensors and listening on radio.
 *
 * See also, for previous version of this code:
 *     Arduino/snapshots/20150110-r4107-V0p2-Arduino-REV8-boiler-relay-test
 */

// INCLUDES & DEFINES
// Debug output flag
#define DEBUG
// REV8 / DORM1 boiler controller and secure hub, secure TX.
#define CONFIG_REV8_SECURE_BHR
// Get defaults for valve applications.
#include <OTV0p2_valve_ENABLE_defaults.h>
// All-in-one valve unit (DORM1).
#include <OTV0p2_CONFIG_REV8.h>
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
 * Peripherals present on REV8
 * - LED          Toggle in antiphase to relay.
 * - Buttons      MODE + LEARN + LEARN2
 * - SHT21        read() once.        +
 * - TMP112       read() once.        +
 * - RFM23B       ??                  +
 * - XTAL         Setup and leave running.
 * - Boiler relay Toggle back and forth.
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
static constexpr bool RFM23B_allowRX = true;
 
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
static constexpr int8_t RFM23B_IRQ_PIN = PIN_RFM_NIRQ;
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> PrimaryRadio;//RFM23B;
///* constexpr */ OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B
// Pick an appropriate radio config for RFM23 (if it is the primary radio).
// Nodes talking on fast GFSK channel 0.
static const uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] = {
    // GFSK channel 0 full config, RX/TX, not in itself secure.
    OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true), };

// SHT21 temperature sensor instance
OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.

// HUMIDITY_SENSOR_SUPPORT is defined if at least one humidity sensor has support compiled in.
// Simple implementations can assume that the sensor will be present if defined;
// more sophisticated implementations may wish to make run-time checks.
// If SHT21 support is enabled at compile-time then its humidity sensor may be used at run-time.
// Singleton implementation/instance.
typedef OTV0P2BASE::HumiditySensorSHT21 RelHumidity_t;
RelHumidity_t RelHumidity;

// TMP112 temperature sensor instance
OTV0P2BASE::RoomTemperatureC16_TMP112 TemperatureC16TMP112; // TMP112 impl.


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

    // Give plenty of time for the Xtal to settle.
    delay(5000);

    // Have 32678Hz clock at least running before going any further.
//    if(!::OTV0P2BASE::HWTEST::check32768HzOsc()) { panic(F("xtal")); } // Async clock not running correctly.
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

    // Use the sensors on the TMP112: supply (internal), SHT21, TMP112.
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
    const int heatTMP112 = TemperatureC16TMP112.read();
    DEBUG_SERIAL_PRINT_FLASHSTRING("T(TMP112): ");
    DEBUG_SERIAL_PRINT(heatTMP112);
    DEBUG_SERIAL_PRINTLN();

    // Initialised: turn main/heatcall UI LED off.
    OTV0P2BASE::LED_HEATCALL_OFF();

    // Set up the radio to listen on standard OpenTRV GFSK57600 channel in band 14.
    PrimaryRadio.listen(true);
}


//========================================
// MAIN LOOP
//========================================
/**
 * @brief   Sleep in low power mode, toggle LEDs every 2s.
 */
void loop()
{
    static bool LED_on;

    // Toggle (primary, red0 LED and boiler relay (with its LED) in antiphase.
    if(LED_on) { OTV0P2BASE::LED_HEATCALL_ON(); }
    else { OTV0P2BASE::LED_HEATCALL_OFF(); }
    fastDigitalWrite(OUT_HEATCALL, !LED_on ? HIGH : LOW);

    // Toggle second (green) LED along with first.
    if(LED_on) { OTV0P2BASE::LED_UI2_ON(); }
    else { OTV0P2BASE::LED_UI2_OFF(); }

    // Repeatedly read and print sensor values for fun.
    const int heat = TemperatureC16.read();
    DEBUG_SERIAL_PRINT_FLASHSTRING("T: ");
    DEBUG_SERIAL_PRINT(heat);
    DEBUG_SERIAL_PRINTLN();
    const uint8_t rh = RelHumidity.read();
    DEBUG_SERIAL_PRINT_FLASHSTRING("RH%: ");
    DEBUG_SERIAL_PRINT(rh);
    DEBUG_SERIAL_PRINTLN();
    const int heatTMP112 = TemperatureC16TMP112.read();
    DEBUG_SERIAL_PRINT_FLASHSTRING("T(TMP112): ");
    DEBUG_SERIAL_PRINT(heatTMP112);
    DEBUG_SERIAL_PRINTLN();

    // Very crude poll/RX; likely to miss plenty.
    PrimaryRadio.poll();
    const uint8_t rxed = PrimaryRadio.getRXMsgsQueued();
    DEBUG_SERIAL_PRINT_FLASHSTRING("RXED: ");
    DEBUG_SERIAL_PRINT(rxed);
    DEBUG_SERIAL_PRINTLN();
    if(0 != rxed)
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING("RX 1st byte: ");
      DEBUG_SERIAL_PRINTFMT(*PrimaryRadio.peekRXMsg(), HEX);
      DEBUG_SERIAL_PRINTLN();
      PrimaryRadio.removeRXMsg();
      }


    // =====
    // To sleep, perchance to dream...
    
    // Ensure that serial I/O is off while sleeping.
    OTV0P2BASE::powerDownSerial();
    // Power down most stuff (except radio for hub RX).
    OTV0P2BASE::minimisePowerWithoutSleep();
    // Normal long minimal-power sleep until wake-up interrupt.
    // Rely on interrupt to force quick loop round to I/O poll.
    OTV0P2BASE::sleepUntilInt();

    LED_on = !LED_on;
}

/*
 * Output from running board should look something like this:


OpenTRV: board V0.2 REV8 2017/Jan/08 18:34:37
V: 321
T: 401
RH%: 61
T(TMP112): 393
T: 401
RH%: 61
T(TMP112): 393
RXED: 0
T: 401
RH%: 61
T(TMP112): 393
RXED: 1
RX 1st byte: CF
T: 400
RH%: 61
T(TMP112): 393
RXED: 0

 */
