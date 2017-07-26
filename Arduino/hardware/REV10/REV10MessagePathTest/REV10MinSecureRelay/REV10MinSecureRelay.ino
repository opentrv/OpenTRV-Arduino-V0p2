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
                           Deniz Erbilgin 2015--2016
*/

/*
  Test of minimum relay code path.

  DHD20130417: hardware setup on bare board.
    * 1MHz CPU clock (from 8MHz internal RC clock with /8 prescaler) ATmega328P running at 1.8V--5V (typically 2V--3.3V).
    * Fuse set for BOD-managed additional clock settle time, ie as fast a restart from sleep as possible.
    * All unused pins unconnected and nominally floating (though driven low as output where possible).
    * 32768Hz xtal between pins XTAL1 and XTAL2, async timer 2, for accurate timekeeping and low-power sleep.
    * All unused system modules turned off.

  Basic AVR power consumption ticking an (empty) control loop at ~0.5Hz should be ~1uA.
 */


// GLOBAL flags that alter system build and behaviour.
//#define DEBUG // If defined, do extra checks and serial logging.  Will take more code space and power.

// Ensure that OpenTRV 'standard' UART speed is set unless explicitly overridden.
#define BAUD 4800
// Global flag for REV10 secure BHR
#define CONFIG_REV10_SECURE_BHR // REV10: secure stats relay and boiler hub.

// Get defaults for valve applications.
#include <OTV0p2_valve_ENABLE_defaults.h>
// REV8 + GSM Arduino shield + I2CEXT, see TODO-551.
#include <OTV0p2_CONFIG_REV10.h>
// --------------------------------------------
// Fixups to apply after loading the target config.
#include <OTV0p2_valve_ENABLE_fixups.h>

#include <OTV0p2_Board_IO_Config.h> // I/O pin allocation and setup: include ahead of I/O module headers.

#include <Arduino.h>
#include <OTV0p2Base.h>
#include <OTRadioLink.h>
#include <OTRFM23BLink.h>
#include <OTSIM900Link.h>
#include <OTAESGCM.h>

#include "ipAddress.h"  // IP adress in seperate header to avoid accidentally committing.

/////// / RADIOS
//For EEPROM: TODO make a spec for how config should be stored in EEPROM to make changing them easy
//- Set the first field of SIM900LinkConfig to true.
//- The configs are stored as \0 terminated strings starting at 0x300.
//- You can program the eeprom using ./OTRadioLink/dev/utils/sim900eepromWrite.ino
//  static const void *SIM900_PIN      = (void *)0x0300;
//  static const void *SIM900_APN      = (void *)0x0305;
//  static const void *SIM900_UDP_ADDR = (void *)0x031B;
//  static const void *SIM900_UDP_PORT = (void *)0x0329;
//  const OTSIM900Link::OTSIM900LinkConfig_t SIM900Config(
//                                                  true,
//                                                  SIM900_PIN,
//                                                  SIM900_APN,
//                                                  SIM900_UDP_ADDR,
//                                                  SIM900_UDP_PORT);
//For Flash:
//- Set the first field of SIM900LinkConfig to false.
//- The configs are stored as \0 terminated strings.
//- Where multiple options are available, uncomment whichever you want
  static const char SIM900_PIN[5] PROGMEM       = "1111";

// APN Configs - Uncomment based on what SIM you are using
//  static const char SIM900_APN[] PROGMEM      = "\"everywhere\",\"eesecure\",\"secure\""; // EE
//static const char SIM900_APN[] PROGMEM      = "\"arkessa.net\",\"arkessa\",\"arkessa\""; // Arkessa
static const char SIM900_APN[] PROGMEM      = "\"mobiledata\""; // GeoSIM

// UDP Configs - Edit SIM900_UDP_ADDR for relevant server. NOTE: The server IP address should never be committed to GitHub.
// IP adress in seperate header to avoid accidentally committing.
//static const char SIM900_UDP_ADDR[16] PROGMEM = ""; // Of form "1.2.3.4". 
static const char SIM900_UDP_PORT[5] PROGMEM = "9999";             // Standard port for OpenTRV servers
const OTSIM900Link::OTSIM900LinkConfig_t SIM900Config(
                                                false,
                                                SIM900_PIN,
                                                SIM900_APN,
                                                SIM900_UDP_ADDR,
                                                SIM900_UDP_PORT);


// Pick an appropriate radio config for RFM23 (if it is the primary radio).
// Nodes talking on fast GFSK channel 0.
static constexpr uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] = {
    // GFSK channel 0 full config, RX/TX, not in itself secure.
    OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true)
};
static const OTRadioLink::OTRadioChannelConfig SecondaryRadioConfig(&SIM900Config, true);

// Brings in necessary radio libs.
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
static constexpr int8_t RFM23B_IRQ_PIN = PIN_RFM_NIRQ;
static constexpr bool RFM23B_allowRX = true;
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> RFM23B;
OTSIM900Link::OTSIM900Link<8, 5, RADIO_POWER_PIN, OTV0P2BASE::getSecondsLT> SIM900;

// Assigns radio to PrimaryRadio alias
OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B;

// Assign radio to SecondaryRadio alias.
OTRadioLink::OTRadioLink &SecondaryRadio = SIM900;

//////// SENSORS
// TMP112 instance
OTV0P2BASE::RoomTemperatureC16_TMP112 TemperatureC16;
// Ambient Light Sensor
OTV0P2BASE::SensorAmbientLight AmbLight;

//////// Constants and variables
// Controller's view of Least Significant Digits of the current (local) time, in this case whole seconds.
// TIME_LSD ranges from 0 to TIME_CYCLE_S-1, also major cycle length.
static constexpr uint_fast8_t TIME_CYCLE_S = 60;
// Controller's notion/cache of seconds within major cycle.
static uint_fast8_t TIME_LSD;

// 'Elapsed minutes' count of minute/major cycles; cheaper than accessing RTC and not tied to real time.
// Starts at or just above zero (within the first 4-minute cycle) to help avoid collisions between units after mass power-up.
// Wraps at its maximum (0xff) value.
static uint8_t minuteCount;
// Does some limited board self-test and will panic() if anything is obviously broken.
static constexpr uint8_t PP_OFF_MS = 250;
// Mask for Port B input change interrupts.
static constexpr uint8_t RFM23B_INT_MASK = (1 << (PIN_RFM_NIRQ&7));


//========================================
// LOCAL FUNCTIONS
//========================================
// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
static void panic()
{
    // Reset radio and go into low-power mode.
    PrimaryRadio.panicShutdown();
    // Reset radio and go into low-power mode.
    SecondaryRadio.panicShutdown();
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
static void panic(const __FlashStringHelper *s)
{
    OTV0P2BASE::serialPrintlnAndFlush(); // Start new line to highlight error.  // May fail.
    OTV0P2BASE::serialPrintAndFlush('!'); // Indicate error with leading '!' // May fail.
    OTV0P2BASE::serialPrintlnAndFlush(s); // Print supplied detail text. // May fail.
    panic();
}

// Call this to do an I/O poll if needed; returns true if something useful definitely happened.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// Should also do nothing that interacts with Serial.
// Limits actual poll rate to something like once every 8ms, unless force is true.
//   * force if true then force full poll on every call (ie do not internally rate-limit)
// Note that radio poll() can be for TX as well as RX activity.
// Not thread-safe, eg not to be called from within an ISR.
// FIXME trying to move into utils (for the time being.)
bool pollIO(const bool force = false)
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
    SecondaryRadio.poll();
    }
  return(false);
  }


// Messaging
// Setup frame RX handlers
// Define queue handler
// Currently 4 possible cases for RXing secure frames:
// - Both relay and boiler hub present (e.g. CONFIG_REV10_AS_BHR)
// - Just relay present (e.g. CONFIG_REV10_AS_GSM_RELAY_ONLY)
// - Just boiler hub (e.g. CONFIG_REV8_SECURE_BHR)
// - Unit acting as stats-hub (e.g. CONFIG_REV11_SECURE_STATSHUB)
// relay
inline bool decodeAndHandleSecureFrame(volatile const uint8_t * const msg)
{
  return OTRadioLink::decodeAndHandleOTSecureOFrame<OTRadioLink::SimpleSecureFrame32or0BodyRXV0p2,
                                                    OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleDec_DEFAULT_STATELESS,
                                                   OTV0P2BASE::getPrimaryBuilding16ByteSecretKey,
                                                   OTRadioLink::relayFrameOperation<decltype(SIM900), SIM900>
                                                  >(msg);
}
OTRadioLink::OTMessageQueueHandler< pollIO, V0P2_UART_BAUD,
                                    decodeAndHandleSecureFrame, OTRadioLink::decodeAndHandleDummyFrame
                                   > messageQueue;  //TODO change baud


//========================================
// INTERRUPT SERVICE ROUTINES
//========================================
// Previous state of port B pins to help detect changes.
static volatile uint8_t prevStatePB;
// Interrupt service routine for PB I/O port transition changes.
ISR(PCINT0_vect)
  {
  const uint8_t pins = PINB;
  const uint8_t changes = pins ^ prevStatePB;
  prevStatePB = pins;
  // RFM23B nIRQ falling edge is of interest.
  // Handler routine not required/expected to 'clear' this interrupt.
  // TODO: try to ensure that OTRFM23BLink.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
  if((changes & RFM23B_INT_MASK) && !(pins & RFM23B_INT_MASK)) { PrimaryRadio.handleInterruptSimple(); }
  }


//========================================
// SETUP
//========================================

// Setup routine: runs once after reset.
void setup()
{
    //----------- Low level setup
    // Set appropriate low-power states, interrupts, etc, ASAP.
    OTV0P2BASE::powerSetup();
  
    // IO setup for safety, and to avoid pins floating.
    OTV0P2BASE::IOSetup();
    
    OTV0P2BASE::serialPrintAndFlush(F("\r\nOpenTRV: ")); // Leading CRLF to clear leading junk, eg from bootloader.
    V0p2Base_serialPrintlnBuildVersion();
    
    // Count resets to detect unexpected crashes/restarts.
    const uint8_t oldResetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
    eeprom_write_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT, 1 + oldResetCount);
    
    // Have 32678Hz clock at least running before going any further.
    // Check that the slow clock is running reasonably OK, and tune the fast one to it.
    if(!::OTV0P2BASE::HWTEST::calibrateInternalOscWithExtOsc()) { panic(F("Xtal")); } // Async clock not running or can't tune.
  
    //----------- Flash UI
    // Signal that xtal is running AND give it time to settle.
    OTV0P2BASE::sleepLowPowerMs(1000);
    OTV0P2BASE::LED_HEATCALL_OFF();
  
    OTV0P2BASE::sleepLowPowerMs(PP_OFF_MS); // TODO: use this time to gather entropy.
    OTV0P2BASE::LED_HEATCALL_ON();
    OTV0P2BASE::sleepLowPowerMs(1000); // TODO: use this time to gather entropy.
  
    //----------- Init Radio
    // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
    PrimaryRadio.preinit(NULL);
    // Check that the radio is correctly connected; panic if not...
    if(!PrimaryRadio.configure(nPrimaryRadioChannels, RFM23BConfigs) || !PrimaryRadio.begin()) { panic(F("r1")); }
    
    // Turn power on for SIM900 with PFET for secondary power control.
    fastDigitalWrite(A3, 0); // todo move into sim900link
    pinMode(A3, OUTPUT);
    // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
    SecondaryRadio.preinit(NULL);
    // Check that the radio is correctly connected; panic if not...
    if(!SecondaryRadio.configure(1, &SecondaryRadioConfig) || !SecondaryRadio.begin()) { panic(F("r2")); }
  
    OTV0P2BASE::seedPRNGs();
  
    //----------- Ensure has ID
    // Ensure that the unique node ID is set up (mainly on first use).
    // Have one attempt (don't want to stress an already failing EEPROM) to force-reset if not good, then panic.
    // Needs to have had entropy gathered, etc.
    if(!OTV0P2BASE::ensureIDCreated()) {
        if(!OTV0P2BASE::ensureIDCreated(true)) { panic(F("ID")); }  // Force reset.
    }
  
    // Initialised: turn main/heatcall UI LED off.
    OTV0P2BASE::LED_HEATCALL_OFF();
  
    // Do OpenTRV-specific (late) setup.
    // Radio not listening to start with.
    // Ignore any initial spurious RX interrupts for example.
    PrimaryRadio.listen(false);
    // Set up async edge interrupts.
    // Enable pin change interrupt on PB, RFM23B interrupt pin.
    ATOMIC_BLOCK (ATOMIC_RESTORESTATE) { PCICR = 1; PCMSK0 = RFM23B_INT_MASK; }
    PrimaryRadio.listen(true);  // XXX might be switched off and left that way somewhere
  
    // Init sensors
    TemperatureC16.read();
    AmbLight.read();
    
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

//========================================
// MAIN LOOP
//========================================
// Main loop for OpenTRV radiator control.
// Note: exiting and re-entering can take a little while, handling Arduino background tasks such as serial.
#if 1
inline void stackCheck()
{
    // Complain and keep complaining when getting near stack overflow.
    // TODO: make DEBUG-only when confident all configs OK.
    const int16_t minsp = OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd();
    const uint8_t location = OTV0P2BASE::MemoryChecks::getLocation();
    OTV0P2BASE::serialPrintAndFlush(F("minsp: "));
    OTV0P2BASE::serialPrintAndFlush(minsp, HEX);
    OTV0P2BASE::serialPrintAndFlush(F(" loc:"));
    OTV0P2BASE::serialPrintAndFlush(location);
    OTV0P2BASE::serialPrintlnAndFlush();

    // Force restart if SPAM/heap/stack likely corrupt.
//    OTV0P2BASE::MemoryChecks::forceResetIfStackOverflow();
    if(OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd() <= 0) panic(F("!SOVF"));

    OTV0P2BASE::MemoryChecks::resetMinSP();
}
#endif
void loop()
{
    // CHECK STACK
    // ===============
    // Force restart if SPAM/heap/stack likely corrupt.
    stackCheck();
  
    // SLEEP
    // ===============
    // Sleep in low-power mode (waiting for interrupts) until seconds roll.
    // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
    // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
    // Ensure that serial I/O is off while sleeping.
    OTV0P2BASE::powerDownSerial();
    // Power down most stuff (except radio for hub RX).
    OTV0P2BASE::minimisePowerWithoutSleep();
    uint_fast8_t newTLSD;
    while(TIME_LSD == (newTLSD = OTV0P2BASE::getSecondsLT())) {
        // Poll I/O and process message incrementally (in this otherwise idle time)
        // before sleep and on wakeup in case some IO needs further processing now,
        // eg work was accrued during the previous major slow/outer loop
        // or the in a previous orbit of this loop sleep or nap was terminated by an I/O interrupt.
        // May generate output to host on Serial.
        // Come back and have another go immediately until no work remaining.
        if(messageQueue.handle(true, PrimaryRadio)) { continue; }
    
        // Normal long minimal-power sleep until wake-up interrupt.
        // Rely on interrupt to force quick loop round to I/O poll.
        OTV0P2BASE::sleepUntilInt();
    }
    TIME_LSD = newTLSD;
    // Reset and immediately re-prime the RTC-based watchdog.
    OTV0P2BASE::resetRTCWatchDog();
    OTV0P2BASE::enableRTCWatchdog(true);
  
  
    // START LOOP BODY
    // ===============  
    // Handling the UI may have taken a little while, so process I/O a little.
    messageQueue.handle(true, PrimaryRadio); // Deal with any pending I/O.
    
    // High-priority UI handing, every other/even second.
    // Show status if the user changed something significant.
    // Must take ~300ms or less so as not to run over into next half second if two TXs are done.
    switch(TIME_LSD) { // With V0P2BASE_TWO_S_TICK_RTC_SUPPORT only even seconds are available.
        // Tasks that must be run every minute.
        // Note simple roll-over to 0 at max value.
        case 0: { ++minuteCount; break; }
        // Churn/reseed PRNG(s) a little to improve unpredictability in use: should be lightweight.
        case 2: { if(minuteCount & 3) { OTV0P2BASE::seedRNG8(minuteCount ^ OTV0P2BASE::getCPUCycleCount(), OTV0P2BASE::_getSubCycleTime(), 0xff); } break; }
    }
    
    // End-of-loop processing, that may be slow.
    // Ensure progress on queued messages ahead of slow work.  (TODO-867)
    messageQueue.handle(true, PrimaryRadio); // Deal with any pending I/O.
}

