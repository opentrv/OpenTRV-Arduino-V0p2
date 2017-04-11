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
*/

/*
  V0p2 (V0.2) core/main header file for this project:
  all other project header files should #include this first
  (or at least immediately after std/AVR headers) for consistency,
  and project non-header files should include this via their own header files (or directly).
  */

#ifndef REV10_SECURE_BHR_H
#define REV10_SECURE_BHR_H


// GLOBAL flags that alter system build and behaviour.
//#define DEBUG // If defined, do extra checks and serial logging.  Will take more code space and power.
//#define EST_CPU_DUTYCYCLE // If defined, estimate CPU duty cycle and thus base power consumption.

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
#include <OTRadValve.h>
#include <OTRadioLink.h>
#include <OTRFM23BLink.h>
#include <OTSIM900Link.h>
#include <OTRN2483Link.h>
#include <OTAESGCM.h>

// Indicate that the system is broken in an obvious way (distress flashing of the main UI LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep the distress beacon running for a while.
void panic();
// Panic with fixed message.
void panic(const __FlashStringHelper *s);

// Call this to do an I/O poll if needed; returns true if something useful happened.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// Should also do nothing that interacts with Serial.
// Limits actual poll rate to something like once every 8ms, unless force is true.
//   * force if true then force full poll on every call (ie do not internally rate-limit)
// Not thread-safe, eg not to be called from within an ISR.
// NOTE: implementation may not be in power-management module.
bool pollIO(bool force = false);


////// MESSAGING
extern OTRadioLink::OTRadioLink &PrimaryRadio;
extern OTRadioLink::OTRadioLink &SecondaryRadio;

//For EEPROM:
//- Set the first field of SIM900LinkConfig to true.
//- The configs are stored as \0 terminated strings starting at 0x300.
//- You can program the eeprom using ./OTRadioLink/dev/utils/sim900eepromWrite.ino

extern const OTSIM900Link::OTSIM900LinkConfig_t SIM900Config;

// XXX
static constexpr uint8_t RFM22_PREAMBLE_BYTE = 0xaa; // Preamble byte for RFM22/23 reception.
static constexpr uint8_t RFM22_PREAMBLE_MIN_BYTES = 4; // Minimum number of preamble bytes for reception.
static constexpr uint8_t RFM22_PREAMBLE_BYTES = 5; // Recommended number of preamble bytes for reliable reception.
static constexpr uint8_t RFM22_SYNC_BYTE = 0xcc; // Sync-word trailing byte (with FHT8V primarily).
static constexpr uint8_t RFM22_SYNC_MIN_BYTES = 3; // Minimum number of sync bytes.

// XXX
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

// Returns true if an unencrypted trailing static payload and similar (eg bare stats transmission) is permitted.
// True if the TX_ENABLE value is no higher than stTXmostUnsec.
// Some filtering may be required even if this is true.
#define enableTrailingStatsPayload() (true) // Always allow at least some stats to be TXed.


// Incrementally poll and process I/O and queued messages, including from the radio link.
// Returns true if some work was done.
// This may mean printing them to Serial (which the passed Print object usually is),
// or adjusting system parameters,
// or relaying them elsewhere, for example.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This will attempt to process messages in such a way
// as to avoid internal overflows or other resource exhaustion,
// which may mean deferring work at certain times
// such as the end of minor cycle.
// The Print object pointer must not be NULL.
bool handleQueuedMessages(Print *p, bool wakeSerialIfNeeded, OTRadioLink::OTRadioLink *rl);


/////// CONTROL (EARLY, NOT DEPENDENT ON OTHER SENSORS)
// XXX
// Radiator valve mode (FROST, WARM, BAKE).
extern OTRadValve::ValveMode valveMode;

// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#ifdef ENABLE_OCCUPANCY_SUPPORT
typedef OTV0P2BASE::PseudoSensorOccupancyTracker OccupancyTracker;
#else
// Placeholder class with dummy static status methods to reduce code complexity.
typedef OTV0P2BASE::DummySensorOccupancyTracker OccupancyTracker;
#endif
// Singleton implementation for entire node.
extern OccupancyTracker Occupancy;


////// SENSORS

// Sensor for supply (eg battery) voltage in millivolts.
// Singleton implementation/instance.
extern OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;


// XXX
// Sense ambient lighting level.
#ifdef ENABLE_AMBLIGHT_SENSOR
// Sensor for ambient light level; 0 is dark, 255 is bright.
typedef OTV0P2BASE::SensorAmbientLight AmbientLight;
#else // !defined(ENABLE_AMBLIGHT_SENSOR)
typedef OTV0P2BASE::DummySensorAmbientLight AmbientLight; // Dummy stand-in.
#endif // ENABLE_AMBLIGHT_SENSOR
// Singleton implementation/instance.
extern AmbientLight AmbLight;

// XXX
// Ambient/room temperature sensor, usually on main board.
#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
extern OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.
#elif defined(ENABLE_PRIMARY_TEMP_SENSOR_DS18B20)
  #if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
  // DSB18B20 temperature impl, with slightly reduced precision to improve speed.
  extern OTV0P2BASE::TemperatureC16_DS18B20 TemperatureC16;
  #endif // defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
#else // Don't use TMP112 if SHT21 or DS18B20 have been selected.
extern OTV0P2BASE::RoomTemperatureC16_TMP112 TemperatureC16;
#endif

// Dummy implementation to minimise coding changes.
extern OTV0P2BASE::DummyHumiditySensorSHT21 RelHumidity;



/////// CONTROL

// Special setup for OpenTRV beyond generic hardware setup.
void setupOpenTRV();
// Main loop for OpenTRV radiator control.
void loopOpenTRV();

// XXX
// Select basic parameter set to use (or could define new set here).
#ifndef DHW_TEMPERATURES
// Settings for room TRV.
typedef OTRadValve::DEFAULT_ValveControlParameters PARAMS;
#else
// Default settings for DHW control.
typedef OTRadValve::DEFAULT_DHW_ValveControlParameters PARAMS;
#endif


// Dummy temperature control.
typedef OTRadValve::NULLTempControl TempControl_t;
// XXX
#define TempControl_DEFINED
extern TempControl_t tempControl;

// XXX
// Default minimum on/off time in minutes for the boiler relay.
// Set to 5 as the default valve Tx cycle is 4 mins and 5 mins is a good amount for most boilers.
// This constant is necessary as if V0P2BASE_EE_START_MIN_BOILER_ON_MINS_INV is not set, the boiler relay will never be turned on.
static const constexpr uint8_t DEFAULT_MIN_BOILER_ON_MINS = 5;
#if defined(ENABLE_DEFAULT_ALWAYS_RX)
#define getMinBoilerOnMinutes() (DEFAULT_MIN_BOILER_ON_MINS)
#elif defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)
// Get minimum on (and off) time for pointer (minutes); zero if not in hub mode.
uint8_t getMinBoilerOnMinutes();
// Set minimum on (and off) time for pointer (minutes); zero to disable hub mode.
// Suggested minimum of 4 minutes for gas combi; much longer for heat pumps for example.
void setMinBoilerOnMinutes(uint8_t mins);
#else
#define getMinBoilerOnMinutes() (0) // Always disabled.
#define setMinBoilerOnMinutes(mins) {} // Do nothing.
#endif

// XXX
#if defined(ENABLE_DEFAULT_ALWAYS_RX)
// True: always in central hub/listen mode.
#define inHubMode() (true)
// True: always in stats hub/listen mode.
#define inStatsHubMode() (true)
#elif !defined(ENABLE_RADIO_RX)
// No RX/listening allowed, so never in hub mode.
// False: never in central hub/listen mode.
#define inHubMode() (false)
// False: never in stats hub/listen mode.
#define inStatsHubMode() (false)
#else
// True if in central hub/listen mode (possibly with local radiator also).
#define inHubMode() (0 != getMinBoilerOnMinutes())
// True if in stats hub/listen mode (minimum timeout).
#define inStatsHubMode() (1 == getMinBoilerOnMinutes())
#endif // defined(ENABLE_DEFAULT_ALWAYS_RX)

// XXX
// Period in minutes for simple learned on-time; strictly positive (and less than 256).
#ifndef LEARNED_ON_PERIOD_M
#define LEARNED_ON_PERIOD_M 60
#endif
// Period in minutes for simple learned on-time with comfort bias; strictly positive (and less than 256).
// Defaults to twice LEARNED_ON_PERIOD_M.
// Should be no shorter/less than LEARNED_ON_PERIOD_M to avoid confusion.
#ifndef LEARNED_ON_PERIOD_COMFORT_M
#define LEARNED_ON_PERIOD_COMFORT_M (min(2*(LEARNED_ON_PERIOD_M),255))
#endif
#if defined(ENABLE_SINGLETON_SCHEDULE)
#define SCHEDULER_AVAILABLE
// Singleton scheduler instance.
typedef OTRadValve::SimpleValveSchedule
    <
    LEARNED_ON_PERIOD_M, LEARNED_ON_PERIOD_COMFORT_M,
    decltype(tempControl), &tempControl,
#if defined(ENABLE_OCCUPANCY_SUPPORT)
    decltype(Occupancy), &Occupancy
#else
    OTRadValve::SimpleValveSchedule_PseudoSensorOccupancyTracker, (OTRadValve::SimpleValveSchedule_PseudoSensorOccupancyTracker*)NULL
#endif
    > Scheduler_t;
#else
// Dummy scheduler to simplify coding.
typedef OTRadValve::NULLValveSchedule Scheduler_t;
#endif // defined(ENABLE_SINGLETON_SCHEDULE)
extern Scheduler_t Scheduler;


/////// STATS

// Singleton non-volatile stats store instance.
extern OTV0P2BASE::EEPROMByHourByteStats eeStats;

// Singleton stats-updater object.
typedef 
    OTV0P2BASE::ByHourSimpleStatsUpdaterSampleStats <
      decltype(eeStats), &eeStats,
      // XXX
#if defined(ENABLE_OCCUPANCY_SUPPORT)
      decltype(Occupancy), &Occupancy,
#else
      OTV0P2BASE::SimpleTSUint8Sensor, static_cast<OTV0P2BASE::SimpleTSUint8Sensor*>(NULL), // Save code space when no occupancy tracking.
#endif
      decltype(AmbLight), &AmbLight,
      decltype(TemperatureC16), &TemperatureC16,
      decltype(RelHumidity), &RelHumidity,
      2
      > StatsU_t;
extern StatsU_t statsU;

// XXX
// Mechanism to generate '=' stats line, if enabled.
#if defined(ENABLE_SERIAL_STATUS_REPORT)
typedef OTV0P2BASE::SystemStatsLine<
      decltype(valveMode), &valveMode,
      OTRadValve::AbstractRadValve, (OTRadValve::AbstractRadValve *)NULL,
      decltype(TemperatureC16), &TemperatureC16,
      OTV0P2BASE::HumiditySensorBase, (OTV0P2BASE::HumiditySensorBase *)NULL,
// XXX
#ifdef ENABLE_AMBLIGHT_SENSOR
      decltype(AmbLight), &AmbLight,
#else
      OTV0P2BASE::SensorAmbientLight, (OTV0P2BASE::SensorAmbientLight *)NULL,
#endif
// XXX
#ifdef ENABLE_OCCUPANCY_SUPPORT
      decltype(Occupancy), &Occupancy,
#else
      OTV0P2BASE::PseudoSensorOccupancyTracker, (OTV0P2BASE::PseudoSensorOccupancyTracker*)NULL,
#endif
      decltype(Scheduler), &Scheduler,
#if defined(ENABLE_JSON_OUTPUT) && !defined(ENABLE_TRIMMED_MEMORY)
      true // Enable JSON stats.
#else
      true // Disable JSON stats. // FIXME
#endif
      > StatsLine_t;
extern StatsLine_t statsLine;
// Send a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
// Should be similar to PICAXE V0.1 output to allow the same parser to handle either.
inline void serialStatusReport() { statsLine.serialStatusReport(); }
#else
#define serialStatusReport() { }
#endif // defined(ENABLE_SERIAL_STATUS_REPORT)

// Do bare stats transmission.
// Output should be filtered for items appModelledRadValveComputeTargetTempBasicropriate
// to current channel security and sensitivity level.
// This may be binary or JSON format.
//   * allowDoubleTX  allow double TX to increase chance of successful reception
//   * doBinary  send binary form if supported, else JSON form if supported
// Sends stats on primary radio channel 0 with possible duplicate to secondary channel.
// If sending encrypted then ID/counter fields (eg @ and + for JSON) are omitted
// as assumed supplied by security layer to remote recipent.
void bareStatsTX(bool allowDoubleTX = false, bool doBinary = false);

// Raw notification of received call for heat from remote (eg FHT8V) unit.
// This form has a 16-bit ID (eg FHT8V housecode) and percent-open value [0,100].
// Note that this may include 0 percent values for a remote unit explicitly confirming
// that is is not, or has stopped, calling for heat (eg instead of replying on a timeout).
// This is not filtered, and can be delivered at any time from RX data, from a non-ISR thread.
// Does not have to be thread-/ISR- safe.
void remoteCallForHeatRX(uint16_t id, uint8_t percentOpen);

////// UI
// XXX
// Suggested minimum buffer size for pollUI() to ensure maximum-sized commands can be received.
#if defined(ENABLE_EXTENDED_CLI) || defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
static constexpr uint8_t MAXIMUM_CLI_RESPONSE_CHARS = 1 + OTV0P2BASE::CLI::MAX_TYPICAL_CLI_BUFFER;
#else
static constexpr uint8_t MAXIMUM_CLI_RESPONSE_CHARS = 1 + OTV0P2BASE::CLI::MIN_TYPICAL_CLI_BUFFER;
#endif
static constexpr uint8_t BUFSIZ_pollUI = 1 + MAXIMUM_CLI_RESPONSE_CHARS;

// Used to poll user side for CLI input until specified sub-cycle time.
// A period of less than (say) 500ms will be difficult for
// direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid
// possibility of overrun on long interactions.
// Times itself out after at least a minute or two of inactivity. 
// NOT RE-ENTRANT (eg uses static state for speed and code space).
void pollCLI(uint8_t maxSCT, bool startOfMinute, const OTV0P2BASE::ScratchSpace &s);


////////////////////////// Actuators

// XXX
// DORM1/REV7 direct drive motor actuator.
static constexpr bool binaryOnlyValveControl = true;


#endif // REV10_SECURE_BHR_H

