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

#ifndef V0p2_MAIN_H
#define V0p2_MAIN_H


// GLOBAL flags that alter system build and behaviour.
//#define DEBUG // If defined, do extra checks and serial logging.  Will take more code space and power.
//#define EST_CPU_DUTYCYCLE // If defined, estimate CPU duty cycle and thus base power consumption.

#ifndef BAUD
// Ensure that OpenTRV 'standard' UART speed is set unless explicitly overridden.
#define BAUD 4800
#endif

// Defaults for V0p2 / V0.2; should be '#undef'ined if not required.
//
// Use sleep wakeup (2Hz by default) from external 32768Hz xtal and timer 2.
#define ENABLE_WAKEUP_32768HZ_XTAL
// IF DEFINED: this unit may run on 2xAA cells, preferably rechargeable eg NiMH, ~2V--2.4V, and should monitor supply voltage.
#define ENABLE_SUPPLY_VOLTAGE_LOW_2AA // May require limiting clock speed and using some alternative peripherals/sensors...
// IF DEFINED: enable use AVR's 'idle' mode to stop the CPU but leave I/O clocls (eg Serial) running to save power.
// DHD20150920: NOT RECOMMENDED AS SEEMS TO CAUSE SOME BOARDS (REV1,REV9) TO CRASH.
#undef ENABLE_USE_OF_AVR_IDLE_MODE
// IF DEFINED: use (slow, low energy) 32768Hz-clock-based watchdog to recover from some software hangups.
#define ENABLE_WATCHDOG_SLOW
// IF DEFINED: attempt to tune the internal fast (RC/resonator) clock from the RTC source.
#define ENABLE_TUNE_FAST_OSC_TO_RTC_SOURCE
// IF DEFINED: provide software RTC support by default.
#define ENABLE_RTC_INTERNAL_SIMPLE
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.
#undef ENABLE_MIN_ENERGY_BOOT
//////////////////////////////////////// DEV/MAINT UI OPTIONS (and support for them)
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: enable a full OpenTRV CLI.
#define ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc.
#define ENABLE_FULL_OT_UI
// IF DEFINED: provide CLI read/write access to generic parameter block.
#define ENABLE_GENERIC_PARAM_CLI_ACCESS
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#undef ENABLE_EXTENDED_CLI
// IF DEFINED: physical UI use wakes CLI (not needed when CLI can auto-wake from serial).
#undef ENABLE_UI_WAKES_CLI
//////////////////////////////////////// DEVICE UI OPTIONS (and support for them)
// IF DEFINED: fast temp pot/dial sampling to partly compensate for less good mechanics (at some energy cost).
#define ENABLE_FAST_TEMP_POT_SAMPLING
// IF DEFINED: enable use of second UI LED if available.
#undef ENABLE_UI_LED_2_IF_AVAILABLE
// IF DEFINED: enabled frequent stats TX, eg every minute, for diagnostics.
#undef ENABLE_FREQUENT_STATS_TX
// IF DEFINED: the (>>8) value of this flag is the maximum JSON frame size allowed (bytes).
#undef ENABLE_JSON_STATS_LEN_CAP
// IF DEFINED: unconditionally suppress the "@" ID field (carrier supplies it or equiv) to save bandwidth.
#undef ENABLE_JSON_SUPPRESSED_ID
// IF DEFINED: unconditionally suppress the "+" ID field and aim for minimum JSON frame size, for poor/noisy comms channels.
// NOTE: minimising the JSON frame will overall *reduce* bandwidth efficiency and ability to diagnose TX problems.
#undef ENABLE_JSON_FRAME_MINIMISED
//////////////////////////////////////// SENSOR OPTIONS (and support for them)
// IF DEFINED: allow use of ambient light sensor.
#define ENABLE_AMBLIGHT_SENSOR
// IF DEFINED: use the temperature-setting potentiometer/dial if present.
#define ENABLE_TEMP_POT_IF_PRESENT
// Enable use of OneWire devices.
#undef ENABLE_MINIMAL_ONEWIRE_SUPPORT
// Enable use of DS18B20 as primary temp sensor.
#undef ENABLE_PRIMARY_TEMP_SENSOR_DS18B20
// IF DEFINED: enable use of additional (ie external) DS18B20 temp sensor(s).
#undef ENABLE_EXTERNAL_TEMP_SENSOR_DS18B20
//////////////////////////////////////// OCCUPANCY OPTIONS
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#define ENABLE_OCCUPANCY_SUPPORT
// IF DEFINED: detect occupancy based on ambient light, if available.
#define ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT
// IF DEFINED: detect occupancy based on voice detection, if available. This undefines learn button 2 to use GPIO as input.
#undef ENABLE_OCCUPANCY_DETECTION_FROM_VOICE
//////////////////////////////////////// RADIO OPTIONS
// IF DEFINED: enable (at least) a primary radio module; without, this unit has no radio comms.
#define ENABLE_RADIO_PRIMARY_MODULE
// IF DEFINED: had RFM23B as the primary radio module: default from REV1 to REV11.
#define ENABLE_RADIO_RFM23B
// IF DEFINED: make RFM23B the primary radio.
#define ENABLE_RADIO_PRIMARY_RFM23B
// IF DEFINED: enable a secondary (typically WAN-relay) radio module.
#undef ENABLE_RADIO_SECONDARY_MODULE
// IF DEFINED: enable a WAN-relay radio module, primarily to relay stats outbound.
#undef ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
// IF DEFINED: enable a 'null' radio module; can be used to simplify code for a radio-less unit.
#undef ENABLE_RADIO_NULL
// IF DEFINED: enable periodic secure beacon broadcast.
#undef ENABLE_SECURE_RADIO_BEACON
// IF DEFINED: allow non-secure OpenTRV secure frame RX (as of 2015/12): DISABLED BY DEFAULT.
#undef ENABLE_OTSECUREFRAME_INSECURE_RX_PERMITTED
// IF DEFINED: allow TX of local stats frames (should not affect relaying).
#define ENABLE_STATS_TX
// IF DEFINED: allow minimal binary format in addition to more generic one: ~400 bytes code cost.
#undef ENABLE_MINIMAL_STATS_TXRX
// IF DEFINED: forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#undef ENABLE_DEFAULT_ALWAYS_RX
// GENERIC
#define V0p2_REV 7
// IF DEFINED: simplified mode button behaviour: tapping button invokes BAKE, not mode cycling.
#define ENABLE_SIMPLIFIED_MODE_BAKE
// IF DEFINED: basic FROST/WARM temperatures are settable and stored in EEPROM.
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: support one on and one off time per day (possibly in conjunction with 'learn' button).
#undef ENABLE_SINGLETON_SCHEDULE
// IF DEFINED: initial direct motor drive design.
#define ENABLE_V1_DIRECT_MOTOR_DRIVE
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
#define ENABLE_AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF DEFINED: detect occupancy based on relative humidity, if available.
// DHD20160101: seems to still be set off spuriously by fast drop in temp when rad turns off (TODO-696).
#undef ENABLE_OCCUPANCY_DETECTION_FROM_RH
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: do not allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow radio listen/RX.
#undef ENABLE_RADIO_RX
// IF DEFINED: allow JSON stats frames.
#define ENABLE_JSON_OUTPUT
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#undef ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#define ENABLE_LOCAL_TRV
// IF DEFINED: there is run-time help available for the CLI.
#undef ENABLE_CLI_HELP
// IF DEFINED: reverse DORM1 motor with respect to very first samples.
#define ENABLE_DORM1_MOTOR_REVERSED
// IF DEFINED: try to trim memory (primarily RAM, also code/Flash) space used.
#define ENABLE_TRIMMED_MEMORY
// IF DEFINED: try to trim bandwidth as may be especially expensive/scarce.
#undef ENABLE_TRIMMED_BANDWIDTH
// If DEFINED: attempt proportional (rather than cruder, eg, on/off, control of TRV or other heat source).
#define ENABLE_PROPORTIONAL_VALVE_CONTROL
// IF DEFINED: allow periodic machine- and human- readable status report to serial, starting with "=".
#undef ENABLE_SERIAL_STATUS_REPORT
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: enable support for FS20 carrier for RX or TX.
#undef ENABLE_FS20_CARRIER_SUPPORT
// IF DEFINED: use FHT8V wireless radio module/valve.
#undef ENABLE_FHT8VSIMPLE
// IF DEFINED: enable support for FS20 carrier for TX specifically (to allow RX-only).
#undef ENABLE_FS20_CARRIER_SUPPORT_TX
// IF DEFINED: enable raw preamble injection/framing eg for FS20 over RFM23B.
#undef ENABLE_RFM23B_FS20_RAW_PREAMBLE
// IF DEFINED: enable support for FS20 encoding/decoding, eg to send to FHT8V.
#undef ENABLE_FS20_ENCODING_SUPPORT
// IF DEFINED: enable support for fast (>50kbps) packet-handling carrier (leading length byte).
#define ENABLE_FAST_FRAMED_CARRIER_SUPPORT
// IF DEFINED: enable OpenTRV secure frame encoding/decoding (as of 2015/12).
// DHD20160214: costs 5866 bytes to enable vs 3426 for FS20 support.
#define ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
// IF DEFINED: always allow some kind of stats TX, whatever the privacy settings.
// OK IN THIS CASE BECAUSE ALL COMMS SECURE.
#define ENABLE_ALWAYS_TX_ALL_STATS
// IF DEFINED: allow setting of ID from CLI to replace devices in situ; not recommended by default to avoid confusion.
#define ENABLE_ID_SET_FROM_CLI
// IF DEFINED: enable a CLI-settable setback lockout (hours/days) to establish a baseline before engaging energy saving setbacks.
#define ENABLE_SETBACK_LOCKOUT_COUNTDOWN


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

static constexpr uint8_t RFM22_PREAMBLE_BYTE = 0xaa; // Preamble byte for RFM22/23 reception.
static constexpr uint8_t RFM22_PREAMBLE_MIN_BYTES = 4; // Minimum number of preamble bytes for reception.
static constexpr uint8_t RFM22_PREAMBLE_BYTES = 5; // Recommended number of preamble bytes for reliable reception.
static constexpr uint8_t RFM22_SYNC_BYTE = 0xcc; // Sync-word trailing byte (with FHT8V primarily).
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


// Returns true if an unencrypted trailing static payload and similar (eg bare stats transmission) is permitted.
// True if the TX_ENABLE value is no higher than stTXmostUnsec.
// Some filtering may be required even if this is true.
#define enableTrailingStatsPayload() (true) // Always allow at least some stats to be TXed.
// Stub version for when RX not enabled.
extern OTRadioLink::OTMessageQueueHandlerNull messageQueue;

/////// CONTROL (EARLY, NOT DEPENDENT ON OTHER SENSORS)

// Radiator valve mode (FROST, WARM, BAKE).
extern OTRadValve::ValveMode valveMode;

// IF DEFINED: support for general timed and multi-input occupancy detection / use.
typedef OTV0P2BASE::PseudoSensorOccupancyTracker OccupancyTracker;
// Singleton implementation for entire node.
extern OccupancyTracker Occupancy;


////// SENSORS

// Sensor for supply (eg battery) voltage in millivolts.
// Singleton implementation/instance.
extern OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;

#ifdef ENABLE_TEMP_POT_IF_PRESENT
#if (V0p2_REV >= 2) && defined(TEMP_POT_AIN) // Only supported in REV2/3/4/7.
#define TEMP_POT_AVAILABLE
// Sensor for temperature potentiometer/dial UI control.
typedef OTV0P2BASE::SensorTemperaturePot
    <
    decltype(Occupancy), &Occupancy,
#if (V0p2_REV == 7)
    48, 296, // Correct for DORM1/TRV1 with embedded REV7.
    false // REV7 does not drive pot from IO_POWER_UP.
#elif defined(TEMP_POT_REVERSE)
    1023, 0
#endif
    > TempPot_t;
extern TempPot_t TempPot;
#endif
#endif // ENABLE_TEMP_POT_IF_PRESENT

// Sensor for ambient light level; 0 is dark, 255 is bright.
typedef OTV0P2BASE::SensorAmbientLight AmbientLight;
// Singleton implementation/instance.
extern AmbientLight AmbLight;

// Ambient/room temperature sensor, usually on main board.
extern OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.

// HUMIDITY_SENSOR_SUPPORT is defined if at least one humidity sensor has support compiled in.
// Simple implementations can assume that the sensor will be present if defined;
// more sophisticated implementations may wish to make run-time checks.
// If SHT21 support is enabled at compile-time then its humidity sensor may be used at run-time.
#define HUMIDITY_SENSOR_SUPPORT // Humidity sensing available.

// Singleton implementation/instance.
typedef OTV0P2BASE::HumiditySensorSHT21 RelHumidity_t;
extern RelHumidity_t RelHumidity;


/////// CONTROL

// Special setup for OpenTRV beyond generic hardware setup.
void setupOpenTRV();
// Main loop for OpenTRV radiator control.
void loopOpenTRV();

// Settings for room TRV.
typedef OTRadValve::DEFAULT_ValveControlParameters PARAMS;

// Choose which subtype to use depending on enabled settings and board type.
#if defined(TEMP_POT_AVAILABLE) // Eg REV2/REV7.
typedef OTRadValve::TempControlTempPot<decltype(TempPot), &TempPot, PARAMS, decltype(RelHumidity), &RelHumidity> TempControl_t;
#elif defined(ENABLE_SETTABLE_TARGET_TEMPERATURES) // Eg REV1.
typedef OTRadValve::TempControlSimpleEEPROMBacked<PARAMS> TempControl_t;
#else
// Dummy temperature control.
typedef OTRadValve::NULLTempControl TempControl_t;
#endif
#define TempControl_DEFINED
extern TempControl_t tempControl;

// FIXME
static constexpr bool enableDefaultAlwaysRX = false;
static constexpr bool enableRadioRX = false;
static constexpr bool allowGetMinBoilerOnMFromEEPROM = false;
extern OTRadValve::OTHubManager<enableDefaultAlwaysRX, enableRadioRX, allowGetMinBoilerOnMFromEEPROM> hubManager;

// Period in minutes for simple learned on-time; strictly positive (and less than 256).
#define LEARNED_ON_PERIOD_M 60
// Period in minutes for simple learned on-time with comfort bias; strictly positive (and less than 256).
// Defaults to twice LEARNED_ON_PERIOD_M.
// Should be no shorter/less than LEARNED_ON_PERIOD_M to avoid confusion.
#define LEARNED_ON_PERIOD_COMFORT_M (min(2*(LEARNED_ON_PERIOD_M),255))
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

#define ENABLE_MODELLED_RAD_VALVE
#define ENABLE_NOMINAL_RAD_VALVE
// Singleton implementation for entire node.
extern OTRadValve::ModelledRadValve NominalRadValve;

/////// STATS

// Singleton non-volatile stats store instance.
extern OTV0P2BASE::EEPROMByHourByteStats eeStats;

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
extern StatsU_t statsU;


// Mechanism to generate '=' stats line, if enabled.
#if defined(ENABLE_SERIAL_STATUS_REPORT)
typedef OTV0P2BASE::SystemStatsLine<
      decltype(valveMode), &valveMode,
#if defined(ENABLE_LOCAL_TRV)
      decltype(NominalRadValve), &NominalRadValve,
#else
      OTRadValve::AbstractRadValve, (OTRadValve::AbstractRadValve *)NULL,
#endif // defined(ENABLE_LOCAL_TRV)
      decltype(TemperatureC16), &TemperatureC16,
#if defined(HUMIDITY_SENSOR_SUPPORT)
      decltype(RelHumidity), &RelHumidity,
#else
      OTV0P2BASE::HumiditySensorBase, (OTV0P2BASE::HumiditySensorBase *)NULL,
#endif // defined(HUMIDITY_SENSOR_SUPPORT)
#ifdef ENABLE_AMBLIGHT_SENSOR
      decltype(AmbLight), &AmbLight,
#else
      OTV0P2BASE::SensorAmbientLight, (OTV0P2BASE::SensorAmbientLight *)NULL,
#endif
#ifdef ENABLE_OCCUPANCY_SUPPORT
      decltype(Occupancy), &Occupancy,
#else
      OTV0P2BASE::PseudoSensorOccupancyTracker, (OTV0P2BASE::PseudoSensorOccupancyTracker*)NULL,
#endif
      decltype(Scheduler), &Scheduler,
#if defined(ENABLE_JSON_OUTPUT) && !defined(ENABLE_TRIMMED_MEMORY)
      true // Enable JSON stats.
#else
      true // Disable JSON stats.
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
void bareStatsTX(bool doBinary = false);

////// UI

// Valve physical UI controller.
#define valveUI_DEFINED
  typedef OTRadValve::ModeButtonAndPotActuatorPhysicalUI valveUI_t;
extern valveUI_t valveUI;


// Suggested minimum buffer size for pollUI() to ensure maximum-sized commands can be received.
static constexpr uint8_t MAXIMUM_CLI_RESPONSE_CHARS = 1 + OTV0P2BASE::CLI::MAX_TYPICAL_CLI_BUFFER;
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

// DORM1/REV7 direct drive motor actuator.
#ifdef ENABLE_PROPORTIONAL_VALVE_CONTROL
static constexpr bool binaryOnlyValveControl = false;
#else
static constexpr bool binaryOnlyValveControl = true;
#endif
  static constexpr uint8_t m1 = MOTOR_DRIVE_ML;
  static constexpr uint8_t m2 = MOTOR_DRIVE_MR;
typedef OTRadValve::ValveMotorDirectV1<OTRadValve::ValveMotorDirectV1HardwareDriver, m1, m2, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN, OTRadValve::MOTOR_DRIVE_NSLEEP_UNUSED, decltype(Supply_cV), &Supply_cV> ValveDirect_t;

extern ValveDirect_t ValveDirect;


#endif

