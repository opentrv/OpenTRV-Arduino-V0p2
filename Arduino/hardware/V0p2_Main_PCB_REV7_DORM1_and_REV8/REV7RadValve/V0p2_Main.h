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

#define V0p2_REV 7


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

// Sensor for temperature potentiometer/dial UI control.
typedef OTV0P2BASE::SensorTemperaturePot
    <
    decltype(Occupancy), &Occupancy,
    48, 296, // Correct for DORM1/TRV1 with embedded REV7.
    false // REV7 does not drive pot from IO_POWER_UP.
    > TempPot_t;
extern TempPot_t TempPot;

// Sensor for ambient light level; 0 is dark, 255 is bright.
typedef OTV0P2BASE::SensorAmbientLight AmbientLight;
// Singleton implementation/instance.
extern AmbientLight AmbLight;

// Ambient/room temperature sensor, usually on main board.
extern OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.

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
typedef OTRadValve::TempControlTempPot<decltype(TempPot), &TempPot, PARAMS, decltype(RelHumidity), &RelHumidity> TempControl_t;
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
// Dummy scheduler to simplify coding.
typedef OTRadValve::NULLValveSchedule Scheduler_t;
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

// Do bare stats transmission.
// Output should be filtered for items appModelledRadValveComputeTargetTempBasicropriate
// to current channel security and sensitivity level.
// This may be binary or JSON format.
//   * allowDoubleTX  allow double TX to increase chance of successful reception
// Sends stats on primary radio channel 0 with possible duplicate to secondary channel.
// If sending encrypted then ID/counter fields (eg @ and + for JSON) are omitted
// as assumed supplied by security layer to remote recipent.
void bareStatsTX();

////// UI

// Valve physical UI controller.
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
static constexpr bool binaryOnlyValveControl = false;
static constexpr uint8_t m1 = MOTOR_DRIVE_ML;
static constexpr uint8_t m2 = MOTOR_DRIVE_MR;
typedef OTRadValve::ValveMotorDirectV1<OTRadValve::ValveMotorDirectV1HardwareDriver, m1, m2, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN, OTRadValve::MOTOR_DRIVE_NSLEEP_UNUSED, decltype(Supply_cV), &Supply_cV> ValveDirect_t;

extern ValveDirect_t ValveDirect;


#endif

