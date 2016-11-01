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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2016
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
#define BAUD 4800 // Ensure that OpenTRV 'standard' UART speed is set unless explicitly overridden.
#endif

#include <Arduino.h>
#include <OTV0p2Base.h>
#include <OTRadioLink.h>
#include <OTRadValve.h>
#include "V0p2_Generic_Config.h" // Config switches and module dependencies.
#include <OTV0p2_Board_IO_Config.h> // I/O pin allocation and setup: include ahead of I/O module headers.


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


////// UI

// Valve physical UI controller.
#if defined(ENABLE_LOCAL_TRV) && !defined(NO_UI_SUPPORT)
#define valveUI_DEFINED
  #if defined(ENABLE_SIMPLIFIED_MODE_BAKE)
  typedef OTRadValve::ModeButtonAndPotActuatorPhysicalUI valveUI_t;
  #else
  typedef OTRadValve::CycleModeAndLearnButtonsAndPotActuatorPhysicalUI<BUTTON_MODE_L> valveUI_t;
  #endif
extern valveUI_t valveUI;
#endif // ENABLE_LOCAL_TRV && !NO_UI_SUPPORT

// Check/apply the user's schedule, at least once each minute, and act on any timed events.
#if defined(SCHEDULER_AVAILABLE)
void checkUserSchedule();
#else
#define checkUserSchedule() // Reduce to a no-op.
#endif // defined(SCHEDULER_AVAILABLE)

// Sends a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
// Should be similar to PICAXE V0.1 output to allow the same parser to handle either.
#ifdef ENABLE_SERIAL_STATUS_REPORT
void serialStatusReport();
#else
#define serialStatusReport() { }
#endif

// Reset CLI active timer to the full whack before it goes inactive again (ie makes CLI active for a while).
// Thread-safe.
void resetCLIActiveTimer();

// Returns true if the CLI is (or should currently be) active, at least intermittently.
// Thread-safe.
bool isCLIActive();

// Used to poll user side for CLI input until specified sub-cycle time.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// Times itself out after at least a minute or two of inactivity. 
// NOT RENTRANT (eg uses static state for speed and code space).
void pollCLI(uint8_t maxSCT, bool startOfMinute);


////// SENSORS

// Sensor for supply (eg battery) voltage in millivolts.
// Singleton implementation/instance.
extern OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;

#ifdef ENABLE_TEMP_POT_IF_PRESENT
#if (V0p2_REV >= 2) && defined(TEMP_POT_AIN) // Only supported in REV2/3/4/7.
#define TEMP_POT_AVAILABLE
// Sensor for temperature potentiometer/dial.
extern OTV0P2BASE::SensorTemperaturePot TempPot;
#endif
#endif // ENABLE_TEMP_POT_IF_PRESENT

// Sense ambient lighting level.
#ifdef ENABLE_AMBLIGHT_SENSOR
// Sensor for ambient light level; 0 is dark, 255 is bright.
typedef OTV0P2BASE::SensorAmbientLight AmbientLight;
#else // !defined(ENABLE_AMBLIGHT_SENSOR)
typedef OTV0P2BASE::DummySensorAmbientLight AmbientLight; // Dummy stand-in.
#endif // ENABLE_AMBLIGHT_SENSOR
// Singleton implementation/instance.
extern AmbientLight AmbLight;

// Create very light-weight standard-speed OneWire(TM) support if a pin has been allocated to it.
// Meant to be similar to use to OneWire library V2.2.
// Supports search but not necessarily CRC.
// Designed to work with 1MHz/1MIPS CPU clock.
#if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
#define SUPPORTS_MINIMAL_ONEWIRE
extern OTV0P2BASE::MinimalOneWire<> MinOW_DEFAULT_OWDQ;
#endif

// Cannot have internal and external use of same DS18B20 at same time...
#if defined(ENABLE_EXTERNAL_TEMP_SENSOR_DS18B20) && !defined(ENABLE_PRIMARY_TEMP_SENSOR_DS18B20) && defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
#define SENSOR_EXTERNAL_DS18B20_ENABLE_0 // Enable sensor zero.
extern OTV0P2BASE::TemperatureC16_DS18B20 extDS18B20_0;
#endif

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

// HUMIDITY_SENSOR_SUPPORT is defined if at least one humidity sensor has support compiled in.
// Simple implementations can assume that the sensor will be present if defined;
// more sophisticated implementations may wish to make run-time checks.
// If SHT21 support is enabled at compile-time then its humidity sensor may be used at run-time.
#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
#define HUMIDITY_SENSOR_SUPPORT // Humidity sensing available.
#endif

#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
// Singleton implementation/instance.
typedef OTV0P2BASE::HumiditySensorSHT21 RelHumidity_t;
extern RelHumidity_t RelHumidity;
#else
// Dummy implementation to minimise coding changes.
extern OTV0P2BASE::DummyHumiditySensorSHT21 RelHumidity;
#endif

#ifdef ENABLE_VOICE_SENSOR
extern OTV0P2BASE::VoiceDetectionQM1 Voice;
#endif

////////////////////////// Actuators

// DORM1/REV7 direct drive motor actuator.
#if /* defined(ENABLE_LOCAL_TRV) && */ defined(ENABLE_V1_DIRECT_MOTOR_DRIVE)
#define HAS_DORM1_VALVE_DRIVE
// Singleton implementation/instance.
#ifdef ENABLE_DORM1_MOTOR_REVERSED // Reversed vs sample 2015/12.
extern OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_ML, MOTOR_DRIVE_MR, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#else
extern OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_MR, MOTOR_DRIVE_ML, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#endif // HAS_DORM1_MOTOR_REVERSED
#endif

// Singleton FHT8V valve instance (to control remote FHT8V valve by radio).
#ifdef ENABLE_FHT8VSIMPLE
static const uint8_t _FHT8V_MAX_EXTRA_TRAILER_BYTES = (1 + max(OTV0P2BASE::MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES, OTV0P2BASE::FullStatsMessageCore_MAX_BYTES_ON_WIRE));
extern OTRadValve::FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V;
#if defined(ENABLE_LOCAL_TRV) || defined(ENABLE_SLAVE_TRV)
inline bool localFHT8VTRVEnabled() { return(!FHT8V.isUnavailable()); }
#else
#define localFHT8VTRVEnabled() (false) // Local FHT8V TRV disabled.
#endif
#endif // ENABLE_FHT8VSIMPLE


#endif

