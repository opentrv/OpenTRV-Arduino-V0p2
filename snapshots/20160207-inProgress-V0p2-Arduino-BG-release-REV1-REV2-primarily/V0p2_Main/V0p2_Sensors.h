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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2016
                           Deniz Erbilgin 2015
*/

/*
 * Header for common on-board and external sensors and actuators for V0p2 variants.
 */

#ifndef V0P2_SENSORS_H
#define V0P2_SENSORS_H

#include <util/atomic.h>
#include <Arduino.h>
#include <OTV0p2Base.h>
#include <OTRadioLink.h>
#include <OTRadValve.h>

#include "V0p2_Main.h"

#include "Messaging.h"


// Sensor for supply (eg battery) voltage in millivolts.
// Singleton implementation/instance.
extern OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;


#ifdef ENABLE_TEMP_POT_IF_PRESENT
#if (V0p2_REV >= 2) && defined(TEMP_POT_AIN) // Only supported in REV2/3/4/7.
#define TEMP_POT_AVAILABLE
// Sensor for temperature potentiometer/dial; 0 is coldest, 255 is hottest.
// Note that if the callbacks are enabled, the following are implemented:
//   * Any operation of the pot calls the occupancy/"UI used" callback.
//   * Force FROST mode when dial turned right down to bottom.
//   * Start BAKE mode when dial turned right up to top.
//   * Cancel BAKE mode when dial/temperature turned down.
//   * Force WARM mode when dial/temperature turned up.
// Singleton implementation/instance.
extern OTV0P2BASE::SensorTemperaturePot TempPot;
#endif
#endif // ENABLE_TEMP_POT_IF_PRESENT


// Sense (usually non-linearly) over full likely internal ambient lighting range of a (UK) home,
// down to levels too dark to be active in (and at which heating could be set back for example).
// This suggests a full scale of at least 50--100 lux, maybe as high as 300 lux, eg see:
// http://home.wlv.ac.uk/~in6840/Lightinglevels.htm
// http://www.engineeringtoolbox.com/light-level-rooms-d_708.html
// http://www.pocklington-trust.org.uk/Resources/Thomas%20Pocklington/Documents/PDF/Research%20Publications/GPG5.pdf
// http://www.vishay.com/docs/84154/appnotesensors.pdf
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
extern OTV0P2BASE::HumiditySensorSHT21 RelHumidity;
#else
// Dummy implementation to minimise coding changes.
extern OTV0P2BASE::DummyHumiditySensorSHT21 RelHumidity;
#endif


#ifdef ENABLE_VOICE_SENSOR
// TODO
extern OTV0P2BASE::VoiceDetectionQM1 Voice;
#endif


////////////////////////// Actuators


// DORM1/REV7 direct drive motor actuator.
#if /* defined(ENABLE_LOCAL_TRV) && */ defined(DIRECT_MOTOR_DRIVE_V1)
#define HAS_DORM1_VALVE_DRIVE
// Singleton implementation/instance.
#ifdef ENABLE_DORM1_MOTOR_REVERSED // Reversed vs sample 2015/12
extern OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_ML, MOTOR_DRIVE_MR, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#else
extern OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_MR, MOTOR_DRIVE_ML, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#endif // HAS_DORM1_MOTOR_REVERSED
#endif


// FHT8V radio-controlled actuator.
#ifdef ENABLE_FHT8VSIMPLE
// Singleton FHT8V valve instance (to control remote FHT8V valve by radio).
static const uint8_t _FHT8V_MAX_EXTRA_TRAILER_BYTES = (1 + max(OTV0P2BASE::MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES, OTV0P2BASE::FullStatsMessageCore_MAX_BYTES_ON_WIRE));
extern OTRadValve::FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V;
// This unit may control a local TRV.
// Returns TRV if valve/radiator is to be controlled by this unit.
// Usually the case, but may not be for (a) a hub or (b) a not-yet-configured unit.
// Returns false if house code parts are set to invalid or uninitialised values (>99).
#if defined(ENABLE_LOCAL_TRV) || defined(ENABLE_SLAVE_TRV)
inline bool localFHT8VTRVEnabled() { return(!FHT8V.isUnavailable()); }
#else
#define localFHT8VTRVEnabled() (false) // Local FHT8V TRV disabled.
#endif
// Clear both housecode parts (and thus disable local valve).
void FHT8VClearHC();
// Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
// Will cache in FHT8V instance for speed.
void FHT8VSetHC1(uint8_t hc);
void FHT8VSetHC2(uint8_t hc);
// Get (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control (will be 0xff until set).
// Used FHT8V instance as a transparent cache of the values for speed.
uint8_t FHT8VGetHC1();
uint8_t FHT8VGetHC2();
inline uint16_t FHT8VGetHC() { return(FHT8VGetHC2() | (((uint16_t) FHT8VGetHC1()) << 8)); }
// Load EEPROM house codes into primary FHT8V instance at start-up or once cleared in FHT8V instance.
void FHT8VLoadHCFromEEPROM();
#endif // ENABLE_FHT8VSIMPLE


#endif
