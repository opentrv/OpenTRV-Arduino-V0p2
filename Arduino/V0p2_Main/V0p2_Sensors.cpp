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
                           John Harvey 2014 (DS18B20 code)
                           Deniz Erbilgin 2015--2016
*/

/*
 * Header for common on-board and external sensors and actuators for V0p2 variants.
 */
#include <stdint.h>
#include <limits.h>
#include <util/atomic.h>

#include <Wire.h> // Arduino I2C library.
#include <OTV0p2Base.h>

#include "V0p2_Main.h"
#include <OTV0p2_Board_IO_Config.h> // I/O pin allocation and setup: include ahead of I/O module headers.
#include "V0p2_Sensors.h" // I/O code access.
#include "Control.h"
#include "UI_Minimal.h"


// Sensor for supply (eg battery) voltage in millivolts.
// Singleton implementation/instance.
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;


#ifdef TEMP_POT_AVAILABLE
// Singleton implementation/instance.
#if defined(TEMP_POT_REVERSE)
OTV0P2BASE::SensorTemperaturePot TempPot(OTV0P2BASE::SensorTemperaturePot::TEMP_POT_RAW_MAX, 0);
#else
#if (V0p2_REV != 7) // For DORM1/REV7 natural direction for temp dial pot is correct.
OTV0P2BASE::SensorTemperaturePot TempPot(0, OTV0P2BASE::SensorTemperaturePot::TEMP_POT_RAW_MAX);
#else
// DORM1 / REV7 initial unit range ~[45,293] DHD20160211 (seen <45 to >325).
// Thus could be ~30 points per item on scale: * 16 17 18 >19< 20 21 22 BOOST
// Actual precision/reproducability of pot is circa +/- 4.
static const uint16_t REV7_pot_low = 48;
static const uint16_t REV7_pot_high = 296;
OTV0P2BASE::SensorTemperaturePot TempPot(REV7_pot_low, REV7_pot_high);
#endif // (V0p2_REV != 7)
#endif // defined(TEMP_POT_REVERSE)
#endif


#ifdef ENABLE_AMBLIGHT_SENSOR
// Normal 2 bit shift between raw and externally-presented values.
static const uint8_t shiftRawScaleTo8Bit = 2;
#ifdef ENABLE_AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// This implementation expects a phototransitor TEPT4400 (50nA dark current, nominal 200uA@100lx@Vce=50V) from IO_POWER_UP to LDR_SENSOR_AIN and 220k to ground.
// Measurement should be taken wrt to internal fixed 1.1V bandgap reference, since light indication is current flow across a fixed resistor.
// Aiming for maximum reading at or above 100--300lx, ie decent domestic internal lighting.
// Note that phototransistor is likely far more directionally-sensitive than LDR and its response nearly linear.
// This extends the dynamic range and switches to measurement vs supply when full-scale against bandgap ref, then scales by Vss/Vbandgap and compresses to fit.
// http://home.wlv.ac.uk/~in6840/Lightinglevels.htm
// http://www.engineeringtoolbox.com/light-level-rooms-d_708.html
// http://www.pocklington-trust.org.uk/Resources/Thomas%20Pocklington/Documents/PDF/Research%20Publications/GPG5.pdf
// http://www.vishay.com/docs/84154/appnotesensors.pdf
#if (7 == V0p2_REV) // REV7 initial board run especially uses different phototransistor (not TEPT4400).
// Note that some REV7s from initial batch were fitted with wrong device entirely,
// an IR device, with very low effective sensitivity (FSD ~ 20 rather than 1023).
static const int LDR_THR_LOW = 180U;
static const int LDR_THR_HIGH = 250U;
#else // REV4 default values.
static const int LDR_THR_LOW = 270U;
static const int LDR_THR_HIGH = 400U;
#endif
#else // LDR (!defined(ENABLE_AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400))
// This implementation expects an LDR (1M dark resistance) from IO_POWER_UP to LDR_SENSOR_AIN and 100k to ground.
// Measurement should be taken wrt to supply voltage, since light indication is a fraction of that.
// Values below from PICAXE V0.09 impl approx multiplied by 4+ to allow for scale change.
#ifdef ENABLE_AMBLIGHT_EXTRA_SENSITIVE // Define if LDR not exposed to much light, eg for REV2 cut4 sideways-pointing LDR (TODO-209).
static const int LDR_THR_LOW = 50U;
static const int LDR_THR_HIGH = 70U;
#else // Normal settings.
static const int LDR_THR_LOW = 160U; // Was 30.
static const int LDR_THR_HIGH = 200U; // Was 35.
#endif // ENABLE_AMBLIGHT_EXTRA_SENSITIVE
#endif // ENABLE_AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// Singleton implementation/instance.
AmbientLight AmbLight(LDR_THR_HIGH >> shiftRawScaleTo8Bit);
#endif // ENABLE_AMBLIGHT_SENSOR


#if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
OTV0P2BASE::MinimalOneWire<> MinOW_DEFAULT;
#endif


#if defined(SENSOR_EXTERNAL_DS18B20_ENABLE_0) // Enable sensor zero.
OTV0P2BASE::TemperatureC16_DS18B20 extDS18B20_0(MinOW_DEFAULT, 0);
#endif


#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
// Singleton implementation/instance.
OTV0P2BASE::HumiditySensorSHT21 RelHumidity;
#else
OTV0P2BASE::DummyHumiditySensorSHT21 RelHumidity;
#endif


// Ambient/room temperature sensor, usually on main board.
#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.
#elif defined(ENABLE_PRIMARY_TEMP_SENSOR_DS18B20)
#if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
// DSB18B20 temperature impl, with slightly reduced precision to improve speed.
OTV0P2BASE::TemperatureC16_DS18B20 TemperatureC16(MinOW_DEFAULT, OTV0P2BASE::TemperatureC16_DS18B20::MAX_PRECISION - 1);
#endif
#else // Don't use TMP112 if SHT21 or DS18B20 are selected.
OTV0P2BASE::RoomTemperatureC16_TMP112 TemperatureC16;
#endif


#ifdef ENABLE_VOICE_SENSOR
// TODO
OTV0P2BASE::VoiceDetectionQM1 Voice;
#endif


////////////////////////// Actuators


// DORM1/REV7 direct drive actuator.
#ifdef HAS_DORM1_VALVE_DRIVE
//#ifdef ENABLE_V1_DIRECT_MOTOR_DRIVE
// Singleton implementation/instance.
#ifdef ENABLE_DORM1_MOTOR_REVERSED // Reversed vs sample 2015/12.
OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_ML, MOTOR_DRIVE_MR, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#else
OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_MR, MOTOR_DRIVE_ML, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#endif // HAS_DORM1_MOTOR_REVERSED
#endif


// FHT8V radio-controlled actuator.
#ifdef ENABLE_FHT8VSIMPLE
// Function to append stats trailer (and 0xff) to FHT8V/FS20 TX buffer.
// Assume enough space in buffer for largest possible stats message.
#if defined(ENABLE_STATS_TX)
uint8_t *appendStatsToTXBufferWithFF(uint8_t *bptr, const uint8_t bufSize)
{
  OTV0P2BASE::FullStatsMessageCore_t trailer;
  populateCoreStats(&trailer);
  // Ensure that no ID is encoded in the message sent on the air since it would be a repeat from the FHT8V frame.
  trailer.containsID = false;

#if defined(ENABLE_MINIMAL_STATS_TXRX)
  // As bandwidth optimisation just write minimal trailer if only temp&power available.
  if (trailer.containsTempAndPower &&
      !trailer.containsID && !trailer.containsAmbL)
  {
    writeTrailingMinimalStatsPayload(bptr, &(trailer.tempAndPower));
    bptr += 3;
    *bptr = (uint8_t)0xff; // Terminate TX bytes.
  }
  else
#endif
  {
    // Assume enough space in buffer for largest possible stats message.
    bptr = encodeFullStatsMessageCore(bptr, bufSize, OTV0P2BASE::getStatsTXLevel(), false, &trailer);
  }
  return (bptr);
}
#else
#define appendStatsToTXBufferWithFF NULL // Do not append stats.
#endif
#endif // ENABLE_FHT8VSIMPLE

#ifdef ENABLE_FHT8VSIMPLE
OTRadValve::FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V(appendStatsToTXBufferWithFF);
#endif // ENABLE_FHT8VSIMPLE

