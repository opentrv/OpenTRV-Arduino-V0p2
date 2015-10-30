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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2015
*/

/*
 * Header for main on-board sensors and actuators for V0p2 variants.
 */

#ifndef V0P2_SENSORS_H
#define V0P2_SENSORS_H

#include <util/atomic.h>
#include <Arduino.h>
#include <OTV0p2Base.h>

#include "V0p2_Main.h"


// Create very light-weight standard-speed OneWire(TM) support if a pin has been allocated to it.
// Meant to be similar to use to OneWire library V2.2.
// Supports search but not necessarily CRC.
// Designed to work with 1MHz/1MIPS CPU clock.


#if defined(PIN_OW_DQ_DATA) && defined(SUPPORT_ONEWIRE)
#define SUPPORTS_MINIMAL_ONEWIRE
extern OTV0P2BASE::MinimalOneWire<PIN_OW_DQ_DATA> MinOW;
#endif



#if defined(SENSOR_EXTERNAL_DS18B20_ENABLE) // Needs defined(SUPPORTS_MINIMAL_ONEWIRE)
// External/off-board DS18B20 temperature sensor in nominal 1/16 C.
// Requires OneWire support.
// Will in future be templated on:
//   * the MinimalOneWire instance to use
//   * precision (9, 10, 11 or 12 bits, 12 for the full C/16 resolution),
//     noting that lower precision is faster,
//     and for example 1C will be 0x1X
//     with more bits of the final nibble defined for with higher precision
//   * enumeration order of this device on the OW bus,
//     with 0 (the default) being the first found by the usual deterministic scan
//   * whether the CRC should de checked for incoming data
//     to improve reliability on long connections at a code and CPU cost
// Multiple DS18B20s can nominally be supported on one or multiple OW buses.
// Not all template parameter combinations may be supported.
// Provides temperature as a signed int value with 0C == 0 at all precisions.
//template <template <class = float> class T> struct A 
//template <template <uint8_t DigitalPin> class MOW, uint8_t bitsAfterPoint = 4, uint8_t busOrder = 0>
class ExtTemperatureDS18B20C16 : public OTV0P2BASE::Sensor<int>
  {
  private:
    // Ordinal of this DS18B20 on the OW bus.
    // FIXME: not currently used.
    const uint8_t busOrder;

    // Precision [9,12].
    uint8_t precision;

    // Address of DS18B20 being used, else [0] == 0 if none found.
    uint8_t address[8];

    // True once initialised.
    bool initialised;

    // Current value in (shifted) C.      
    int value;

    // Initialise the device (if any) before first use.
    // Returns true iff successful.
    // Uses specified order DS18B20 found on bus.
    // May need to be reinitialised if precision changed.
    bool init();

  public:
    // Minimum supported precision, in bits, corresponding to 1/2 C resolution.
    static const uint8_t MIN_PRECISION = 9;
    // Maximum supported precision, in bits, corresponding to 1/16 C resolution.
    static const uint8_t MAX_PRECISION = 12;
    // Default precision; defaults to minimum for speed.
    static const uint8_t DEFAULT_PRECISION = MIN_PRECISION;

    // Error value returned if device unavailable or not yet read.
    // Negative and below minimum value that DS18B20 can return legitimately (-55C). 
    static const int INVALID_TEMP = -128 * 16; // Nominally -128C.

    // Create instance with given bus ordinal and precision.
    ExtTemperatureDS18B20C16(uint8_t _busOrder = 0, uint8_t _precision = DEFAULT_PRECISION) : busOrder(_busOrder), initialised(false), value(INVALID_TEMP)
      {
      // Coerce precision to be valid.
      precision = constrain(_precision, MIN_PRECISION, MAX_PRECISION);
      }

    // Get current precision in bits [9,12]; 9 gives 1/2C resolution, 12 gives 1/16C resolution.
    int getPrecisionBits() { return(precision); }
 
    // Force a read/poll of temperature and return the value sensed in nominal units of 1/16 C.
    // At sub-maximum precision lsbits will be zero or undefined.
    // Expensive/slow.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual int read();

    // Return last value fetched by read(); undefined before first read().
    // Fast.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual int get() const { return(value); }
  };

#define SENSOR_EXTERNAL_DS18B20_ENABLE_0 // Enable sensor zero.
extern ExtTemperatureDS18B20C16 extDS18B20_0;

#endif




// Sense (maybe non-linearly) over full likely internal ambient lighting range of a (UK) home,
// down to levels too dark to be active in (and at which heating could be set back for example).
// This suggests a full scale of at least 50--100 lux, maybe as high as 300 lux, eg see:
// http://home.wlv.ac.uk/~in6840/Lightinglevels.htm
// http://www.engineeringtoolbox.com/light-level-rooms-d_708.html
// http://www.pocklington-trust.org.uk/Resources/Thomas%20Pocklington/Documents/PDF/Research%20Publications/GPG5.pdf
// http://www.vishay.com/docs/84154/appnotesensors.pdf

#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION
// Sensor for ambient light level; 0 is dark, 255 is bright.
class AmbientLight : public OTV0P2BASE::SimpleTSUint8Sensor
  {
  private:
    // Raw ambient light value [0,1023] dark--light.
    uint16_t rawValue;

    // True iff room is lit well enough for activity.
    // Marked volatile for thread-safe (simple) lock-free access.
    volatile bool isRoomLitFlag;

    // Number of minutes (read() calls) that teh room has been continuously dark for [0,255].
    // Does not roll over from maximum value.
    // Reset to zero in light.
    uint8_t darkTicks;

  public:
    // Force a read/poll of the ambient light level and return the value sensed [0,1023] (dark to light).
    // Potentially expensive/slow.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual uint8_t read();

    // Preferred poll interval (in seconds); should bve called at constant rate, usually 1/60s.
    virtual uint8_t preferredPollInterval_s() const { return(60); }

    // Returns a suggested (JSON) tag/field/key name including units of get(); NULL means no recommended tag.
    // The lifetime of the pointed-to text must be at least that of the Sensor instance.
    virtual const char *tag() const { return("L"); }

    // Get raw ambient light value in range [0,1023].
    uint16_t getRaw() { return(rawValue); }

    // Returns true if room is lit enough for someone to be active.
    // False if unknown.
    // Thread-safe and usable within ISRs (Interrupt Service Routines).
    bool isRoomLit() const { return(isRoomLitFlag); }

    // Returns true if room is light enough for someone to be active.
    // False if unknown.
    // Thread-safe and usable within ISRs (Interrupt Service Routines).
    bool isRoomDark() const { return(!isRoomLitFlag); }

    // Get number of minutes (read() calls) that teh room has been continuously dark for [0,255].
    // Does not roll over from maximum value.
    // Reset to zero in light.
    uint8_t getDarkMinutes() { return(darkTicks); }

#ifdef UNIT_TESTS
    // Set new value(s) for unit test only.
    // Makes this more usable as a mock for testing other components.
    virtual void _TEST_set_multi_(uint16_t newRawValue, bool newRoomLitFlag, uint8_t newDarkTicks)
      { rawValue = newRawValue; value = newRawValue >> 2; isRoomLitFlag = newRoomLitFlag; darkTicks = newDarkTicks; }
#endif
  };
#else
// Placeholder namespace with dummy static status methods to reduce code complexity.
class AmbientLight
  {
  public:
    // Not available, so always returns false.
    static bool isAvailable() { return(false); }

    // Unknown, so always false.
    // Thread-safe and usable within ISRs (Interrupt Service Routines).
    static bool isRoomLit() { return(false); }

    // Unknown, so always false.
    // Thread-safe and usable within ISRs (Interrupt Service Routines).
    static bool isRoomDark() { return(false); }
  };
#endif
// Singleton implementation/instance.
extern AmbientLight AmbLight;








// Room/ambient temperature, usually from the on-board sensor.

// Sensor for ambient/room temperature in 1/16th of one degree Celsius.
// An error may be indicated by returning a zero or (very) negative value.
class RoomTemperatureC16 : public OTV0P2BASE::Sensor<int>
  {
  private:
    // Room temperature in 16*C, eg 1 is 1/16 C, 32 is 2C, -64 is -4C.
    int value;

  public:
    RoomTemperatureC16() : value(0) { }

    // Force a read/poll of room temperature and return the value sensed in units of 1/16 C.
    // Should be called at regular intervals (1/60s) if isJittery() is true.
    // Expensive/slow.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual int read();

    // Preferred poll interval (in seconds).
    // This should be called at a regular rate, usually 1/60, so make stats such as velocity measurement easier.
    virtual uint8_t preferredPollInterval_s() const { return(60); }

    // Return last value fetched by read(); undefined before first read().
    // Fast.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual int get() const { return(value); }

    // Returns a suggested (JSON) tag/field/key name including units of get(); NULL means no recommended tag.
    // The lifetime of the pointed-to text must be at least that of the Sensor instance.
    virtual const char *tag() const { return("T|C16"); }

    // Returns true if the underlying sensor precision (or accuracy) is coarser than 1/16th C.
    // If true this implies an actual precision of about 1/8th C.
#ifdef SENSOR_DS18B20_ENABLE
#define ROOM_TEMP_REDUCED_PRECISION
    bool isLowPrecision() const { return(true); } // Requests lower precision from DS18B20 to give an acceptable conversion time.
#else
    bool isLowPrecision() const { return(false); }
#endif
  };
// Singleton implementation/instance.
extern RoomTemperatureC16 TemperatureC16;








// High and low bounds on relative humidity for comfort and (eg) mite/mould growth.
// See http://www.cdc.gov/niosh/topics/indoorenv/temperature.html: "The EPA recommends maintaining indoor relative humidity between 30 and 60% to reduce mold growth [EPA 2012]."
#define HUMIDTY_HIGH_RHPC 70
#define HUMIDTY_LOW_RHPC 30
// Epsilon bounds (absolute % +/- around thresholds) for accuracy and hysteresis.
#define HUMIDITY_EPSILON_RHPC 5
#if ((HUMIDTY_HIGH_RHPC + HUMIDITY_EPSILON_RHPC) >= 100)
#error bad RH constants!
#endif
#if ((HUMIDTY_LOW_RHPC - HUMIDITY_EPSILON_RHPC) <= 0)
#error bad RH constants!
#endif

// HUMIDITY_SENSOR_SUPPORT is defined if at least one humdity sensor has support compiled in.
// Simple implementations can assume that the sensor will be present if defined;
// more sophisticated implementations may wish to make run-time checks.

// IF SHT21 support is enabled at compile-time then its humidity sensor may be used at run-time.
// There may be other alternatived
#if defined(SENSOR_SHT21_ENABLE)
#define HUMIDITY_SENSOR_SUPPORT // Humidity sensing available.
#endif

// If humidity is supported, provide definitions.
#if defined(SENSOR_SHT21_ENABLE)
// Functionality and code only enabled if SENSOR_SHT21_ENABLE is defined.
// Sensor for relative humidity percentage; 0 is dry, 100 is condensing humid, 255 for error.
class HumiditySensorSHT21 : public OTV0P2BASE::SimpleTSUint8Sensor
  {
  private:
    // True if RH% is high, with hysteresis.
    // Marked volatile for thread-safe lock-free access.
    volatile bool highWithHyst;

  public:
    HumiditySensorSHT21() : SimpleTSUint8Sensor(255), highWithHyst(false) { }

    // Force a read/poll of the relative humidity % and return the value sensed [0,100] (dry to wet).
    // Initially (and in case of error) the value 255 is returned as a fail-safe.
    // Potentially expensive/slow.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual uint8_t read();

    // Returns true if the sensor reading value passed is potentially valid, ie in range [0,100].
    virtual bool isValid(const uint8_t value) const { return(value <= 100); }

    // Returns a suggested (JSON) tag/field/key name including units of get(); NULL means no recommended tag.
    // The lifetime of the pointed-to text must be at least that of the Sensor instance.
    virtual const char *tag() const { return("H|%"); }

    // True if RH% high.
    // Thread-safe and usable within ISRs (Interrupt Service Routines).
    bool isRHHigh() { return(get() > (HUMIDTY_HIGH_RHPC+HUMIDITY_EPSILON_RHPC)); }

    // True if RH% high with a hysteresis band of 2 * HUMIDITY_EPSILON_RHPC.
    // Thread-safe and usable within ISRs (Interrupt Service Routines).
    bool isRHHighWithHyst() { return(highWithHyst); }
  };
#else
// Placeholder namespace with dummy static status methods to reduce code complexity.
class HumiditySensorSHT21
  {
  public:
    // Not available, so always returns false.
    // Thread-safe and usable within ISRs (Interrupt Service Routines).
    static bool isAvailable() { return(false); }

    // Unknown, so always false.
    // Thread-safe and usable within ISRs (Interrupt Service Routines).
    static bool isRHHigh() { return(false); }

    // Unknown, so always false.
    // Thread-safe and usable within ISRs (Interrupt Service Routines).
    static bool isRHHighWithHyst() { return(false); }
  };
#endif
// Singleton implementation/instance.
extern HumiditySensorSHT21 RelHumidity;






#if (V0p2_REV >= 2) && defined(TEMP_POT_AIN) // Only supported in REV2/3/4/7.
#define TEMP_POT_AVAILABLE

// Maximum 'raw' temperature pot/dial value.
#define TEMP_POT_RAW_MAX 1023 // Max value.

// Sensor for temperature potentiometer/dial; 0 is coldest, 255 is hottest.
class TemperaturePot : public OTV0P2BASE::SimpleTSUint8Sensor
  {
  private:
    // Raw pot value [0,1023] if extra precision is required.
    uint16_t raw;

  public:
    // Initialise to cautious values.
    TemperaturePot() : raw(0) { }

    // Force a read/poll of the temperature pot and return the value sensed [0,255] (cold to hot).
    // Potentially expensive/slow.
    // This value has some hysteresis applied to reduce noise.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual uint8_t read();

    // Return last value fetched by read(); undefined before first read()).
    // Fast.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    uint16_t getRaw() const { return(raw); }
  };
// Singleton implementation/instance.
extern TemperaturePot TempPot;
#endif






#ifdef ENABLE_VOICE_SENSOR
/*
 Voice sensor.

 EXPERIMENTAL

 Functionality and code only enabled if ENABLE_VOICE_SENSOR is defined.
 */
// Sensor for supply (eg battery) voltage in millivolts.
class VoiceDetection : public OTV0P2BASE::SimpleTSUint8Sensor
  {
  private:
    // Activity count.
    // Marked volatile for thread-safe (simple) lock-free access.
    volatile uint8_t count;
    // True if voice is detected.
    // Marked volatile for thread-safe lock-free access.
    volatile bool isDetected;
 
  public:
    // Initialise to cautious values.
    VoiceDetection() : count(0), isDetected(false) { }

    // Force a read/poll of the voice level and return the value sensed.
    // Potentially expensive/slow.
    // Thread-safe and usable within ISRs (Interrupt Service Routines), though not recommended.
    virtual uint8_t read();

    // Returns preferred poll interval (in seconds); non-zero.
    virtual uint8_t preferredPollInterval_s() const { return(60); }

    // Handle simple interrupt.
    // Fast and ISR (Interrupt Service Routines) safe.
    // Returns true if interrupt was successfully handled and cleared
    // else another interrupt handler in the chain may be called
    // to attempt to clear the interrupt.
    virtual bool handleInterruptSimple();

    // Returns true if voice has been detected in this or previous poll period.
    bool isVoiceDetected() { return(isDetected); }
  };
// Singleton implementation/instance.
extern VoiceDetection Voice;
#endif


#endif
