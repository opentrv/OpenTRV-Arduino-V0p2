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

#include "Sensor.h"




// Sense (maybe non-linearly) over full likely internal ambient lighting range of a (UK) home,
// down to levels too dark to be active in (and at which heating could be set back for example).
// This suggests a full scale of at least 50--100 lux, maybe as high as 300 lux, eg see:
// http://home.wlv.ac.uk/~in6840/Lightinglevels.htm
// http://www.engineeringtoolbox.com/light-level-rooms-d_708.html
// http://www.pocklington-trust.org.uk/Resources/Thomas%20Pocklington/Documents/PDF/Research%20Publications/GPG5.pdf
// http://www.vishay.com/docs/84154/appnotesensors.pdf

#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION
// Sensor for ambient light level; 0 is dark, 255 is bright.
class AmbientLight : public SimpleTSUint8Sensor
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
  }
#endif
// Singleton implementation/instance.
extern AmbientLight AmbLight;










// Room/ambient temperature.

// Sensor for ambient/room temperature in 1/16th of one degree Celsius.
// An error may be indicated by returning a zero or (very) negative value.
class RoomTemperatureC16 : public Sensor<int>
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
    // This implies an actual precision of about 1/8th C.
    bool isLowPrecision() const { return(false); }
  };
// Singleton implementation/instance.
extern RoomTemperatureC16 TemperatureC16;








// High and low bounds on relative humidity for comfort and (eg) mite/mould growth.
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
class HumiditySensorSHT21 : public SimpleTSUint8Sensor
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






#if V0p2_REV >= 2 // Only supported in REV2 onwards.
#define TEMP_POT_AVAILABLE

// Maximum 'raw' temperature pot/dial value.
#define TEMP_POT_RAW_MAX 1023 // Max value.

// Sensor for temperature potentiometer/dial; 0 is coldest, 255 is hottest.
class TemperaturePot : public SimpleTSUint8Sensor
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
class VoiceDetection : public SimpleTSUint8Sensor
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
    bool isVoiceDelected() { return(isDetected); }
  };
// Singleton implementation/instance.
extern VoiceDetection Voice;
#endif


#endif
