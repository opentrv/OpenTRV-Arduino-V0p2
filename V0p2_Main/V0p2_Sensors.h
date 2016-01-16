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
 * Header for main on-board sensors and actuators for V0p2 variants.
 */

#ifndef V0P2_SENSORS_H
#define V0P2_SENSORS_H

#include <util/atomic.h>
#include <Arduino.h>
#include <OTV0p2Base.h>

#include "V0p2_Main.h"

#include <OTRadioLink.h>
#include <OTRadValve.h>

#include "Messaging.h"


// Create very light-weight standard-speed OneWire(TM) support if a pin has been allocated to it.
// Meant to be similar to use to OneWire library V2.2.
// Supports search but not necessarily CRC.
// Designed to work with 1MHz/1MIPS CPU clock.


#if defined(PIN_OW_DQ_DATA) && defined(SUPPORT_ONEWIRE)
#define SUPPORTS_MINIMAL_ONEWIRE
extern OTV0P2BASE::MinimalOneWire<PIN_OW_DQ_DATA> MinOW;
#endif


// Sensor for supply (eg battery) voltage in millivolts.
// Singleton implementation/instance.
extern OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;


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



// Sense (usually non-linearly) over full likely internal ambient lighting range of a (UK) home,
// down to levels too dark to be active in (and at which heating could be set back for example).
// This suggests a full scale of at least 50--100 lux, maybe as high as 300 lux, eg see:
// http://home.wlv.ac.uk/~in6840/Lightinglevels.htm
// http://www.engineeringtoolbox.com/light-level-rooms-d_708.html
// http://www.pocklington-trust.org.uk/Resources/Thomas%20Pocklington/Documents/PDF/Research%20Publications/GPG5.pdf
// http://www.vishay.com/docs/84154/appnotesensors.pdf

#ifdef ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT
// Sensor for ambient light level; 0 is dark, 255 is bright.
typedef OTV0P2BASE::SensorAmbientLight AmbientLight;
#else // !defined(ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT)
typedef OTV0P2BASE::DummySensorAmbientLight AmbientLight; // Dummy stand-in.
#endif // ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT
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

// If RH% rises by at least this per hour, then it may indicate occupancy.
#define HUMIDITY_OCCUPANCY_PC_MIN_RISE_PER_H 3

// HUMIDITY_SENSOR_SUPPORT is defined if at least one humdity sensor has support compiled in.
// Simple implementations can assume that the sensor will be present if defined;
// more sophisticated implementations may wish to make run-time checks.

// If SHT21 support is enabled at compile-time then its humidity sensor may be used at run-time.
#if defined(ENABLE_SENSOR_SHT21)
#define HUMIDITY_SENSOR_SUPPORT // Humidity sensing available.
#endif

// If humidity is supported, provide definitions.
#if defined(ENABLE_SENSOR_SHT21)
// Functionality and code only enabled if ENABLE_SENSOR_SHT21 is defined.
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
class TemperaturePot : public OTV0P2BASE::SimpleTSUint8Sensor
  {
  public:
    // Maximum 'raw' temperature pot/dial value.
    static const uint16_t TEMP_POT_RAW_MAX = 1023;

  private:
    // Raw pot value [0,1023] if extra precision is required.
    uint16_t raw;

    // Occupancy callback function (for good confidence of human presence); NULL if not used.
    // Also indicates that the manual UI has been used.
    // If not NULL, is called when this sensor detects indications of occupancy.
    void (*occCallback)();

    // WARM/FROST and BAKE start/cancel callbacks.
    // If not NULL, are called when the pot is adjusted appropriately.
    // Typically at most one of these callbacks would be made on any approriate pot adjustment.
    void (*warmModeCallback)(bool);
    void (*bakeStartCallback)(bool);

  public:
    // Initialise raw to distinct/special value and all pointers to NULL.
    TemperaturePot(uint16_t minExpected_ = 0, uint16_t maxExpected_ = TEMP_POT_RAW_MAX)
      : raw(~0U),
        occCallback(NULL), warmModeCallback(NULL), bakeStartCallback(NULL),
        minExpected(minExpected_), maxExpected(maxExpected_)
      { }

    // Lower and upper bounds of expected pot movement/output each in range [0,TEMP_POT_RAW_MAX].
    // The values must be different and further apart at least than the noise threshold (~8).
    // Max is lower than min if the pot value is to be reversed.
    // Conservative values (ie with actual travel outside the specified range) should be specified
    // if end-stop detection is to work (callbacks on hitting the extremes).
    // The output is not rebased based on these values, though is reversed if necessary;
    // whatever uses the pot output should map to the desired values.
    // These read-only values are exposed to assist with any such mapping.
    const uint16_t minExpected, maxExpected;
    // Returns true if the pot output is to be reversed from the natural direction.
    inline bool isReversed() { return(minExpected > maxExpected); }

    // Force a read/poll of the temperature pot and return the value sensed [0,255] (cold to hot).
    // Potentially expensive/slow.
    // This value has some hysteresis applied to reduce noise.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual uint8_t read();

    // Set occupancy callback function (for good confidence of human presence); NULL for no callback.
    // Also indicates that the manual UI has been used.
    void setOccCallback(void (*occCallback_)()) { occCallback = occCallback_; }

    // Set WARM/FROST and BAKE start/cancel callbacks.
    // If not NULL, are called when the pot is adjusted appropriately.
    // Typically at most one of these callbacks would be made on any approriate pot adjustment.
    void setWFBCallbacks(void (*warmModeCallback_)(bool), void (*bakeStartCallback_)(bool))
      { warmModeCallback = warmModeCallback_; bakeStartCallback = bakeStartCallback_; }

    // Return last value fetched by read(); undefined before first read()).
    // Fast.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    uint16_t getRaw() const { return(raw); }
  };
// Singleton implementation/instance.
extern TemperaturePot TempPot;
#endif
#endif // ENABLE_TEMP_POT_IF_PRESENT





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
    // Last time sensor was polled
    // Marked volatile for thread-safe (simple) lock-free access.
//    volatile uint16_t endOfLocking;
//    // True if there is new data to poll
//    // Marked volatile for thread-safe (simple) lock-free access.
//    volatile bool isTriggered;
//    // Lock out time after interrupt
//    // only needs to be > 10secs, but go for between 2 mins to make sure (we have a 4 min cycle anyway)
//    static const uint8_t lockingPeriod = 2;

 
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

    // Returns true if more than a minute has passed since last interrupt and sensor has not been polled.
//    bool isVoiceReady() { return (isTriggered && (OTV0P2BASE::getMinutesSinceMidnightLT() >= endOfLocking)); }

    // Returns a suggested (JSON) tag/field/key name including units of get(); NULL means no recommended tag.
    // The lifetime of the pointed-to text must be at least that of the Sensor instance.
    virtual const char *tag() const { return("av"); }

  };
// Singleton implementation/instance.
extern VoiceDetection Voice;
#endif


////////////////////////// Actuators


// DORM1/REV7 direct drive motor actuator.
#if /* defined(LOCAL_TRV) && */ defined(DIRECT_MOTOR_DRIVE_V1)
#define HAS_DORM1_VALVE_DRIVE
// Singleton implementation/instance.
#ifdef ENABLE_DORM1_MOTOR_REVERSED // Reversed vs sample 2015/12
extern OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_ML, MOTOR_DRIVE_MR, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#else
extern OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_MR, MOTOR_DRIVE_ML, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#endif // HAS_DORM1_MOTOR_REVERSED
#endif


// FHT8V radio-controlled actuator.
#ifdef USE_MODULE_FHT8VSIMPLE
// Singleton FHT8V valve instance (to control remote FHT8V valve by radio).
static const uint8_t _FHT8V_MAX_EXTRA_TRAILER_BYTES = (1 + max(MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES, FullStatsMessageCore_MAX_BYTES_ON_WIRE));
extern OTRadValve::FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V;
// This unit may control a local TRV.
// Returns TRV if valve/radiator is to be controlled by this unit.
// Usually the case, but may not be for (a) a hub or (b) a not-yet-configured unit.
// Returns false if house code parts are set to invalid or uninitialised values (>99).
#if defined(LOCAL_TRV) || defined(SLAVE_TRV)
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
#endif // USE_MODULE_FHT8VSIMPLE


//#if defined(ENABLE_BOILER_HUB)
//// Singleton implementation/instance.
//extern BoilerDriver BoilerControl;
//#endif




#endif
