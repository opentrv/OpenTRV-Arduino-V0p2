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
 V0p2 boards physical sensor support.
 */
#include <stdint.h>
#include <limits.h>
#include <util/atomic.h>

#include <Wire.h> // Arduino I2C library.
#include <OTV0p2Base.h>

#include "V0p2_Main.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "V0p2_Sensors.h" // I/O code access.
#include "Control.h"
#include "UI_Minimal.h"

#if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
OTV0P2BASE::MinimalOneWire<> MinOW_DEFAULT;
#endif

// Singleton implementation/instance.
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;


#ifdef TEMP_POT_AVAILABLE
// Singleton implementation/instance.
#if defined(TEMP_POT_REVERSE)
OTV0P2BASE::SensorTemperaturePot TempPot(OTV0P2BASE::SensorTemperaturePot::TEMP_POT_RAW_MAX, 0);
#else
OTV0P2BASE::SensorTemperaturePot TempPot(0, OTV0P2BASE::SensorTemperaturePot::TEMP_POT_RAW_MAX);
#endif // defined(TEMP_POT_REVERSE)
#endif


#ifdef ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT

// Normal 2 bit shift between raw and externally-presented values.
static const uint8_t shiftRawScaleTo8Bit = 2;

//// Note on: phototransistor variant.
//// Note that if AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400 is defined
//// This expects a current-response phototransistor in place of the LDR
//// with roughly full-scale value in full light against internal 1.1V reference
//// not against supply voltage as usual.
//
#ifdef AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
//#define ALREFERENCE INTERNAL // Internal 1.1V reference.
//
//#ifdef AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400_WRONG_WAY // Schematic error in one board!
//#define ALREFERENCE DEFAULT // Supply voltage as reference for REV9 cut1.  HACK HACK!
//#endif // AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400_WRONG_WAY
//
//// If defined, then allow adaptive compression of top part of range when would otherwise max out.
//// This may be somewhat supply-voltage dependent, eg capped by the supply voltage.
//// Supply voltage is expected to be 2--3 times the bandgap reference, typically.
////#define ADAPTIVE_THRESHOLD 896U // (1024-128) Top ~10%, companding by 8x.
////#define ADAPTIVE_THRESHOLD 683U // (1024-683) Top ~33%, companding by 4x.
////static const uint16_t scaleFactor = (extendedScale - ADAPTIVE_THRESHOLD) / (normalScale - ADAPTIVE_THRESHOLD);
//// Assuming typical V supply of 2--3 times Vbandgap,
//// compress above threshold to extend top of range by a factor of two.
//// Ensure that scale stays monotonic in face of calculation lumpiness, etc...
//// Scale all remaining space above threshold to new top value into remaining space.
//// Ensure scaleFactor is a power of two for speed.
//
//#ifndef ENABLE_AMBLIGHT_EXTRA_SENSITIVE 
//// Don't extend the dynamic range of this ambient light sensor
//// if the photosensor is badly located or otherwise needs to be made more sensitive.
////
//// IF DEFINED: extend optosensor range as far as possible at the expense of loss of linearity.
//#define EXTEND_OPTO_SENSOR_RANGE
//// Switch to measuring against supply voltage to maximise range.
//// Will be severely non-linear at top end
//// and will change with battery voltage at bottom end,
//// but rely on dynamic adaptation to deal with some of that.
//// Note that with low supply voltage headroom there may not be much linear range if any.
//#undef ALREFERENCE
//#define ALREFERENCE DEFAULT
////static const uint16_t extendedScale = 2048;
////static const uint8_t shiftExtendedToRawScale = 1;
//#endif
//
// This implementation expects a phototransitor TEPT4400 (50nA dark current, nominal 200uA@100lx@Vce=50V) from IO_POWER_UP to LDR_SENSOR_AIN and 220k to ground.
// Measurement should be taken wrt to internal fixed 1.1V bandgap reference, since light indication is current flow across a fixed resistor.
// Aiming for maximum reading at or above 100--300lx, ie decent domestic internal lighting.
// Note that phototransistor is likely far more directionally-sensitive than LDR and its response nearly linear.
// This extends the dynamic range and switches to measurement vs supply when full-scale against bandgap ref, then scales by Vss/Vbandgap and compresses to fit.
// http://home.wlv.ac.uk/~in6840/Lightinglevels.htm
// http://www.engineeringtoolbox.com/light-level-rooms-d_708.html
// http://www.pocklington-trust.org.uk/Resources/Thomas%20Pocklington/Documents/PDF/Research%20Publications/GPG5.pdf
// http://www.vishay.com/docs/84154/appnotesensors.pdf
//
#if (7 == V0p2_REV) // REV7 initial board run especially uses different phototransistor (not TEPT4400).
// Note that some REV7s from initial batch were fitted with wrong device entirely,
// an IR device, with very low effective sensitivity (FSD ~ 20 rather than 1023).
static const int LDR_THR_LOW = 180U;
static const int LDR_THR_HIGH = 250U;
#else // REV4 default values.
static const int LDR_THR_LOW = 270U;
static const int LDR_THR_HIGH = 400U;
#endif

#else // LDR (!defined(AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400))

// This implementation expects an LDR (1M dark resistance) from IO_POWER_UP to LDR_SENSOR_AIN and 100k to ground.
// Measurement should be taken wrt to supply voltage, since light indication is a fraction of that.
// Values below from PICAXE V0.09 impl approx multiplied by 4+ to allow for scale change.
#define ALREFERENCE DEFAULT // Supply voltage as reference.

#ifdef ENABLE_AMBLIGHT_EXTRA_SENSITIVE // Define if LDR not exposed to much light, eg for REV2 cut4 sideways-pointing LDR (TODO-209).
static const int LDR_THR_LOW = 50U;
static const int LDR_THR_HIGH = 70U; 
#else // Normal settings.
static const int LDR_THR_LOW = 160U; // Was 30.
static const int LDR_THR_HIGH = 200U; // Was 35.
#endif // ENABLE_AMBLIGHT_EXTRA_SENSITIVE

#endif // AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400


// Singleton implementation/instance.
AmbientLight AmbLight(LDR_THR_HIGH >> shiftRawScaleTo8Bit);
#endif // ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT











// TMP102 and TMP112 should be interchangeable: latter has better guaranteed accuracy.
#define TMP102_I2C_ADDR 72
#define TMP102_REG_TEMP 0 // Temperature register.
#define TMP102_REG_CTRL 1 // Control register.
#define TMP102_CTRL_B1 0x31 // Byte 1 for control register: 12-bit resolution and shutdown mode (SD).
#define TMP102_CTRL_B1_OS 0x80 // Control register: one-shot flag in byte 1.
#define TMP102_CTRL_B2 0x0 // Byte 2 for control register: 0.25Hz conversion rate and not extended mode (EM).

#if !defined(ENABLE_SENSOR_SHT21) && !defined(SENSOR_DS18B20_ENABLE) // Don't use TMP112 if SHT21 or DS18B20 are available.
// Measure/store/return the current room ambient temperature in units of 1/16th C.
// This may contain up to 4 bits of information to the right of the fixed binary point.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
// This will simulate a zero temperature in case of detected error talking to the sensor as fail-safe for this use.
// Check for errors at certain critical places, not everywhere.
static int16_t TMP112_readTemperatureC16()
  {
  const bool neededPowerUp = OTV0P2BASE::powerUpTWIIfDisabled();
  
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("TMP102 needed power-up: ");
  DEBUG_SERIAL_PRINT(neededPowerUp);
  DEBUG_SERIAL_PRINTLN();
#endif

  // Force start of new one-shot temperature measurement/conversion to complete.
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write((byte) TMP102_REG_CTRL); // Select control register.
  Wire.write((byte) TMP102_CTRL_B1); // Clear OS bit.
  //Wire.write((byte) TMP102_CTRL_B2);
  Wire.endTransmission();
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write((byte) TMP102_REG_CTRL); // Select control register.
  Wire.write((byte) TMP102_CTRL_B1 | TMP102_CTRL_B1_OS); // Start one-shot conversion.
  //Wire.write((byte) TMP102_CTRL_B2);
  if(Wire.endTransmission()) { return(RoomTemperatureC16::INVALID_TEMP); } // Exit if error.


  // Wait for temperature measurement/conversion to complete, in low-power sleep mode for the bulk of the time.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("TMP102 waiting for conversion...");
#endif
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write((byte) TMP102_REG_CTRL); // Select control register.
  if(Wire.endTransmission()) { return(0); } // Exit if error.
  for(int i = 8; --i; ) // 2 orbits should generally be plenty.
    {
    if(i <= 0) { return(0); } // Exit if error.
    if(Wire.requestFrom(TMP102_I2C_ADDR, 1) != 1) { return(RoomTemperatureC16::INVALID_TEMP); } // Exit if error.
    const byte b1 = Wire.read();
    if(b1 & TMP102_CTRL_B1_OS) { break; } // Conversion completed.
    ::OTV0P2BASE::nap(WDTO_15MS); // One or two of these naps should allow typical ~26ms conversion to complete...
    }

  // Fetch temperature.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("TMP102 fetching temperature...");
#endif
  Wire.beginTransmission(TMP102_I2C_ADDR);
  Wire.write((byte) TMP102_REG_TEMP); // Select temperature register (set ptr to 0).
  if(Wire.endTransmission()) { return(RoomTemperatureC16::INVALID_TEMP); } // Exit if error.
  if(Wire.requestFrom(TMP102_I2C_ADDR, 2) != 2)  { return(0); }
  if(Wire.endTransmission()) { return(RoomTemperatureC16::INVALID_TEMP); } // Exit if error.

  const byte b1 = Wire.read(); // MSByte, should be signed whole degrees C.
  const uint8_t b2 = Wire.read(); // Avoid sign extension...

  if(neededPowerUp) { OTV0P2BASE::powerDownTWI(); }

  // Builds 12-bit value (assumes not in extended mode) and sign-extends if necessary for sub-zero temps.
  const int t16 = (b1 << 4) | (b2 >> 4) | ((b1 & 0x80) ? 0xf000 : 0);

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("TMP102 temp: ");
  DEBUG_SERIAL_PRINT(b1);
  DEBUG_SERIAL_PRINT_FLASHSTRING("C / ");
  DEBUG_SERIAL_PRINT(temp16);
  DEBUG_SERIAL_PRINTLN();
#endif

  return(t16);
  }
#endif



// Functionality and code only enabled if ENABLE_SENSOR_SHT21 is defined.
#if defined(ENABLE_SENSOR_SHT21)

#define SHT21_I2C_ADDR 0x40
#define SHT21_I2C_CMD_TEMP_HOLD	0xe3
#define SHT21_I2C_CMD_TEMP_NOHOLD 0xf3
#define SHT21_I2C_CMD_RH_HOLD	0xe5
#define SHT21_I2C_CMD_RH_NOHOLD 0xf5
#define SHT21_I2C_CMD_USERREG 0xe7 // User register...

// If defined, sample 8-bit RH (for for 1%) and 12-bit temp (for 1/16C).
// This should save time and energy.
#define SHT21_USE_REDUCED_PRECISION 1

// Set true once SHT21 has been initialised.
static volatile bool SHT21_initialised;

// Initialise/configure SHT21, usually once only.
// TWI must already be powered up.
static void SHT21_init()
  {
#if defined(SHT21_USE_REDUCED_PRECISION)
  // Soft reset in order to sample at reduced precision.
  Wire.beginTransmission(SHT21_I2C_ADDR);
  Wire.write((byte) SHT21_I2C_CMD_USERREG); // Select control register.
  Wire.endTransmission();
  Wire.requestFrom(SHT21_I2C_ADDR, 1);
  while(Wire.available() < 1)
    {
    // Wait for data, but avoid rolling over the end of a minor cycle...
    if(OTV0P2BASE::getSubCycleTime() >= OTV0P2BASE::GSCT_MAX-2)
      {
      return; // Failed, and not initialised.
      }
    }
  const uint8_t curUR = Wire.read();
//  DEBUG_SERIAL_PRINT_FLASHSTRING("UR: ");
//  DEBUG_SERIAL_PRINTFMT(curUR, HEX);
//  DEBUG_SERIAL_PRINTLN();

  // Preserve reserved bits (3, 4, 5) and sample 8-bit RH (for for 1%) and 12-bit temp (for 1/16C).
  const uint8_t newUR = (curUR & 0x38) | 3;
  Wire.beginTransmission(SHT21_I2C_ADDR);
  Wire.write((byte) SHT21_I2C_CMD_USERREG); // Select control register.
  Wire.write((byte) newUR);
  Wire.endTransmission();

#endif
  SHT21_initialised = true;
  }

// Measure and return the current ambient temperature in units of 1/16th C.
// This may contain up to 4 bits of information to the right of the fixed binary point.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
static int Sensor_SHT21_readTemperatureC16()
  {
  const bool neededPowerUp = OTV0P2BASE::powerUpTWIIfDisabled();

  // Initialise/config if necessary.
  if(!SHT21_initialised) { SHT21_init(); }

  // Max RH measurement time:
  //   * 14-bit: 85ms
  //   * 12-bit: 22ms
  //   * 11-bit: 11ms
  // Use blocking data fetch for now.
  Wire.beginTransmission(SHT21_I2C_ADDR);
  Wire.write((byte) SHT21_I2C_CMD_TEMP_HOLD); // Select control register.
#if defined(SHT21_USE_REDUCED_PRECISION)
  OTV0P2BASE::nap(WDTO_30MS); // Should cover 12-bit conversion (22ms).
#else
  OTV0P2BASE::sleepLowPowerMs(90); // Should be plenty for slowest (14-bit) conversion (85ms).
#endif
  //delay(100);
  Wire.endTransmission();
  Wire.requestFrom(SHT21_I2C_ADDR, 3);
  while(Wire.available() < 3)
    {
    // Wait for data, but avoid rolling over the end of a minor cycle...
    if(OTV0P2BASE::getSubCycleTime() >= OTV0P2BASE::GSCT_MAX-2)
      {
      return(RoomTemperatureC16::INVALID_TEMP); // Failure value: may be able to to better.
      }
    }
  uint16_t rawTemp = (Wire.read() << 8);
  rawTemp |= (Wire.read() & 0xfc); // Clear status ls bits.

  // Power down TWI ASAP.
  if(neededPowerUp) { OTV0P2BASE::powerDownTWI(); }

  // TODO: capture entropy if (transformed) value has changed.

  // Nominal formula: C = -46.85 + ((175.72*raw) / (1L << 16));
  const int c16 = -750 + ((5623L * rawTemp) >> 17); // FIXME: find a faster approximation...

  return(c16);
  }

// Measure and return the current relative humidity in %; range [0,100] and 255 for error.
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
// Returns 255 (~0) in case of error.
uint8_t HumiditySensorSHT21::read()
  {
  const bool neededPowerUp = OTV0P2BASE::powerUpTWIIfDisabled();

  // Initialise/config if necessary.
  if(!SHT21_initialised) { SHT21_init(); }

  // Get RH%...
  // Max RH measurement time:
  //   * 12-bit: 29ms
  //   *  8-bit:  4ms
  // Use blocking data fetch for now.
  Wire.beginTransmission(SHT21_I2C_ADDR);
  Wire.write((byte) SHT21_I2C_CMD_RH_HOLD); // Select control register.
#if defined(SHT21_USE_REDUCED_PRECISION)
  OTV0P2BASE::sleepLowPowerMs(5); // Should cover 8-bit conversion (4ms).
#else
  OTV0P2BASE::nap(WDTO_30MS); // Should cover even 12-bit conversion (29ms).
#endif
  Wire.endTransmission();
  Wire.requestFrom(SHT21_I2C_ADDR, 3);
  while(Wire.available() < 3)
    {
    // Wait for data, but avoid rolling over the end of a minor cycle...
    if(OTV0P2BASE::getSubCycleTime() >= OTV0P2BASE::GSCT_MAX)
      {
//      DEBUG_SERIAL_PRINTLN_FLASHSTRING("giving up");
      return(~0);
      }
    }
  const uint8_t rawRH = Wire.read();
  const uint8_t rawRL = Wire.read();

  // Power down TWI ASAP.
  if(neededPowerUp) { OTV0P2BASE::powerDownTWI(); }

  const uint16_t raw = (((uint16_t)rawRH) << 8) | (rawRL & 0xfc); // Clear status ls bits.
  const uint8_t result = -6 + ((125L * raw) >> 16);

  // Capture entropy from raw status bits
  // iff (transformed) reading has changed.
  if(value != result) { OTV0P2BASE::addEntropyToPool(rawRL ^ rawRH, 1); }

  value = result;
  if(result > (HUMIDTY_HIGH_RHPC+HUMIDITY_EPSILON_RHPC)) { highWithHyst = true; }
  else if(result < (HUMIDTY_HIGH_RHPC-HUMIDITY_EPSILON_RHPC)) { highWithHyst = false; }
  return(result);
  }
// Singleton implementation/instance.
HumiditySensorSHT21 RelHumidity;
#endif




// Functionality and code only enabled if SENSOR_DS18B20_ENABLE is defined.
#if defined(SENSOR_DS18B20_ENABLE)

#define DS1820_PRECISION_MASK 0x60
#define DS1820_PRECISION_9 0x00
#define DS1820_PRECISION_10 0x20
#define DS1820_PRECISION_11 0x40 // 1/8C @ 375ms.
#define DS1820_PRECISION_12 0x60 // 1/16C @ 750ms.

// Run reduced precision (11 bit, 1/8C) for acceptable conversion time (375ms).
#define DS1820_PRECISION DS1820_PRECISION_11 // 1/8C @ 375ms.

//// Handle on device.
//static OneWire ds18b20(DS1820_ONEWIRE_PIN);  

// Set true once DS18B20 has been searched for and initialised.
static bool sensor_DS18B10_initialised;
// Address of (first) DS18B20 found, else [0] == 0 if none found.
static uint8_t first_DS18B20_address[8];

// Initialise the device (if any) before first use.
// Returns true iff successful.
// Uses first DS18B20 found on bus.
static bool Sensor_DS18B10_init()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("DS18B20 init...");
  bool found = false;

  // Ensure no bad search state.
  minOW.reset_search();

  for( ; ; )
    {
    if(!minOW.search(first_DS18B20_address))
      {
      minOW.reset_search(); // Be kind to any other OW search user.
      break;
      }

#if 0 && defined(DEBUG)
    // Found a device.
    DEBUG_SERIAL_PRINT_FLASHSTRING("addr:");
    for(int i = 0; i < 8; ++i)
      {
      DEBUG_SERIAL_PRINT(' ');
      DEBUG_SERIAL_PRINTFMT(first_DS18B20_address[i], HEX);
      }
    DEBUG_SERIAL_PRINTLN();
#endif

    if(0x28 != first_DS18B20_address[0])
      {
#if 0 && defined(DEBUG)
      DEBUG_SERIAL_PRINTLN_FLASHSTRING("Not a DS18B20, skipping...");
#endif
      continue;
      }

#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("Setting precision...");
#endif
    minOW.reset();
    // Write scratchpad/config
    minOW.select(first_DS18B20_address);
    minOW.write(0x4e);
    minOW.write(0); // Th: not used.
    minOW.write(0); // Tl: not used.
    minOW.write(DS1820_PRECISION | 0x1f); // Config register; lsbs all 1.

    // Found one and configured it!
    found = true;
    }

  // Search has been run (whether DS18B20 was found or not).
  sensor_DS18B10_initialised = true;

  if(!found)
    {
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("DS18B20 not found");
    first_DS18B20_address[0] = 0; // Indicate no DS18B20 found.
    }
  return(found);
  }

// Returns temperature in C*16.
// Returns <= 0 for some sorts of error as failsafe (RoomTemperatureC16::INVALID_TEMP if failed to initialise).
static int16_t Sensor_DS18B10_readTemperatureC16()
  {
  if(!sensor_DS18B10_initialised) { Sensor_DS18B10_init(); }
  if(0 == first_DS18B20_address[0]) { return(RoomTemperatureC16::INVALID_TEMP); }

  // Start a temperature reading.
  minOW.reset();
  minOW.select(first_DS18B20_address);
  minOW.write(0x44); // Start conversion without parasite power.
  //delay(750); // 750ms should be enough.
  // Poll for conversion complete (bus released)...
  while(minOW.read_bit() == 0) { OTV0P2BASE::nap(WDTO_30MS); }

  // Fetch temperature (scratchpad read).
  minOW.reset();
  minOW.select(first_DS18B20_address);    
  minOW.write(0xbe);
  // Read first two bytes of 9 available.  (No CRC config or check.)
  const uint8_t d0 = minOW.read();
  const uint8_t d1 = minOW.read();
  // Terminate read and let DS18B20 go back to sleep.
  minOW.reset();

  // Extract raw temperature, masking any undefined lsbit.
  const int16_t rawC16 = (d1 << 8) | (d0 & ~1);

  return(rawC16);
  }
#endif


//// Median filter.
//// Find mean of interquatile range of group of ints where sum can be computed in an int without loss.
//// FIXME: needs a unit test or three.
//template<uint8_t N> int smallIntIQMean(const int data[N])
//  {
//  // Copy array content.
//  int copy[N];
//  for(int8_t i = N; --i >= 0; ) { copy[i] = data[i]; }
//  // Sort in place with a bubble sort (yeuck) assuming the array to be small.
//  // FIXME: replace with insertion sort for efficiency.
//  // FIXME: break out sort as separate subroutine.
//  uint8_t n = N;
//  do
//    {
//    uint8_t newn = 0;
//    for(uint8_t i = 0; ++i < n; )
//      {
//      const int c0 = copy[i-1];
//      const int c1 = copy[i];
//      if(c0 > c1)
//         {
//         copy[i] = c0;
//         copy[i-1] = c1;
//         newn = i;
//         }
//      }
//    n = newn;
//    } while(0 != n);
//#if 0 && defined(DEBUG)
//DEBUG_SERIAL_PRINT_FLASHSTRING("sorted: ");
//for(uint8_t i = 0; i < N; ++i) { DEBUG_SERIAL_PRINT(copy[i]); DEBUG_SERIAL_PRINT(' '); }
//DEBUG_SERIAL_PRINTLN();
//#endif
//  // Extract mean of interquartile range.
//  const size_t sampleSize = N/2;
//  const size_t start = N/4;
//  // Assume values will be nowhere near the extremes.
//  int sum = 0;
//  for(uint8_t i = start; i < start + sampleSize; ++i) { sum += copy[i]; }
//  // Compute rounded-up mean.
//  return((sum + sampleSize/2) / sampleSize);
//  }



// Singleton implementation/instance.
RoomTemperatureC16 TemperatureC16;

// Temperature read uses/selects one of the implementations/sensors.
int16_t RoomTemperatureC16::read()
  {
#if defined(SENSOR_DS18B20_ENABLE)
  const int raw = Sensor_DS18B10_readTemperatureC16();
#elif defined(ENABLE_SENSOR_SHT21)
  const int raw = Sensor_SHT21_readTemperatureC16();
#else
  const int raw = TMP112_readTemperatureC16();
#endif

  value = raw;
  return(value);
  }



#if defined(SENSOR_EXTERNAL_DS18B20_ENABLE) && defined(SUPPORTS_MINIMAL_ONEWIRE)
//// Initialise the device (if any) before first use.
//// Returns true iff successful.
//// Uses specified order DS18B20 found on bus.
//// May need to be reinitialised if precision changed.
//bool TemperatureC16_DS18B20::init()
//  {
////  DEBUG_SERIAL_PRINTLN_FLASHSTRING("DS18B20 init...");
//  bool found = false;
//
//  // Ensure no bad search state.
//  minOW.reset_search();
//
//  for( ; ; )
//    {
//    if(!minOW.search(address))
//      {
//      minOW.reset_search(); // Be kind to any other OW search user.
//      break;
//      }
//
//#if 0 && defined(DEBUG)
//    // Found a device.
//    DEBUG_SERIAL_PRINT_FLASHSTRING("addr:");
//    for(int i = 0; i < 8; ++i)
//      {
//      DEBUG_SERIAL_PRINT(' ');
//      DEBUG_SERIAL_PRINTFMT(address[i], HEX);
//      }
//    DEBUG_SERIAL_PRINTLN();
//#endif
//
//    if(0x28 != address[0])
//      {
//#if 0 && defined(DEBUG)
//      DEBUG_SERIAL_PRINTLN_FLASHSTRING("Not a DS18B20, skipping...");
//#endif
//      continue;
//      }
//
//#if 0 && defined(DEBUG)
//    DEBUG_SERIAL_PRINTLN_FLASHSTRING("Setting precision...");
//#endif
//    minOW.reset();
//    // Write scratchpad/config
//    minOW.select(address);
//    minOW.write(0x4e);
//    minOW.write(0); // Th: not used.
//    minOW.write(0); // Tl: not used.
////    MinOW.write(DS1820_PRECISION | 0x1f); // Config register; lsbs all 1.
//    minOW.write(((precision - 9) << 6) | 0x1f); // Config register; lsbs all 1.
//
//    // Found one and configured it!
//    found = true;
//    }
//
//  // Search has been run (whether DS18B20 was found or not).
//  initialised = true;
//
//  if(!found)
//    {
//    DEBUG_SERIAL_PRINTLN_FLASHSTRING("DS18B20 not found");
//    address[0] = 0; // Indicate that no DS18B20 was found.
//    }
//  return(found);
//  }
//
//// Force a read/poll of temperature and return the value sensed in nominal units of 1/16 C.
//// At sub-maximum precision lsbits will be zero or undefined.
//// Expensive/slow.
//// Not thread-safe nor usable within ISRs (Interrupt Service Routines).
//int16_t TemperatureC16_DS18B20::read()
//  {
//  if(!initialised) { init(); }
//  if(0 == address[0]) { value = INVALID_TEMP; return(INVALID_TEMP); }
//
//  // Start a temperature reading.
//  minOW.reset();
//  minOW.select(address);
//  minOW.write(0x44); // Start conversion without parasite power.
//  //delay(750); // 750ms should be enough.
//  // Poll for conversion complete (bus released)...
//  while(minOW.read_bit() == 0) { OTV0P2BASE::nap(WDTO_15MS); }
//
//  // Fetch temperature (scratchpad read).
//  minOW.reset();
//  minOW.select(address);    
//  minOW.write(0xbe);
//  // Read first two bytes of 9 available.  (No CRC config or check.)
//  const uint8_t d0 = minOW.read();
//  const uint8_t d1 = minOW.read();
//  // Terminate read and let DS18B20 go back to sleep.
//  minOW.reset();
//
//  // Extract raw temperature, masking any undefined lsbit.
//  // TODO: mask out undefined LSBs if precision not maximum.
//  const int16_t rawC16 = (d1 << 8) | (d0);
//
//  return(rawC16);
//  }
#endif

#if defined(SENSOR_EXTERNAL_DS18B20_ENABLE_0) // Enable sensor zero.
TemperatureC16_DS18B20 extDS18B20_0(MinOW_DEFAULT, 0);
#endif


#ifdef ENABLE_VOICE_SENSOR
// If count meets or exceeds this threshold in one poll period then
// the room is deemed to be occupied.
// Strictly positive.
// DHD20151119: even now it seems a threshold of >= 2 is needed to avoid false positives.
#define VOICE_DETECTION_THRESHOLD 4

// Force a read/poll of the voice level and return the value sensed.
// Thread-safe and ISR-safe.
uint8_t VoiceDetection::read()
  {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
	  isDetected = ((value = count) >= VOICE_DETECTION_THRESHOLD);
	  // clear count and detection flag
//	  isTriggered = false;
      count = 0;
    }
  return(value);
  }

// Handle simple interrupt.
// Fast and ISR (Interrupt Service Routines) safe.
// Returns true if interrupt was successfully handled and cleared
// else another interrupt handler in the chain may be called
// to attempt to clear the interrupt.
bool VoiceDetection::handleInterruptSimple()
  {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
    // Count of voice activations since last poll, avoiding overflow.
    if((count < 255) && (++count >= VOICE_DETECTION_THRESHOLD))
      {
      // Act as soon as voice is detected.
      isDetected = true;
      // Don't regard this as a very strong indication,
      // as it could be a TV or radio on in the room.
      Occupancy.markAsPossiblyOccupied();
      }
    }

//    // Flag that interrupt has occurred
//    endOfLocking = OTV0P2BASE::getMinutesSinceMidnightLT() + lockingPeriod;
//    isTriggered = true;
    // No further work to be done to 'clear' interrupt.
    return(true);
  }

// Singleton implementation/instance.
VoiceDetection Voice;
#endif


////////////////////////// Actuators


// DORM1/REV7 direct drive actuator.
#ifdef HAS_DORM1_VALVE_DRIVE
//#ifdef DIRECT_MOTOR_DRIVE_V1
// Singleton implementation/instance.
#ifdef ENABLE_DORM1_MOTOR_REVERSED // Reversed vs sample 2015/12
OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_ML, MOTOR_DRIVE_MR, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#else
OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_MR, MOTOR_DRIVE_ML, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#endif // HAS_DORM1_MOTOR_REVERSED
#endif


// FHT8V radio-controlled actuator.
#ifdef ENABLE_FHT8VSIMPLE
// Function to append stats trailer (and 0xff) to FHT8V/FS20 TX buffer.
// Assume enough space in buffer for largest possible stats message.
#if defined(ALLOW_STATS_TX)
uint8_t *appendStatsToTXBufferWithFF(uint8_t *bptr, const uint8_t bufSize)
  {
  OTV0P2BASE::FullStatsMessageCore_t trailer;
  populateCoreStats(&trailer);
  // Ensure that no ID is encoded in the message sent on the air since it would be a repeat from the FHT8V frame.
  trailer.containsID = false;

#if defined(ENABLE_MINIMAL_STATS_TXRX)
  // As bandwidth optimisation just write minimal trailer if only temp&power available.
  if(trailer.containsTempAndPower &&
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
  return(bptr);
  }
#else
#define appendStatsToTXBufferWithFF NULL // Do not append stats.
#endif
#endif // ENABLE_FHT8VSIMPLE

#ifdef ENABLE_FHT8VSIMPLE
OTRadValve::FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V(appendStatsToTXBufferWithFF);
#endif

// Clear both housecode parts (and thus disable local valve).
// Does nothing if FHT8V not in use.
#ifdef ENABLE_FHT8VSIMPLE
void FHT8VClearHC()
  {
  FHT8V.clearHC();
  OTV0P2BASE::eeprom_smart_erase_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC1);
  OTV0P2BASE::eeprom_smart_erase_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC2);
  }
#endif

// Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
// Also set value in FHT8V rad valve model.
// Does nothing if FHT8V not in use.
#ifdef ENABLE_FHT8VSIMPLE
void FHT8VSetHC1(uint8_t hc)
  {
  FHT8V.setHC1(hc);
  OTV0P2BASE::eeprom_smart_update_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC1, hc);
  }
void FHT8VSetHC2(uint8_t hc)
  {
  FHT8V.setHC2(hc);
  OTV0P2BASE::eeprom_smart_update_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC2, hc);
  }
#endif

// Get (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control (will be 0xff until set).
// FHT8V instance values are used as a cache.
// Does nothing if FHT8V not in use.
#ifdef ENABLE_FHT8VSIMPLE
uint8_t FHT8VGetHC1()
  {
  const uint8_t vv = FHT8V.getHC1();
  // If cached value in FHT8V instance is valid, return it.
  if(OTRadValve::FHT8VRadValveBase::isValidFHTV8HouseCode(vv))
    { return(vv); }
  // Else if EEPROM value is valid, then cache it in the FHT8V instance and return it.
  const uint8_t ev = eeprom_read_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC1);
  if(OTRadValve::FHT8VRadValveBase::isValidFHTV8HouseCode(ev))
      { FHT8V.setHC1(ev); }
  return(ev);
  }
uint8_t FHT8VGetHC2()
  {
  const uint8_t vv = FHT8V.getHC2();
  // If cached value in FHT8V instance is valid, return it.
  if(OTRadValve::FHT8VRadValveBase::isValidFHTV8HouseCode(vv))
    { return(vv); }
  // Else if EEPROM value is valid, then cache it in the FHT8V instance and return it.
  const uint8_t ev = eeprom_read_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC2);
  if(OTRadValve::FHT8VRadValveBase::isValidFHTV8HouseCode(ev))
      { FHT8V.setHC2(ev); }
  return(ev);
  }
#endif // ENABLE_FHT8VSIMPLE

#ifdef ENABLE_FHT8VSIMPLE
// Load EEPROM house codes into primary FHT8V instance at start-up or once cleared in FHT8V instance.
void FHT8VLoadHCFromEEPROM()
  {
  // Uses side-effect to cache/save in FHT8V instance.
  FHT8VGetHC1();
  FHT8VGetHC2();
  }
#endif // ENABLE_FHT8VSIMPLE


//#if defined(ENABLE_BOILER_HUB)
//// Boiler output control.
//// Singleton implementation/instance.
//extern BoilerDriver BoilerControl;
//#endif
