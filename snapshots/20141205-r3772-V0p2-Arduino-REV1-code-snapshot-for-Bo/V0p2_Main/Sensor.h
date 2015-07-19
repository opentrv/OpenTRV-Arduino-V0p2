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

Author(s) / Copyright (s): Damon Hart-Davis 2014
*/

/*
 Base sensor type for simple sensors returning scalar values.

 Most sensors should derive from this.

 May also be used for pseudo-sensors
 such as synthesised from multiple sensors combined.
 */

#ifndef SENSOR_H
#define SENSOR_H

//#include "Security.h"

// Base sensor type.
// Templated on sensor value type, typically uint8_t or uint16_t or int.
template <class T>
class Sensor
  {
  public:
    // Force a read/poll of this sensor and return the value sensed.
    // May be expensive/slow.
    // Unlikely to be thread-safe or usable within ISRs (Interrupt Service Routines).
    // Individual implementations can document alternative behaviour.
    virtual T read() = 0;
 
    // Return last value fetched by read(); undefined before first read().
    // Usually fast.
    // Often likely to be thread-safe or usable within ISRs (Interrupt Service Routines),
    // BUT READ IMPLEMENTATION DOCUMENTATION BEFORE TREATING AS thread/ISR-safe.
    virtual T get() const = 0;

    // Returns true if this sensor reading value passed is potentially valid, eg in-range.
    // Default is to always return true, ie all values potentially valid.
    virtual bool isValid(T value) const { return(true); }
 
    // Returns non-zero if this implementation requires a regular call to read() to operate correctly.
    // Preferred poll interval (in seconds) or 0 if no regular poll() call required.
    // Default returns 0 indicating regular call to read() not required,
    // only as required to fetch new values from the underlying sensor.
    virtual uint8_t preferredPollInterval_s() const { return(0); }

//    // Returns a suggested (JSON) tag/field/key name including units of value from get(); non-NULL.
//    // The lifetime of the pointed-to text is at least that of the Sensor instance.
//    virtual const char *tag() const { return(""); }
//
//    // Returns a suggested privacy/sensitivity level of the data from this sensor.
//    // The default sensitivity is set to just forbid transmission at default (255) leaf settings.
//    virtual uint8_t sensitivity() const { return(254 /* stTXsecOnly */ ); }

    // Handle simple interrupt for this sensor.
    // Must be fast and ISR (Interrupt Service Routine) safe.
    // Returns true if interrupt was successfully handled and cleared
    // else another interrupt handler in the chain may be called
    // to attempt to clear the interrupt.
    // By default does nothing (and returns false).
    virtual bool handleInterruptSimple() { return(false); }

    // Begin access to this sensor if applicable and not already begun.
    // Returns true if it needed to be begun.
    // Allows logic to end() if required at the end of a block, etc.
    // Defaults to do nothing (and return false).
    virtual bool begin() { return(false); }

    // Returns true if this sensor is currently available.
    // True by default unless implementation overrides.
    // For those sensors that need starting this will be false before begin().
    virtual bool isAvailable() const { return(true); }

    // End access to this sensor if applicable and not already ended.
    // Returns true if it needed to be ended.
    // Defaults to do nothing (and return false).
    virtual bool end() { return(false); }

#if 0 // Defining the virtual destructor uses ~800+ bytes of Flash by forcing use of malloc()/free().
    // Ensure safe instance destruction when derived from.
    // by default attempts to shut down the sensor and otherwise free resources when done.
    // This uses ~800+ bytes of Flash by forcing use of malloc()/free().
    virtual ~Sensor() { end(); }
#else
#define SENSOR_NO_VIRT_DEST // Beware, no virtual destructor so be careful of use via base pointers.
#endif
  };


// Simple mainly thread-safe uint8_t-valued sensor.
// Made thread-safe in get() by marking the value volatile
// providing that read() is careful to do any compound operations on value
// (eg more than simple read or simple write, or involving unwanted intermediate states)
// under a proper lock, eg excluding interrupts.
class SimpleTSUint8Sensor : public Sensor<uint8_t>
  {
  protected:
      volatile uint8_t value;

      // By default initialise the value to zero.
      SimpleTSUint8Sensor() : value(0) { }
      // Can initialise to a chosen value.
      SimpleTSUint8Sensor(const uint8_t v) : value(v) { }

  public:
    // Return last value fetched by read(); undefined before first read().
    // Usually fast.
    // Often likely to be thread-safe or usable within ISRs (Interrupt Service Routines),
    // BUT READ IMPLEMENTATION DOCUMENTATION BEFORE TREATING AS thread/ISR-safe.
    virtual uint8_t get() const { return(value); }
  };

#endif
