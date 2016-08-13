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

// Base sensor type.
// Templated on sensor value type, typically uint8_t or uint16_t or int.
template <class T>
class Sensor
  {
  public:
    // Force a read/poll of the sensor and return the value sensed.
    // May be expensive/slow.
    // Unlikely to be thread-safe or usable within ISRs (Interrupt Service Routines).
    // Individual implementations can document alternative behaviour.
    virtual T read() = 0;
 
    // Return last value fetched by read(); undefined before first read().
    // Usually fast.
    // Often likely to be thread-safe or usable within ISRs (Interrupt Service Routines),
    // BUT READ IMPLEMENTATION DOCUMENTATION BEFORE TREATING AS thread/ISR-safe.
    virtual T get() = 0;

    // Returns true if the sensor reading value passed is potentially valid, eg in-range.
    virtual bool isValid(T value) = 0;
 
    // Returns true if this implementation requires a regular call to read() to operate correctly.
    // Preferred poll interval (in seconds) or 0 if no regular poll() call required.
    // Default returns 0 indicating regular call to read() not required,
    // only as required to fetch new values from the underlying sensor.
    virtual uint8_t preferredPollInterval_s() { return(0); }

    // Handle simple interrupt.
    // Must be fast and ISR (Interrupt Service Routines) safe.
    // Returns true if interrupt was successfully handled and cleared
    // else another interrupt handler in the chain may be called
    // to attempt to clear the interrupt.
    // By default does nothing (and returns false).
    virtual bool handleInterruptSimple() { return(false); }

    // Begin access to the sensor if applicable and not already begun.
    // Returns true if it needed to be begun.
    // Allows logic to end() if required at the end of a block, etc.
    // Defaults to do nothing (and return false).
    virtual bool begin() { return(false); }

    // Returns true if sensor is currently available.
    // True by default unless implementation overrides.
    // For those sensors that need starting this will be false before begin().
    virtual bool isAvailable() { return(true); }

    // End access to the sensor if applicable and not already ended.
    // Returns true if it needed to be ended.
    // Defaults to do nothing (and return false).
    virtual bool end() { return(false); }

    // Ensure safe instance destruction when derived from.
    // by default attempts to shut down the sensor and otherwise free resources when done.
    virtual ~Sensor() { end(); }
  };

#endif
