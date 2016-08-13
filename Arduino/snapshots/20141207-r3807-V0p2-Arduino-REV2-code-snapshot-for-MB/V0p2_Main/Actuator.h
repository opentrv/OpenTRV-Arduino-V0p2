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
 Base actuator type for simple actuators accepting scalar values.

 Most actuators should derive from this.

 May also be used for pseudo-sensors
 such as synthesised from multiple sensors combined.
 */

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "Sensor.h"

// Base Actuator type.
// All actuators are sensors for their requsted actuator setting/position by default.
// Templated on sensor value type, typically uint8_t or uint16_t or int.
template <class T>
class Actuator : public Sensor<T>
  {
  };


// Simple mainly thread-safe uint8_t-valued actuator.
// May be a virtual actuator or physical.
class SimpleTSUint8Actuator : public Actuator<uint8_t>
  {
  protected:
    // Requested acuator value/position.
    volatile uint8_t value;

    // By default initialise the value to zero.
    SimpleTSUint8Actuator() : value(0) { }
    // Can initialise to a chosen value.
    SimpleTSUint8Actuator(const uint8_t v) : value(v) { }

//    // Set new value.
//    // This assumes that isValid() applies to actuator values for set() also.
//    // Usually fast.
//    // Often likely to be thread-safe or usable within ISRs (Interrupt Service Routines),
//    // BUT READ IMPLEMENTATION DOCUMENTATION BEFORE TREATING AS thread/ISR-safe.
//    // Note that visibility is protected to prevent direct setting of the target value
//    // unless a more visible version is provided.
//    virtual bool set(const uint8_t newValue)
//      {
//      if(!isValid(newValue)) { return(false); }
//      value = newValue;
//      return(true);
//      }

  public:
    // Return last value fetched by read(); undefined before first read().
    // Usually fast.
    // Often likely to be thread-safe or usable within ISRs (Interrupt Service Routines),
    // BUT READ IMPLEMENTATION DOCUMENTATION BEFORE TREATING AS thread/ISR-safe.
    virtual uint8_t get() const { return(value); }
  };

#endif
