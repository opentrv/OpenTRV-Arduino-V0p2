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

Author(s) / Copyright (s): John Harvey 2014
*/

/*
 * Header for main on-board sensors for V0p2 variants.
 */

#ifndef DS_SENSORS_H
#define DS_SENSORS_H

#include "V0p2_Sensors.h"


// Room/ambient temperature.

// Sensor for ambient/room temperature in 1/16th of one degree Celcius.
// An error may be indicated by returning a zero or (very) negative value.
class DS1820: public RoomTemperatureC16
  {
  private:
    // Room temperature in 16*C, eg 1 is 1/16 C, 32 is 2C, -64 is -4C.
    int value;

  public:
    // Initialise to cautious values.
    DS1820() : value(0) { }

    // Force a read/poll of room temperature and return the value sensed in units of 1/16 C.
    // Expensive/slow.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual int read();

    // Return last value fetched by read(); undefined before first read().
    // Fast.
    // Not thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual int get() const { return(value); }
  };
// Singleton implementation/instance.
extern DS1820 TemperatureDS1820;

#endif

