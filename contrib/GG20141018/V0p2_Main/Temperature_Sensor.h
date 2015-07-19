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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2014
                           Gary Gladman 2014
*/

/*
 Temperature sensor module.
 */

#ifndef TEMPERATURE_SENSOR_H
#define TEMPERATURE_SENSOR_H

#include <stdint.h>

#include "V0p2_Main.h"

// Class encapsulating all temperature related behaviour i.e. more than just sensor
// TODO: inline single use routines e.g. maxLight, onHour, toStat?
// TODO: make temperature a type?
typedef int tempC16_t;
typedef uint8_t tempC_t;
class Temp {
  private:
    static tempC16_t tempC16;

    // static const uint8_t maxLight(const uint8_t *sE) /* const */;

  protected:
  public:
    // TODO: more detailed thought?
    Temp() {/* tempC16 = 0; */ readTemperatureC16(); };

    // Measure/store/return the current room ambient temperature in units of 1/16th C.
    // This may contain up to 4 bits of information to the right of the fixed binary point.
    // This may consume significant power and time.
    // Probably no need to do this more than (say) once per minute.
    // The first read will initialise the device as necessary and leave it in a low-power mode afterwards.
    // This will simulate a zero temperature in case of detected error talking to the sensor as fail-safe for this use.
    static const tempC16_t readTemperatureC16();
    // Return previously-read (with readTemperatureC16()) temperature; very fast.
    static inline const tempC16_t getTemperatureC16() /* const */ {return (tempC16);};
    // Function C16 as C.
    static inline const tempC_t asTemperatureC(const tempC16_t tempC16) /* const */ {return (tempC_t)(tempC16 >> 4);};
    // Return read C16 as C.
    static inline const tempC_t getTemperatureC() /* const */ {return(asTemperatureC(tempC16));};
    // Function C as C16.
    static inline const tempC16_t asTemperatureC16(const tempC_t tempC) /* const */ {return (tempC16_t)(tempC << 4);};
    // Function C16 16ths.
    static inline const uint8_t asTemperature16ths(const tempC16_t tempC16) /* const */ {return (uint8_t)(tempC16 & 0xf);};
    // Return read C16 16ths.
    static inline const uint8_t getTemperature16ths() /* const */ {return(asTemperature16ths(tempC16));};

    // Return temperature relation -ve: under, 0: close (within hysteresis), +ve: over
    static const int8_t temperatureRelation(const tempC_t tempC) /* const */ ;
    // Return temperature relation -ve: under, 0: close (within hysteresis), +ve: over
    static const int8_t temperatureRelation16ths(const tempC_t tempC) /* const */ ;

    // This may consume significant power and time.
    static void onHour();

    // Average, scale and constrain total ambient-light value to valid range for stats; very top of range is compressed to retain maximum gamut.
    static const uint8_t toStat(const tempC16_t total, const uint8_t count) /* const */ ;

#if 0
    // Measure/store/return the current microcontroller temperature in units of 1/16th C.
    // This may contain up to 4 bits of information to the right of the fixed binary point.
    // This may consume significant power and time.
    // Probably no need to do this more than (say) once per minute.
    // This is likely highly inaccurate, eg +/- 10C according to ATmega328P datasheet.
    static const int readInternalTemperatureC16();
#endif

};

#endif
