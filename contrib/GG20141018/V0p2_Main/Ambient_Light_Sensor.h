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
 Ambient light sensor module.
 */

#ifndef AMBIENT_LIGHT_SENSOR_H
#define AMBIENT_LIGHT_SENSOR_H

#include <stdint.h>

#include "V0p2_Main.h"


// Aim is to be able to sense (maybe non-linearly) over full likely
// internal ambient lighting range of a (UK) home,
// down to levels too dark to be active in (and at which heating could be set back for example).
// This suggests a full scale of at least 50--100 lux, maybe as high as 300 lux, eg see:
// http://home.wlv.ac.uk/~in6840/Lightinglevels.htm
// http://www.engineeringtoolbox.com/light-level-rooms-d_708.html
// http://www.pocklington-trust.org.uk/Resources/Thomas%20Pocklington/Documents/PDF/Research%20Publications/GPG5.pdf
// http://www.vishay.com/docs/84154/appnotesensors.pdf



#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION

// Note on: phototransistor variant.
// Note that if AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400 is defined
// This expects a current-response phototransistor in place of the LDR
// with roughly full-scale value in full light against internal 1.1V reference
// not against supply voltage as usual.

// Class encapsulating all light related behaviour i.e. more than just sensor
// TODO: inline single use routines e.g. maxLight, onHour, toStat?
// TODO: make light level a type?
typedef int lightLevel_t;
class Light {
  private:
    static lightLevel_t ambientLightLevel;
    static bool roomLit; // TODO: might want to refactor imp. - most calls on isRoomDark

    static const uint8_t maxLight(const uint8_t *sE) /* const */;

  protected:
  public:
    // TODO: more detailed thought?
    Light() {/* roomLit = false; */ ambientLightLevel = 255; /* ~ day lit */ readAmbientLight(); };

    // Measure/store/return the current room ambient light levels in range [0,1023].
    // This may consume significant power and time.
    // Probably no need to do this more than (say) once per minute.
    static const lightLevel_t readAmbientLight();

    // Return previously-read (with readAmbientLight()) ambient light level in range [0,1023]; very fast.
    static inline const lightLevel_t getAmbientLight() /* const */ {return (ambientLightLevel);};
    // Returns true if room/environs appear well enough lit for normal activity.
    // Based on results of last call to readAmbientLight().
    // Defaults to false if OMIT_MODULE_LDROCCUPANCYDETECTION defined.
    static inline const bool isRoomLit() /* const */ {return (roomLit);};
    // Returns true if room/environs do not appear well enough lit for normal activity.
    // Usually the complement of isRoomLit() but both can be false if ambient light status not known.
    // Defaults to false if OMIT_MODULE_LDROCCUPANCYDETECTION defined.
    static inline const bool isRoomDark() /* const */ {return (!isRoomLit());};

    // This may consume significant power and time.
    static void onHour();

    // Average, scale and constrain total ambient-light value to valid range for stats; very top of range is compressed to retain maximum gamut.
    static const uint8_t toStat(const lightLevel_t total, const uint8_t count) /* const */ ;
};

#else

class Light {
  private:
  protected:
  public:
    static inline const int readAmbientLight() {return (0);};
    static inline const int getAmbientLight() /* const */ {return (0);};
    static inline const bool isRoomLit() /* const */ {return (false);};
    static inline const bool isRoomDark() /* const */ {return (false);};

};

#endif

#endif


