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
*/

/*
 Ambient light sensor module.
 */

#include "Ambient_Light_Sensor.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "Control.h"
#include "Security.h"
#include "Serial_IO.h"
#include "Power_Management.h"
#include "UI_Minimal.h"

#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION


#ifdef AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
const int reference = INTERNAL; // Internal 1.1V reference.
// If defined, then allow adaptive compression of top part of range when would otherwise max out.
// This may be somewhat supply-voltage dependent, eg capped by the supply voltage.
// Supply voltage is expected to be 2--3 times the bandgap reference, typically.
#define ADAPTIVE_THRESHOLD 896 // (1024-128) Top ~10%, companding by 8x.

// This implementation expects a phototransitor TEPT4400 (50nA dark current, nominal 200uA@100lx@Vce=50V) from IO_POWER_UP to LDR_SENSOR_AIN and 220k to ground.
// Measurement should be taken wrt to internal fixed 1.1V bandgap reference, since light indication is current flow across a fixed resistor.
// Aiming for maximum reading at or above 100--300lx, ie decent domestic internal lighting.
// Note that phototransistor is likely far more directionally-sensitive than LDR.
// http://home.wlv.ac.uk/~in6840/Lightinglevels.htm
// http://www.engineeringtoolbox.com/light-level-rooms-d_708.html
// http://www.pocklington-trust.org.uk/Resources/Thomas%20Pocklington/Documents/PDF/Research%20Publications/GPG5.pdf
// http://www.vishay.com/docs/84154/appnotesensors.pdf
static const int LDR_THR_LOW = 4;
static const int LDR_THR_HIGH = 8;

// TODO: could extend dynamic range and switch to Vss measurement when full-scale against bandgap ref, then scale by Vss/Vbandgap and compress to fit.

#else // LDR

// This implementation expects an LDR (1M dark resistance) from IO_POWER_UP to LDR_SENSOR_AIN and 100k to ground.
// Measurement should be taken wrt to supply voltage, since light indication is a fraction of that.
// Values below from PICAXE V0.09 impl approx multiplied by 4+ to allow for scale change.
const int reference = DEFAULT; // Supply voltage as reference.

#ifdef LDR_EXTRA_SENSITIVE // Define if LDR not exposed to much light, eg for REV2 cut4 sideways-pointing LDR (TODO-209).
static const int LDR_THR_LOW = 50;
static const int LDR_THR_HIGH = 70; 
#else // Normal settings.
static const int LDR_THR_LOW = 160; // Was 30.
static const int LDR_THR_HIGH = 200; // Was 35.
#endif

#endif

static bool isRoomLitFlag;

// Returns true if room/environs well enough lit for normal activity.
// Based on results of last call to readAmbientLight().
bool isRoomLit() { return(isRoomLitFlag); }

// Ambient light levels in range [0,1023].
static int ambientLightLevel;

// Return previously-read (with readAmbientLight()) ambient light level in range [0,1023]; very fast.
int getAmbientLight() { return(ambientLightLevel); }

// Measure/store/return the current room ambient light levels in range [0,1023].
// This may consume significant power and time.
// Probably no need to do this more than (say) once per minute.
// This implementation expects LDR (1M dark resistance) from IO_POWER_UP to LDR_SENSOR_AIN and 100k to ground.
// (Not intended to be called from ISR.)
int readAmbientLight()
  {
  power_intermittent_peripherals_enable(false); // No need to wait for anything to stablise as direct of IO_POWER_UP.
  const int al0 = analogueNoiseReducedRead(LDR_SENSOR_AIN, reference);
#if defined(ADAPTIVE_THRESHOLD)
  int al;
  if(al0 >= ADAPTIVE_THRESHOLD)
    {
    const int al1 = analogueNoiseReducedRead(LDR_SENSOR_AIN, DEFAULT); // Vsupply reference.
    const int vbg = read1V1wrtBattery(); // Vbandgap wrt Vsupply.
    // Compute value in extended range up to ~1024 * Vsupply/Vbandgap.
    const int ale = ((al1 << 5) / ((vbg+16) >> 5)); // Faster int-only approximation to (int)((al1 * 1024L) / vbg)).
    // Assuming typical V supply of 2--3 times Vbandgap,
    // compress above threshold to extend top of range by a factor of two.
    // Ensure that scale stays monotonic in face of calculation lumpiness, etc...
    // Scale all remaining space above threshold to new top value into remaining space.
    // TODO: ensure scaleFactor a power of two for speed.
    const int scaleFactor = (2048 - ADAPTIVE_THRESHOLD) / (1024 - ADAPTIVE_THRESHOLD);
    al = fnmin(1023,
        ADAPTIVE_THRESHOLD + fnmax(0, ((ale - ADAPTIVE_THRESHOLD) / scaleFactor)));
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("Ambient raw: ");
    DEBUG_SERIAL_PRINT(al0);
    DEBUG_SERIAL_PRINT_FLASHSTRING(", against Vcc: ");
    DEBUG_SERIAL_PRINT(al1);
    DEBUG_SERIAL_PRINT_FLASHSTRING(", Vref against Vcc: ");
    DEBUG_SERIAL_PRINT(vbg);
    DEBUG_SERIAL_PRINT_FLASHSTRING(", extended scale value: ");
    DEBUG_SERIAL_PRINT(ale);
    DEBUG_SERIAL_PRINT_FLASHSTRING(", compressed value: ");
    DEBUG_SERIAL_PRINT(al);
    DEBUG_SERIAL_PRINTLN();
#endif
    }
  else { al = al0; }
#else
  const int al = al0;
#endif
  power_intermittent_peripherals_disable();

  // Capture entropy from changed LS bits.
  if((uint8_t)al != (uint8_t)ambientLightLevel) { addEntropyToPool((uint8_t)al ^ (uint8_t)ambientLightLevel, 0); } // Claim zero entropy as may be forced by Eve.

  // Adjust room-lit flag, with hysteresis.
  if(al <= LDR_THR_LOW)
    { isRoomLitFlag = false; }
  else if(al > LDR_THR_HIGH)
    {
    // Treat a sharp transition from dark to light as a possible/weak indication of occupancy, eg light flicked on.
    // TODO: consider refusal to trigger from zero to avoid power-up in light conditions causing transition. 
    if((!isRoomLitFlag) && (ambientLightLevel < LDR_THR_LOW)) { markAsPossiblyOccupied(); }

    isRoomLitFlag = true;
    }

  // Store new value.
  ambientLightLevel = al;

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Ambient light: ");
  DEBUG_SERIAL_PRINT(al);
  DEBUG_SERIAL_PRINTLN();
#endif

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("isRoomLit: ");
  DEBUG_SERIAL_PRINT(isRoomLitFlag);
  DEBUG_SERIAL_PRINTLN();
#endif

  return(al);
  }

#endif


