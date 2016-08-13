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
 Temperature pot module.
 */

#include "Temp_Pot.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "Control.h"
#include "Serial_IO.h"
#include "Power_Management.h"
#include "UI_Minimal.h"

#if V0p2_REV >= 2 // Only supported in REV2 onwards.

// Ambient light levels in range [0,1023].
static int tempPot;

// Minimum change (hysteresis) enforced in 'reduced noise' version value; must be greater than 1.
// Aim to provide reasonable noise immunity, even from an ageing carbon-track pot.
// Allow reasonable remaining granularity of response, at least 10s of distinct positions (>=5 bits).
#define RN_HYST 4

// Bottom and top parts of reduced noise range reserved for forcing FROST or BOOST.
// Should be big enough to hit easily (and must be larger than RN_HYST)
// but not so big as to really constrain the temperature range or cause confusion.
#define RN_FRBO (max(8, 2*RN_HYST))

// Reduced-noise temp pot value in range [0,255].
static uint8_t tempPotReducedNoise;

// Return previously-read (with readTempPot()) temperature pot user control level in range [0,1023]; very fast.
int getTempPot() { return(tempPot); }

// Get reduced-noise temperature pot user control value (previously read with readTempPot()) in range [0,255]; very fast.
// In particular this value should not jitter between readings even if the pot is sitting at an ADC boundary level.
uint8_t getTempPotReducedNoise() { return(tempPotReducedNoise); }

// Read the user 'temperature pot' setting in range [0,1023]; higher value implies higher target temperature.
// This may consume significant power and time.
int readTempPot()
  {
  power_intermittent_peripherals_enable(false); // No need to wait for anything to stablise as direct of IO_POWER_UP.
  const int tpRaw = analogueNoiseReducedRead(TEMP_POT_AIN, DEFAULT); // Vcc reference.
  power_intermittent_peripherals_disable();

#if defined(TEMP_POT_REVERSE)
  const int tp = TEMP_POT_RAW_MAX - tpRaw; // Travel is in opposite direction to natural!
#else
  const int tp = tpRaw;
#endif

  // TODO: capture entropy from changed LS bits esp if reduced-noise version doesn't change.

  // Store new value.
  tempPot = tp;

  // Capture reduced-noise value with a little hysteresis.
  const int shifted = tp >> 2; // Keep signed to avoid wrap-round confusion.
  if(((shifted > tempPotReducedNoise) && (shifted - tempPotReducedNoise >= RN_HYST)) ||
     ((shifted < tempPotReducedNoise) && (tempPotReducedNoise - shifted >= RN_HYST)))
    {
    const uint8_t rn = (uint8_t) shifted;

    // Smart responses to adjustment/movement of temperature pot.
    // Possible to get reasonable functionality without using MODE button.
    //
    // NOTE: without ignoredFirst this will also respond to the initial position of the pot
    //   as the first reading is taken, ie may force to WARM or BAKE.
    static bool ignoredFirst;
    if(!ignoredFirst) { ignoredFirst = true; }
    // Force FROST mode when right at bottom of dial.
    else if(rn < RN_FRBO) { setWarmModeDebounced(false); }
#ifdef SUPPORT_BAKE // IF DEFINED: this unit supports BAKE mode.
    // Start BAKE mode when dial turned up to top.
    else if(rn > (255-RN_FRBO)) { startBakeDebounced(); }
#endif
    // Force WARM mode if pot/temperature turned up.
    else if(rn > tempPotReducedNoise) { setWarmModeDebounced(true); }

    tempPotReducedNoise = rn;
    markUIControlUsed(); // Note user operation of pot.
    }

#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Temp pot: ");
  DEBUG_SERIAL_PRINT(tp);
  DEBUG_SERIAL_PRINT_FLASHSTRING(", rn: ");
  DEBUG_SERIAL_PRINT(tempPotReducedNoise);
  DEBUG_SERIAL_PRINTLN();
#endif

  return(tp);
  }

#endif


