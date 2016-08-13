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

Author(s) / Copyright (s): Damon Hart-Davis 2013
*/

/*
 Control/model for TRV and boiler.
 */

#include "Control.h"

#include "Ambient_Light_Sensor.h"
#include "EEPROM_Utils.h"
#include "PRNG.h"
#include "RTC_Support.h"
#include "Serial_IO.h"
#include "Schedule.h"
#include "Temperature_Sensor.h"
#include "UI_Minimal.h"

// Percentage open for local TRV being controlled in range [0,100]; 0 is closed/off and is also the initial state.
static uint8_t TRVPercentOpen;
uint8_t getTRVPercentOpen() { return(TRVPercentOpen); }

// Target temperature in Centigrade.
static uint8_t targetTempC;
uint8_t getTargetTempC() { return(targetTempC); }

// Get 'FROST' protection target in C; no higher than getWARMTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
#ifdef SETTABLE_TARGET_TEMPERATURES
// Note that this value is non-volatile (stored in EEPROM).
uint8_t getFROSTTargetC()
  {
  // Get persisted value, if any.
  const uint8_t stored = eeprom_read_byte((uint8_t *)EE_START_FROST_C);
  // If out of bounds or no stored value then use default.
  if((stored < MIN_TARGET_C) || (stored > MAX_TARGET_C)) { return(FROST); }
  // Return valid persisted value.
  return(stored);
  }
#else
#define getFROSTTargetC() (FROST) // Fixed value.
#endif

// Get 'WARM' target in C; no lower than getFROSTTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
#ifdef SETTABLE_TARGET_TEMPERATURES
// Note that this value is non-volatile (stored in EEPROM).
uint8_t getWARMTargetC()
  {
  // Get persisted value, if any.
  const uint8_t stored = eeprom_read_byte((uint8_t *)EE_START_WARM_C);
  // If out of bounds or no stored value then use default (or frost value if set and higher).
  if((stored < MIN_TARGET_C) || (stored > MAX_TARGET_C)) { return(fnmax((uint8_t)WARM, getFROSTTargetC())); }
  // Return valid persisted value (or frost value if set and higher).
  return(fnmax(stored, getFROSTTargetC()));
  }
#else
#define getWARMTargetC() ((uint8_t) (WARM)) // Fixed value.
#endif

#ifdef SETTABLE_TARGET_TEMPERATURES
// Set (non-volatile) 'FROST' protection target in C; no higher than getWARMTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Returns false if not set, eg because outside range [MIN_TARGET_C,MAX_TARGET_C], else returns true.
bool setFROSTTargetC(uint8_t tempC)
  {
  if((tempC < MIN_TARGET_C) || (tempC > MAX_TARGET_C)) { return(false); } // Invalid temperature.
  if(tempC > getWARMTargetC()) { return(false); } // Cannot set above WARM target.
  eeprom_smart_update_byte((uint8_t *)EE_START_FROST_C, tempC); // Update in EEPROM if necessary.
  return(true); // Assume value correctly written.
  }
// Set 'WARM' target in C; no lower than getFROSTTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
// Returns false if not set, eg because below FROST setting or outside range [MIN_TARGET_C,MAX_TARGET_C], else returns true.
bool setWARMTargetC(uint8_t tempC)
  {
  if((tempC < MIN_TARGET_C) || (tempC > MAX_TARGET_C)) { return(false); } // Invalid temperature.
  if(tempC < getFROSTTargetC()) { return(false); } // Cannot set below FROST target.
  eeprom_smart_update_byte((uint8_t *)EE_START_WARM_C, tempC); // Update in EEPROM if necessary.
  return(true); // Assume value correctly written.
  }
#endif


#ifndef getMinBoilerOnMinutes
// Get minimum on (and off) time for pointer (minutes); zero if not in hub mode.
uint8_t getMinBoilerOnMinutes() { return(~eeprom_read_byte((uint8_t *)EE_START_MIN_BOILER_ON_MINS_INV)); }
#endif

#ifndef setMinBoilerOnMinutes
// Set minimum on (and off) time for pointer (minutes); zero to disable hub mode.
// Suggested minimum of 4 minutes for gas combi; much longer for heat pumps for example.
void setMinBoilerOnMinutes(uint8_t mins) { eeprom_smart_update_byte((uint8_t *)EE_START_MIN_BOILER_ON_MINS_INV, ~(mins)); }
#endif

// Minimum slew/error % distance in central range; should be larger than smallest temperature-sensor-driven step (6) to be effective; [1,100].
// Note: keeping TRV_MIN_SLEW_PC sufficiently high largely avoids spurious hunting back and forth from single-ulp noise.
#ifndef TRV_MIN_SLEW_PC
#define TRV_MIN_SLEW_PC 7
#endif
// Set maximum valve slew rate (percent/minute) when close to target temperture.
// Note: keeping TRV_MAX_SLEW_PC_PER_MIN small reduces noise and overshoot and surges of water
// (eg for when charged by the m^3 in district heating systems)
// and will likely work better with high-thermal-mass / slow-response systems such as UFH.
// Should be << 100%/min, and probably << 30%/min, given that 30% may be the effective control range of many rad valves.
#ifndef TRV_MAX_SLEW_PC_PER_MIN
#ifndef TRV_SLEW_GLACIAL
#define TRV_MAX_SLEW_PC_PER_MIN 5 // Maximum normal slew rate (%/min), eg to fully open from off when well under target; [1,100].
#else
#define TRV_MAX_SLEW_PC_PER_MIN 1 // Minimal slew rate (%/min) to keep flow rates as low as possible.
#endif
#endif

// Derived from basic slew values.
#ifndef TRV_SLEW_GLACIAL
#define TRV_SLEW_PC_PER_MIN_VFAST (min(34,(4*TRV_MAX_SLEW_PC_PER_MIN))) // Takes ~3 minutes for full travel.
#define TRV_SLEW_PC_PER_MIN_FAST (min(20,(2*TRV_MAX_SLEW_PC_PER_MIN))) // Takes ~5 minutes for full travel.
#else
#define TRV_SLEW_PC_PER_MIN_FAST TRV_MAX_SLEW_PC_PER_MIN
#define TRV_SLEW_PC_PER_MIN_VFAST TRV_MAX_SLEW_PC_PER_MIN
#endif




#ifdef OCCUPANCY_SUPPORT
// Number of minutes that room is regarded as occupied after markAsOccupied(); strictly positive.
// DHD20130528: no activity for 30 minutes usually enough to declare room empty in my experience.
// Should probably be at least as long as, or a little longer than, the BAKE timeout.
// Should probably be significantly shorter than normal 'learn' on time to allow savings from that in empty rooms.
#define OCCUPATION_TIMEOUT_M (min(max(SETBACK_FULL_M, 30), 255))

// Time until room regarded as unoccupied, in minutes; initially zero (ie treated as unoccupied at power-up).
// (Not volatile since not expected to be used from ISRs.)
static uint8_t occupationCountdownM;

// Returns true if the room appears to be likely occupied (with active users) now or recently.
// Operates on a timeout; calling markAsOccupied() restarts the timer.
// Defaults to false (and API still exists) when OCCUPANCY_SUPPORT not defined.
// Do not call from an ISR.
bool isLikelyOccupied() { return(0 != occupationCountdownM); }

// Returns true if the room appears to be likely occupied (with active users) recently.
// This uses the same timer as isOccupied() (restarted by markAsOccupied())
// but returns to false somewhat sooner for example to allow ramping up more costly occupancy detection methods
// and to allow some simple graduated occupancy responses.
// Do not call from an ISR.
bool isLikelyRecentlyOccupied() { return(occupationCountdownM > OCCUPATION_TIMEOUT_M/2); }

// Call when some strong evidence of room occupation has occurred.
// Such evidence may include operation of buttons (etc) on the unit or PIR.
// Do not call from (for example) 'on' schedule change.
// Do not call from an ISR.
void markAsOccupied() { occupationCountdownM = OCCUPATION_TIMEOUT_M; }

// Call when some/weak evidence of room occupation, such as light going on.
// Also use to simulate demand on behalf of user, eg for some part of schedule.
// In this implementation sets the timeout to half the usual time (unless already higher),
// which shouldn't force the room to appear recently occupied.
// Do not call from an ISR.
void markAsPossiblyOccupied()
  { occupationCountdownM = fmax(occupationCountdownM, OCCUPATION_TIMEOUT_M/2); }

#endif





// Returns true iff there is a full set of stats (none unset) and this 3/4s of the values are higher than supplied sample.
//   * s is start of (24) sample set in EEPROM
//   * sample to be tested for being in lower quartile
static bool inBottomQuartile(uint8_t *sE, const uint8_t sample)
  {
  uint8_t valuesHigher = 0;
  for(int8_t hh = 24; --hh >= 0; ++sE)
    {
    const uint8_t v = eeprom_read_byte(sE); 
    if(STATS_UNSET_INT == v) { return(false); } // Abort if not a full set of stats (eg at least one full days' worth). 
    if(v > sample) { if(++valuesHigher >= 18) { return(true); } } // Stop as soon as known to be in lower quartile.
    }
  return(false); // Not in lower quartile.
  }


// Returns true iff room likely to be occupied and need warming at the specified hour's sample point based on collected stats.
// Used for predictively warming a room in smart mode and for choosing setback depths.
// Returns false if no good evidence to warm the room at the given time based on past history over about one week.
//   * hh hour to check for predictive warming [0,23]
bool shouldBeWarmedAtHour(const uint_least8_t hh)
  {
#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION
  // Return false if the sample hour's historic ambient light level falls in the bottom quartile.
  // Thus avoid any 'smart' warming for at least 25% of the daily cycle.
  const uint8_t smoothedAmbLight = eeprom_read_byte((uint8_t *)(EE_START_LAST_AMBLIGHT_BY_HOUR_SMOOTHED + hh));
  if((STATS_UNSET_INT != smoothedAmbLight) && inBottomQuartile((uint8_t *)EE_START_LAST_AMBLIGHT_BY_HOUR_SMOOTHED, smoothedAmbLight))
    { return(false); }
#endif

  // Return false if no WARM mode this hour for the last week (ie the unit needs reminding at least once per week).
  // Return true if this hour was in WARM mode yesterday or a week ago, and at least one other day.
  const uint8_t warmHistory = eeprom_read_byte((uint8_t *)(EE_START_LAST_WARMMODE_BY_HOUR + hh));
  if(0 == (0x80 & warmHistory)) // This hour has a history.
    {
    if(0 == warmHistory) // No explicit WARM for a week at this hour, so prevent 'smart' warming.
      { return(false); }
    if((0 != (0x41 & warmHistory)) && (0 != (0x3e & warmHistory)))
      { return(true); }
    }

  // Return true if the sample hour is usually warm, ie at or above WARM target.
  const int smoothedTempHHNext = expandTempC16(eeprom_read_byte((uint8_t *)(EE_START_LAST_TEMP_BY_HOUR_SMOOTHED + hh)));
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Smoothed C for ");
  DEBUG_SERIAL_PRINT(hh);
  DEBUG_SERIAL_PRINT_FLASHSTRING("h is ");
  DEBUG_SERIAL_PRINT(smoothedTempHHNext >> 4);
  DEBUG_SERIAL_PRINTLN();
#endif
  if((STATS_UNSET_INT != smoothedTempHHNext) && (((smoothedTempHHNext+8)>>4) >= getWARMTargetC()))
    { return(true); }

  // No good evidence for room to be warmed for specified hour.
  return(false);
  }


// Compute target temperature.
static void computeTargetTemperature()
  {
  if(!inWarmMode()) // In FROST mode.
    {
    const uint8_t frostC = getFROSTTargetC();
    targetTempC = frostC; // Apply FROST safety target temperature by default in FROST mode; no setbacks apply.

#ifdef ENABLE_ANTICIPATION
    if(inSmartMode())
      {
      // Iff in 'smart' mode... (ie don't provide *additional* heating using additional energy unless smartness is enabled)
      // 1) Warm this room if appropriate for the current hour (based on collected stats and current occupancy).
      // 2) Else pre-warm the room (to a setback temperature) if warming would be appropriate for the next hour based on collected stats.
      //
      // Typically this may pre-warm a room before getting up time in winter, getting back from work, or going to bed,
      // and go on warming at the same times each day as long as the user confirms typically once per week.
      //
      // Shouldn't anticipate too far in advance to avoid wasting energy or (eg) waking the user in the morning.
      const uint8_t warmTarget = getWARMTargetC();
      const int currentTempC16 = getTemperatureC16();
      const uint8_t currentTempC = (uint8_t)(currentTempC16 >> 4);
      if(currentTempC < warmTarget) // Only bother with potentially-expensive computations below if not already achieving WARM target!
        {
        const uint_least8_t hh = getHoursLT();
        // Pre-warm target temperature
        const uint8_t preWarmTempC = max(warmTarget - SETBACK_FULL, frostC); // Putative pre-warm temperature...

        // Do 'smart' learned warming for current hour with extra-conservative setbacks unless reasonable evidence of current occupation/activity.
        if(shouldBeWarmedAtHour(hh))
          { targetTempC = (isRoomDark() && isLikelyUnoccupied()) ? preWarmTempC : warmTarget; }
        // Else do predictive pre-warming for next hour or so.
        else if(currentTempC <= preWarmTempC) // Only need to think about pre-warming if not already achieving pre-warm target!
          {
          const uint_least8_t hhNext = (hh < 23) ? (hh+1) : 0;
          // Pre-warm the room, by an extra degree (eg slightly reduced setback) in the last half hour.
          if(shouldBeWarmedAtHour(hhNext))
            { targetTempC = ((currentTempC == preWarmTempC) && (getMinutesLT() >= 30)) ? (preWarmTempC+1) : preWarmTempC; }
          }
        }
      }
#endif
    }

#ifdef SUPPORT_BAKE
  else if(inBakeMode()) // If in BAKE mode then use elevated target.
    {
    // dec bakeCountdownM // Moved management of counter to UI code.
    targetTempC = fnmin((uint8_t)(getWARMTargetC() + BAKE_UPLIFT), (uint8_t)MAX_TARGET_C); // No setbacks apply in BAKE mode.
    }
#endif

  else // In 'WARM' mode with possible setback.
    {
    // TODO: If no schedule set and no occupancy detected for over 1 day, then invoke full setback assuming people are away, eg on holiday.

    // Set back target temperature a little if room is too dark for activity AND room seems unoccupied.
    // TODO: with full occupancy support then allow setback simply based on lack of occupancy.
    if(isRoomDark() && isLikelyUnoccupied())
      {
#ifdef ENABLE_ANTICIPATION
      // Set a 'smarter' setback target temperature based on predicted occupancy, etc, even if not explicitly in 'smart' mode.
      const uint_least8_t hh = getHoursLT();
      const uint8_t sb = shouldBeWarmedAtHour(hh) ? SETBACK : SETBACK_FULL; 
      targetTempC = fnmax((uint8_t)(getWARMTargetC() - sb), getFROSTTargetC());
#else // Lighter-weight code, eg for PICAXE.
      targetTempC = fnmax((uint8_t)(getWARMTargetC() - SETBACK), (uint8_t)MIN_TARGET_C); // Target must never be set low enough to create a frost/freeze hazard.
#endif
      }
    else
      { targetTempC = getWARMTargetC(); } // Room not known to be too dark for normal activity so use WARM target directly.
    }
  }

// Set heat demand with some hysteresis and a hint of proportional control.
// Always be willing to turn off quickly, but on slowly (AKA "slow start" algorithm),
// and try to eliminate unnecessary 'hunting' which makes noise and uses actuator energy.
static bool computeRequiredTRVPercentOpen()
  {
  bool changed = false;

  const int currentTempC16 = getTemperatureC16();
  const uint8_t currentTempC = (uint8_t)(currentTempC16 >> 4);

  if(currentTempC < targetTempC) // (Well) under temp target: open valve.
    {
    // Limit valve open slew to help minimise overshoot and actuator noise.
    // This should also reduce nugatory setting changes when occupancy (etc) is fluctuating.
    // Thus it may take several minutes to turn the radiator fully on,
    // though probably opening the first 30% will allow near-maximum heat output in practice.
    if(TRVPercentOpen != 100)
      {
#if defined(SUPPORT_BAKE) && !defined(TRV_SLEW_GLACIAL)
      // If room is well below target then in BAKE mode immediately open to 100%, unless always glacial.
      // FIXME: use debounced bake mode value to avoid spurious slamming open of the valve if user cycles through modes.
      if(inBakeMode()) { TRVPercentOpen = 100; }
      else
#endif
        {
#if !defined(TRV_SLEW_GLACIAL) // Unless glacial, open faster than usual even with eco bias, and even faster with comfort.
        const uint8_t tmp = TRVPercentOpen + ((!hasEcoBias()) ? TRV_SLEW_PC_PER_MIN_VFAST : TRV_SLEW_PC_PER_MIN_FAST);
#else
        const uint8_t tmp = TRVPercentOpen + TRV_MAX_SLEW_PC_PER_MIN;
#endif
        if(tmp < 100) { TRVPercentOpen = tmp; } else { TRVPercentOpen = 100; } // Capped at 100%.
        }      
      changed = true; // TRV setting has been changed.
      }
    }
  else if(currentTempC > targetTempC) // (Well) over temp target: close valve if not yet closed.
    {
    cancelBake(); // Ensure BAKE mode cancelled immediately if over target (eg when target is BAKE).
    if(TRVPercentOpen != 0)
      {
      // Continue shutting valve as not yet closed.
#if defined(VALVE_TURN_OFF_LINGER) && (DEFAULT_MIN_VALVE_PC_REALLY_OPEN > 0)
      // TODO-117: allow very slow final turn off to help systems with poor bypass, ~1% per minute.
      // Special slow-turn-off rules for final part of travel at/below DEFAULT_MIN_VALVE_PC_REALLY_OPEN floor.
      const uint8_t lingerThreshold = DEFAULT_MIN_VALVE_PC_REALLY_OPEN - 1; // Should be below 'call for heat' threshold.
      if(TRVPercentOpen <= lingerThreshold)
        {
        // If lingered long enough then do final chunk in one burst to help avoid valve hiss and temperature overshoot.
        if((DEFAULT_MAX_RUN_ON_TIME_M < DEFAULT_MIN_VALVE_PC_REALLY_OPEN) && (TRVPercentOpen < DEFAULT_MIN_VALVE_PC_REALLY_OPEN - DEFAULT_MAX_RUN_ON_TIME_M))
          { TRVPercentOpen = 0; } // Shut right off.
        else
          { --TRVPercentOpen; } // Turn down as slowly as reasonably possible to help boiler cool.
        }
     else
#else
      // No special linger threshold.
      const uint8_t lingerThreshold = 0; // Fully off.
#endif    
      // TODO-109: with comfort bias (or at hub because of self-heating temp errors) slew to off relatively slowly.
      if(((!hasEcoBias()) || inHubMode()) && (TRVPercentOpen > lingerThreshold + TRV_SLEW_PC_PER_MIN_VFAST)) { TRVPercentOpen -= TRV_SLEW_PC_PER_MIN_VFAST; }
      // Else (by default) force to (nearly) off immediately when requested, ie eagerly stop heating to conserve energy.
      // In any case percentage open should now be low enough to stop calling for heat immediately.
      else { TRVPercentOpen = lingerThreshold; }
      // TRV setting has been changed (always closes at least a little each time until fully closed).
      changed = true;
      }
    }
  else // Close to temp target: set valve partly open to try to tightly regulate.
    {
    // Use currentTempC16 lsbits to set valve percentage for proportional feedback
    // to provide more efficient and quieter TRV drive and probably more stable room temperature.
    uint8_t tmp = (uint8_t) (currentTempC16 & 0xf); // Only interested in lsbits.
    tmp = 16 - tmp; // Now in range 1 (at warmest end of 'correct' temperature) to 16 (coolest).
    const uint8_t ulpStep = 6;
    // Get to nominal range 6 to 96, eg valve nearly shut just below top of 'correct' temperature window.
    const uint8_t targetPORaw = tmp * ulpStep;
#if defined(VALVE_TURN_OFF_LINGER)
    // Constrain from below to likely minimum-open value, in part to deal with TODO-117 'linger open' in lieu of boiler bypass.
    const uint8_t targetPO = fmax(targetPORaw, DEFAULT_MIN_VALVE_PC_REALLY_OPEN);
#else
    // Use as-is.
    const uint8_t targetPO = targetPORaw;
#endif
    // Reduce spurious valve/boiler adjustment by avoiding movement at all unless current error is significant.
    if(targetPO < TRVPercentOpen) // Currently open more than required.
      {
      const uint8_t slew = TRVPercentOpen - targetPO;
      if(slew >= max((1+ulpStep), TRV_MIN_SLEW_PC)) // Ensure no hunting for 1ulp temperature wobble.
        {
        if(slew > TRV_MAX_SLEW_PC_PER_MIN)
            { TRVPercentOpen -= TRV_MAX_SLEW_PC_PER_MIN; } // Cap slew rate.
        else
            { TRVPercentOpen = targetPO; } // Adjust directly to target.
        changed = true; // TRV setting has been changed.
        }
      }
    else if(targetPO > TRVPercentOpen) // Currently open less than required.
      {
      const uint8_t slew = targetPO - TRVPercentOpen;
      if(slew >= max((1+ulpStep), TRV_MIN_SLEW_PC)) // Ensure no hunting for 1ulp temperature wobble.
        {
        // Slew open faster in BAKE mode or with comfort bias (unless always glacial).
#if !defined(TRV_SLEW_GLACIAL)
        const uint8_t maxSlew = (inBakeMode() || !hasEcoBias()) ? TRV_SLEW_PC_PER_MIN_FAST : TRV_MAX_SLEW_PC_PER_MIN;
#else
        const uint8_t maxSlew = TRV_MAX_SLEW_PC_PER_MIN;
#endif
        if(slew > maxSlew)
            { TRVPercentOpen += maxSlew; } // Cap slew rate.
        else
            { TRVPercentOpen = targetPO; } // Adjust directly to target.
        changed = true; // TRV setting has been changed.
        }
      }
    }

  return(changed);
  }

// Compute target temperature and set heat demand for TRV and boiler.
// CALL APPROXIMATELY ONCE PER MINUTE TO ALLOW SIMPLE TIME-BASED CONTROLS.
// Inputs are inWarmMode(), isRoomLit().
// The inputs must be valid (and recent).
// Values set are targetTempC, TRVPercentOpen.
// This may also prepare data such as TX command sequences for the TRV, boiler, etc.
// This routine may take significant CPU time; no I/O is done, only internal state is updated.
// Returns true if valve target changed and thus messages may need to be recomputed/sent/etc.
bool computeTargetAndDemand()
  {
  // Run down occupation timer if need be.
  if(occupationCountdownM > 0) { --occupationCountdownM; }

  computeTargetTemperature();
  return(computeRequiredTRVPercentOpen());
  }


// The STATS_SMOOTH_SHIFT is chosen to retain some reasonable precision within a byte and smooth over a weekly cycle.
#define STATS_SMOOTH_SHIFT 3 // Number of bits of shift for smoothed value: larger => larger time-constant; strictly positive.

// Compute new linearly-smoothed value given old smoothed value and new value.
// Guaranteed not to produce a value higher than the max of the old smoothed value and the new value.
// Uses stochastic rounding to nearest to allow nominally sub-lsb values to have an effect over time.
// Usually only made public for unit testing.
uint8_t smoothStatsValue(const uint8_t oldSmoothed, const uint8_t newValue)
  {
  if(oldSmoothed == newValue) { return(oldSmoothed); } // Optimisation: smoothed value is unchanged if new value is the same as extant.
  // Compute and update with new stochastically-rounded exponentially-smoothed ("Brown's simple exponential smoothing") value.
  // Stochastic rounding allows sub-lsb values to have an effect over time.
  const uint8_t stocAdd = randRNG8() & ((1 << STATS_SMOOTH_SHIFT) - 1); // Allows sub-lsb values to have an effect over time.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("stocAdd=");
  DEBUG_SERIAL_PRINT(stocAdd);
  DEBUG_SERIAL_PRINTLN();
#endif
  // Do arithmetic in 16 bits to avoid over-/under- flows.
  return((uint8_t) (((((uint16_t) oldSmoothed) << STATS_SMOOTH_SHIFT) - ((uint16_t)oldSmoothed) + ((uint16_t)newValue) + stocAdd) >> STATS_SMOOTH_SHIFT));
  }

// Sample statistics once per hour as background to simple monitoring and adaptive behaviour.
// Call this once per hour with fullSample==true, as near the end of the hour as possible;
// this will update the non-volatile stats record for the current hour.
// Optionally call this at a small (2--10) even number of evenly-spaced number of other times thoughout the hour
// with fullSample=false to sub-sample (and these may receive lower weighting or be ignored).
// (EEPROM wear should not be an issue at this update rate in normal use.)
void sampleStats(const bool fullSample)
  {
  // (Sub-)sample processing.
  static uint8_t sampleCount; // General sub-sample count; initially zero, and zeroed after each full sample.
  const bool firstSample = (0 == sampleCount++);
  // WARM mode count.
  static int8_t warmCount; // Sub-sample WARM count; initially zero, and zeroed after each full sample.
  if(inWarmMode()) { ++warmCount; } else { --warmCount; }
  // Ambient light.
  const int ambLight = getAmbientLight();
  static int ambLightTotal;
  ambLightTotal = firstSample ? ambLight : (ambLightTotal + ambLight);
  const int tempC16 = getTemperatureC16();
  static int tempC16Total;
  tempC16Total = firstSample ? tempC16 : (tempC16Total + tempC16);
  if(!fullSample) { return; } // Only accumulate values cached until a full sample.

  const uint_least8_t hh = getHoursLT(); // Get the current local-time hour...

  // Scale and constrain last-read temperature to valid range for stats.
  const uint8_t temp = compressTempC16((tempC16Total + (sampleCount/2)) / sampleCount);
  // Update the last-sample slot using the mean samples value.
  eeprom_smart_update_byte((uint8_t *)(EE_START_LAST_TEMP_BY_HOUR + hh), temp);
  // If existing smoothed value unset or invalid, use new one as is, else fold in.
  uint8_t *const phT = (uint8_t *)(EE_START_LAST_TEMP_BY_HOUR_SMOOTHED + hh);
  const uint8_t tempSmoothed = eeprom_read_byte(phT);
  if(tempSmoothed > MAX_STATS_TEMP) { eeprom_smart_update_byte(phT, temp); }
  else { eeprom_smart_update_byte(phT, smoothStatsValue(tempSmoothed, temp)); }

  // Scale and constrain mean ambient-light value to valid range for stats; very top of range is compressed to retain maximum gamut.
  const uint8_t ambLShifted = (uint8_t) ((ambLightTotal + (sampleCount<<1)) / (sampleCount<<2));
  const uint8_t ambL = min(ambLShifted, MAX_STATS_AMBLIGHT);
  // Update the last-sample slot using the mean samples value.
  eeprom_smart_update_byte((uint8_t *)(EE_START_LAST_AMBLIGHT_BY_HOUR + hh), ambL);
  // If existing smoothed value unset or invalid, use new one as is, else fold in.
  uint8_t *const phA = (uint8_t *)(EE_START_LAST_AMBLIGHT_BY_HOUR_SMOOTHED + hh);
  const uint8_t ambLSmoothed = eeprom_read_byte(phA);
  if(ambLSmoothed > MAX_STATS_AMBLIGHT) { eeprom_smart_update_byte(phA, ambL); }
  else { eeprom_smart_update_byte(phA, smoothStatsValue(ambLSmoothed, ambL)); }

  // Update sampled WARM-mode value.
  // 0xff when unset/erased; first use will set all history bits to the initial sample value.
  // When in use, bit 7 (msb) is always 0 (to distinguish from unset).
  // Bit 6 is 1 if most recent day's sample was in WARM (or BAKE) mode, 0 if in FROST mode.
  // At each new sampling, bits 6--1 are shifted down and the new bit 6 set as above.
  // Designed to enable low-wear no-write or selective erase/write use much of the time;
  // periods which are always the same mode will achieve a steady-state value (eliminating most EEPROM wear)
  // while even some of the rest (while switching over from all-WARM to all-FROST) will only need pure writes (no erase).
  uint8_t *const phW = (uint8_t *)(EE_START_LAST_WARMMODE_BY_HOUR + hh);
  const uint8_t warmHistory = eeprom_read_byte(phW);
  if(warmHistory & 0x80) { eeprom_smart_clear_bits(phW, inWarmMode() ? 0x7f : 0); } // First use sets all history bits to current sample value.
  else // Shift in today's sample bit value for this hour at bit 6...
    {
    uint8_t newWarmHistory = (warmHistory >> 1) & 0x3f;
    if(warmCount > 0) { newWarmHistory |= 0x40; } // Treat as warm iff more WARM than FROST (sub-)samples.
    eeprom_smart_update_byte(phW, newWarmHistory);
    }
  // Reset WARM sub-sample count after full sample.
  warmCount = 0;



  // TODO: other stats measures...


  // Reset generical sub-sample count to initial state after fill sample.
  sampleCount = 0;
  }

// Get raw stats value for hour HH [0,23] from stats set N from non-volatile (EEPROM) store.
// A value of 0xff (255) means unset (or out of range); other values depend on which stats set is being used.
// The stats set is determined by the order in memory.
uint8_t getByHourStat(uint8_t hh, uint8_t statsSet)
  {
  if(statsSet > (EE_END_STATS - EE_START_STATS) / EE_STATS_SET_SIZE) { return((uint8_t) 0xff); } // Invalid set.
  if(hh > 23) { return((uint8_t) 0xff); } // Invalid hour.
  return(eeprom_read_byte((uint8_t *)(EE_START_STATS + (statsSet * (int)EE_STATS_SET_SIZE) + (int)hh)));
  }


// Clear all collected statistics, eg when moving device to a new room or at a major time change.
// Requires 1.8ms per byte for each byte that actually needs erasing.
//   * maxBytesToErase limit the number of bytes erased to this; strictly positive, else 0 to allow 65536
// Returns true if finished with all bytes erased.
bool zapStats(uint16_t maxBytesToErase)
  {
  for(uint8_t *p = (uint8_t *)EE_START_STATS; p <= (uint8_t *)EE_END_STATS; ++p)
    { if(eeprom_smart_erase_byte(p)) { if(--maxBytesToErase == 0) { return(false); } } } // Stop if out of time...
  return(true); // All done.
  }


// Range-compress an signed int 16ths-Celsius temperature to a unsigned single-byte value < 0xff.
// This preserves at least the first bit after the binary point for all values,
// and three bits after binary point for values in the most interesting mid range around normal room temperatures,
// with transitions at whole degrees Celsius.
// Input values below 0C are treated as 0C, and above 100C as 100C, thus allowing air and DHW temperature values.
#define COMPRESSION_C16_FLOOR_VAL 0 // Floor input value to compression.
#define COMPRESSION_C16_LOW_THRESHOLD (16<<4) // Values in range [COMPRESSION_LOW_THRESHOLD_C16,COMPRESSION_HIGH_THRESHOLD_C16[ have maximum precision.
#define COMPRESSION_C16_LOW_THR_AFTER (COMPRESSION_C16_LOW_THRESHOLD>>3) // Low threshold after compression.
#define COMPRESSION_C16_HIGH_THRESHOLD (24<<4)
#define COMPRESSION_C16_HIGH_THR_AFTER (COMPRESSION_C16_LOW_THR_AFTER + ((COMPRESSION_C16_HIGH_THRESHOLD-COMPRESSION_C16_LOW_THRESHOLD)>>1)) // High threshold after compression.
#define COMPRESSION_C16_CEIL_VAL (100<<4) // Ceiling input value to compression.
#define COMPRESSION_C16_CEIL_VAL_AFTER (COMPRESSION_C16_HIGH_THR_AFTER + ((COMPRESSION_C16_CEIL_VAL-COMPRESSION_C16_HIGH_THRESHOLD) >> 3)) // Ceiling input value after compression.
uint8_t compressTempC16(int tempC16)
  {
  if(tempC16 <= 0) { return(0); } // Clamp negative values to zero.
  if(tempC16 < COMPRESSION_C16_LOW_THRESHOLD) { return(tempC16 >> 3); } // Preserve 1 bit after the binary point (0.5C precision).
  if(tempC16 < COMPRESSION_C16_HIGH_THRESHOLD)
    { return(((tempC16 - COMPRESSION_C16_LOW_THRESHOLD) >> 1) + COMPRESSION_C16_LOW_THR_AFTER); }
  if(tempC16 < COMPRESSION_C16_CEIL_VAL)
    { return(((tempC16 - COMPRESSION_C16_HIGH_THRESHOLD) >> 3) + COMPRESSION_C16_HIGH_THR_AFTER); }
  return(COMPRESSION_C16_CEIL_VAL_AFTER);
  }

// Reverses range compression done by compressTempC16(); results in range [0,100], with varying precision based on original value.
// 0xff (or other invalid) input results in STATS_UNSET_INT. 
int expandTempC16(uint8_t cTemp)
  {
  if(cTemp < COMPRESSION_C16_LOW_THR_AFTER) { return(cTemp << 3); }
  if(cTemp < COMPRESSION_C16_HIGH_THR_AFTER)
    { return(((cTemp - COMPRESSION_C16_LOW_THR_AFTER) << 1) + COMPRESSION_C16_LOW_THRESHOLD); }
  if(cTemp <= COMPRESSION_C16_CEIL_VAL_AFTER)
    { return(((cTemp - COMPRESSION_C16_HIGH_THR_AFTER) << 3) + COMPRESSION_C16_HIGH_THRESHOLD); }
  return(STATS_UNSET_INT); // Invalid/unset input.
  }


// Returns true if system is in 'learn'/smart mode.
// If in 'smart' mode can anticipate user demand to pre-warm rooms, maintain customary temperatures, etc.
// Currently true if any simple schedule is set.
// TODO: maybe only if schedule characteristic of having been set by the learn button.
bool inSmartMode()
  {
  return(isSimpleScheduleSet());
  }

