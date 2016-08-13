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
 Control/model for TRV and boiler.
 */

#include "Control.h"

#include "Ambient_Light_Sensor.h"
#include "EEPROM_Utils.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "Humidity_Sensor.h"
#include "PRNG.h"
#include "Power_Management.h"
#include "RFM22_Radio.h"
#include "RTC_Support.h"
#include "Security.h"
#include "Serial_IO.h"
#include "Schedule.h"
#include "Temperature_Sensor.h"
#include "Temp_Pot.h"
#include "UI_Minimal.h"


// Percentage open for local TRV being controlled in range [0,100]; 0 is closed/off and is also the initial state.
static uint8_t TRVPercentOpen;
uint8_t getTRVPercentOpen() { return(TRVPercentOpen); }

// Target temperature in Centigrade.
static uint8_t targetTempC;
uint8_t getTargetTempC() { return(targetTempC); }

// Get 'FROST' protection target in C; no higher than getWARMTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
#if defined(TEMP_POT_AVAILABLE)
// Derived from temperature pot position.
uint8_t getFROSTTargetC()
  {
  // Crudely select between eco/comfort FROST levels at the half-way mark.
  // Should not be critical enough to worry over.
  // Prevent falling to lowest frost temperature if relative humidity is high (eg to avoid mould).
  if(!hasEcoBias() || (isRHAvailable() && isRHHigh())) { return(BIASCOM_FROST); }
  return(BIASECO_FROST); // Default is 'eco' notion of frost protection.
  }
#elif defined(SETTABLE_TARGET_TEMPERATURES)
// Note that this value is non-volatile (stored in EEPROM).
uint8_t getFROSTTargetC()
  {
  // Get persisted value, if any.
  const uint8_t stored = eeprom_read_byte((uint8_t *)EE_START_FROST_C);
  // If out of bounds or no stored value then use default.
  if((stored < MIN_TARGET_C) || (stored > MAX_TARGET_C)) { return(FROST); }
  // Prevent falling to lowest frost temperature if relative humidity is high (eg to avoid mould).
  if(!hasEcoBias() || (isRHAvailable() && isRHHigh())) { return(fmax(stored, BIASCOM_FROST)); }
  // Return valid persisted value.
  return(stored);
  }
#else
#define getFROSTTargetC() (FROST) // Fixed value.
#endif

// Get 'WARM' target in C; no lower than getFROSTTargetC() returns, strictly positive, in range [MIN_TARGET_C,MAX_TARGET_C].
#if defined(TEMP_POT_AVAILABLE)
// Derived from temperature pot position, 0 for coldest (most eco), 255 for hotest (comfort).
// Temp ranges from eco-1C to comfort+1C levels across full (reduced jitter) [0,255] pot range.
// Should be fastest computing values at extreme ends of range.
uint8_t computeWARMTargetC(const uint8_t pot)
  {
  const uint8_t low = (BIASECO_WARM-1);
  const uint8_t high = (BIASCOM_WARM+1);
  const uint8_t range = high - low + 1;
  const uint8_t band = 256 / range; // Width of band for each degree C...

  // If relatively small number of distinct temperature values...
  if(pot >= 256 - band) { return(high); } // At top... (optimisation / robustness)
  if(range < 10)
    {
    uint8_t result = low;
    for(uint8_t ppot = band; ppot < pot; ++result) { ppot += band; }
    return(result);
    }
  else
    {
    if(pot < band) { return(low); } // At bottom... (optimisation / robustness)
    return((pot / band) + low); // Intermediate (requires expensive run-time division).
    }
  }

// Exposed implementation.
// Uses cache to avoid expensive recomputation.
// NOT safe in face of interrupts.
uint8_t getWARMTargetC()
  {
  const uint8_t pot = getTempPotReducedNoise();

  // Cached input and result values; initially zero.
  static uint8_t potLast;
  static uint8_t resultLast;
  // Force recompuation if pot value changed
  // or apparently no calc done yet (unlikely/impossible zero cached result).
  if((potLast != pot) || (0 == resultLast))
    {
    const uint8_t result = computeWARMTargetC(pot);
    // Cache input/result.
    resultLast = result;
    potLast = pot;
    return(result);
    }

  // Return cached result.
  return(resultLast);
  }


#elif defined(SETTABLE_TARGET_TEMPERATURES)
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

#if defined(SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
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
#define TRV_MIN_SLEW_PC_PER_MIN 1 // Minimal slew rate (%/min) to keep flow rates as low as possible.
#define TRV_MAX_SLEW_PC_PER_MIN 5 // Maximum normal slew rate (%/min), eg to fully open from off when well under target; [1,100].
#else
#define TRV_MAX_SLEW_PC_PER_MIN TRV_MIN_SLEW_PC_PER_MIN
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

// Returns true if the room appears to be likely occupied (with active users) now.
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

// Two-bit occupancy: (00 not disclosed,) 1 not occupied, 2 possibly occupied, 3 probably occupied.
// 0 is not returned by this implementation.
uint8_t twoBitOccupancyValue() { return(isLikelyRecentlyOccupied() ? 3 : (isLikelyOccupied() ? 2 : 1)); }

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
// Can be called as often as require though may be slow/expensive.
// Will be called by computeCallForHeat().
void computeTargetTemperature()
  {
#if defined(TEMP_POT_AVAILABLE)
  // Force up-to-date reading of temperature pot.
  readTempPot();
#endif

  if(!inWarmModeDebounced()) // In FROST mode.
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
  else if(inBakeModeDebounced()) // If in BAKE mode then use elevated target.
    {
    // dec bakeCountdownM // Moved management of counter to UI code.
    targetTempC = fnmin((uint8_t)(getWARMTargetC() + BAKE_UPLIFT), (uint8_t)MAX_TARGET_C); // No setbacks apply in BAKE mode.
    }
#endif

  else // In 'WARM' mode with possible setback.
    {
    // TODO: If no schedule set and no occupancy detected for over 1 day, then invoke full setback assuming people are away, eg on holiday.

    // Set back target temperature a little if room is too dark for activity AND room seems unoccupied
    // AND the unit has an eco bias or no schedule is on WARM at the moment (TODO-111).
    // TODO: with full occupancy support then allow setback simply based on lack of occupancy.
    const bool scheduledOn = isAnyScheduleOnWARMNow();
    if(((!scheduledOn) || hasEcoBias()) && // TODO-111: suppress set-back from lack of occupancy in scheduled on-time in comfort mode.
       isRoomDark() && isLikelyUnoccupied()) // Only setback if apparently unoccupied.  
      {
      const uint8_t wt = getWARMTargetC();
      // Use a bigger setback if extreme eco bias, unless in scheduled on-time.
      const uint8_t setback = ((!scheduledOn) && isEcoTemperature(wt)) ? SETBACK_ECO : SETBACK_DEFAULT;
#ifdef ENABLE_ANTICIPATION
      // Set a 'smarter' setback target temperature based on predicted occupancy, etc, even if not explicitly in 'smart' mode.
      const uint_least8_t hh = getHoursLT();
      const uint8_t sba = shouldBeWarmedAtHour(hh) ? setback : SETBACK_FULL; 
      targetTempC = fnmax((uint8_t)(wt - sba), getFROSTTargetC());
#else // Lighter-weight code, eg for PICAXE.
      targetTempC = fnmax((uint8_t)(wt - setback), (uint8_t)MIN_TARGET_C); // Target must never be set low enough to create a frost/freeze hazard.
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
      if(inBakeModeDebounced()) { TRVPercentOpen = 100; }
      else
#endif
        {
#if !defined(TRV_SLEW_GLACIAL) // Unless glacial, open faster than usual even with eco bias, and even faster with comfort.
        const uint8_t tmp = TRVPercentOpen + ((!hasEcoBias()) ? TRV_SLEW_PC_PER_MIN_VFAST : TRV_SLEW_PC_PER_MIN_FAST);
#else
        const uint8_t tmp = TRVPercentOpen + TRV_MAX_SLEW_PC_PER_MIN;
#endif
        if(tmp > 100) { TRVPercentOpen = 100; } // Capped at 100%.
        else
          {
#if defined(VALVE_TURN_OFF_LINGER) && (DEFAULT_MIN_VALVE_PC_REALLY_OPEN > TRV_MAX_SLEW_PC_PER_MIN)
          // Ensure valve will be immediately significantly opened (and that linger can work properly).
          if(tmp < DEFAULT_MIN_VALVE_PC_REALLY_OPEN) { TRVPercentOpen = DEFAULT_MIN_VALVE_PC_REALLY_OPEN; }
          else
#endif
            { TRVPercentOpen = tmp; }
          }
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
  else // Close to (or at) temp target: set valve partly open to try to tightly regulate.
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
    const uint8_t targetPO = fmax(targetPORaw, getMinValvePcReallyOpen());
#else
    // Use as-is.
    const uint8_t targetPO = targetPORaw;
#endif
    // Reduce spurious valve/boiler adjustment by avoiding movement at all unless current error is significant.
    if(targetPO != TRVPercentOpen)
      {
      // Compute the minimum/epsilon slew adjustment allowed.
      // Raise this minimum in dark/quiet/unoccupied room to minimise disturbance and battery use.
      const bool minimiseSlew = isRoomDark() || isLikelyUnoccupied();
      const uint8_t minAbsSlew = fmax((1+ulpStep), minimiseSlew ? (2*TRV_MIN_SLEW_PC) : TRV_MIN_SLEW_PC);
      if(targetPO < TRVPercentOpen) // Currently open more than required.
        {
        const uint8_t slew = TRVPercentOpen - targetPO;
        if(slew >= minAbsSlew) // Ensure no hunting for 1ulp temperature wobble.
          {
          if(slew > TRV_MAX_SLEW_PC_PER_MIN)
              { TRVPercentOpen -= TRV_MAX_SLEW_PC_PER_MIN; } // Cap slew rate.
          else
              { TRVPercentOpen = targetPO; } // Adjust directly to target.
          changed = true; // TRV setting has been changed.
          }
        }
      else // if(targetPO > TRVPercentOpen) // Currently open less than required.
        {
        const uint8_t slew = targetPO - TRVPercentOpen;
        if(slew >= minAbsSlew) // Ensure no hunting for 1ulp temperature wobble.
          {
          // Slew open faster in BAKE mode or with comfort bias (unless always glacial).
#if !defined(TRV_SLEW_GLACIAL)
          const uint8_t maxSlew = (inBakeModeDebounced() || !hasEcoBias()) ? TRV_SLEW_PC_PER_MIN_FAST : TRV_MAX_SLEW_PC_PER_MIN;
#else
          const uint8_t maxSlew = TRV_MIN_SLEW_PC_PER_MIN;
#endif
          if(slew > maxSlew)
              { TRVPercentOpen += maxSlew; } // Cap slew rate.
          else
              { TRVPercentOpen = targetPO; } // Adjust directly to target.
          changed = true; // TRV setting has been changed.
          }
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
bool computeCallForHeat()
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
  if(inWarmModeDebounced()) { ++warmCount; } else { --warmCount; }
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
  if(warmHistory & 0x80) { eeprom_smart_clear_bits(phW, inWarmModeDebounced() ? 0x7f : 0); } // First use sets all history bits to current sample value.
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
  return(isAnySimpleScheduleSet());
  }


// If true (the default) then the system has an 'Eco' energy-saving bias, else it has a 'comfort' bias.
// Several system parameters are adjusted depending on the bias,
// with 'eco' slanted toward saving energy, eg with lower target temperatures and shorter on-times.
#ifndef hasEcoBias // If not a macro...
#if defined(TEMP_POT_AVAILABLE)
// Optimisiation/simplication: true if temp pot less than half way.
bool hasEcoBias() { return(getTempPotReducedNoise() < 128); }
#else
// True if WARM temperature at/below halfway mark between eco and comfort levels.
bool hasEcoBias() { return(getWARMTargetC() <= ((BIASECO_WARM + BIASCOM_WARM)/2)); }
#endif
#endif





// Return minimum valve percentage open to be considered actually/significantly open; [1,100].
// At the boiler hub this is also the threshold precentage-open on eavesdropped requests that will call for heat.
// If no override is set then DEFAULT_MIN_VALVE_PC_REALLY_OPEN is used.
// NOTE: raising this value temporarily (and shutting down the boiler immediate if possible) is one way to implement dynamic demand.
uint8_t getMinValvePcReallyOpen()
  {
  const uint8_t stored = eeprom_read_byte((uint8_t *)EE_START_MIN_VALVE_PC_REALLY_OPEN);
  if((stored > 0) && (stored <= 100)) { return(stored); }
  return(DEFAULT_MIN_VALVE_PC_REALLY_OPEN);
  }

// Set percent open to be considered really open.
// Applies to local valve and, at hub, to calls for remote calls for heat.
// Any out-of-range value (eg >100) clears the override and DEFAULT_MIN_VALVE_PC_REALLY_OPEN will be used.
void setMinValvePcReallyOpen(uint8_t percent)
  {
  if((percent > 100) || (percent == 0) || (percent == DEFAULT_MIN_VALVE_PC_REALLY_OPEN))
    {
    // Bad / out-of-range / default value so erase stored value if not already so.
    eeprom_smart_erase_byte((uint8_t *)EE_START_MIN_VALVE_PC_REALLY_OPEN);
    return;
    }
  // Store specified value with as low wear as possible.
  eeprom_smart_update_byte((uint8_t *)EE_START_MIN_VALVE_PC_REALLY_OPEN, percent);
  }




// Clear and populate core stats structure with information from this node.
// Exactly what gets filled in will depend on sensors on the node,
// and may depend on stats TX security level (eg if collecting some sensitive items is also expensive).
void populateCoreStats(FullStatsMessageCore_t *const content)
  {
  clearFullStatsMessageCore(content); // Defensive programming: all fields should be set explicitly below.
  if(localFHT8VTRVEnabled())
    {
    // Use FHT8V house codes if available.
    content->id0 = FHT8VGetHC1();
    content->id1 = FHT8VGetHC2();
    }
  else
    {
    // Use OpenTRV unique ID if no other higher-priority ID.
    content->id0 = eeprom_read_byte(0 + (uint8_t *)EE_START_ID);
    content->id1 = eeprom_read_byte(1 + (uint8_t *)EE_START_ID);
    }
  content->containsID = true;
  content->tempAndPower.tempC16 = getTemperatureC16();
  content->tempAndPower.powerLow = isBatteryLow();
  content->containsTempAndPower = true;
  content->ambL = fnmax(1, fnmin(254, getAmbientLight() >> 2)); // Coerce to allowed value in range [1,254]. Bug-fix c/o Gary Gladman!
  content->containsAmbL = true;
  // OC1/OC2 = Occupancy: 00 not disclosed, 01 not occupied, 10 possibly occupied, 11 probably occupied.
  // The encodeFullStatsMessageCore() route should omit data not appopriate for security reasons.
  content->occ = twoBitOccupancyValue();
  }








// Call this to do an I/O poll if needed; returns true if something useful happened.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// Limits actual poll rate to something like once every 32ms, unless force is true.
//   * force if true then force full poll on every call (ie do not internally rate-limit)
bool pollIO(const bool force)
  {
#if defined(ENABLE_BOILER_HUB) && defined(USE_MODULE_FHT8VSIMPLE)
  if(inHubMode())
    {
    static volatile uint8_t _pO_lastPoll;

    // Poll RX at most about every ~32ms to help approx match spil rate when called in loop with 30ms nap.
    const uint8_t sct = getSubCycleTime();
    if(force || ((0 == (sct & 3)) && (sct != _pO_lastPoll)))
      {
      _pO_lastPoll = sct;
      if(FHT8VCallForHeatPoll()) // Check if call-for-heat has been overheard.
        { return(true); }
      }
    }
#endif
  return(false);
  }










// 'Elapsed minutes' count of minute/major cycles; cheaper than accessing RTC and not tied to real time.
static uint8_t minuteCount;

#if defined(ENABLE_BOILER_HUB)
// Ticks until locally-controlled boiler should be turned off; boiler should be on while this is positive.
// Ticks are the mail loop time, 1s or 2s.
// Used in hub mode only.
static uint16_t boilerCountdownTicks;
// Minutes since boiler last on as result of remote call for heat.
// Reducing listening if quiet for a while helps reduce self-heating temperature error
// (~2C as of 2013/12/24 at 100% RX, ~100mW heat dissipation in V0.2 REV1 box) and saves some energy.
// Time thresholds could be affected by eco/comfort switch.
#define RX_REDUCE_MIN_M 20 // Minimum minutes quiet before considering reducing RX duty cycle listening for call for heat; [1--255], 10--60 typical.
// IF DEFINED then give backoff threshold to minimise duty cycle.
// #define RX_REDUCE_MAX_M 240 // Minutes quiet before considering maximally reducing RX duty cycle; ]RX_REDUCE_MIN_M--255], 30--240 typical.
static uint8_t boilerNoCallM;
#endif


// Controller's view of Least Significiant Digits of the current (local) time, in this case whole seconds.
// See PICAXE V0.1/V0.09/DHD201302L0 code.
#define TIME_LSD_IS_BINARY // TIME_LSD is in binary (cf BCD).
#define TIME_CYCLE_S 60 // TIME_LSD ranges from 0 to TIME_CYCLE_S-1, also major cycle length.
static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.

void setupOpenTRV()
  {
  // Set appropriate loop() values just before entering it.
  TIME_LSD = getSecondsLT();
  }

// Main loop for OpenTRV radiator control.
// Note: exiting and re-entering can take a little while, handling Arduino background tasks such as serial.
void loopOpenTRV()
  {
#if 0 && defined(DEBUG) // Indicate loop start.
  DEBUG_SERIAL_PRINT('L');
  DEBUG_SERIAL_PRINT(TIME_LSD);
  DEBUG_SERIAL_PRINTLN();
#endif


  // Set up some variables before sleeping to minimise delay/jitter after the RTC tick.
  bool showStatus = false; // Show status at end of loop?

  // Use the zeroth second in each minute to force extra deep device sleeps/resets, etc.
  const bool second0 = (0 == TIME_LSD);
  // Sensor readings are taken late in each minute (where they are taken)
  // and if possible noise and heat and light should be minimised in this part of each minute to improve readings.
//  const bool sensorReading30s = (TIME_LSD >= 30);
  // Sensor readings and (stats transmissions) are nominally on a 4-minute cycle.
  const uint8_t minuteFrom4 = (minuteCount & 3);
  // The 0th minute in each group of four is always used for measuring where possible (possibly amongst others)
  // and where possible locally-generated noise and heat and light should be minimised in this minute to give the best possible readings.
  // This is the first (0th) minute in each group of four.
  const bool minute0From4ForSensors = (0 == minuteFrom4);
  // True the minute after all sensors should have been sampled.
  const bool minute1From4AfterSensors = (1 == minuteFrom4);

  // Note last-measured battery status.
  const bool batteryLow = isBatteryLow();

  // Run some tasks less often when not demanding heat (at the valve or boiler), so as to conserve battery/energy.
  const bool conserveBattery =
    (batteryLow || !inWarmModeDebounced()) && // Don't spare the batteries unless low, or in FROST mode (which should be most of the time).
#if defined(ENABLE_BOILER_HUB)
    (0 == boilerCountdownTicks) && // Unless the boiler is off, stay responsive.
#endif
    (!isControlledValveOpen()) &&  // Run at full speed until the FHT8V valve should actually have shut and the boiler gone off.
    (0 == getTRVPercentOpen()); // Run at full speed until not nominally demanding heat, eg even during FROST mode or pre-heating.

  // Try if very near to end of cycle and thus causing an overrun.
  // Conversely, if not true, should have time to savely log outputs, etc.
  const uint8_t nearOverrunThreshold = GSCT_MAX - 8; // ~64ms/~32 serial TX chars of grace time...
  bool tooNearOverrun = false; // Set flag that can be checked later.

  // Is this unit currently in central hub listener mode?
  const bool hubMode = inHubMode();

  // Check (early) for any remote stats arriving to dump.
  // This is designed to be easy to pick up by reading the serial output.
  // The output is terse to avoid taking too long and possibly delaying other stuff too far.
  // Avoid doing this at all if too near the end of the cycle and risking overrun,
  // leaving any message queued, hoping it does not get overwritten.
  // TODO: safely process more than one pending message if present.
  // TODO: move to process in a batch periodically, eg when CLI is due.
  if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }
  else
    {
    // Look for binary-format message.
    FullStatsMessageCore_t stats;
    getLastCoreStats(&stats);
    if(stats.containsID)
      {
      // Dump (remote) stats field '@<hexnodeID>;TnnCh[P;]'
      // where the T field shows temperature in C with a hex digit after the binary point indicated by C
      // and the optional P field indicates low power.
      serialPrintAndFlush(LINE_START_CHAR_RSTATS);
      serialPrintAndFlush((((uint16_t)stats.id0) << 8) | stats.id1, HEX);
      if(stats.containsTempAndPower)
        {
        serialPrintAndFlush(F(";T"));
        serialPrintAndFlush(stats.tempAndPower.tempC16 >> 4, DEC);
        serialPrintAndFlush('C');
        serialPrintAndFlush(stats.tempAndPower.tempC16 & 0xf, HEX);
        if(stats.tempAndPower.powerLow) { serialPrintAndFlush(F(";P")); } // Insert power-low field if needed.
        }
      if(stats.containsAmbL)
        {
        serialPrintAndFlush(F(";L"));
        serialPrintAndFlush(stats.ambL);
        }
      if(0 != stats.occ)
        {
        serialPrintAndFlush(F(";O"));
        serialPrintAndFlush(stats.occ);
        }
      serialPrintlnAndFlush();
      }
    // Check for JSON/text-format message if no binary message waiting.
    else
      {
      char buf[MSG_JSON_MAX_LENGTH+1];
      getLastJSONStats(buf);
      if('\0' != *buf)
        {
        // Dump contained JSON message as-is at start of line.
        serialPrintAndFlush(buf);
        serialPrintlnAndFlush();
        }
      }
    }

  // IF IN CENTRAL HUB MODE: listen out for OpenTRV units calling for heat.
  // Power optimisation 1: when >> 1 TX cycle (of ~2mins) need not listen, ie can avoid enabling receiver.
  // Power optimisation 2: TODO: when (say) >>30m since last call for heat then only sample listen for (say) 3 minute in 10 (not at a TX cycle multiple).
  // TODO: These optimisation are more important when hub unit is running a local valve
  // to avoid temperature over-estimates from self-heating,
  // and could be disabled if no local valve is being run to provide better response to remote nodes.
  bool hubModeBoilerOn = false; // If true then remote call for heat is in progress.
#if defined(USE_MODULE_FHT8VSIMPLE)
  bool needsToEavesdrop = false; // By default assume no need to eavesdrop.
#endif
  if(hubMode)
    {
#if defined(USE_MODULE_FHT8VSIMPLE)
    // Final poll to to cover up to end of previous minor loop.
    // Keep time from here to following SetupToEavesdropOnFHT8V() as short as possible to avoid missing remote calls.
    FHT8VCallForHeatPoll();

    // Fetch and clear current pending sample house code calling for heat.
    const uint16_t hcRequest = FHT8VCallForHeatHeardGetAndClear();
    const bool heardIt = (hcRequest != ((uint16_t)~0));
    // Don't log call for hear if near overrun,
    // and leave any error queued for next time.
    if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }
    else
      {
      if(heardIt)
        {
        DEBUG_SERIAL_TIMESTAMP();
        DEBUG_SERIAL_PRINT(' ');
        serialPrintAndFlush(F("CfH ")); // Call for heat from 
        serialPrintAndFlush((hcRequest >> 8) & 0xff);
        serialPrintAndFlush(' ');
        serialPrintAndFlush(hcRequest & 0xff);
        serialPrintlnAndFlush();
        }
      else
        {
        // Check for error if nothing received.
        const uint8_t err = FHT8VLastRXErrGetAndClear();
        if(0 != err)
          {
          serialPrintAndFlush(F("!RXerr F"));
          serialPrintAndFlush(err);
          serialPrintlnAndFlush();
          }
        }
      }

    // Record call for heat, both to start boiler-on cycle and to defer need to listen again. 
    // Optimisation: may be able to stop RX if boiler is on for local demand (can measure local temp better: less self-heating).
    if(heardIt)
      {
      if(0 == boilerCountdownTicks)
        {
        if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }
        else { serialPrintlnAndFlush(F("RCfH1")); } // Remote call for heat on.
        }
      boilerCountdownTicks = getMinBoilerOnMinutes() * (60/MAIN_TICK_S);
      boilerNoCallM = 0; // No time has passed since the last call.
      }
    // Else count down towards boiler off.
    else if(boilerCountdownTicks > 0)
      {
      if(0 == --boilerCountdownTicks)
        {
        if(getSubCycleTime() >= nearOverrunThreshold) { tooNearOverrun = true; }
        else { serialPrintlnAndFlush(F("RCfH0")); } // Remote call for heat off
        }
      }
    // Else already off so count up quiet minutes...
    else if(second0 && (boilerNoCallM < (uint8_t)~0)) { ++boilerNoCallM; }         

    // Turn boiler output on or off in response to calls for heat.
    hubModeBoilerOn = (boilerCountdownTicks > 0);

    // If not running a local TRV, and this without local temperature measurement problems from self-heating,
    // then just listen all the time for maximum simplicity and responsiveness at some cost in extra power consumption.
    // (At least as long as power is not running low for some reasons.)
    if(!localFHT8VTRVEnabled() && !batteryLow)
      { needsToEavesdrop = true; }
    // Try to avoid listening in the 'quiet' sensor minute in order to minimise noise and power consumption and self-heating.
    // Optimisation: if just heard a call need not listen on this next cycle.
    // Optimisation: if boiler timeout is a long time away (>> one FHT8V TX cycle, ~2 minutes excl quiet minute), then can avoid listening for now.
    //    Longish period without any RX listening may allow hub unit to cool and get better sample of local temperature if marginal.
    // Aim to listen in one stretch for greater than full FHT8V TX cycle of ~2m to avoid missing a call for heat.
    // MUST listen for all of final 2 mins of boiler-on to avoid missing TX (without forcing boiler over-run).
    else if((boilerCountdownTicks <= ((MAX_FHT8V_TX_CYCLE_HS+1)/(2*MAIN_TICK_S))) && // Don't miss a final TX that would keep the boiler on...
       (boilerCountdownTicks != 0)) // But don't force unit to listen/RX all the time if no recent call for heat.
      { needsToEavesdrop = true; }
    else if((!heardIt) &&
       (!minute0From4ForSensors) &&
       (boilerCountdownTicks <= (RX_REDUCE_MIN_M*(60/MAIN_TICK_S)))) // Listen eagerly for fresh calls for heat for last few minutes before turning boiler off.
      {
#if defined(RX_REDUCE_MAX_M) && defined(LOCAL_TRV)
      // Skip the minute before the 'quiet' minute also in very quiet mode to improve local temp measurement.
      // (Should still catch at least one TX per 4 minutes at worst.)
      needsToEavesdrop =
          ((boilerNoCallM <= RX_REDUCE_MAX_M) || (3 != (minuteCount & 3)));
#else
      needsToEavesdrop = true;
#endif
      }

#endif
    }

#if defined(USE_MODULE_FHT8VSIMPLE)
  // Act on eavesdropping need, setting up or clearing down hooks as required.
  if(needsToEavesdrop)
    {
    // Ensure radio is in RX mode rather than standby, and possibly hook up interrupts if available (REV1 board).
    SetupToEavesdropOnFHT8V(second0); // Start listening (if not already so).
#if 0 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("hub listen, on/cd ");
    DEBUG_SERIAL_PRINT(boilerCountdownTicks);
    DEBUG_SERIAL_PRINT_FLASHSTRING("t quiet ");
    DEBUG_SERIAL_PRINT(boilerNoCallM);
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("m");
#endif
    }
  else
    {
    // Power down and clear radio state (if currently eavesdropping).
    StopEavesdropOnFHT8V(second0);
    // Clear any RX state so that nothing stale is carried forward.
    FHT8VCallForHeatHeardGetAndClear();
    }
#endif


  // Set BOILER_OUT as appropriate for local and/or remote calls for heat.
  // FIXME: local valve-driven boiler on does not obey normal on/off run-time rules.
  fastDigitalWrite(OUT_HEATCALL, ((hubModeBoilerOn || isControlledValveOpen()) ? HIGH : LOW));


  // Sleep in low-power mode (waiting for interrupts) until seconds roll.
  // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
  // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*E"); // End-of-cycle sleep.
#endif
  powerDownSerial(); // Ensure that serial I/O is off.
  // Power down most stuff (except radio for hub RX).
  minimisePowerWithoutSleep();
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = getSecondsLT()))
    {
#if defined(ENABLE_BOILER_HUB) && defined(USE_MODULE_FHT8VSIMPLE) // Deal with FHT8V eavesdropping if needed.
    // Poll for RX of remote calls-for-heat if needed.
    if(needsToEavesdrop) { nap30AndPoll(); continue; }
#endif
#if defined(USE_MODULE_RFM22RADIOSIMPLE) // Force radio to power-saving standby state if appropriate.
    // Force radio to known-low-power state from time to time (not every time to avoid unnecessary SPI work, LED flicker, etc.)
    if(batteryLow || second0) { RFM22ModeStandbyAndClearState(); }
#endif
    sleepUntilInt(); // Normal long minimal-power sleep until wake-up interrupt.
    }
  TIME_LSD = newTLSD;
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif

#if defined(ENABLE_BOILER_HUB) && defined(USE_MODULE_FHT8VSIMPLE) // Deal with FHT8V eavesdropping if needed.
  // Check RSSI...
  if(needsToEavesdrop)
    {
    const uint8_t rssi = RFM22RSSI();
    static uint8_t lastRSSI;
    if((rssi > 0) && (lastRSSI != rssi))
      {
      lastRSSI = rssi;
      addEntropyToPool(rssi, 0); // Probably some real entropy but don't assume it.
#if 0 && defined(DEBUG)
      DEBUG_SERIAL_PRINT_FLASHSTRING("RSSI=");
      DEBUG_SERIAL_PRINT(rssi);
      DEBUG_SERIAL_PRINTLN();
#endif
      }
    }
#endif

#if 0 && defined(DEBUG) // Show CPU cycles.
  DEBUG_SERIAL_PRINT('C');
  DEBUG_SERIAL_PRINT(cycleCountCPU());
  DEBUG_SERIAL_PRINTLN();
#endif


  // START LOOP BODY
  // ===============


  // Warn if too near overrun before.
  if(tooNearOverrun) { serialPrintlnAndFlush(F("?near overrun")); }


  // Get current power supply voltage.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Vcc: ");
  DEBUG_SERIAL_PRINT(readBatterymV());
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("mV");
#endif


#if defined(USE_MODULE_FHT8VSIMPLE)
  // FHT8V is highest priority and runs first.
  // ---------- HALF SECOND #0 -----------
  bool useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_First(!conserveBattery); // Time for extra TX before UI.
//  if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@0"); }
#endif


  // High-priority UI handing, every other/even second.
  // Show status if the user changed something significant.
  // Must take ~300ms or less so as not to run over into next half second if two TXs are done.
  bool recompute = false; // Set true an extra recompute of target temperature should be done.
#if !defined(TWO_S_TICK_RTC_SUPPORT)
  if(0 == (TIME_LSD & 1))
#endif
    {
    if(tickUI(TIME_LSD))
      {
      showStatus = true;
      recompute = true;
      }
    }
 
  
  if(recompute || recentUIControlUse())
    {
    computeTargetTemperature(); // Force recompute of temperature for (UI) responsiveness.
    }


#if defined(USE_MODULE_FHT8VSIMPLE)
  if(useExtraFHT8VTXSlots)
    {
    // Time for extra TX before other actions, but don't bother if minimising power in frost mode.
    // ---------- HALF SECOND #1 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(!conserveBattery); 
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@1"); }
    }
#endif


// While debugging switchover to phototransistor...
//  DEBUG_SERIAL_PRINT('L');
//  DEBUG_SERIAL_PRINT(readAmbientLight());
//  DEBUG_SERIAL_PRINTLN();


  // DO SCHEDULING

  // Once-per-minute tasks: all must take << 0.3s.
  // Run tasks spread throughout the minute to be as kind to batteries (etc) as possible.
  // Only when runAll is true run less-critical tasks that be skipped sometimes when particularly conserving energy.
  // TODO: coordinate temperature reading with time when radio and other heat-generating items are off for more accurate readings.
  // TODO: ensure only take ambient light reading at times when all LEDs are off.
  const bool runAll = (!conserveBattery) || minute0From4ForSensors;

  switch(TIME_LSD) // With TWO_S_TICK_RTC_SUPPORT only even seconds are available.
    {
    case 0:
      {
      // Tasks that must be run every minute.
      ++minuteCount;
      checkUserSchedule(); // Force to user's programmed settings, if any, at the correct time.
      // Ensure that the RTC has been persisted promptly when necessary.
      persistRTC();
      break;
      }

    // Churn/reseed PRNG(s) a little to improve unpredictability in use: should be lightweight.
    case 2: { if(runAll) { seedRNG8(minuteCount ^ cycleCountCPU() ^ (uint8_t)getBatterymV(), getSubCycleTime() ^ (uint8_t)getAmbientLight(), (uint8_t)getTemperatureC16()); } break; }
    // Force read of battery voltage; measure and recompute status less often when already thought to be low, eg when conserving.
    case 4: { if(runAll) { readBatterymV(); } break; }

#if defined(USE_MODULE_FHT8VSIMPLE) && defined(FHT8V_ALLOW_EXTRA_TXES)
    // Re-transmit slot for additional comms with boiler hub, eg when valve wide open, to help ensure reliable/fast call for heat.
    // This is entirely optional, and just improves odds of an urgent call for heat being acted on quickly,
    // so it can be cancelled with any reasonable excuse to save some energy and bandwidth.
    // This should always be safe, ie not done unless valve actually open, etc,
    // though it may cause the boiler to overrun a little into a closing valve under some circumstances.
    case 8:
      {
      if((!hubMode) && // Hub doesn't need to send extra TXes to itself!
         (!batteryLow) && // Don't send if battery is low.
         (!useExtraFHT8VTXSlots) && // Don't send if there's an immediately pending TX.
         inWarmModeDebounced() && // Only do extra TX if still in a warming mode, ie don't TX if mode just changed to FROST.
         (isControlledValveOpen()) && // Valve should be open already so we won't hurt the pump/boiler with call for heat.
         ((getTRVPercentOpen() >= 75) || inBakeModeDebounced())) // Valve fairly wide open, eg for BAKE or because boiler not hearing us reliably.
          {
#if 1 && defined(DEBUG)
          DEBUG_SERIAL_PRINTLN_FLASHSTRING("Extra FTH8V TX");
#endif
          pollIO(); // Deal with any pending I/O.
          sleepLowPowerLessThanMs(1 | (randRNG8() & 0x7f)); // Sleep randomly up to 127ms to spread transmissions and help avoid collisions.
          pollIO(); // Deal with any pending I/O.
          FHT8VDoSafeExtraTXToHub(); // Takes up to ~80ms.
          }
      break;
      }
#endif

    // Regular transmission of stats if NOT driving a local valve (else stats can be piggybacked onto that).
    case 10:
      {
//      if(hubMode) { break; } // Hub can't allow random switch to TX mode.
      if(!enableTrailingMinimalStatsPayload()) { break; } // Not allowed to send stuff like this.
#if defined(USE_MODULE_FHT8VSIMPLE)
      // Avoid transmit conflict with FS20; just drop the slot.
      // We should possibly choose between this and piggybacking stats to avoid busting duty-cycle rules.
      if(localFHT8VTRVEnabled() && useExtraFHT8VTXSlots) { break; }
#endif

      // Generally only attempt stats TX in the minute after all sensors should have been polled (so that readings are fresh).
      if(minute1From4AfterSensors ||
        (!batteryLow && (0 == (0x24 & randRNG8())))) // Occasional additional TX when not conserving power.
        {
        // Create (insecure) message.

#if 0 || !defined(HUMIDITY_SENSOR_SUPPORT) // Send hand-crafted binary message...
        uint8_t buf[RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES + FullStatsMessageCore_MAX_BYTES_ON_WIRE + 1];
          {
          uint8_t *bptr = buf;
          // Start with RFM23-friendly preamble which ends with with the aacccccc sync word.
          memset(bptr, RFM22_PREAMBLE_BYTE, RFM22_PREAMBLE_BYTES);
          bptr += RFM22_PREAMBLE_BYTES;
          memset(bptr, RFM22_SYNC_BYTE, RFM22_SYNC_MIN_BYTES);
          bptr += RFM22_SYNC_MIN_BYTES;
          // Gather core stats.
          FullStatsMessageCore_t content;
          populateCoreStats(&content);
          const uint8_t *msg1 = encodeFullStatsMessageCore(bptr, sizeof(buf) - (bptr-buf), getStatsTXLevel(), false, &content);
          if(NULL == msg1)
            {
DEBUG_SERIAL_PRINTLN_FLASHSTRING("Bin msg gen err!");
            break;
            }
          }
#else
        uint8_t buf[RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES + MSG_JSON_MAX_LENGTH + 1 + 1];
          {
          uint8_t *bptr = buf;
          // Start with RFM23-friendly preamble which ends with with the aacccccc sync word.
          memset(bptr, RFM22_PREAMBLE_BYTE, RFM22_PREAMBLE_BYTES);
          bptr += RFM22_PREAMBLE_BYTES;
          memset(bptr, RFM22_SYNC_BYTE, RFM22_SYNC_MIN_BYTES);
          bptr += RFM22_SYNC_MIN_BYTES;
          // Now append JSON text and closing 0xff...
          // Use letters that correspond to the values in ParsedRemoteStatsRecord and when displaying/parsing @ status records.
          const int8_t wrote = sprintfRawSimpleJSONMessage((char *)bptr,
            F("{\"@\":\"%0.2hhx%0.2hhx\",\"T|C16\":%d,\"H|%%\":%d,\"L\":%d,\"B|cV\":%d}"),
                (unsigned char)eeprom_read_byte(0 + (uint8_t *)EE_START_ID),
                (unsigned char)eeprom_read_byte(1 + (uint8_t *)EE_START_ID),
            (int)readTemperatureC16(),
            (int)Sensor_SHT21_readRHpc(),
            (int)(readAmbientLight()/4),
            (int)(readBatterymV()/10));
//          const int8_t wrote = sprintfRawSimpleJSONMessage((char *)bptr,
//            F("{\"id\":\"%0.2hhx%0.2hhx\",\"t|C16\":%d,\"RH|%%\":%hhd,\"l\":%d,\"b|cV\":%d}"),
//                (unsigned char)eeprom_read_byte(0 + (uint8_t *)EE_START_ID),
//                (unsigned char)eeprom_read_byte(1 + (uint8_t *)EE_START_ID),
//            (int)readTemperatureC16(),
//            (unsigned char)Sensor_SHT21_readRHpc(),
//            (int)(readAmbientLight()/4),
//            (int)(readBatterymV()/10));
          if(wrote < 0)
            {
DEBUG_SERIAL_PRINTLN_FLASHSTRING("JSON msg gen err!");
            break;
            }
          // Record stats as if local, and treat channel as secure.
          recordJSONStats(true, (const char *)bptr);
#if 0 && defined(DEBUG)
          DEBUG_SERIAL_PRINT_FLASHSTRING("JSON sent: ");
          DEBUG_SERIAL_PRINT((const char *)bptr);
          DEBUG_SERIAL_PRINTLN();
#endif
          bptr += wrote;
          // Should consider use of 0x5B CRC for body up to 7 chars, 0x48 (CRC-7/G704) beyond that.
          *bptr++ = 0; // FIXME: Add 7-bit CRC for on-the-wire check.
          *bptr = 0xff; // Terminate message.
#if 1 && defined(DEBUG)
          if(bptr - buf > 64)
            {
            DEBUG_SERIAL_PRINT_FLASHSTRING("Msg too long for RFM2x: ");
            DEBUG_SERIAL_PRINT((int)(bptr - buf));
            DEBUG_SERIAL_PRINTLN();
            }
#endif
          }
#endif

        pollIO(); // Deal with any pending I/O.
        sleepLowPowerLessThanMs(1 + (randRNG8() & 0x7f)); // Sleep randomly up to 128ms to spread transmissions and help avoid collisions.
        pollIO(); // Deal with any pending I/O.

        // TODO: put in listen before TX to reduce collisions (CSMA)
        // Send message!
        // Assume RFM22/23 support for now.
        RFM22QueueCmdToFF(buf);
        RFM22TXFIFO(); // Send it!  Approx 1.6ms/byte.     
        
#if defined(ENABLE_BOILER_HUB)
      if(hubMode)
        { SetupToEavesdropOnFHT8V(); } // Revert to hub listening...
      else
#endif
      { RFM22ModeStandbyAndClearState(); } // Go to standby to conserve energy.

DEBUG_SERIAL_PRINTLN_FLASHSTRING("Bare stats TX");
        }
      break;
      }

#ifdef TEMP_POT_AVAILABLE
    // Sample the user-selected WARM temperature target.
    case 48: { if(runAll) { readTempPot(); } break; }
#endif

    // Read all environmental inputs, late in the cycle.
#ifdef HUMIDITY_SENSOR_SUPPORT
    // Sample humidity.
    case 50: { if(runAll) { readRHpc(); } break; }
#endif
    // Sample ambient light levels.
    case 52: { if(runAll) { readAmbientLight(); } break; }
    // At a hub, sample temperature as late as possible in (and only in the 'quiet') minute, to reduce valve hunting from self-heating.
    case 54: { if(hubMode ? minute0From4ForSensors : runAll) { readTemperatureC16(); } break; }

    // Compute targets and heat demand based on environmental inputs.
    // Note: ensure that valve-shut message is always conveyed quickly to valve even in slow/'conserve' mode.
    // Also drives OUT_HEATCALL to control local boiler if in central hub mode.
    case 56:
      {
      static bool boilerOn; // Internal record of current boiler-out state.

      // Recompute target, valve position and call for heat, etc.
      // Should be called once per minute to work correctly.
      if(computeCallForHeat() ||
         (minute1From4AfterSensors && enableTrailingMinimalStatsPayload()))
        {
        // If there was a change in target valve position,
        // or periodically in a minute after all sensors should have been read,
        // precompute some or all of any outgoing frames/stats/etc ready for transmission.
#if defined(USE_MODULE_FHT8VSIMPLE)
        // Recompute FHT8V command to send if target valve setting has changed...
        if(localFHT8VTRVEnabled()) { FHT8VCreateValveSetCmdFrame(); }
#endif
        }

#if defined(ENABLE_BOILER_HUB)
      // Track how long since remote call for heat last heard.
      if(hubMode)
        {
        if(boilerCountdownTicks != 0)
          {
#if 1 && defined(DEBUG)
          DEBUG_SERIAL_PRINT_FLASHSTRING("Boiler on, s left: ");
          DEBUG_SERIAL_PRINT(boilerCountdownTicks * MAIN_TICK_S);
          DEBUG_SERIAL_PRINTLN();
#endif
          }
        }
#endif

      // Show current status if appropriate.
      if(runAll) { showStatus = true; }
      break;
      }

    // Stats samples; should never be missed.
    case 58:
      {
      // Take full stats sample as near the end of the hour as reasonably possible (without danger of overrun),
      // and with other optional non-full samples evenly spaced throughout the hour (if not low on battery).
      if(minute0From4ForSensors) // Hope to take lowest-noise samples on the special minute out of each 4.
        {
        const uint_least8_t mm = getMinutesLT();
        switch(mm)
          {
          case 16: case 17: case 18: case 19:
          case 36: case 37: case 38: case 39:
            { if(!batteryLow) { sampleStats(false); } break; } // Skip sub-samples if short of juice.
          case 56: case 57: case 58: case 59:
            { sampleStats(true); break; } // Always take the full sample at end of hour.
          }
        }
      break;
      }
    }

#if defined(USE_MODULE_FHT8VSIMPLE) && defined(TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // Time for extra TX before other actions, but don't bother if minimising power in frost mode.
    // ---------- HALF SECOND #2 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(!conserveBattery); 
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@2"); }
    }
#endif

  // Generate periodic status reports.
  if(showStatus) { serialStatusReport(); }

#if defined(USE_MODULE_FHT8VSIMPLE) && defined(TWO_S_TICK_RTC_SUPPORT)
  if(useExtraFHT8VTXSlots)
    {
    // ---------- HALF SECOND #3 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8VPollSyncAndTX_Next(!conserveBattery); 
//    if(useExtraFHT8VTXSlots) { DEBUG_SERIAL_PRINTLN_FLASHSTRING("ES@3"); }
    }
#endif

  // Command-Line Interface (CLI) polling.
  // If a reasonable chunk of the minor cycle remains after all other work is done
  // AND the CLI is / should be active OR a status line has just been output
  // then poll/prompt the user for input
  // using a timeout which should safely avoid missing the next basic tick
  // and which should also allow some energy-saving sleep.
#if 1 && defined(SUPPORT_CLI)
  const bool humanCLIUse = isCLIActive(); // Keeping CLI active for human interaction rather than for automated interaction.
  if(showStatus || humanCLIUse)
    {
    const uint8_t sct = getSubCycleTime();
    const uint8_t listenTime = max(GSCT_MAX/16, CLI_POLL_MIN_SCT);
    if(sct < (GSCT_MAX - 2*listenTime))
      // Don't listen beyond the last 16th of the cycle,
      // or a minimal time if only prodding for interaction with automated front-end,
      // as listening for UART RX uses lots of power.
      { pollCLI(humanCLIUse ? (GSCT_MAX-listenTime) : (sct+CLI_POLL_MIN_SCT)); }
    }
#endif



#if 0 && defined(DEBUG)
  const int tDone = getSubCycleTime();
  if(tDone > 1) // Ignore for trivial 1-click time.
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("done in "); // Indicates what fraction of available loop time was used / 256.
    DEBUG_SERIAL_PRINT(tDone);
    DEBUG_SERIAL_PRINT_FLASHSTRING(" @ ");
    DEBUG_SERIAL_TIMESTAMP();
    DEBUG_SERIAL_PRINTLN();
    }
#endif

  // Detect and handle (actual or near) overrun, if it happens, though it should not.
  if(TIME_LSD != getSecondsLT())
    {
    // Increment the overrun counter (stored inverted, so 0xff initialised => 0 overruns).
    const uint8_t orc = 1 + ~eeprom_read_byte((uint8_t *)EE_START_OVERRUN_COUNTER);
    eeprom_smart_update_byte((uint8_t *)EE_START_OVERRUN_COUNTER, ~orc);
#if 1 && defined(DEBUG)
    DEBUG_SERIAL_PRINT_FLASHSTRING("!ERROR: loop overrun ");
    DEBUG_SERIAL_PRINT(orc);
    DEBUG_SERIAL_PRINTLN();
#endif
#if defined(USE_MODULE_FHT8VSIMPLE)
    FHT8VSyncAndTXReset(); // Assume that sync with valve may have been lost, so re-sync.
#endif
    TIME_LSD = getSecondsLT(); // Prepare to sleep until start of next full minor cycle.
    }
#if 0 && defined(DEBUG) // Expect to pick up near overrun at start of next loop.
  else if(getSubCycleTime() >= nearOverrunThreshold)
    {
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("?O"); // Near overrun.  Note 2ms/char to send...
    }
#endif
  }


#if !defined(ALT_MAIN_LOOP)
// Interrupt service routine for I/O port transition changes.
ISR(PCINT1_vect)
  {
  }
#endif
