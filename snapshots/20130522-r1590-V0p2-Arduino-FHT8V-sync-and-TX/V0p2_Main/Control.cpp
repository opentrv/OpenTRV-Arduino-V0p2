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
  if((stored < MIN_TARGET_C) || (stored > MAX_TARGET_C)) { return(max(WARM, getFROSTTargetC())); }
  // Return valid persisted value (or frost value if set and higher).
  return(max(stored, getFROSTTargetC()));
  }
#else
#define getWARMTargetC() (WARM) // Fixed value.
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


#ifndef TRV_MIN_SLEW_PC
#define TRV_MIN_SLEW_PC 7 // Minimum slew/error distance in central range; should be larger than smallest temperature-sensor-driven step (6) to be effective; [1,100].
// Note: keeping TRV_MIN_SLEW_PC sufficiently high largely avoids spurious hunting back and forth from single-ulp noise.
#endif
#ifndef TRV_MAX_SLEW_PC_PER_MIN
#ifndef TRV_SLEW_GLACIAL
#define TRV_MAX_SLEW_PC_PER_MIN 10 // Maximum slew rate, eg to fully open from off when well under target; [1,100].
#else
#define TRV_MAX_SLEW_PC_PER_MIN = 1 // Minimal slew rate to keep flow rates as low as possible.
#endif
// Note: keeping TRV_MAX_SLEW_PC_PER_MIN small reduces noise and overshoot and surges of water (eg when charged by the m^3).
// Very low values of TRV_MAX_SLEW_PC_PER_MIN may help avoid drawing excess water eg from district heating schemes.
#endif


// Compute target temperature.
static void computeTargetTemperature()
  {
  if(!inWarmMode()) // In FROST mode.
    { targetTempC = getFROSTTargetC(); } // No setbacks apply in FROST mode.

#ifdef SUPPORT_BAKE
  else if(inBakeMode()) // If in BAKE mode then use elevated target.
    {
    // dec bakeCountdownM // Moved management of counter to UI code.
    targetTempC = min(getWARMTargetC() + BAKE_UPLIFT, MAX_TARGET_C); // No setbacks apply in BAKE mode.
    }
#endif

  else // In 'WARM' mode with possible setback.
    {
#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION
    // Set back target temperature a little if room is too dark for activity.
    if(!isRoomLit())
      { targetTempC = max(getWARMTargetC() - SETBACK, MIN_TARGET_C); } // Target must never be below low enough for real frost hazard.
    else
      { targetTempC = getWARMTargetC(); } // Room light enough for normal activity so use WARM target directly.
#else
    targetTempC = getWARMTargetC(); // No LDR, no setback.
#endif
    }
  }

// Set heat demand with some hysteresis and a hint of proportional control.
// Always be willing to turn off quickly, but on slowly (AKA "slow start" algorithm),
// and try to eliminate unnecessary 'hunting' which makes noise and uses actuator energy.
static bool computeRequiredTRVPercentOpen()
  {
  bool changed = false;

  const int currentTempC16 = getTemperatureC16();
  const int currentTempC = currentTempC16 >> 4;

  if(currentTempC < targetTempC) // (Well) under temp target: open valve.
    {
    // Limit value open slew to help minimise overshoot and actuator noise.
    // This should also reduce nugatory setting changes when occupancy (etc) is fluctuating.
    // Thus it may take several minutes to turn the radiator fully on,
    // though probably opening the first 30% will allow near-maximum heat output in practice.
    if(TRVPercentOpen != 100)
      {
      const uint8_t tmp = TRVPercentOpen + TRV_MAX_SLEW_PC_PER_MIN;
      if(tmp < 100) { TRVPercentOpen = tmp; } else { TRVPercentOpen = 100; } // Capped at 100%.        
      changed = true; // TRV setting has been changed.
      }
    }
  else if(currentTempC > targetTempC) // (Well) over temp target: close valve.
    {
#if defined(SUPPORT_BAKE)
    cancelBake(); // Ensure bake mode cancelled immediately if over target (eg when target is BAKE).
#endif
    if(TRVPercentOpen != 0)
      {
      TRVPercentOpen = 0; // Always force to off immediately when requested.  (Eagerly stop heating to conserve.)
      changed = true; // TRV setting has been changed.
      }
    }
  else // Close to temp target: set valve partly open to try to tightly regulate.
    {
    // Use currentTempC16 lsbits to set valve percentage for proportional feedback
    // to provide more efficient and quieter TRV drive and probably more stable room temperature.
    uint8_t tmp = currentTempC16 & 0xf; // Only interested in lsbits.
    tmp = 16 - tmp; // Now in range 1 (at warmest end of 'correct' temperature) to 16 (coolest).
    const uint8_t targetPO = tmp * 6; // Now in range 6 to 96, eg valve nearly shut just below top of 'correct' temperature window.
    // Reduce spurious valve/boiler adjustment by avoiding movement at all unless current error is significant.
    if(targetPO < TRVPercentOpen) // Currently open more than required.
      {
      const uint8_t slew = TRVPercentOpen - targetPO;
      if(slew >= TRV_MIN_SLEW_PC)
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
      if(slew >= TRV_MIN_SLEW_PC)
        {
        if(slew > TRV_MAX_SLEW_PC_PER_MIN)
            { TRVPercentOpen += TRV_MAX_SLEW_PC_PER_MIN; } // Cap slew rate.
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
  computeTargetTemperature();
  return(computeRequiredTRVPercentOpen());
  }

#if 0 // PICAXE V0.09/V0.1 code.
symbol TRV_MIN_SLEW_PC = 10 ; Minimum slew/error distance; should be larger than smallest temperature-sensor-driven step to be effective.
symbol TRV_MAX_SLEW_PC_PER_MIN = 20 ; Maximum slew rate determining minutes to fully on from off; should usually be larger than TRV_MIN_SLEW_PC.
computeTargetAndDemand:
    ; Compute target.
    if isWarmMode = 0 then ; In frost mode.
        targetTempC = FROST ; No setbacks apply.
#ifdef SUPPORT_BAKE
    else if bakeCountdownM != 0 then ; If (still) in bake mode then use high target.
        dec bakeCountdownM
        targetTempC = WARM + BAKE_UPLIFT ; No setbacks apply.
#endif
    else ; In 'warm' mode with possible setback.
#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION
        ; Set back target temperature a little if room is too dark for activity.
        if isRoomLit = 0 then
            targetTempC = WARM - SETBACK min FROST ; Target must never be below FROST.
        else
            targetTempC = WARM
        endif
#else
        targetTempC = WARM ; No LDR, no setback.
#endif
    endif

    ; Set heat demand with some hysteresis and a hint of proportional control.
    ; Always be willing to turn off quickly, but on slowly (AKA "slow start" algorithm),
    ; and try to eliminate unnecessary 'hunting' which makes noise and uses actuator energy.
    ; If tempB2 is non-zero then a change has been made to TRVPercentOpen.
    tempB2 = 0
    if currentTempC < targetTempC then
        ; Limit value open slew to help minimise overshoot and actuator noise.
        ; This should also reduce nugatory setting changes when occupancy (etc) is fluctuating.
        ; Thus it may take several minutes to turn the radiator fully on,
        ; though probably opening the first 30% will allow near-maximum heat output in practice.
        if TRVPercentOpen != 100 then
            TRVPercentOpen = TRVPercentOpen + TRV_MAX_SLEW_PC_PER_MIN max 100 ; Evaluated strictly left-to-right, so slews at max rate and caps at 100.
            tempB2 = 1 ; TRV setting has been changed.
        endif
    else if currentTempC > targetTempC then
#ifdef SUPPORT_BAKE
        bakeCountdownM = 0 ; Ensure bake mode cancelled immediately if over target (eg when target is BAKE).
#endif
        if TRVPercentOpen != 0 then
            TRVPercentOpen = 0 ; Always force to off immediately when requested.  (Eagerly stop heating to conserve.)
            tempB2 = 1 ; TRV setting has been changed.
        endif
    else
        ; Use currentTempC16 lsbits to set valve percentage for proportional feedback
        ; to provide more efficient and quieter TRV drive and probably more stable room temperature.
        tempB0 = currentTempC16 & $f ; Only interested in lsbits.
        tempB0 = 16 - tempB0 ; Now in range 1 (at warmest end of 'correct' temperature) to 16 (coolest).
        tempB0 = tempB0 * 6 ; Now in range 6 to 96, eg valve nearly shut just below top of 'correct' temperature window.
        ; Reduce spurious valve/boiler adjustment by avoiding movement at all unless current error is significant.
        if tempB0 < TRVPercentOpen then
            tempB1 = TRVPercentOpen - tempB0
            if tempB1 >= TRV_MIN_SLEW_PC then
                if tempB1 > TRV_MAX_SLEW_PC_PER_MIN then
                    TRVPercentOpen = TRVPercentOpen - TRV_MAX_SLEW_PC_PER_MIN ; Cap slew rate.
                else
                    TRVPercentOpen = tempB0
                endif
                tempB2 = 1 ; TRV setting has been changed.
            endif
        else if tempB0 > TRVPercentOpen then
            tempB1 = tempB0 - TRVPercentOpen
            if tempB1 >= TRV_MIN_SLEW_PC then
                if tempB1 > TRV_MAX_SLEW_PC_PER_MIN then
                    TRVPercentOpen = TRVPercentOpen + TRV_MAX_SLEW_PC_PER_MIN ; Cap slew rate.
                else
                    TRVPercentOpen = tempB0
                endif
                tempB2 = 1 ; TRV setting has been changed.
            endif
        endif
    endif

    ; Recompute anything necessary to support TRV activity.
#ifdef USE_MODULE_FHT8VSIMPLE_TX
    ; TODO: force update if command buffer is empty (leading $ff).
    if tempB2 != 0 then
        bptr = FHT8VTXCommandArea
        gosub FHT8VCreateValveSetCmdFrame
        slowOpDone = 1
    endif
#endif

#endif

