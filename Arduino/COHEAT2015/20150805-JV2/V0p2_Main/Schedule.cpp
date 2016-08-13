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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2015
*/

/*
 Schedule support for TRV.
 */

#include <avr/eeprom.h>
#include <util/atomic.h>

#include "Schedule.h"

#include "Control.h"
#include "RTC_Support.h"
#include "EEPROM_Utils.h"



#if defined(UNIT_TESTS)
// Support for unit tests to force particular apparent schedule state.
// Current override state; 0 (default) means no override.
static _TEST_schedule_override _soUT_override;
// Set the override value (or remove the override).
void _TEST_set_schedule_override(const _TEST_schedule_override override)
  { _soUT_override = override; }
#endif






//#define EE_START_SIMPLE_SCHEDULE_ON 16 // 2-byte 'minutes after midnight' on time, if any.
//#define EE_START_SIMPLE_SCHEDULE_OFF 18 // 2-byte 'minutes after midnight' off time, if any.

// All EEPROM activity is made atomic by locking out interrupts where necessary.

// Maximum mins-after-midnight compacted value in one byte.
#define MAX_COMPRESSED_MINS_AFTER_MIDNIGHT ((MINS_PER_DAY / SIMPLE_SCHEDULE_GRANULARITY_MINS) - 1)


// If LEARN_BUTTON_AVAILABLE then what is the schedule on time?
#ifdef LEARN_BUTTON_AVAILABLE
// Number of minutes of schedule on time to use.
// Will depend on eco bias.
// TODO: make gradual.
static uint8_t onTime()
  {
//  // Simple and fast binary choice.
//  return(hasEcoBias() ? LEARNED_ON_PERIOD_M : LEARNED_ON_PERIOD_COMFORT_M);

  // Three-way split based on current WARM target temperature.
  const uint8_t wt = getWARMTargetC();
  if(isEcoTemperature(wt)) { return(LEARNED_ON_PERIOD_M); }
  else if(isComfortTemperature(wt)) { return(LEARNED_ON_PERIOD_COMFORT_M); }
  else { return((LEARNED_ON_PERIOD_M + LEARNED_ON_PERIOD_COMFORT_M) / 2); }
  }
#endif

// Pre-warm time before learned/scheduled WARM period.
#define PREWARM_MINS ((SIMPLE_SCHEDULE_GRANULARITY_MINS/2) + (LEARNED_ON_PERIOD_M>>2))
// Setback period before WARM period to help ensure that the WARM target can be reached on time.
// Important for slow-to-heat rooms that have become very cold.
// Similar to PREWARM_MINS so that we can safely use this without causing distress, eg waking people up.
#define PREPREWARM_MINS (PREWARM_MINS)

// Get the simple/primary schedule on time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
// Will usually include a pre-warm time before the actual time set.
// Note that unprogrammed EEPROM value will result in invalid time, ie schedule not set.
//   * which  schedule number, counting from 0
uint_least16_t getSimpleScheduleOn(const uint8_t which)
  {
  if(which >= MAX_SIMPLE_SCHEDULES) { return(~0); } // Invalid schedule number.
  uint8_t startMM;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    { startMM = eeprom_read_byte((uint8_t*)(EE_START_SIMPLE_SCHEDULE0_ON + which)); }
  if(startMM > MAX_COMPRESSED_MINS_AFTER_MIDNIGHT) { return(~0); } // No schedule set.
  // Compute start time from stored shedule value.
  uint_least16_t startTime = SIMPLE_SCHEDULE_GRANULARITY_MINS * startMM;
// If LEARN_BUTTON_AVAILABLE then in the absence of anything better SUPPORT_SINGLETON_SCHEDULE should be supported.
#ifdef LEARN_BUTTON_AVAILABLE
  const uint8_t windBackM = PREWARM_MINS; // Wind back start time by about 25% of full interval.
  if(windBackM > startTime) { startTime += MINS_PER_DAY; } // Allow for wrap-around at midnight.
  startTime -= windBackM;
#endif
  return(startTime);
  }

// Get the simple/primary schedule off time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
// This is based on specifed start time and some element of the current eco/comfort bias.
//   * which  schedule number, counting from 0
uint_least16_t getSimpleScheduleOff(const uint8_t which)
  {
  const uint_least16_t startMins = getSimpleScheduleOn(which);
  if(startMins == (uint_least16_t)~0) { return(~0); }
  // Compute end from start, allowing for wrap-around at midnight.
  uint_least16_t endTime = startMins + PREWARM_MINS + onTime();
  if(endTime >= MINS_PER_DAY) { endTime -= MINS_PER_DAY; } // Allow for wrap-around at midnight.
  return(endTime);
  }

// Set the simple/primary simple on time.
//   * startMinutesSinceMidnightLT  is start/on time in minutes after midnight [0,1439]
//   * which  schedule number, counting from 0
// Invalid parameters will be ignored and false returned,
// else this will return true and isSimpleScheduleSet() will return true after this.
// NOTE: over-use of this routine can prematurely wear out the EEPROM.
bool setSimpleSchedule(const uint_least16_t startMinutesSinceMidnightLT, const uint8_t which)
  {
  if(which >= MAX_SIMPLE_SCHEDULES) { return(false); } // Invalid schedule number.
  if(startMinutesSinceMidnightLT >= MINS_PER_DAY) { return(false); } // Invalid time.

  // Set the schedule, minimising wear.
  const uint8_t startMM = startMinutesSinceMidnightLT / SIMPLE_SCHEDULE_GRANULARITY_MINS; // Round down...
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    { eeprom_smart_update_byte((uint8_t*)(EE_START_SIMPLE_SCHEDULE0_ON + which), startMM); }
  return(true); // Assume EEPROM programmed OK...
  }

// Clear a simple schedule.
// There will be neither on nor off events from the selected simple schedule once this is called.
//   * which  schedule number, counting from 0
void clearSimpleSchedule(const uint8_t which)
  {
  if(which >= MAX_SIMPLE_SCHEDULES) { return; } // Invalid schedule number.
  // Clear the schedule back to 'unprogrammed' values, minimising wear.
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    { eeprom_smart_erase_byte((uint8_t*)(EE_START_SIMPLE_SCHEDULE0_ON + which)); }
  }

// Returns true if any simple schedule is set, false otherwise.
// This implementation just checks for any valid schedule 'on' time.
// In unit-test override mode is true for soon/now, false for off.
bool isAnySimpleScheduleSet()
  {
#if defined(UNIT_TESTS)
  // Special behaviour for unit tests.
  switch(_soUT_override)
    {
    case _soUT_off: return(false);
    case _soUT_soon: return(true);
    case _soUT_now: return(true);
    }
#endif

  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    for(uint8_t which = 0; which < MAX_SIMPLE_SCHEDULES; ++which)
      {
      if(eeprom_read_byte((uint8_t*)(EE_START_SIMPLE_SCHEDULE0_ON + which)) <= MAX_COMPRESSED_MINS_AFTER_MIDNIGHT)
        { return(true); }
      }
    }
  return(false);
  }


// True iff any schedule is currently 'on'/'WARM' even when schedules overlap.
// May be relatively slow/expensive.
// Can be used to suppress all 'off' activity except for the final one.
// Can be used to suppress set-backs during on times.
// In unit-test override mode is true for now, false for soon/off.
bool isAnyScheduleOnWARMNow()
  {
#if defined(UNIT_TESTS)
  // Special behaviour for unit tests.
  switch(_soUT_override)
    {
    case _soUT_off: return(false);
    case _soUT_soon: return(false);
    case _soUT_now: return(true);
    }
#endif

  const uint_least16_t mm = getMinutesSinceMidnightLT();

  for(uint8_t which = 0; which < MAX_SIMPLE_SCHEDULES; ++which)
    {
    const uint_least16_t s = getSimpleScheduleOn(which);
    if(mm < s) { continue; } // Also deals with case where this schedule is not set at all (s == ~0);
    uint_least16_t e = getSimpleScheduleOff(which);
    if(e < s) { e += MINS_PER_DAY; } // Cope with schedule wrap around midnight.
    if(mm < e) { return(true); }
    }

  return(false);
  }


// True iff any schedule is due 'on'/'WARM' soon even when schedules overlap.
// May be relatively slow/expensive.
// Can be used to allow room to be brought up to at least a set-back temperature
// if very cold when a WARM period is due soon (to help ensure that WARM target is met on time).
// In unit-test override mode is true for soon, false for now/off.
bool isAnyScheduleOnWARMSoon()
  {
#if defined(UNIT_TESTS)
  // Special behaviour for unit tests.
  switch(_soUT_override)
    {
    case _soUT_off: return(false);
    case _soUT_soon: return(true);
    case _soUT_now: return(false);
    }
#endif

  const uint_least16_t mm0 = getMinutesSinceMidnightLT() + PREPREWARM_MINS; // Look forward...
  const uint_least16_t mm = (mm0 >= MINS_PER_DAY) ? (mm0 - MINS_PER_DAY) : mm0;

  for(uint8_t which = 0; which < MAX_SIMPLE_SCHEDULES; ++which)
    {
    const uint_least16_t s = getSimpleScheduleOn(which);
    if(mm < s) { continue; } // Also deals with case where this schedule is not set at all (s == ~0);
    uint_least16_t e = getSimpleScheduleOff(which);
    if(e < s) { e += MINS_PER_DAY; } // Cope with schedule wrap around midnight.
    if(mm < e) { return(true); }
    }

  return(false);
  }

