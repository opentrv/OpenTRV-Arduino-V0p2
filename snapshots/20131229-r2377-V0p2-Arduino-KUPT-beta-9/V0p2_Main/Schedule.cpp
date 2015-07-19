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
 Schedule support for TRV.
 */

#include <avr/eeprom.h>
#include <util/atomic.h>

#include "Schedule.h"

#include "EEPROM_Utils.h"


//#define EE_START_SIMPLE_SCHEDULE_ON 16 // 2-byte 'minutes after midnight' on time, if any.
//#define EE_START_SIMPLE_SCHEDULE_OFF 18 // 2-byte 'minutes after midnight' off time, if any.

// All EEPROM activity is made atomic by locking out interrupts where necessary.

// Maximum mins-after-midnight compacted value in one byte.
#define MAX_COMPRESSED_MINS_AFTER_MIDNIGHT ((1440 / SIMPLE_SCHEDULE_GRANULARITY_MINS) - 1)

// Get the simple schedule on time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
// Note that unprogrammed EEPROM value will result in invalid time, ie not set.
uint_least16_t getSimpleScheduleOn()
  {
  uint8_t startMM;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    { startMM = eeprom_read_byte((uint8_t*)EE_START_SIMPLE_SCHEDULE_ON); }
  return(SIMPLE_SCHEDULE_GRANULARITY_MINS * startMM);
  }

// Get the simple schedule off time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
uint_least16_t getSimpleScheduleOff()
  {
  uint8_t endMM;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    { endMM = eeprom_read_byte((uint8_t*)EE_START_SIMPLE_SCHEDULE_OFF); }
  return(SIMPLE_SCHEDULE_GRANULARITY_MINS * endMM);
  }

// Set a simple on/off schedule.
//   * startMinutesSinceMidnightLT  is start/on time in minutes after midnight [0,1439]
//   * durationMinutes  is duration in minutes in range [1,1439]
// Invalid parameters will be ignored and false returned,
// else this will return true and isSimpleScheduleSet() will return true after this.
// NOTE: over-use of this routine can prematurely wear our the EEPROM.
bool setSimpleSchedule(const uint_least16_t startMinutesSinceMidnightLT, const uint_least16_t durationMinutes)
  {
  if((startMinutesSinceMidnightLT >= 1440) || (durationMinutes == 0) || (durationMinutes >= 1439)) { return(false); }
  
  const uint8_t startMM = startMinutesSinceMidnightLT / SIMPLE_SCHEDULE_GRANULARITY_MINS; // Round down...
  const uint16_t em = startMM + (uint16_t) max((uint8_t)(durationMinutes/SIMPLE_SCHEDULE_GRANULARITY_MINS), (uint8_t)1);
  const uint8_t endMM = (em <= MAX_COMPRESSED_MINS_AFTER_MIDNIGHT) ? em : (em - (MAX_COMPRESSED_MINS_AFTER_MIDNIGHT + 1));

  // Set the schedule.
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    eeprom_smart_update_byte((uint8_t*)EE_START_SIMPLE_SCHEDULE_ON, startMM);
    eeprom_smart_update_byte((uint8_t*)EE_START_SIMPLE_SCHEDULE_OFF, endMM);
    }
  return(true); // Assume EEPROM programmed OK...
  }

// Clear simple schedule.
// There will be no on nor off events from the simple schedule once this is called,
// and isSimpleScheduleSet() will return false.
void clearSimpleSchedule()
  {
  // Clear the schedule back to 'unprogrammed' values.
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    eeprom_smart_erase_byte((uint8_t*)EE_START_SIMPLE_SCHEDULE_ON);
    eeprom_smart_erase_byte((uint8_t*)EE_START_SIMPLE_SCHEDULE_OFF);
    }
  }

// Returns true if a simple schedule is set, false otherwise.
// This implementation just checks for a valid 'on' time.
bool isSimpleScheduleSet()
  {
  uint8_t startMM;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    { startMM = eeprom_read_byte((uint8_t*)EE_START_SIMPLE_SCHEDULE_ON); }
  return(startMM <= MAX_COMPRESSED_MINS_AFTER_MIDNIGHT);
  }



