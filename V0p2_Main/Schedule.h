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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2016
*/

/*
 Simple schedule support for TRV.
 */

#ifndef SCHEDULE_H
#define SCHEDULE_H

#include "V0p2_Main.h"

// Granularity of simple schedule in minutes (values may be rounded/truncated to nearest); strictly positive.
static const uint8_t SIMPLE_SCHEDULE_GRANULARITY_MINS = 6;

// Expose number of supported schedules.
// Can be more than the number of buttons, but later schedules will be CLI-only.
static const uint8_t MAX_SIMPLE_SCHEDULES = V0P2BASE_EE_START_MAX_SIMPLE_SCHEDULES;

// Get the simple schedule on time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
// Will usually include a pre-warm time before the actual time set.
// Note that unprogrammed EEPROM value will result in invalid time, ie schedule not set.
//   * which  schedule number, counting from 0
uint_least16_t getSimpleScheduleOn(uint8_t which);

// Get the simple schedule off time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
// This is based on specified start time and some element of the current eco/comfort bias.
//   * which  schedule number, counting from 0
uint_least16_t getSimpleScheduleOff(uint8_t which);

// Set the simple simple on time.
//   * startMinutesSinceMidnightLT  is start/on time in minutes after midnight [0,1439]
//   * which  schedule number, counting from 0
// Invalid parameters will be ignored and false returned,
// else this will return true and isSimpleScheduleSet() will return true after this.
// NOTE: over-use of this routine can prematurely wear out the EEPROM.
bool setSimpleSchedule(uint_least16_t startMinutesSinceMidnightLT, uint8_t which);

// Clear a simple schedule.
// There will be neither on nor off events from the selected simple schedule once this is called.
//   * which  schedule number, counting from 0
void clearSimpleSchedule(uint8_t which);

// True iff any schedule is 'on'/'WARN' even when schedules overlap.
// Can be used to suppress all 'off' activity except for the final one.
// Can be used to suppress set-backs during on times.
bool isAnyScheduleOnWARMNow();

// True iff any schedule is due 'on'/'WARM' soon even when schedules overlap.
// May be relatively slow/expensive.
// Can be used to allow room to be brought up to at least a set-back temperature
// if very cold when a WARM period is due soon (to help ensure that WARM target is met on time).
bool isAnyScheduleOnWARMSoon();

// True iff any schedule is currently 'on'/'WARM' even when schedules overlap.
// May be relatively slow/expensive.
// Can be used to suppress all 'off' activity except for the final one.
// Can be used to suppress set-backs during on times.
bool isAnySimpleScheduleSet();


#if defined(UNIT_TESTS)
// Support for unit tests to force particular apparent schedule state (without EEPROM writes).
enum _TEST_schedule_override
  {
    _soUT_normal = 0, // No override
    _soUT_off, // All schedules off.
    _soUT_soon, // Schedule due on WARM soon.
    _soUT_now, // Schedule active now.
  };
#define _TEST_schedule_override_MAX _soUT_now // Max legit _TEST_schedule_override value.
// Set the override value (or remove the override).
void _TEST_set_schedule_override(_TEST_schedule_override override);
#endif


#endif


