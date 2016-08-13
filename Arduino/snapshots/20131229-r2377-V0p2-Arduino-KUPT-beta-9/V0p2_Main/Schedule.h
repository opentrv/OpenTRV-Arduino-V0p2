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

#ifndef SCHEDULE_H
#define SCHEDULE_H

#include "V0p2_Main.h"


// Granularity of simple schedule in minutes (values may be rounded/truncated to nearest); strictly positive.
#define SIMPLE_SCHEDULE_GRANULARITY_MINS 6

// Get the simple schedule on time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
// Note that unprogrammed EEPROM value will result in invalid time, ie not set.
uint_least16_t getSimpleScheduleOn();

// Get the simple schedule off time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
// Note that unprogrammed EEPROM value will result in invalid time, ie not set.
uint_least16_t getSimpleScheduleOff();

// Set a simple on/off schedule.
//   * startMinutesSinceMidnightLT  is start/on time in minutes after midnight [0,1439]
//   * durationMinutes  is duration in minutes in range [1,1439]
// Invalid parameters will be ignored and false returned,
// else this will return true and isSimpleScheduleSet() will return true after this.
// NOTE: over-use of this routine can prematurely wear our the EEPROM.
bool setSimpleSchedule(uint_least16_t startMinutesSinceMidnightLT, uint_least16_t durationMinutes);

// Clear simple schedule.
// There will be no on nor off events from the simple schedule once this is called,
// and isSimpleScheduleSet() will return false.
void clearSimpleSchedule();

// Returns true if a simple schedule is set, false otherwise.
bool isSimpleScheduleSet();


#endif


