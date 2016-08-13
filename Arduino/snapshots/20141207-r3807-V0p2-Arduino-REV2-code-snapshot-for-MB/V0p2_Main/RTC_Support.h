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

#ifndef RTC_SUPPORT_H
#define RTC_SUPPORT_H

#include <stdint.h>

#include "V0p2_Main.h"

// Number of minutes per day.
#define MINS_PER_DAY 1440

// Seconds for local time (and assumed UTC) in range [0,59].
// Volatile to allow for async update.
// Maintained locally or shadowed from external RTC.
// Read and write accesses assumed effectively atomic.
// NOT FOR DIRECT ACCESS OUTSIDE RTC ROUTINES.
extern volatile uint_fast8_t _secondsLT;

// Minutes since midnight for local time in range [0,1439].
// Must be accessed with interrupts disabled and as if volatile.
// Maintained locally or shadowed from external RTC.
// NOT FOR DIRECT ACCESS OUTSIDE RTC ROUTINES.
extern volatile uint_least16_t _minutesSinceMidnightLT;

// Whole days since the start of 2000/01/01 (ie the midnight between 1999 and 2000), local time.
// Must be accessed with interrupts disabled and as if volatile.
// This will roll in about 2179.
// NOT FOR DIRECT ACCESS OUTSIDE RTC ROUTINES.
extern volatile uint_least16_t _daysSince1999LT;




// Persist software RTC information to non-volatile (EEPROM) store.
// This does not attempt to store full precision of time down to seconds,
// but enough to help avoid the clock slipping too much during (say) a battery change.
// There is no point calling this more than (say) once per minute,
// though it will simply return relatively quickly from redundant calls.
// The RTC data is stored so as not to wear out AVR EEPROM for at least several years.
void persistRTC();

// Restore software RTC information from non-volatile (EEPROM) store, if possible.
// Returns true if the persisted data seemed valid and was restored, in full or part.
bool restoreRTC();


// Get local time seconds from RTC [0,59].
// Is as fast as reasonably practical.
// Thread-safe and ISR-safe: returns a consistent atomic snapshot.
#define getSecondsLT() (_secondsLT) // Assumed atomic.

// Get local time minutes from RTC [0,59].
// Thread-safe and ISR-safe.
uint_least8_t getMinutesLT();

// Get local time hours from RTC [0,23].
// Thread-safe and ISR-safe.
uint_least8_t getHoursLT();

// Get minutes since midnight local time [0,1439].
// Useful to fetch time atomically for scheduling purposes.
// Thread-safe and ISR-safe.
uint_least16_t getMinutesSinceMidnightLT();

// Get whole days since the start of 2000/01/01 (ie the midnight between 1999 and 2000), local time.
// This will roll in about 2179.
// Thread-safe and ISR-safe.
uint_least16_t getDaysSince1999LT();


// Set time as hours [0,23] and minutes [0,59].
// Will ignore attempts to set bad values and return false in that case.
// Returns true if all OK and the time has been set.
// Does not attempt to set seconds.
// Thread/interrupt safe, but do not call this from an ISR.
// Will persist time to survive reset as neceessary.
bool setHoursMinutesLT(int hours, int minutes);


// Length of main loop and wakeup cycle/tick in seconds.
#if defined(TWO_S_TICK_RTC_SUPPORT)
#define MAIN_TICK_S 2
#else
#define MAIN_TICK_S 1
#endif



#ifdef USE_RTC_INTERNAL_SIMPLE // Provide software RTC support by default.
// Declared inline to allow optimal generation of ISR by compiler (eg avoiding some register push/pop).
#if defined(TWO_S_TICK_RTC_SUPPORT)
// Call to indicate that two seconds have passed/rolled.
// May be called from an ISR, so must not:
//   * do anything expensive,
//   * access EEPROM,
//   * enable interrupts,
//   * alter interrupt state from how it was on entry.
// If not being called from an ISR then locking round this call that works with the getXXX() functions should be considered.
static inline void tickDoubleSecondISR()
#else
// Call to indicate that a second has passed/rolled.
// May be called from an ISR, so must not do anything expensive,
// must not enable interrupts, and must leave interrupt state as was on entry.
// If not being called from an ISR then locking round this call that works with the getXXX() functions should be considered.
static inline void tickSecondISR()
#endif
  {
  register uint_fast8_t sTemp = _secondsLT; // Avoid some redundant memory accesses.
  sTemp += MAIN_TICK_S;
  if(sTemp > 59)
    {
    sTemp = 0; // Seconds roll.
    register uint_least16_t mTemp = _minutesSinceMidnightLT + 1; // Avoid some redundant memory accesses.
    if(mTemp > 1439)
      {
      mTemp = 0; // Minutes/hours roll.
      // Increment the day.
      ++_daysSince1999LT; // Don't currently prevent roll.
      }
    _minutesSinceMidnightLT = mTemp;
    }
  _secondsLT = sTemp;
  }
#endif

#endif


