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
                           Gary Gladman 2014
*/

#ifndef RTC_SUPPORT_H
#define RTC_SUPPORT_H

#include <stdint.h>

#include "V0p2_Main.h"

// Number of minutes per day.
#define MINS_PER_DAY 1440

// Length of main loop and wakeup cycle/tick in seconds.
#if defined(TWO_S_TICK_RTC_SUPPORT)
#define MAIN_TICK_S 2
#else
#define MAIN_TICK_S 1
#endif


class RTC {
  private:
    // Minutes since midnight for local time in range [0,1439].
    // Must be accessed with interrupts disabled and as if volatile.
    // Maintained locally or shadowed from external RTC.
    // NOT FOR DIRECT ACCESS OUTSIDE RTC ROUTINES.
    static volatile uint_least16_t _minutesSinceMidnightLT;

    // Whole days since the start of 2000/01/01 (ie the midnight between 1999 and 2000), local time.
    // This will roll in about 2179.
    static uint_least16_t daysSince1999LT;

    // Seconds for local time (and assumed UTC) in range [0,59].
    // Volatile to allow for async update.
    // Maintained locally or shadowed from external RTC.
    // Read and write accesses assumed effectively atomic.
    // NOT FOR DIRECT ACCESS OUTSIDE RTC ROUTINES.
    static volatile uint_fast8_t _secondsLT;

#if defined(DATE)

    // Roughly which "month" [0..11] of the current year.
    // Slight bias towards heating season/winter and "december".
    static uint8_t y12;

#endif

  protected:
  public:
    // TODO: more detailed thought?
    RTC() {};

    // Persist software RTC information to non-volatile (EEPROM) store.
    // This does not attempt to store full precision of time down to seconds,
    // but enough to help avoid the clock slipping too much during (say) a battery change.
    // There is no point calling this more than (say) once per minute,
    // though it will simply return relatively quickly from redundant calls.
    // The RTC data is stored so as not to wear out AVR EEPROM for at least several years.
    static void persistRTC();

    // Restore software RTC information from non-volatile (EEPROM) store, if possible.
    // Returns true if the persisted data seemed valid and was restored, in full or part.
    static const bool restoreRTC();

    // Get local time seconds from RTC [0,59].
    // Is as fast as reasonably practical.
    // Returns a consistent atomic snapshot.
    static inline const uint_fast8_t getSecondsLT() /* const */ {return(_secondsLT);}; // Assumed atomic.

    // Get local time minutes from RTC [0,59].
    static const uint_least8_t getMinutesLT() /* const */;

    // Get local time hours from RTC [0,23].
    static const uint_least8_t getHoursLT() /* const */;

    // Get minutes since midnight local time [0,1439].
    // Useful to fetch time atomically for scheduling purposes.
    static const uint_least16_t getMinutesSinceMidnightLT() /* const */;

    // FIXME Private - subsumed by getDaysLT
    // Get whole days since the start of 2001/01/01 (ie the midnight between 2000 and 2001), local time [0,].
    // This will roll in about 2179, by which time I will not care.
    static inline const uint_least16_t getDaysSince1999LT() /* const */ {return(daysSince1999LT);};

    // Set time as hours [0,23] and minutes [0,59].
    // Will ignore attempts to set bad values and return false in that case.
    // Returns true if all OK and the time has been set.
    // Does not attempt to set seconds.
    // Thread/interrupt safe, but do not call this from an ISR.
    // Will persist time to survive reset as neceessary.
    static const bool setHoursMinutesLT(const int hours, const int minutes);

#if defined(DATE)

    // Roughly which "month" of the current year [0,11].
    // Slight bias towards heating season/winter and "December".
    static inline const uint8_t getY12() /* const */ {return(y12);};

    // Whole days since date baseline of 2001/01/01 (ie the midnight between 2000 and 2001), local time [0,].
    // Alternate base date simplifies calculations.
    // This will roll in about 2179, by which time I will not care.
    static inline const uint_least16_t getDaysLT() /* const */ {return(getDaysSince1999LT());};

    // Set days as days
    // Will ignore attempts to set bad values and return false in that case.
    // Returns true if all OK and the time has been set.
    // Thread/interrupt safe, but do not call this from an ISR.
    // Will persist time to survive reset as neceessary.
    static const bool setDaysLT(const uint_least16_t days);

    // Calculate date ancillary attributes from days.
    static void calcDateLT(const uint_least16_t days);

    // Tick RTC minute timer.
    static void tickMinuteRTC();

#else

    static void calcDateLT(const uint_least16_t days) {};  // No-op for convenience
    static void tickMinuteRTC() {};  // No-op for convenience

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
          }
        _minutesSinceMidnightLT = mTemp;
        }
      _secondsLT = sTemp;
      }
#endif

#endif

};

