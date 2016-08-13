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
 EEPROM space allocation and utilites.
 */

#ifndef EEPROM_UTILS_H
#define EEPROM_UTILS_H

#include <avr/eeprom.h>


#include "V0p2_Main.h"



// Define if not present in our AVR header, eg see: http://arduino.cc/forum/index.php?topic=37849.0
#if 0
#ifndef eeprom_update_byte
#define eeprom_update_byte(loc, val) \
    do \
      { \
      if((uint8_t)(val) != eeprom_read_byte((loc))) \
          { eeprom_write_byte((loc), (val)); } \
      } while(false)
#endif
#endif



// ATmega328P has 1kByte of EEPROM, with an underlying page size (datasheet section 27.5) of 4 bytes for wear purposes.
// Endurance may be per page (or per bit-change), rather than per byte, eg: http://www.mail-archive.com/avr-libc-dev@nongnu.org/msg02456.html
// Also see AVR101: High Endurance EEPROM Storage: http://www.atmel.com/Images/doc2526.pdf
// Also see AVR103: Using the EEPROM Programming Modes http://www.atmel.com/Images/doc2578.pdf
// Note that with split erase/program operations specialised bitwise programming can be achieved with lower wear.
#if defined(__AVR_ATmega328P__)
#define EEPROM_SIZE 1024
#define EEPROM_PAGE_SIZE 4
#define EEPROM_SPLIT_ERASE_WRITE // Separate erase and write are possible.
#endif

// Unit test location for erase/write.
// Also may be more vulnerable to damage during resets/brown-outs.
#define EE_START_TEST_LOC 0 // 1-byte test location.
// Second unit test location for erase/write.
#define EE_START_TEST_LOC2 1 // 1-byte test location.
// Store a few bits of (non-secure) random seed/entropy from one run to another.
// Used in a way that increases likely EEPROM endurance.
#define EE_START_SEED 2 // 2-byte store for part of (non-crypto) random seed.

// Space for RTC to persist current day/date.
#define EE_START_RTC_DAY_PERSIST 4 // 2-byte store for RTC to persist day/date.
// Space for RTC to persist current time in 15-minute increments with a low-wear method.
#define EE_START_RTC_HHMM_PERSIST 6 // 1-byte store for RTC to persist time of day.  Not in same page as anything else updated frequently.

// 2-byte block to support simple schedule, if in use.
#define EE_START_SIMPLE_SCHEDULE_ON 8 // 1-byte encoded 'minutes after midnight' on time, if any.
#define EE_START_SIMPLE_SCHEDULE_OFF 9 // 1-byte encoded 'minutes after midnight' off time, if any.

// For FHT8V wireless radiator valve control.
#define EE_START_FHT8V_HC1 10 // (When controlling FHT8V rad valve): 1-byte value for house-code 1.
#define EE_START_FHT8V_HC2 11 // (When controlling FHT8V rad valve): 1-byte value for house-code 2.

// For overrides of default FROST and WARM target values (in C); 0xff means 'use defaults'.
#define EE_START_FROST_C 12 // Overridde for FROST target/threshold.
#define EE_START_WARM_C 13 // Overridde for WARM target/threshold.


// Updates an EEEPROM byte iff not current correct.
// May be able to selectively erase or write (ie reduce wear) to reach the desired value.
// Returns true if a write was done.
bool eeprom_smart_update_byte(uint8_t *p, uint8_t value);

// Erases (sets to 0xff) the specified EEPROM byte, avoiding a (redundant) write if possible.
// If the target byte is already 0xff this does nothing at all.
// This saves a bit of time and power and possibly a little wear also.
// Without split erase/write this degenerates to a specialised eeprom_update_byte().
// As with the AVR eeprom_XXX_byte() macros, interrupts should be avoided/disabled during this call.
// Returns true iff an erase was performed.
bool eeprom_smart_erase_byte(uint8_t *p);

// ANDs the supplied mask into the specified EEPROM byte, avoiding an initial (redundant) erase if possible.
// This can be used to ensure that specific bits are 0 while leaving others untouched.
// If ANDing in the mask has no effect then this does nothing at all beyond an initial read.
// This saves a bit of time and power and possibly a little EEPROM cell wear also.
// Without split erase/write this degenerates to a specialised eeprom_update_byte().
// As with the AVR eeprom_XXX_byte() macros, interrupts should be avoided/disabled during this call.
// Returns true iff a write was performed.
bool eeprom_smart_clear_bits(uint8_t *p, uint8_t mask);

#endif
