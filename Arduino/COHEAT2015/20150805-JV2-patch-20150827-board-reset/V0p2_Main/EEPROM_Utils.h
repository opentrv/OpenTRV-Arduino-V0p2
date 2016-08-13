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
 EEPROM space allocation and utilities.

 NOTE: NO EEPROM ACCESS SHOULD HAPPEN FROM ANY ISR CODE ELSE VARIOUS FAILURE MODES ARE POSSIBLE
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
// Deliberately crosses an EEPROM page boundary.
#define EE_START_SEED 2 // *4-byte* store for part of (non-crypto) persistent random seed.
#define EE_LEN_SEED 4 // *4-byte* store for part of (non-crypto) persistent random seed.

// Reset/restart count (least-significant 2 bytes) for diagnostic and crypto (eg nonce) purposes.
#define EE_START_RESET_COUNT 6 // Modulo-256 reset count (ls byte), for diagnostic and crypto purposes.
#define EE_START_RESET_COUNT2 7 // Second byte of reset count, for diagnostic and crypto purposes..

// Space for RTC to persist current day/date.
#define EE_START_RTC_DAY_PERSIST 8 // 2-byte store for RTC to persist day/date.
// Space for RTC to persist current time in 15-minute increments with a low-wear method.
// Nothing else receiving frequent updates should go in this EEPROM page if possible.
#define EE_START_RTC_HHMM_PERSIST 10 // 1-byte store for RTC to persist time of day.  Not in same page as anything else updated frequently.
#define EE_START_RTC_RESERVED 11 // Reserved byte for future use.  (Could store real minutes of power-fail on last gasp.)

// 2-byte block to support primary simple 7-day schedule, if in use.
#define EE_START_SIMPLE_SCHEDULE0_ON 12 // 1-byte encoded 'minutes after midnight' / 6 primary on-time, if any.
#define EE_START_MAX_SIMPLE_SCHEDULES 2 // Maximum number of 'ON' schedules that can be stored, starting with schedule 0.

// For overrides of default FROST and WARM target values (in C); 0xff means 'use defaults'.
#define EE_START_FROST_C 14 // Override for FROST target/threshold.
#define EE_START_WARM_C 15 // Override for WARM target/threshold.

// For FHT8V wireless radiator valve control.
#define EE_START_FHT8V_HC1 16 // (When controlling FHT8V rad valve): 1-byte value for house-code 1, 0xff if not in use.
#define EE_START_FHT8V_HC2 17 // (When controlling FHT8V rad valve): 1-byte value for house-code 2, 0xff if not in use.

// One byte BITWISE-INVERTED minimum number of minutes of boiler time ON; ~0 (erased/default) means NOT in hub/boiler mode.
// Bitwise-inverted so that erased/unset 0xff (~0) value leaves boiler mode disabled.
#define EE_START_MIN_BOILER_ON_MINS_INV 18 // Bitwise inverted minimum boiler on-time or zero if not in central boiler hub mode.

// Minimum (percentage) threshold that local radiator valve is considered open.
#define EE_START_MIN_VALVE_PC_REALLY_OPEN 19 // Ignored entirely if outside range [1,100], eg if default/unprogrammed 0xff.

// Generic (8-byte) node ID, of which usually only 2 first bytes are used in OpenTRV-native messages.
// All valid ID bytes have the high bit set but are not 0xff, ie in the range [128,254].
// An 0xff value indicates not set and the system may automatically generate a new ID byte.
// Note that in the presence of (say) a house-code, that takes precedence;
// Since FHT8V house codes are in the range [0,99] there is no ambiguity between these values.
#define EE_START_ID 20 // (When controlling FHT8V rad valve): 1-byte value for house-code 1, 0xff if not in use.
#define EE_LEN_ID 8 // 64-bit unique ID.  Only first 2 bytes generally used in OpenTRV-native messages.

// 1-byte valuye used to enable/disable stats transmissions.
// A combination of this value and available channel security determines how much is transmitted.
#define EE_START_STATS_TX_ENABLE 28 // 0xff disables all avoidable stats transmissions; 0 enables all.
// A 1-byte overrun counter, inverted so 0xff means 0.
#define EE_START_OVERRUN_COUNTER 29

// Maximum (percentage) that local radiator value is allowed to open.
#define EE_START_MAX_VALVE_PC_OPEN 30 // Ignored entirely if outside range [1,100], eg if default/unprogrammed 0xff.

// Minimum (total percentage across all rads) that all rads should be on before heating should fire.
// Default might be (say) DEFAULT_VALVE_PC_MODERATELY_OPEN eg 33%, or twice the minimum.
#define EE_START_MIN_TOTAL_VALVE_PC_OPEN 31 // Ignored entirely if outside range [1,100], eg if default/unprogrammed 0xff.




// Housecode filter at central hub.
// Intended to fit snug up before stats area.
#define EE_START_HUB_HC_FILTER 240
#define EE_HUB_HC_FILTER_COUNT 8 // Max count of house codes (each 2 bytes) filtered for.
#define EE_END_HUB_HC_FILTER (EE_START_HUB_HC_FILTER+2*(EE_HUB_HC_FILTER_COUNT)-1)

// Bulk data storage: should fit within 1kB EEPROM of ATmega328P or 512B of ATmega164P.
#define EE_START_STATS 256 // INCLUSIVE START OF BULK STATS AREA.
#define EE_STATS_SET_SIZE 24 // Size in entries/bytes of one normal EEPROM-resident hour-of-day stats set.

// Stats set numbers, 0 upwards, contiguous.
// Generally even-numbered values are 'last' values and odd-numbered are 'smoothed' nominally over a week.
#define EE_STATS_SET_TEMP_BY_HOUR               0  // Last companded temperature samples in each hour in range [0,248].
#define EE_STATS_SET_TEMP_BY_HOUR_SMOOTHED      1  // Smoothed hourly companded temperature samples in range [0,248].
#define EE_STATS_SET_AMBLIGHT_BY_HOUR           2  // Last ambient light level samples in each hour in range [0,254].
#define EE_STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED  3  // Smoothed ambient light level samples in each hour in range [0,254].
#define EE_STATS_SET_OCCPC_BY_HOUR              4  // Last hourly observed occupancy percentage [0,100].
#define EE_STATS_SET_OCCPC_BY_HOUR_SMOOTHED     5  // Smoothed hourly observed occupancy percentage [0,100].
#define EE_STATS_SET_RHPC_BY_HOUR               6  // Last hourly relative humidity % samples in range [0,100].
#define EE_STATS_SET_RHPC_BY_HOUR_SMOOTHED      7  // Smoothed hourly relative humidity % samples in range [0,100].
#define EE_STATS_SET_USER1_BY_HOUR              8  // Last hourly user-defined stats value in range [0,254].
#define EE_STATS_SET_USER1_BY_HOUR_SMOOTHED     9  // Smoothed hourly user-defined stats value in range [0,254].
//#define EE_STATS_SET_WARMMODE_BY_HOUR_OF_WK    10  // Each sample is last 7 days' WARM mode bitset by hour. [0,127].

#define EE_STATS_SETS 10 // Number of stats sets in range [0,EE_STATS_SETS-1].

// Compute start of stats set n (in range [0,EE_STATS_SETS-1]) in EEPROM.
// Eg use as EE_STATS_START_ADDR(EE_STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED) in
//   const uint8_t smoothedAmbLight = eeprom_read_byte((uint8_t *)(EE_STATS_START_ADDR(EE_STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED) + hh));
#define EE_STATS_START_ADDR(n) (EE_START_STATS + 24*(n))
// INCLUSIVE END OF BULK STATS AREA: must point to last byte used.
#define EE_END_STATS (EE_STATS_START_ADDR(EE_STATS_SETS+1)-1)



#if EE_END_HUB_HC_FILTER >= EE_START_STATS
#error EEPROM allocation problem: Hub HC filter overlaps with stats
#endif


// Updates an EEPROM byte iff not currently at the specified target value.
// May be able to selectively erase or write (ie reduce wear) to reach the desired value.
// As with the AVR eeprom_XXX_byte() macros, not safe to use outside and within ISRs as-is.
// Returns true iff an erase and/or write was performed.
bool eeprom_smart_update_byte(uint8_t *p, uint8_t value);

// Erases (sets to 0xff) the specified EEPROM byte, avoiding a (redundant) write if possible.
// If the target byte is already 0xff then this does nothing at all beyond an initial read.
// This saves a bit of time and power and possibly a little wear also.
// Without split erase/write this degenerates to a specialised eeprom_update_byte().
// As with the AVR eeprom_XXX_byte() macros, not safe to use outside and within ISRs as-is.
// Returns true iff an erase was performed.
bool eeprom_smart_erase_byte(uint8_t *p);

// ANDs the supplied mask into the specified EEPROM byte, avoiding an initial (redundant) erase if possible.
// This can be used to ensure that specific bits are 0 while leaving others untouched.
// If ANDing in the mask has no effect then this does nothing at all beyond an initial read.
// This saves a bit of time and power and possibly a little EEPROM cell wear also.
// Without split erase/write this degenerates to a specialised eeprom_smart_update_byte().
// As with the AVR eeprom_XXX_byte() macros, not safe to use outside and within ISRs as-is.
// Returns true iff a write was performed.
bool eeprom_smart_clear_bits(uint8_t *p, uint8_t mask);

#endif

