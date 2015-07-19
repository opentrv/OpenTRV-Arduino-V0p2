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

#include <util/atomic.h>

#include "EEPROM_Utils.h"







// Updates an EEEPROM byte iff not currently at the specified target value.
// May be able to selectively erase or write (ie reduce wear) to reach the desired value.
// As with the AVR eeprom_XXX_byte() macros, not safe to use outside and within ISRs as-is.
// Returns true iff an erase and/or write was performed.
// TODO: make smarter, eg don't wait/read twice...
bool eeprom_smart_update_byte(uint8_t *p, uint8_t value)
  {
  // If target byte is 0xff then attempt smart erase rather than more generic write or erase+write.
  if((uint8_t) 0xff == value) { return(eeprom_smart_erase_byte(p)); }
  // More than an erase might be required...
  const uint8_t oldValue = eeprom_read_byte(p);
  if(value == oldValue) { return(false); } // No change needed.
#ifdef EEPROM_SPLIT_ERASE_WRITE // Can do selective write.
  if(value == (value & oldValue)) { return(eeprom_smart_clear_bits(p, value)); } // Can use pure write to clear bits to zero.
#endif
  eeprom_write_byte(p, value); // Needs to set some (but not all) bits to 1, so needs erase and write.
  return(true); // Performed an update.
  }

// Erases (sets to 0xff) the specified EEPROM byte, avoiding a following (redundant) write if possible.
// If the target byte is already 0xff then this does nothing at all beyond an initial read.
// This saves a bit of time and power and possibly a little EEPROM cell wear also.
// Without split erase/write this degenerates to a specialised eeprom_update_byte().
// As with the AVR eeprom_XXX_byte() macros, not safe to use outside and within ISRs as-is.
// Returns true iff an erase was performed.
bool eeprom_smart_erase_byte(uint8_t *p)
  {
#ifndef EEPROM_SPLIT_ERASE_WRITE // No split erase/write so do as a slightly smart update...
  if((uint8_t) 0xff == eeprom_read_byte(p)) { return(false); } // No change/erase needed.
  eeprom_write_byte(p, 0xff); // Set to 0xff.
  return(true); // Performed an erase (and probably a write, too).
#else

  // Wait until EEPROM is idle/ready.
  eeprom_busy_wait();

  // Following block equivalent to:
  //     if((uint8_t) 0xff == eeprom_read_byte(p)) { return; }
  // but leaves EEAR[L] set up appropriately for any erase or write.
#if E2END <= 0xFF
  EEARL = (uint8_t)p;
#else
  EEAR = (uint16_t)p;
#endif
  // Ignore problems that some AVRs have with EECR and STS instructions (ATmega64 errata).
  EECR = _BV(EERE); // Start EEPROM read operation.
  const uint8_t oldValue = EEDR; // Get old EEPROM value.
  if((uint8_t) 0xff != oldValue) // Needs erase...
    {
    ATOMIC_BLOCK (ATOMIC_RESTORESTATE) // Avoid timing problems from interrupts.
      {
      // Erase to 0xff; no write needed.
      EECR = _BV(EEMPE) | _BV(EEPM0); // Set master write-enable bit and erase-only mode.
      EECR |= _BV(EEPE);  // Start erase-only operation.
      }
    return(true); // Performed the erase.
    }
  return(false);
#endif
  }

// ANDs the supplied mask into the specified EEPROM byte, avoiding an initial (redundant) erase if possible.
// This can be used to ensure that specific bits are 0 while leaving others untouched.
// If ANDing in the mask has no effect then this does nothing at all beyond an initial read.
// This saves a bit of time and power and possibly a little EEPROM cell wear also.
// Without split erase/write this degenerates to a specialised eeprom_update_byte().
// As with the AVR eeprom_XXX_byte() macros, not safe to use outside and within ISRs as-is.
// Returns true iff a write was performed.
bool eeprom_smart_clear_bits(uint8_t *p, uint8_t mask)
  {
#ifndef EEPROM_SPLIT_ERASE_WRITE // No split erase/write so do as a slightly smart update...
  const uint8_t oldValue = eeprom_read_byte(p);
  const uint8_t newValue = oldValue & mask;
  if(oldValue == newValue) { return(false); } // No change/write needed.
  eeprom_write_byte(p, newValue); // Set to masked value.
  return(true); // Performed a write (and probably an erase, too).
#else

  // Wait until EEPROM is idle/ready.
  eeprom_busy_wait();

  // Following block equivalent to:
  //     oldValue = eeprom_read_byte(p);
  // and leaves EEAR[L] set up appropriately for any erase or write.
#if E2END <= 0xFF
  EEARL = (uint8_t)p;
#else
  EEAR = (uint16_t)p;
#endif
  // Ignore problems that some AVRs have with EECR and STS instructions (ATmega64 errata).
  EECR = _BV(EERE); // Start EEPROM read operation.
  const uint8_t oldValue = EEDR; // Get old EEPROM value.
  const uint8_t newValue = oldValue & mask;
  if(oldValue != newValue) // Write is needed...
    {
    // Do the write: no erase is needed.
    EEDR = newValue; // Set EEPROM data register to required new value.
    ATOMIC_BLOCK (ATOMIC_RESTORESTATE) // Avoid timing problems from interrupts.
      {
      EECR = _BV(EEMPE) | _BV(EEPM1); // Set master write-enable bit and write-only mode.
      EECR |= _BV(EEPE);  // Start write-only operation.
      }
    return(true); // Performed the write.
    }
  return(false);
#endif
  }


