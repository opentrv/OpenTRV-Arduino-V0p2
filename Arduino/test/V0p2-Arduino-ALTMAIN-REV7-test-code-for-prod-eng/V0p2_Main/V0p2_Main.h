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

/*
  V0p2 (V0.2) core/main header file for this project:
  all other project header files should #include this first
  (or at least immediately after std/AVR headers) for consistency,
  and project non-header files should include this via their own header files (or directly).
  */

#ifndef V0p2_MAIN_H
#define V0p2_MAIN_H


// GLOBAL flags that alter system build and behaviour.
#define DEBUG // If defined, do extra checks and serial logging.  Will take more code space and power.
#define ALT_MAIN_LOOP // If defined, normal main loop and POST are REPLACED with alternates, for non-OpenTRV builds.
//#define UNIT_TESTS // If defined, normal main loop is REPLACED with a unit test cycle.  Usually define DEBUG also for get serial logging.
//#define EST_CPU_DUTYCYCLE // If defined, estimate CPU duty cycle and thus base power consumption.

//#define COMPAT_UNO // If defined, allow code to run on stock Arduino UNO board.  NOT IMPLEMENTED

#define TWO_S_TICK_RTC_SUPPORT // Wake up every 2 seconds, rather than every 1s, to save power.
//#define DONT_USE_TIMER0 // Avoid using timer 0 and thus delay(), millis(), PWM pins 5&6, etc.

#ifndef BAUD
#define BAUD 4800 // Ensure that OpenTRV 'standard' UART speed is set unless explicitly overridden.
#endif

#include <Arduino.h>
#include "V0p2_Generic_Config.h" // Config switches and module dependencies.
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.


// Link in support for alternate Power On Self-Test (startup) and main loop if required.
#if defined(ALT_MAIN_LOOP)
extern void POSTalt();
extern void loopAlt();
#endif


// Indicate that the system is broken in an obvious way (distress flashing of the main UI LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep the distress beacon running for a while.
void panic();
// Panic with fixed message.
void panic(const __FlashStringHelper *s);


// Version (code/board) information printed as one line to serial (with line-end, and flushed); machine- and human- parseable.
// Format: "board VXXXX REVY; code YYYY/Mmm/DD HH:MM:SS".
void serialPrintlnBuildVersion();




// Templated function versions of min/max that do not evaluate the arguments twice.
template <class T> const T& fnmin(const T& a, const T& b) { return((a>b)?b:a); }
template <class T> const T& fnmax(const T& a, const T& b) { return((a<b)?b:a); }


// Fast read of digital pins where pin number is constant.
// Avoids lots of logic (many 10s of CPU cycles) in normal digitalRead()/digitalWrite() calls,
// and this saves time and energy on (critical) paths polling I/O.
// Does not do any error checking: beware.
// Only really intended for ATmega328P.
#ifdef __AVR_ATmega328P__
// READ GENERIC
/* Register: PIND for 0--7, PINB for 8--13, 14--19 PINC (ADC/AI). */
/* Bit: 0--7 as-is, 8--13 subtract 8, else subtract 14. */
// Handle quickly constant-value pins that we know about; fall back to generic run-time routine for rest.
#define fastDigitalRead(pin) \
    ((__builtin_constant_p((pin)) && ((pin) >= 0) && ((pin) < 8)) ? ((int) ((PIND >> (pin)) & 1)) : \
    ((__builtin_constant_p((pin)) && ((pin) >= 8) && ((pin) < 14)) ? ((int) ((PINB >> max(((pin)-8),0)) & 1)) : \
    ((__builtin_constant_p((pin)) && ((pin) >= 14) && ((pin) < 20)) ? ((int) ((PINC >> max(((pin)-14),0)) & 1)) : \
    digitalRead((pin))))) // Fall back to generic routine.    
// WRITE GENERIC
/* Register: PORTD for 0--7, PORTB for 8--13, (eg 13 is PORTB), 14--19 PINC (ADC/AI). */
/* Bit: 0--7 as-is, 8--13 subtract 8, else subtract 14. */
// Handle quickly constant-value pins that we know about; fall back to generic run-time routine for rest.
#define fastDigitalWrite(pin, value) do { \
    if(__builtin_constant_p((pin)) && (__builtin_constant_p(value) && ((pin) >= 0) && ((pin) < 8))) { bitWrite(PORTD, (pin), (value)); } \
    else if(__builtin_constant_p((pin)) && (__builtin_constant_p((value)) && ((pin) >= 8) && ((pin) < 14))) { bitWrite(PORTB, max((pin)-8,0), (value)); } \
    else if(__builtin_constant_p((pin)) && (__builtin_constant_p((value)) && ((pin) >= 14) && ((pin) < 20))) { bitWrite(PORTC, max((pin)-14,0), (value)); } \
    else { digitalWrite((pin), (value)); } } while(false) // Fall back to generic routine.    
#else
#define fastDigitalRead(pin) digitalRead((pin)) // Don't know about other AVRs.
#define fastDigitalWrite(pin, value) digitalWrite((pin), (value)) // Don't know about other AVRs.
#endif


#endif


