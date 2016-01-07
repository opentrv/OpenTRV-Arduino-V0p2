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
  V0p2 (V0.2) core/main header file for this project:
  all other project header files should #include this first
  (or at least immediately after std/AVR headers) for consistency,
  and project non-header files should include this via their own header files (or directly).
  */

#ifndef V0p2_MAIN_H
#define V0p2_MAIN_H


// GLOBAL flags that alter system build and behaviour.
#define DEBUG // If defined, do extra checks and serial logging.  Will take more code space and power.
//#define ALT_MAIN_LOOP // If defined, normal main loop and POST are REPLACED with alternates, for non-OpenTRV builds.
//#define UNIT_TESTS // If defined, normal main loop is REPLACED with a unit test cycle.  Usually define DEBUG also for get serial logging.
//#define EST_CPU_DUTYCYCLE // If defined, estimate CPU duty cycle and thus base power consumption.

//#define COMPAT_UNO // If defined, allow code to run on stock Arduino UNO board.  NOT IMPLEMENTED

//#define DONT_USE_TIMER0 // Avoid using timer 0 and thus delay(), millis(), PWM pins 5&6, etc.

#ifndef BAUD
#define BAUD 4800 // Ensure that OpenTRV 'standard' UART speed is set unless explicitly overridden.
#endif

#include <Arduino.h>
#include <OTV0p2Base.h>
#include <OTRadioLink.h>
#include <OTRadValve.h>
#include "V0p2_Generic_Config.h" // Config switches and module dependencies.
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

// Link in support for alternate Power On Self-Test (startup) and main loop if required.
#if defined(ALT_MAIN_LOOP) // Exclude code from production systems.
extern void POSTalt();
extern void loopAlt();
#endif

#if defined(UNIT_TESTS) // Exclude code from production systems.
// To be called from loop() instead of main code when running unit tests.
extern void loopUnitTest();
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


// Call this to do an I/O poll if needed; returns true if something useful happened.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// Limits actual poll rate to something like once every 32ms, unless force is true.
//   * force if true then force full poll on every call (ie do not internally rate-limit)
// NOTE: implementation may not be in power-management module.
bool pollIO(bool force = false);
// Nap productively polling I/O, etc, across the system while spending time in low-power mode if possible.
// Typically sleeps for about 30ms; tries to allow earlier wakeup if interrupt is received, etc.
// True iff watchdog timer expired; false if something else woke the CPU.
static bool inline nap15AndPoll() { const bool wd = ::OTV0P2BASE::nap(WDTO_15MS, true); pollIO(!wd); return(wd); }

// Call this to productively burn tens to hundreds of CPU cycles, and poll I/O, eg in a busy-wait loop.
// This may churn PRNGs or gather entropy for example.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// May capture some entropy in secure and non-secure PRNGs.
inline void burnHundredsOfCyclesProductivelyAndPoll()
  {
  if(pollIO()) { OTV0P2BASE::seedRNG8(OTV0P2BASE::getCPUCycleCount(), 37 /* _watchdogFired */, OTV0P2BASE::_getSubCycleTime()); }
  else { OTV0P2BASE::captureEntropy1(); }
  }



#endif


