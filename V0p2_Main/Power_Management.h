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
Utilities to assist with minimal power usage,
including interrupts and sleep.
*/

#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <util/delay_basic.h>
#include <stdint.h>

#include "V0p2_Main.h"

#include <OTV0p2Base.h>


// Call from setup() to turn off unused modules, set up timers and interrupts, etc, for OpenTRV V0p2 board.
// I/O pin configuration is not done here.
void powerSetup();

// Selectively turn off all modules that need not run continuously so as to minimise power without sleeping.
// Suitable for start-up and for belt-and-braces use before main sleep on each cycle,
// to ensure that nothing power-hungry is accidentally left on.
// Any module that may need to run all the time should not be turned off here.
// May be called from panic(), so do not be too clever.
// Does NOT attempt to power down the radio, eg in case that needs to be left in RX mode.
// Does NOT attempt to adjust serial power state.
void minimisePowerWithoutSleep();


// Sensor for supply (eg battery) voltage in millivolts.
// Singleton implementation/instance.
extern OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;


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

#if !defined(OTV0P2BASE_IDLE_NOT_RECOMMENDED) && defined(ENABLE_USE_OF_AVR_IDLE_MODE)
// Idle productively polling I/O, etc, across the system while spending time in low-power mode if possible.
// Typically sleeps for nominally up to 30ms; tries to allow earlier wakeup if interrupt is received, etc.
// (Will often be prematurely woken by timer0 with ~16ms interval.)
// True iff watchdog timer expired; false if something else woke the CPU.
// Only use this if not disallowed for board type, eg with ENABLE_USE_OF_AVR_IDLE_MODE.
static bool inline idle15AndPoll() { const bool wd = ::OTV0P2BASE::_idleCPU(WDTO_15MS, true); pollIO(!wd); return(wd); }
#endif

// Call this to productively burn tens to hundreds of CPU cycles, and poll I/O, eg in a busy-wait loop.
// This may churn PRNGs or gather entropy for example.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// May capture some entropy in secure and non-secure PRNGs.
void burnHundredsOfCyclesProductivelyAndPoll();


#endif

