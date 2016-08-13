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

//#define USE_WDT_FOR_SHORT_DELAYS // If defined use WDT rather than slowing main system clock.

// If CPU clock is 1MHz then *assume* that it is the 8MHz internal RC clock prescaled by 8 unless DEFAULT_CPU_PRESCALE is defined.
#if F_CPU == 1000000L
#ifndef DEFAULT_CPU_PRESCALE
#define DEFAULT_CPU_PRESCALE 3
#endif
#endif

//#ifndef DEFAULT_CPU_PRESCALE
//// Default prescale value at start-up computed and stored once.
//extern const clock_div_t DEFAULT_CPU_PRESCALE;
//#endif

#define MAX_CPU_PRESCALE clock_div_256 // At least for the ATmega328P...
#define MIN_CPU_HZ ((F_CPU) >> (((int) MAX_CPU_PRESCALE) - (DEFAULT_CPU_PRESCALE)))

// By default allow use of IDLE mode to save some power while leaving most things running.
#ifndef DISABLE_AVR_IDLE_MODE
#define ENABLE_AVR_IDLE_MODE
#endif

// Sleep for specified number of _delay_loop2() loops at minimum available CPU speed.
// Each loop takes 4 cycles at that minimum speed, but entry and exit overheads may take the equivalent of a loop or two.
// Note: inlining is prevented so as to avoid migrating anything into the section where the CPU is running slowly.
void sleepLowPowerLoopsMinCPUSpeed(uint16_t loops) __attribute__ ((noinline));

// Sleep/spin for approx specified strictly-positive number of milliseconds, in as low-power mode as possible.
// This may be achieved in part by dynamically slowing the CPU clock if possible.
// Macro to allow some constant folding at compile time where the sleep-time argument is constant.
// Should be good for values up to at least 1000, ie 1 second.
// Assumes MIN_CPU_HZ >> 4000.
#define sleepLowPowerMs(ms) sleepLowPowerLoopsMinCPUSpeed((((MIN_CPU_HZ * (ms)) + 2000) / 4000) - ((MIN_CPU_HZ>=12000)?2:((MIN_CPU_HZ>=8000)?1:0)))

// Sleep/spin for (typically a little less than) strictly-positive specified number of milliseconds, in as low-power mode as possible.
// This may be achieved in part by dynamically slowing the CPU clock if possible.
// Macro to allow some constant folding at compile time where the sleep-time argument is constant.
// Should be good for values up to at least 1000, ie 1 second.
// Uses formulation likely to be quicker for non-constant argument values,
// and that results in a somewhat shorter sleep than sleepLowPowerMs(ms).
// Assumes MIN_CPU_HZ >> 4000.
#define sleepLowPowerLessThanMs(ms) sleepLowPowerLoopsMinCPUSpeed(((MIN_CPU_HZ/4000) * (ms)) - ((MIN_CPU_HZ>=12000)?2:((MIN_CPU_HZ>=8000)?1:0)))




// Call from setup() to turn off unused modules, set up timers and interrupts, etc.
// I/O pin setting is not done here.
void powerSetup();

// Get power supply voltage in mV; non-negative.
// Only accurate to +/- 10%.
// May set sleep mode to SLEEP_MODE_ADC, and disables sleep on exit.
uint16_t readBatterymV();

// Get power supply voltage in mV as last read by readBatterymV(); non-negative, intially zero until first readBatterymV().
uint16_t getBatterymV();

// True if battery voltage was low when last read.
// For a 2xAA NiMH configuration this is ~2.0V, where the BOD may force a reset at 1.8V.
bool isBatteryLow();


// Get approximate internal temperature in nominal C/16.
// Only accurate to +/- 10C uncalibrated.
// May set sleep mode to SLEEP_MODE_ADC, and disables sleep on exit.
int readInternalTemperatureC16();

// Selectively turn off all modules that need not run continuously so as to minimise power without sleeping.
// Suitable for start-up and for belt-and-braces use before main sleep on each cycle,
// to ensure that nothing is accidentally left on.
// Any module that may need to run all the time should not be turned off here.
// Does NOT power down radio, eg to allow RX during sleep.
void minimisePowerWithoutSleep();

// Sleep with BOD disabled in power-save mode; will wake on any interrupt.
// This particular API is not guaranteed to be maintained: please use sleepUntilInt() instead.
void sleepPwrSaveWithBODDisabled();

// Sleep indefinitely in as lower-power mode as possible until a specified watchdog time expires, or another interrupt.
// May be useful to call minimsePowerWithoutSleep() first, when not needing any modules left on.
#define sleepUntilInt() sleepPwrSaveWithBODDisabled()

// Sleep briefly in as lower-power mode as possible until the specified (watchdog) time expires.
//   * watchdogSleep is one of the WDTO_XX values from <avr/wdt.h>
// May be useful to call minimsePowerWithoutSleep() first, when not needing any modules left on.
void nap(int_fast8_t watchdogSleep);

// Sleep briefly in as lower-power mode as possible until the specified (watchdog) time expires, or another interrupt.
//   * watchdogSleep is one of the WDTO_XX values from <avr/wdt.h>
//   * allowPrematureWakeup if true then if woken before watchdog fires return false; default false
// Returns false if the watchdog timer did not go off.
// May be useful to call minimsePowerWithoutSleep() first, when not needing any modules left on.
bool nap(int_fast8_t watchdogSleep, bool allowPrematureWakeup);

#ifdef ENABLE_AVR_IDLE_MODE
// Idle the CPU for specified time but leave everything else running (eg UART), returning on any interrupt or the watchdog timer.
// Should reduce power consumption vs spinning the CPU >> 3x, though not nearly as much as nap().
// True iff watchdog timer expired; false if something else woke the CPU.
bool idleCPU(int_fast8_t watchdogSleep);
#endif

// Call this to do an I/O poll if needed; returns true if something useful happened.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// Limits actual poll rate to something like once every 32ms, unless force is true.
//   * force if true then force full poll on every call (ie do not internally rate-limit)
// NOTE: implementation may not be in power-management module.
bool pollIO(bool force = false);
// Nap productively polling I/O, etc, across the system while spending time in low-power mode if possible.
// Typically sleeps for about 30ms; tries to allow ealier wakeup if interrupt is received, etc.
static void inline nap30AndPoll() { nap(WDTO_30MS); pollIO(true); }
#ifdef ENABLE_AVR_IDLE_MODE
// Idle productively polling I/O, etc, across the system while spending time in low-power mode if possible.
// Typically sleeps for about 30ms; tries to allow ealier wakeup if interrupt is received, etc.
static void inline idle30AndPoll() { idleCPU(WDTO_30MS); pollIO(true); }
#endif

// Call this to productively burn tens to hundreds of CPU cycles, and poll I/O, eg in a busy-wait loop.
// This may churn PRNGs or gather entropy for example.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
void burnHundredsOfCyclesProductivelyAndPoll();


// Use WDT-based timer for xxxPause() routines.
// Very tiny low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define VERYTINY_PAUSE_MS 5
static void inline veryTinyPause() { sleepLowPowerMs(VERYTINY_PAUSE_MS); }
// Tiny low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define TINY_PAUSE_MS 15
static void inline tinyPause() { nap(WDTO_15MS); } // 15ms vs 18ms nominal for PICAXE V0.09 impl.
// Small low-power sleep.
#define SMALL_PAUSE_MS 30
static void inline smallPause() { nap(WDTO_30MS); }
// Medium low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define MEDIUM_PAUSE_MS 60
static void inline mediumPause() { nap(WDTO_60MS); } // 60ms vs 144ms nominal for PICAXE V0.09 impl.
// Big low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define BIG_PAUSE_MS 120
static void inline bigPause() { nap(WDTO_120MS); } // 120ms vs 288ms nominal for PICAXE V0.09 impl.


// Get fraction of the way through the basic cycle in range [0,255].
// This can be used for precision timing during the cycle,
// or to avoid overrunning a cycle with tasks of variable timing.
#define getSubCycleTime() ((uint8_t)TCNT2)
// Maximum value for getSubCycleTime(); full cycle length is this + 1.
// So ~4ms per count for a 1s cycle time, ~8ms per count for a 2s cycle time.
#define GSCT_MAX 255
// Basic cycle length in milliseconds; strictly positive.
#if defined(TWO_S_TICK_RTC_SUPPORT)
#define BASIC_CYCLE_MS 2000
#define SUB_CYCLE_TICKS_PER_S ((GSCT_MAX+1)/2) // Sub-cycle ticks per second.
#else
#define BASIC_CYCLE_MS 1000
#define SUB_CYCLE_TICKS_PER_S (GSCT_MAX+1) // Sub-cycle ticks per second.
#endif
// Approx (rounded down) milliseconds per tick of getSubCycleTime(); strictly positive.
#define SUBCYCLE_TICK_MS_RD (BASIC_CYCLE_MS / (GSCT_MAX+1))
// Approx (rounded to nearest) milliseconds per tick of getSubCycleTime(); strictly positive and no less than SUBCYCLE_TICK_MS_R
#define SUBCYCLE_TICK_MS_RN ((BASIC_CYCLE_MS + ((GSCT_MAX+1)/2)) / (GSCT_MAX+1))

// Returns (rounded-down) approx milliseconds until end of current basic cycle; non-negative.
// Upper limit is set by length of basic cycle, thus 1000 or 2000 typically.
#define msRemainingThisBasicCycle() (SUBCYCLE_TICK_MS_RD * (GSCT_MAX-getSubCycleTime()))

// Sleep in reasonably low-power mode until specified target subcycle time.
// Returns true if OK, false if specified time already passed or significantly missed (eg by more than one tick).
// May use a combination of techniques to hit the required time.
// Requesting a sleep until at or near the end of the cycle risks overrun and may be unwise.
// Using this to sleep less then 2 ticks may prove unreliable as the RTC rolls on underneath...
// This is NOT intended to be used to sleep over the end of a minor cycle.
bool sleepUntilSubCycleTime(uint8_t sleepUntil);


// Return some approximate/fast measure of CPU cycles elapsed.  Will not count when (eg) CPU/TIMER0 not running.
// Rather depends on Arduino/wiring setup for micros()/millis().
#ifndef DONT_USE_TIMER0
#if defined(TCNT0)
#define cycleCountCPU() ((uint8_t)TCNT0)
#elif defined(TCNT0L)
#define cycleCountCPU() ((uint8_t)TCNT0L)
#else
#error TIMER0 not defined
#endif
#else
#define cycleCountCPU() ((uint8_t)0) // Fixed result if TIMER0 is not used (for normal Arduino purposes).
#endif





// If ADC was disabled, power it up, do Serial.begin(), and return true.
// If already powered up then do nothing other than return false.
// If this returns true then a matching powerDownSerial() may be advisable.
bool powerUpADCIfDisabled(); 
// Power ADC down.
void powerDownADC();

// If serial (UART/USART0) was disabled, power it up, do Serial.begin(), and return true.
// If already powered up then do nothing other than return false.
// If this returns true then a matching powerDownSerial() may be advisable.
bool powerUpSerialIfDisabled();
// Flush any pending serial (UART/USART0) output and power it down.
void powerDownSerial();
#ifdef __AVR_ATmega328P__
// Returns true if hardware USART0 buffer in ATMmega328P is non-empty; may occasionally return a spurious false.
// There may still be a byte in the process of being transmitted when this is false.
// This should not interfere with HardwareSerial's handling.
#define serialTXInProgress() (!(UCSR0A & _BV(UDRE0)))
// Does a Serial.flush() attempting to do some useful work (ef I/O polling) while waiting for output to drain.
// Assumes hundreds of CPU cycles available for each character queued for TX.
// Does not change CPU clock speed or disable or mess with USART0, though may poll it.
void flushSerialProductive();
// Does a Serial.flush() idling for 30ms at a time while waiting for output to drain.
// Does not change CPU clock speed or disable or mess with USART0, though may poll it.
// Sleeps in IDLE mode for up to 15ms at a time (using watchdog) waking early on interrupt
// so the caller must be sure RX overrun (etc) will not be an issue.
// Switches to flushSerialProductive() behaviour
// if in danger of overrunning a minor cycle while idling.
void flushSerialSCTSensitive();
#else
#define flushSerialProductive() Serial.flush()
#define flushSerialSCTSensitive() Serial.flush()
#endif


// If TWI (I2C) was disabled, power it up, do Wire.begin(), and return true.
// If already powered up then do nothing other than return false.
// If this returns true then a matching powerDownTWI() may be advisable.
bool powerUpTWIIfDisabled();
// Power down TWI (I2C).
void powerDownTWI();

// If SPI was disabled, power it up, enable it as master and with a sensible clock speed, etc, and return true.
// If already powered up then do nothing other than return false.
// If this returns true then a matching powerDownSPI() may be advisable.
bool powerUpSPIIfDisabled();
// Power down SPI.
void powerDownSPI();


// Enable power to intermittent peripherals.
//   * waitUntilStable  wait long enough (and maybe test) for I/O power to become stable.
void power_intermittent_peripherals_enable(bool waitUntilStable = true);

// Disable/remove power to intermittent peripherals.
void power_intermittent_peripherals_disable();

// Read ADC/analogue input with reduced noise if possible, in range [0,1023].
// If ADC not powered up, then powers up ADC and turns off again afterwards.
uint16_t analogueNoiseReducedRead(int pinNumber, uint8_t mode);


// Capture a little system entropy.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or do I/O, or sleep.
void captureEntropy1();

// Capture a little entropy from clock jitter between CPU and WDT clocks; possibly one bit of entropy captured.
// Expensive in terms of CPU time and thus energy.
uint_fast8_t clockJitterWDT();

// Capture a little entropy from clock jitter between CPU and 32768Hz RTC clocks; possibly up to 2 bits of entropy captured.
// Expensive in terms of CPU time and thus energy.
uint_fast8_t clockJitterRTC();

// Combined clock jitter techniques to generate approximately 8 bits (the entire result byte) of entropy efficiently on demand.
// Expensive in terms of CPU time and thus energy, though possibly more efficient than basic clockJitterXXX() routines.
uint_fast8_t clockJitterEntropyByte();

#endif

