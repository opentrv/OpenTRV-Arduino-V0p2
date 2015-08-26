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


// Sleep for specified number of _delay_loop2() loops at minimum available CPU speed.
// Each loop takes 4 cycles at that minimum speed, but entry and exit overheads may take the equivalent of a loop or two.
// Note: inlining is prevented so as to avoid migrating anything into the section where the CPU is running slowly.
// Deprecated as may interact badly with interrupts if used naively (eg ISR code runs very slowly).
// This may only be safe to use with interrupts disabled.
void _sleepLowPowerLoopsMinCPUSpeed(uint16_t loops) __attribute__ ((noinline));

// Sleep/spin for approx specified strictly-positive number of milliseconds, in as low-power mode as possible.
// This may be achieved in part by dynamically slowing the CPU clock if possible.
// Macro to allow some constant folding at compile time where the sleep-time argument is constant.
// Should be good for values up to at least 1000, ie 1 second.
// Assumes MIN_CPU_HZ >> 4000.
// TODO: break out to non-inlined routine where arg is not constant (__builtin_constant_p).
// Deprecated as may interact badly with interrupts if used naively (eg ISR code runs very slowly).
static void inline _sleepLowPowerMs(const uint16_t ms) { _sleepLowPowerLoopsMinCPUSpeed((((MIN_CPU_HZ * (ms)) + 2000) / 4000) - ((MIN_CPU_HZ>=12000)?2:((MIN_CPU_HZ>=8000)?1:0))); }
// Sleep/spin for (typically a little less than) strictly-positive specified number of milliseconds, in as low-power mode as possible.
// This may be achieved in part by dynamically slowing the CPU clock if possible.
// Macro to allow some constant folding at compile time where the sleep-time argument is constant.
// Should be good for values up to at least 1000, ie 1 second.
// Uses formulation likely to be quicker than _sleepLowPowerMs() for non-constant argument values,
// and that results in a somewhat shorter sleep than _sleepLowPowerMs(ms).
// Assumes MIN_CPU_HZ >> 4000.
// TODO: break out to non-inlined routine where arg is not constant (__builtin_constant_p).
// Deprecated as may interact badly with interrupts if used naively (eg ISR code runs very slowly).
static void inline _sleepLowPowerLessThanMs(const uint16_t ms) { _sleepLowPowerLoopsMinCPUSpeed(((MIN_CPU_HZ/4000) * (ms)) - ((MIN_CPU_HZ>=12000)?2:((MIN_CPU_HZ>=8000)?1:0))); }

// Sleep/spin for approx specified strictly-positive number of milliseconds, in as low-power mode as possible.
// Nap() may be more efficient for intervals of longer than 15ms.
// Interrupts are blocked for about 1ms at a time.
// Should be good for the full range of values and should take no time where 0ms is specified.
static void inline sleepLowPowerMs(uint16_t ms) { while(ms-- > 0) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { _sleepLowPowerMs(1); } } }
// Sleep/spin for (typically a little less than) strictly-positive specified number of milliseconds, in as low-power mode as possible.
// Nap() may be more efficient for intervals of longer than 15ms.
// Interrupts are blocked for about 1ms at a time.
// Should be good for the full range of values and should take no time where 0ms is specified.
static void inline sleepLowPowerLessThanMs(uint16_t ms) { while(ms-- > 0) { ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { _sleepLowPowerLessThanMs(1); } } }

// Call from setup() to turn off unused modules, set up timers and interrupts, etc.
// I/O pin setting is not done here.
void powerSetup();



// Sensor for supply (eg battery) voltage in millivolts.
class SupplyVoltageMilliVolts : public OTV0P2BASE::Sensor<uint16_t>
  {
  private:
    // Internal bandgap (1.1V nominal, 1.0--1.2V) as fraction of Vcc [0,1023].
    uint16_t rawInv;
    // Last measured supply voltage (mV);
    uint16_t mV;
    // True if last-measured voltage was low.
    bool isLow;
 
  public:
    // Initialise to cautious values.
    SupplyVoltageMilliVolts() : mV(0), isLow(true) { }

    // Force a read/poll of the supply voltage and return the value sensed.
    // Expensive/slow.
    // NOT thread-safe or usable within ISRs (Interrupt Service Routines).
    virtual uint16_t read();

    // Return last value fetched by read(); undefined before first read()).
    // Fast.
    // NOT thread-safe nor usable within ISRs (Interrupt Service Routines).
    virtual uint16_t get() const { return(mV); }

    // Returns a suggested (JSON) tag/field/key name including units of get(); NULL means no recommended tag.
    // The lifetime of the pointed-to text must be at least that of the Sensor instance.
    virtual const char *tag() const { return("B|mV"); }

    // Get internal bandgap (1.1V nominal, 1.0--1.2V) as fraction of Vcc.
    uint16_t getRawInv() const { return(rawInv); }

    // Returns true if the supply voltage is low/marginal.
    // This depends on the AVR and other hardware components (eg sensors) in use.
    bool isSupplyVoltageLow() const { return(isLow); }

    // Returns true if the supply appears to be something like mains, that does not need monitoring.
    // This assumes that anything at/above 3V is mains or at least a long way from needing monitoring.
    bool isMains() const { return(!isLow && (mV >= 3000)); }
  };
// Singleton implementation/instance.
extern SupplyVoltageMilliVolts Supply_mV;



// Get approximate internal temperature in nominal C/16.
// Only accurate to +/- 10C uncalibrated.
// May set sleep mode to SLEEP_MODE_ADC, and disables sleep on exit.
int readInternalTemperatureC16();



// Selectively turn off all modules that need not run continuously so as to minimise power without sleeping.
// Suitable for start-up and for belt-and-braces use before main sleep on each cycle,
// to ensure that nothing power-hungry is accidentally left on.
// Any module that may need to run all the time should not be turned off here.
// May be called from panic(), so do not be too clever.
// Does NOT attempt to power down the radio, eg in case that needs to be left in RX mode.
// Does NOT attempt to adjust serial power state.
void minimisePowerWithoutSleep();

//// Sleep with BOD disabled in power-save mode; will wake on any interrupt.
//// This particular API is not guaranteed to be maintained: please use sleepUntilInt() instead.
//void sleepPwrSaveWithBODDisabled();
//
//// Sleep indefinitely in as lower-power mode as possible until a specified watchdog time expires, or another interrupt.
//// May be useful to call minimsePowerWithoutSleep() first, when not needing any modules left on.
//static inline void sleepUntilInt() { sleepPwrSaveWithBODDisabled(); }
//
//// Idle the CPU for specified time but leave everything else running (eg UART), returning on any interrupt or the watchdog timer.
////   * watchdogSleep is one of the WDTO_XX values from <avr/wdt.h>
//// Should reduce power consumption vs spinning the CPU more than 3x, though not nearly as much as nap().
//// True iff watchdog timer expired; false if something else woke the CPU.
//// Only use this if not disallowed for board type, eg with ENABLE_USE_OF_AVR_IDLE_MODE.
//bool idleCPU(int_fast8_t watchdogSleep);
//
//// Sleep briefly in as lower-power mode as possible until the specified (watchdog) time expires.
////   * watchdogSleep is one of the WDTO_XX values from <avr/wdt.h>
//// May be useful to call minimsePowerWithoutSleep() first, when not needing any modules left on.
//// NOTE: will stop clocks for UART, etc.
//void nap(int_fast8_t watchdogSleep);
//
//// Sleep briefly in as lower-power mode as possible until the specified (watchdog) time expires, or another interrupt.
////   * watchdogSleep is one of the WDTO_XX values from <avr/wdt.h>
////   * allowPrematureWakeup if true then if woken before watchdog fires return false; default false
//// Returns false if the watchdog timer did not go off, true if it did.
//// May be useful to call minimsePowerWithoutSleep() first, when not needing any modules left on.
//// NOTE: will stop clocks for UART, etc.
//bool nap(int_fast8_t watchdogSleep, bool allowPrematureWakeup);

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
// Idle productively polling I/O, etc, across the system while spending time in low-power mode if possible.
// Typically sleeps for nominally up to 30ms; tries to allow earlier wakeup if interrupt is received, etc.
// (Will often be prematurely woken by timer0 with ~16ms interval.)
// True iff watchdog timer expired; false if something else woke the CPU.
// Only use this if not disallowed for board type, eg with ENABLE_USE_OF_AVR_IDLE_MODE.
static bool inline idle15AndPoll() { const bool wd = ::OTV0P2BASE::idleCPU(WDTO_15MS, true); pollIO(!wd); return(wd); }

// Call this to productively burn tens to hundreds of CPU cycles, and poll I/O, eg in a busy-wait loop.
// This may churn PRNGs or gather entropy for example.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// May capture some entropy in secure and non-secure PRNGs.
void burnHundredsOfCyclesProductivelyAndPoll();


// Use WDT-based timer for xxxPause() routines.
// Very tiny low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define VERYTINY_PAUSE_MS 5
static void inline veryTinyPause() { sleepLowPowerMs(VERYTINY_PAUSE_MS); }
// Tiny low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define TINY_PAUSE_MS 15
static void inline tinyPause() { ::OTV0P2BASE::nap(WDTO_15MS); } // 15ms vs 18ms nominal for PICAXE V0.09 impl.
// Small low-power sleep.
#define SMALL_PAUSE_MS 30
static void inline smallPause() { ::OTV0P2BASE::nap(WDTO_30MS); }
// Medium low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
// Premature wakeups MAY be allowed to avoid blocking I/O polling for too long.
#define MEDIUM_PAUSE_MS 60
static void inline mediumPause() { ::OTV0P2BASE::nap(WDTO_60MS); } // 60ms vs 144ms nominal for PICAXE V0.09 impl.
// Big low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
// Premature wakeups MAY be allowed to avoid blocking I/O polling for too long.
#define BIG_PAUSE_MS 120
static void inline bigPause() { ::OTV0P2BASE::nap(WDTO_120MS); } // 120ms vs 288ms nominal for PICAXE V0.09 impl.

#if defined(WAKEUP_32768HZ_XTAL) || 1 // FIXME: avoid getSubCycleTime() where slow clock NOT available.
// Get fraction of the way through the basic cycle in range [0,255].
// This can be used for precision timing during the cycle,
// or to avoid overrunning a cycle with tasks of variable timing.
// Only valid if running the slow (32768Hz) clock.
#define getSubCycleTime() ((uint8_t)TCNT2)
// Approximation which is allowed to be zero if true value not available.
#define _getSubCycleTime() (getSubCycleTime())
#else
// Approximation which is allowed to be zero if true value not available.
#define _getSubCycleTime() (0)
#endif

// Maximum value for getSubCycleTime(); full cycle length is this + 1.
// So ~4ms per count for a 1s cycle time, ~8ms per count for a 2s cycle time.
#define GSCT_MAX 255
// Basic cycle length in milliseconds; strictly positive.
#if defined(V0P2BASE_TWO_S_TICK_RTC_SUPPORT)
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
// This does not power up the analogue comparator; this needs to be manually enabled if required.
// If this returns true then a matching powerDownADC() may be advisable.
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
// Does a Serial.flush() attempting to do some useful work (eg I/O polling) while waiting for output to drain.
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

// NOW SUPPLIED BY LIBRARY.
//// If SPI was disabled, power it up, enable it as master and with a sensible clock speed, etc, and return true.
//// If already powered up then do nothing other than return false.
//// If this returns true then a matching powerDownSPI() may be advisable.
//bool powerUpSPIIfDisabled();
//// Power down SPI.
//void powerDownSPI();


// Enable power to intermittent peripherals.
//   * waitUntilStable  wait long enough (and maybe test) for I/O power to become stable.
// Waiting for stable may only be necessary for those items hung from IO_POWER cap;
// items powered direct from IO_POWER_UP may need no such wait.
void power_intermittent_peripherals_enable(bool waitUntilStable = false);

// Disable/remove power to intermittent peripherals.
void power_intermittent_peripherals_disable();

// Read ADC/analogue input with reduced noise if possible, in range [0,1023].
//   * aiNumber is the analogue input number [0,7] for ATMega328P
//   * mode  is the analogue reference, eg DEFAULT (Vcc).
// May set sleep mode to SLEEP_MODE_ADC, and disable sleep on exit.
// Nominally equivalent to analogReference(mode); return(analogRead(pinNumber));
uint16_t analogueNoiseReducedRead(uint8_t aiNumber, uint8_t mode);

// Read from the specified analogue input vs the band-gap reference; true means AI > Vref.
//   * aiNumber is the analogue input number [0,7] for ATMega328P
//   * napToSettle  if true then take a minimal sleep/nap to allow voltage to settle
//       if input source relatively high impedance (>>10k)
// Assumes that the band-gap reference is already running,
// eg from being used for BOD; if not, it must be given time to start up.
// For input settle time explanation please see for example:
//   * http://electronics.stackexchange.com/questions/67171/input-impedance-of-arduino-uno-analog-pins
bool analogueVsBandgapRead(uint8_t aiNumber, bool napToSettle);

// Attempt to capture maybe one bit of noise/entropy with an ADC read, possibly more likely in the lsbits if at all.
// If requested (and needed) powers up extra I/O during the reads.
//   powerUpIO if true then power up I/O (and power down after if so)
uint8_t noisyADCRead(bool powerUpIO = true);

// Capture a little system entropy.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or do I/O, or sleep.
// Should inject some noise into secure (TBD) and non-secure (RNG8) PRNGs, or at least churn them.
void captureEntropy1();

// Capture a little entropy from clock jitter between CPU and WDT clocks; possibly one bit of entropy captured.
// Expensive in terms of CPU time and thus energy.
uint_fast8_t clockJitterWDT();

// Capture a little entropy from clock jitter between CPU and 32768Hz RTC clocks; possibly up to 2 bits of entropy captured.
// Expensive in terms of CPU time and thus energy.
#ifdef WAKEUP_32768HZ_XTAL
uint_fast8_t clockJitterRTC();
#else
#define NO_clockJitterRTC
#endif

// Combined clock jitter techniques to generate approximately 8 bits (the entire result byte) of entropy efficiently on demand.
// Expensive in terms of CPU time and thus energy, though possibly more efficient than basic clockJitterXXX() routines.
#ifdef WAKEUP_32768HZ_XTAL
uint_fast8_t clockJitterEntropyByte();
#else
#define NO_clockJitterEntropyByte
#endif

#endif

