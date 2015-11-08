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


// Sleep in reasonably low-power mode until specified target subcycle time.
// Returns true if OK, false if specified time already passed or significantly missed (eg by more than one tick).
// May use a combination of techniques to hit the required time.
// Requesting a sleep until at or near the end of the cycle risks overrun and may be unwise.
// Using this to sleep less then 2 ticks may prove unreliable as the RTC rolls on underneath...
// This is NOT intended to be used to sleep over the end of a minor cycle.
bool sleepUntilSubCycleTime(uint8_t sleepUntil);


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

#endif

