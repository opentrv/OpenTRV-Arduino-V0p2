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


#include <assert.h>
#include <avr/wdt.h>
#include <util/crc16.h>

#include <Wire.h>
#include <OTV0p2Base.h> // Underlying hardware support definitions.

#include "Power_Management.h"

#include "Radio.h"
#include "Serial_IO.h"


#ifdef WAKEUP_32768HZ_XTAL
static void timer2XtalIntSetup()
 {
  // Set up TIMER2 to wake CPU out of sleep regularly using external 32768Hz crystal.
  // See http://www.atmel.com/Images/doc2505.pdf
  TCCR2A = 0x00;

#if defined(HALF_SECOND_RTC_SUPPORT)
  TCCR2B = (1<<CS22); // Set CLK/64 for overflow interrupt every 0.5s.
#elif defined(V0P2BASE_TWO_S_TICK_RTC_SUPPORT)
  TCCR2B = (1<<CS22)|(1<<CS21); // Set CLK/128 for overflow interrupt every 2s.
#else
  TCCR2B = (1<<CS22)|(1<<CS20); // Set CLK/128 for overflow interrupt every 1s.
#endif
  //TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20); // Set CLK/1024 for overflow interrupt every 8s (slowest possible).

  ASSR = (1<<AS2); // Enable asynchronous operation.
  TIMSK2 = (1<<TOIE2); // Enable the timer 2 interrupt.
  }
#endif

// Call from setup() to turn off unused modules, set up timers and interrupts, etc.
// I/O pin setting is not done here.
void powerSetup()
  {
#ifdef DEBUG
  assert(OTV0P2BASE::DEFAULT_CPU_PRESCALE == clock_prescale_get()); // Verify that CPU prescaling is as expected.
#endif

  // Do normal gentle switch off, including analogue module/control in correct order.
  minimisePowerWithoutSleep();

  // Brutally force off all modules, then re-enable explicitly below any still needed.
  power_all_disable(); 

#if !defined(DONT_USE_TIMER0)
  power_timer0_enable(); // Turning timer 0 off messes up some standard Arduino support such as delay() and millis().
#endif
  
#if defined(WAKEUP_32768HZ_XTAL)
  power_timer2_enable();
  timer2XtalIntSetup();
#endif
  }


// Selectively turn off all modules that need not run continuously so as to minimise power without sleeping.
// Suitable for start-up and for belt-and-braces use before main sleep on each cycle,
// to ensure that nothing power-hungry is accidentally left on.
// Any module that may need to run all the time should not be turned off here.
// May be called from panic(), so do not be too clever.
// Does NOT attempt to power down the radio, eg in case that needs to be left in RX mode.
// Does NOT attempt to power down the hardware serial/UART.
void minimisePowerWithoutSleep()
  {
  // Disable the watchdog timer.
  wdt_disable();

  // Ensure that external peripherals are powered down.
  OTV0P2BASE::power_intermittent_peripherals_disable();

  // Turn off analogue stuff that eats power.
  ADCSRA = 0; // Do before power_[adc|all]_disable() to avoid freezing the ADC in an active state!
  ACSR = (1<<ACD); // Disable the analog comparator.
  DIDR0 = 0x3F; // Disable digital input buffers on all ADC0-ADC5 pins.
    // More subtle approach possible...
    //  // Turn off the digital input buffers on analogue inputs in use as such
    //  // so as to reduce power consumption with mid-supply input voltages.
    //  DIDR0 = 0
    //#if defined(TEMP_POT_AIN)
    //    | (1 << TEMP_POT_AIN)
    //#endif
    //#if defined(LDR_SENSOR_AIN)
    //    | (1 << LDR_SENSOR_AIN)
    //#endif
    //    ;
  DIDR1 = (1<<AIN1D)|(1<<AIN0D); // Disable digital input buffer on AIN1/0.
  power_adc_disable();

  // Ensure that SPI is powered down.
  OTV0P2BASE::powerDownSPI();

  // Ensure that TWI is powered down.
  OTV0P2BASE::powerDownTWI();

  // TIMERS
  // See: http://letsmakerobots.com/node/28278
  //   * For Arduino timer0 is used for the timer functions such as delay(), millis() and micros().
  //   * Servo Library uses timer1 (on UNO).
  //   * tone() function uses at least timer2.
  // Note that timer 0 in normal use sometimes seems to eat a lot of power.

#if defined(DONT_USE_TIMER0)
  power_timer0_disable();
#endif

  power_timer1_disable();

#ifndef WAKEUP_32768HZ_XTAL
  power_timer2_disable();
#endif
  }



// Call this to productively burn tens to hundreds of CPU cycles, and poll I/O, eg in a busy-wait loop.
// This may churn PRNGs or gather entropy for example.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// May capture some entropy in secure and non-secure PRNGs.
void burnHundredsOfCyclesProductivelyAndPoll()
  {
  if(pollIO()) { OTV0P2BASE::seedRNG8(OTV0P2BASE::getCPUCycleCount(), 37 /* _watchdogFired */, OTV0P2BASE::_getSubCycleTime()); }
  else { OTV0P2BASE::captureEntropy1(); }
  }

// Sleep in reasonably low-power mode until specified target subcycle time.
// Returns true if OK, false if specified time already passed or significantly missed (eg by more than one tick).
// May use a combination of techniques to hit the required time.
// Requesting a sleep until at or near the end of the cycle risks overrun and may be unwise.
// Using this to sleep less then 2 ticks may prove unreliable as the RTC rolls on underneath...
// This is NOT intended to be used to sleep over the end of a minor cycle.
// May poll I/O.
bool sleepUntilSubCycleTime(const uint8_t sleepUntil)
  {
  for( ; ; )
    {
    const uint8_t now = OTV0P2BASE::getSubCycleTime();
    if(now == sleepUntil) { return(true); } // Done it!
    if(now > sleepUntil) { return(false); } // Too late...

    // Compute time left to sleep.
    // It is easy to sleep a bit more later if necessary, but oversleeping is bad.
    const uint8_t ticksLeft = sleepUntil - now;
    // Deal with shortest sleep specially to avoid missing target from overheads...
    if(1 == ticksLeft)
      {
      // Take a very short sleep, much less than half a tick,
      // eg as may be some way into this tick already.
      //burnHundredsOfCyclesProductively();
      OTV0P2BASE::sleepLowPowerLessThanMs(1);
      continue;
      }

    // Compute remaining time in milliseconds, rounded down...
    const uint16_t msLeft = ((uint16_t)OTV0P2BASE::SUBCYCLE_TICK_MS_RD) * ticksLeft;

    // If comfortably in the area of nap()s then use one of them for improved energy savings.
    // Allow for nap() to overrun a little as its timing can vary with temperature and supply voltage,
    // and the bulk of energy savings should still be available without pushing the timing to the wire.
    // Note that during nap() timer0 should be stopped and thus not cause premature wakeup (from overflow interrupt).
    if(msLeft >= 20)
      {
      if(msLeft >= 80)
        {
        if(msLeft >= 333)
          {
          ::OTV0P2BASE::nap(WDTO_250MS); // Nominal 250ms sleep.
          continue;
          }
        ::OTV0P2BASE::nap(WDTO_60MS); // Nominal 60ms sleep.
        continue;
        }
      ::OTV0P2BASE::nap(WDTO_15MS); // Nominal 15ms sleep.
      continue;
      }

    // Use low-power CPU sleep for residual time, but being very careful not to over-sleep.
    // Aim to sleep somewhat under residual time, eg to allow for overheads, interrupts, and other slippages.
    // Assumed to be > 1 else would have been special-cased above.
    // Assumed to be << 1s else a nap() would have been used above.
#ifdef DEBUG
    if((msLeft < 2) || (msLeft > 1000)) { panic(); }
#endif
    OTV0P2BASE::sleepLowPowerLessThanMs(msLeft - 1);
    }
  }


// Singleton implementation/instance.
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;


/*
 Power log.
 Basic CPU 1MHz (8MHz RC clock prescaled) + 32768Hz clock running timer 2 async.
 Current draw measured across 100R in Vcc supply on 200mV scale (0.1mV, ie ulp, = 1uA).
 Initially using a 1Hz wake-up from timer 2; later at 0.5Hz.
 USB disconnected for all power measurements unless otherwise stated.
 2013/04/21 11:50 ~5uA@5V in 'frost' mode (no LED flash). USB disconnected (else ~55uA). Using sleepLowPowerLoopsMinCPUSpeed(), ie min CPU speed in wait.
 2013/04/21 15:37 ~4uA@5V,1uA@2.8V in 'frost' mode (no LED flash) using WDT xxxPause(). USB disconnected (else ~55uA).  Possibly less distinct flash lengths.
 2013/04/21 15:37 ~1.5uA@2.6V with readAmbientLight() being called once per second.
 2013/04/25 09:44 Takes ~24--36ms leaving loop() and re-entering after roll to new minor cycle from timer 2 interrupt including loop()-exit Arduino background activity.
 2013/04/25 10:49 ~1uA@2.6V (no readAmbientLight(), no LED flash) with timer 2 wakeup reduced to 0.5Hz.
 2013/04/25 12:48 ~4uA@2.6V with minimal serial status report every 2 seconds (and USB disconnected).
 2013/04/25 14:10 ~1uA@2.6V with minimal serial status report every 60 seconds or on significant change (and USB disconnected).
 2013/04/25 15:24 ~1uA@2.6V having removed #define DONT_USE_TIMER0 so may be benign to leave available for Arduino uses.
 2013/04/25 17:00 ~6.5uA@2.6V adding TMP102 sensor (on SparkFun breakout board) with only Vcc/Gnd connected (default 4Hz continuous conversion).
 2013/04/25 18:18 ~7uA@2.6V with TMP102 SCL/SDA also wired and reading pulled once per 60s (default 4Hz continuous conversion).
 2013/04/25 21:03 ~3uA@2.6V with TMP102 in one-shot mode: TMP102 draws ~2x the current that the ATmega328P does!
 2013/04/26 20:29 ~2.7uA@2.6V 1k resistor in supply line suggests that idle current is 2.7uA; ~1.3uA with TMP102 removed.
 2013/04/27 19:38 ~2.7uA@2.6V still, after all EEPROM / RTC persistence work; surges to very roughly 60uA, once per minute.
 2013/04/30 12:25 ~2.6uA@2.6V multiple small efficiency tweaks and spread out per-minute processing and do less of it in frost mode.
 2013/05/04 17:08 ~1.4mA@2.5V (>1milliAmp!) with RFM22 connected and idle; back to 100R in supply line else won't start up with RFM22 connected.
 2013/05/04 18:47 ~16uA@2.6V with RFM22 powered down with RFM22ModeStandbyAndClearState() including clearing interrupts.
 2013/05/05 10:47 ~3uA@2.6V with all SPI bus pins prevented from floating when idle.  (Measured <3.3uA idle with 1k supply resistor.)
 2013/05/05 12:47 ~3.2uA@2.6V (1k supply resistor) with TWI clock speed pushed up to 62.5kHz, so less time with CPU running.
 2013/05/16 13:53 ~180uA@2.6V (1k supply resistor) with CLI waiting for input ~900ms every 2s (3.3uA when not, and USB disconnected).
 2013/05/21 11:53 ~6.4uA@2.6V (1k supply resistor) with main loop doing nothing but sleepUntilSubCycleTime() for 50% of the minor cycle.
 2013/05/22 12:51 ~1mA@2.6V (100R supply resistor) with IGNORE_FHT_SYNC and in frost mode, ie one FHT8V TX via RFM22 per second.
 2013/05/22 19:16 ~200uA@2.6V (100R supply resistor) in BOOST controlling FHT8V, post sync (& double TXes), LED flashing, USB connected.
 2013/05/22 19:17 ~3uA@2.6V min calculated ~23uA mean in FROST w/ FHT8V, post sync, single TXes, LED off, USB disconn, calced ~50uA mean in WARM mode w/ valve open.
 2013/06/09 16:54 ~40uA@2.6V (100R supply resistor) polling for UART input (CLI active), FHT8V not transmitting.
 2013/06/09 18:21 ~35uA@2.6V (100R supply resistor) polling for UART input (CLI active), FHT8V not transmitting, spending more time in IDLE.
 2014/12/10 18:01 ~4uA@2.5V (100R supply resistor) running current OpenTRV main loop; rises to ~150uA flashing LED in 'FROST' display.
 */


