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

#include <assert.h>
#include <avr/wdt.h>
#include <util/crc16.h>

#include <Wire.h>

#include "Power_Management.h"
#include "RFM22_Radio.h"
#include "RTC_Support.h"
#include "Serial_Debug.h"


#ifdef WAKEUP_32768HZ_XTAL
static void timer2XtalIntSetup()
 {
  // Set up TIMER2 to wake CPU out of sleep regularly using external 32768Hz crystal.
  // See http://www.atmel.com/Images/doc2505.pdf
  TCCR2A = 0x00;

#if defined(HALF_SECOND_RTC_SUPPORT)
  TCCR2B = (1<<CS22); // Set CLK/64 for overflow interrupt every 0.5s.
#elif defined(TWO_S_TICK_RTC_SUPPORT)
  TCCR2B = (1<<CS22)|(1<<CS21); // Set CLK/128 for overflow interrupt every 2s.
#else
  TCCR2B = (1<<CS22)|(1<<CS20); // Set CLK/128 for overflow interrupt every 1s.
#endif
  //TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20); // Set CLK/1024 for overflow interrupt every 8s (slowest possible).

  ASSR = (1<<AS2); // Enable asynchronous operation.
  TIMSK2 = (1<<TOIE2); // Enable the timer 2 interrupt.
  }
#endif


// Selectively turn off all modules that need not run continuously so as to minimise power without sleeping.
// Suitable for start-up and for belt-and-braces use before main sleep on each cycle,
// to ensure that nothing is accidentally left on.
// Any module that may need to run all the time should not be turned off here.
// May be called from panic(), so do not be too clever.
void minimsePowerWithoutSleep()
  {
  // Disable the watchdog timer.
  wdt_disable();
  
  // Turn off external peripherals.
  power_intermittent_peripherals_disable();

  // Turn off analogue stuff that eats power.
  ADCSRA = 0; // Do before power_[adc|all]_disable() to avoid freezing the ADC in an active state!
  ACSR = (1<<ACD); // Disable the analog comparator.
  DIDR0 = 0x3F; // Disable digital input buffers on all ADC0-ADC5 pins.
  DIDR1 = (1<<AIN1D)|(1<<AIN0D); // Disable digital input buffer on AIN1/0.
  power_adc_disable();

  // Disable radio during major sleep.
#ifdef USE_MODULE_RFM22RADIOSIMPLE
  // Put the radio in low-power standby mode.
  //RFM22ModeStandbyAndClearState();
#endif
  // Power down SPI during major sleep.
  powerDownSPI();

  // TIMERS
  // See: http://letsmakerobots.com/node/28278
  //   * For Arduino timer0 is used for the timer functions such as delay(), millis() and micros().
  //   * Servo Library uses timer1 (on UNO).
  //   * tone() function uses at least timer2.
  // Note that timer 0 in normal use sometimes seems to eat a lot of power.

#ifdef DONT_USE_TIMER0
  power_timer0_disable();
#endif

  power_timer1_disable();

#ifndef WAKEUP_32768HZ_XTAL
  power_timer2_disable();
#endif
  }


// Call from setup() to turn off unused modules, set up timers and interrupts, etc.
// I/O pin setting is not done here.
void powerSetup()
  {
#ifdef DEBUG
  assert(DEFAULT_CPU_PRESCALE == clock_prescale_get()); // Verify that CPU prescaling is as expected.
#endif

  // Do normal gentle switch off, including analogue module/control in correct order.
  minimsePowerWithoutSleep();

  // Brutally force off all modules, then re-enable explicitly below any still needed.
  power_all_disable(); 

#ifndef DONT_USE_TIMER0
  power_timer0_enable(); // Turning timer 0 off messes up some standard Arduino support such as delay() and millis().
#endif
  
#if defined(WAKEUP_32768HZ_XTAL)
  power_timer2_enable();
  timer2XtalIntSetup();
#endif
  }

#ifdef WAKEUP_32768HZ_XTAL
ISR(TIMER2_OVF_vect)
  {
  // Maintain RTC.
  // As long as this is very efficient the CPU can be left running slow.
#if defined(TWO_S_TICK_RTC_SUPPORT)
  tickDoubleSecondISR();
#else
  tickSecondISR();
#endif
  }
#endif


// Set true when the watchdog ISR is invoked, ie the watchdog timer has gone off.
// Cleared by the watchdog sleep routine.
volatile bool watchdogFired;

// Catch watchdog timer interrupt to automatically clear WDIE and WDIF.
// This allows use of watchdog for low-power timed sleep.
ISR(WDT_vect)
  {
  // WDIE and WDIF are cleared in hardware upon entering this ISR.
  wdt_disable();
  watchdogFired = true;
  }




//#ifndef DEFAULT_CPU_PRESCALE
//// Default prescale value at start-up, fetched once.  Maybe could compute from prescaler fuse bit instead.
//const clock_div_t DEFAULT_CPU_PRESCALE  = clock_prescale_get();
//#endif

// Sleep for specified number of _delay_loop2() loops at minimum available CPU speed.
// Each loop takes 4 cycles at that minimum speed, but entry and exit overheads may take the equivalent of a loop or two.
// Note: inlining is prevented so as to avoid migrating anything into the section where the CPU is running slowly.
//
// Note: may be dubious to run CPU clock less than 4x 32768Hz crystal speed,
// eg at 31250Hz for 8MHz RC clock and max prescale.
// Don't access timer 2 regs at low CPU speed, eg in ISRs.
__attribute__ ((noinline)) void sleepLowPowerLoopsMinCPUSpeed(uint16_t loops)
  {
  const clock_div_t prescale = clock_prescale_get(); // Capture current prescale value.
  clock_prescale_set(MAX_CPU_PRESCALE); // Reduce clock speed (increase prescale) as far as possible.
  _delay_loop_2(loops); // Burn cycles...
  clock_prescale_set(prescale); // Restore clock prescale.
  }

// Define macro to disable BOD during sleep here if not included in Arduino AVR toolset...
// This is only for "pico-power" variants, eg the "P" in ATmega328P.
#ifndef sleep_bod_disable
#define sleep_bod_disable() \
do { \
  uint8_t tempreg; \
  __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
                       "ori %[tempreg], %[bods_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" "\n\t" \
                       "andi %[tempreg], %[not_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" \
                       : [tempreg] "=&d" (tempreg) \
                       : [mcucr] "I" _SFR_IO_ADDR(MCUCR), \
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE))); \
} while (0)
#endif

// Sleep with BOD disabled in power-save mode; will wake on any interrupt.
void sleepPwrSaveWithBODDisabled()
  {
  set_sleep_mode(SLEEP_MODE_PWR_SAVE); // Stop all but timer 2 and watchdog when sleeping.
  cli();
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  sleep_disable();
  sei();
  }

// Sleep briefly in as lower-power mode as possible until the specified (watchdog) time expires, or another interrupt.
//   * watchdogSleep is one of the WDTO_XX values from <avr/wdt.h>
// May be useful to call minimsePowerWithoutSleep() first, when not needing any modules left on.
void nap(int_fast8_t watchdogSleep)
  {
  // Watchdog should (already) be disabled on entry.
  watchdogFired = false;

  wdt_enable(watchdogSleep);
  WDTCSR |= (1 << WDIE);

  // Keep sleeping until watchdog actually fires.
  for( ; ; )
    {
    sleepPwrSaveWithBODDisabled();
    if(watchdogFired) { return; } // All done!
    }
 }

#if 0
// Sleep briefly in as lower-power mode as possible until the specified (watchdog) time expires, or another interrupt.
//   * watchdogSleep is one of the WDTO_XX values from <avr/wdt.h>
//   * allowPrematureWakeup if true then if woken before watchdog fires return false; default false
// Returns false if the watchdog timer did not go off.
// May be useful to call minimsePowerWithoutSleep() first, when not needing any modules left on.
bool nap(int_fast8_t watchdogSleep, bool allowPrematureWakeup)
  {
  // Watchdog should (already) be disabled on entry.
  watchdogFired = false;

  wdt_enable(watchdogSleep);
  WDTCSR |= (1 << WDIE);

  // Keep sleeping until watchdog actually fires (unless premature return is permitted).
  for( ; ; )
    {
    sleepPwrSaveWithBODDisabled();
    if(watchdogFired) { return(true); } // All done!
    if(allowPrematureWakeup)
      {
        wdt_disable(); // Avoid spurious wakeup later.
        return(false);
      }
    }
  }
#endif

// Sleep in reasonably low-power mode until specified target subcycle time.
// Returns true if OK, false if specified time already passed or significantly missed (eg by more than one tick).
// May use a combination of techniques to hit the required time.
// Requesting a sleep until at or near the end of the cycle risks overrun and may be unwise.
// Using this to sleep less then 2 ticks may prove unreliable as the RTC rolls on underneath...
// This is NOT intended to be used to sleep over the end of a minor cycle.
bool sleepUntilSubCycleTime(const uint8_t sleepUntil)
  {
  for( ; ; )
    {
    const uint8_t now = getSubCycleTime();
    if(now == sleepUntil) { return(true); } // Done it!
    if(now > sleepUntil) { return(false); } // Too late...

    // Compute time left to sleep.
    // It is easy to sleep a bit more later if necessary, but oversleeping is bad.
    const uint8_t ticksLeft = sleepUntil - now;
    // Deal with shortest sleep specially to avoid missing target from overheads...
    if(1 == ticksLeft)
      {
      // Take a very short sleep, less than half a tick,
      // eg as may be some way into this tick already.
      sleepLowPowerLessThanMs(max(SUBCYCLE_TICK_MS_RD / 2, 1)); // Assumed to be a constant expression.
      continue;
      }

    // Compute remaining time in milliseconds, rounded down...
    const uint16_t msLeft = ((uint16_t)SUBCYCLE_TICK_MS_RD) * ticksLeft;

    // If comfortably in the area of nap()s then use one of them for improved energy savings.
    // Allow for nap() to overrun a little as its timing can vary with temperature and supply voltage,
    // and the bulk of energy savings should still be available without pushing their timing to the wire.
    if(msLeft >= 20)
      {
      if(msLeft >= 80)
        {
        if(msLeft >= 333)
          {
          nap(WDTO_250MS); // Nominal 250ms sleep.
          continue;
          }
        nap(WDTO_60MS); // Nominal 60ms sleep.
        continue;
        }
      nap(WDTO_15MS); // Nominal 15ms sleep.
      continue;
      }

    // Use low-power CPU sleep for residual time, but being very careful not to oversleep.
    // Aim to sleep somewhat under residual time, eg to allow for overheads, interrupts, and other slippages.
    // Assumed to be > 1 else would have been special-cased above.
    // Assumed to be << 1s else a nap() would have been used above.
#ifdef DEBUG
    if((msLeft < 2) || (msLeft > 1000)) { panic(); }
#endif
    sleepLowPowerLessThanMs(msLeft - 1);
    }
  }


// Enable power to intermittent peripherals.
//   * waitUntilStable  wait long enough (and maybe test) for I/O power to become stable.
// Switches the digital line to high then output (to avoid ever *discharging* the output cap).
// Note that with 100nF cap, and 330R (or lower) resistor from the output pin,
// then 1ms delay should be plenty for the voltage on the cap to settle.
void power_intermittent_peripherals_enable(bool waitUntilStable)
  {
  digitalWrite(IO_POWER_UP, HIGH);
  pinMode(IO_POWER_UP, OUTPUT);
  // If requested, wait long enough that I/O peripheral power should be stable.
  // Wait in a relatively low-power way...
  if(waitUntilStable) { sleepLowPowerMs(1); }
  }

// Disable/remove power to intermittent peripherals.
// Switches the digital line to input with no pull-up(ie high-Z).
// There should be some sort of load to stop this floating.
void power_intermittent_peripherals_disable()
  {
  pinMode(IO_POWER_UP, INPUT);
  }


// Allow wake from (lower-power) sleep while ADC is running.
static volatile bool ADC_complete;
ISR(ADC_vect) { ADC_complete = true; }

// Read ADC/analogue input with reduced noise if possible, in range [0,1023].
//   * admux  is the value to set ADMUX to
//   * samples  number of samples to take (if one, nap() before); strictly positive
// May set sleep mode to SLEEP_MODE_ADC, and disables sleep on exit.
static uint16_t _analogueNoiseReducedReadM(uint8_t admux, int samples = 3)
  {
  const bool neededEnable = powerUpADCIfDisabled();
  ADMUX = admux;
  if(samples < 2) { nap(WDTO_15MS); } // Allow plenty of time for things to settle if not taking multiple samples.
  set_sleep_mode(SLEEP_MODE_ADC);
  ADCSRB = 0; // Enable free-running mode.
  bitWrite(ADCSRA, ADATE, (samples>1)); // Enable  ADC auto-trigger iff wanting multiple samples.
  bitSet(ADCSRA, ADIE); // Turn on ADC interrupt.
  bitSet(ADCSRA, ADSC); // Start conversion(s).
  // Usually take several readings to improve accuracy.  Discard all but the last...
  while(--samples >= 0)
      {
      ADC_complete = false;
      while(!ADC_complete) { sleep_mode(); }
      }
  bitClear(ADCSRA, ADIE); // Turn off ADC interrupt.
  bitClear(ADCSRA, ADATE); // Turn off ADC auto-trigger.
  sleep_disable();
  const uint8_t l = ADCL; // Capture the low byte and latch the high byte.
  const uint8_t h = ADCH; // Capture the high byte.
  if(neededEnable) { powerDownADC(); }
  return((h << 8) | l);
  }

// Get power supply voltage in mV; strictly positive.
// Only accurate to +/- 10%.
// May set sleep mode to SLEEP_MODE_ADC, and disables sleep on exit.
uint16_t powerSupplyVoltage()
  {
  // Measure internal bandgap (1.1V nominal, 1.0--1.2V) as fraction of Vcc.
  const uint16_t raw = _analogueNoiseReducedReadM(_BV(REFS0) | 14);
  // If Vcc was 1.1V ADC would give 1023.
  // If Vcc was 2.2V ADC would give 511.
  const uint16_t result = ((1023U<<6) / raw) * (1100U>>6);
  return(result);
  }

// Get approximate internal temperature in (rounded) C.
// Only accurate to +/- 10C uncalibrated.  (DHD20130429: much much worse in my testing!)
// May set sleep mode to SLEEP_MODE_ADC, and disables sleep on exit.
int internalTemperatureC()
  {
  // Measure internal temperature sensor against internal voltage source.
  // Response is ~1mv/C with 0C at ~289mV according to the data sheet.
  const int raw = _analogueNoiseReducedReadM(_BV(REFS1) | _BV(REFS0) | _BV(MUX3), 1);
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINT_FLASHSTRING("Int temp raw: ");
  DEBUG_SERIAL_PRINT(raw);
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("");
#endif
  //const int degC = (raw + (raw>>4)) - 289; // Adjust to mV slope (and thus C) given ref at 1.1V and FSD of 1023.
  //const int degC = (raw - 328) ; // Crude fast adjustment for one sensor at ~20C (DHD20130429).
  const int degC = ((raw - 328) * 210 + 128) >> 8; // Slightly less crude adjustment, see http://playground.arduino.cc//Main/InternalTemperatureSensor
  return(degC);
  }

// Read ADC/analogue input with reduced noise if possible, in range [0,1023].
//   * mode  is the analogue reference, eg DEFAULT (Vcc).
// May set sleep mode to SLEEP_MODE_ADC, and disables sleep on exit.
// Nominally equivalent to analogReference(mode); return(analogRead(pinNumber));
int analogueNoiseReducedRead(int pinNumber, uint8_t mode)
  { return(_analogueNoiseReducedReadM((mode << 6) | (pinNumber & 7))); }


// If ADC was disabled, power it up, do Serial.begin(), and return true.
// If already powered up then do nothing other than return false.
// If this returns true then a matching powerDownSerial() may be advisable.
bool powerUpADCIfDisabled()
  {
  if(!(PRR & _BV(PRADC))) { return(false); }
  PRR &= ~_BV(PRADC); // Enable the ADC.
  ADCSRA |= _BV(ADEN);
  return(true);
  }
    
// Power ADC down.
void powerDownADC()
  {
  ADCSRA &= ~_BV(ADEN); // Do before power_[adc|all]_disable() to avoid freezing the ADC in an active state!
  PRR |= _BV(PRADC); // Disable the ADC.
  }


// If serial was disabled, power it up, do Serial.begin(), and return true.
// If already powered up then do nothing other than return false.
// If this returns true then a matching powerDownSerial() may be advisable.
bool powerUpSerialIfDisabled()
  {
  if(!(PRR & _BV(PRUSART0))) { return(false); }
  PRR &= ~_BV(PRUSART0); // Enable the UART.
  Serial.begin(BAUD); // Set it going.
  return(true);
  }

#if 0
// Flush any pending UART TX bytes in the hardware if UART is enabled, eg useful after Serial.flush() and before sleep.
void flushSerialHW()
  {
  if(PRR & _BV(PRUSART0)) { return(); } // UART not running, so nothing to do.
#if 0
  // Try to ensure that the possible two bytes (each 1 start + 8 data + 1 stop bit) still in h/w buffers can get out intact...
  // ~5ms at 4800 baud should suffice...
  sleepLowPowerMs(1 + ((2 * (1+8+1) * 1000) / BAUD)); // Changing clock speed may mess with UART speed too?
#else
  // Snippet c/o http://www.gammon.com.au/forum/?id=11428
  // TODO: try to sleep in a low-ish power mode while waiting for data to clear.
  while(!(UCSR0A & _BV(UDRE0)))  // Wait for empty transmit buffer.
     { UCSR0A |= _BV(TXC0); }  // Mark transmission not complete.
  while(!(UCSR0A & _BV(TXC0))) { } // Wait for the transmission to complete.
#endif
  return;
  }
#endif

// Flush any pending serial output and power it down.
void powerDownSerial()
  {
  // Flush serial output and shut down to minimal power state including minimising leakage to serial lines!
  Serial.flush();
  //flushSerialHW();
  Serial.end();
  pinMode(PIN_SERIAL_RX, INPUT_PULLUP);
  pinMode(PIN_SERIAL_TX, INPUT_PULLUP);
  PRR |= _BV(PRUSART0); // Disable the UART module.
  }

// If TWI (I2C) was disabled, power it up, do Wire.begin(), and return true.
// If already powered up then do nothing other than return false.
// If this returns true then a matching powerDownRWI() may be advisable.
bool powerUpTWIIfDisabled()
  {
  if(!(PRR & _BV(PRTWI))) { return(false); }

  PRR &= ~_BV(PRTWI); // Enable TWI power.
  TWCR |= _BV(TWEN); // Enable TWI.
  Wire.begin(); // Set it going.
  // TODO: reset TWBR and prescaler for our low CPU frequency     (TWBR = ((F_CPU / TWI_FREQ) - 16) / 2 gives -3!)
#if F_CPU <= 1000000
  TWBR = 0; // Implies SCL freq of F_CPU / (16 + 2 * TBWR * PRESC) = 62.5kHz @ F_CPU==1MHz and PRESC==1 (from Wire/TWI code).
#endif
  return(true);
  }

// Power down TWI (I2C).
void powerDownTWI()
  {
  TWCR &= ~_BV(TWEN); // Disable TWI.
  PRR |= _BV(PRTWI); // Disable TWI power.

  // De-activate internal pullups for TWI especially if powering down all TWI devices.
  //digitalWrite(SDA, 0);
  //digitalWrite(SCL, 0);

  // Convert to hi-Z inputs.
  //pinMode(SDA, INPUT);
  //pinMode(SCL, INPUT);
  }

// If SPI was disabled, power it up, enable it as master and with a sensible clock speed, etc, and return true.
// If already powered up then do nothing other than return false.
// If this returns true then a matching powerDownSPI() may be advisable.
bool powerUpSPIIfDisabled()
  {
  if(!(PRR & _BV(PRSPI))) { return(false); }

  pinMode(PIN_SPI_nSS, OUTPUT); // Ensure that nSS is an output to avoid forcing SPI to slave mode by accident.
  fastDigitalWrite(PIN_SPI_nSS, HIGH); // Ensure that nSS is HIGH and thus any slave deselected when powering up SPI.

  PRR &= ~_BV(PRSPI); // Enable SPI power.
  // Configure raw SPI to match better how it was used in PICAXE V0.09 code.
  // CPOL = 0, CPHA = 0
  // Enable SPI, set master mode, set speed.
  const uint8_t ENABLE_MASTER =  _BV(SPE) | _BV(MSTR);
#if F_CPU <= 2000000 // Needs minimum prescale (x2) with slow (<=2MHz) CPU clock.
  SPCR = ENABLE_MASTER; // 2x clock prescale for <=1MHz SPI clock from <=2MHz CPU clock (500kHz SPI @ 1MHz CPU).
  SPSR = _BV(SPI2X);
#elif F_CPU <= 8000000
  SPCR = ENABLE_MASTER; // 4x clock prescale for <=2MHz SPI clock from nominal <=8MHz CPU clock.
  SPSR = 0;
#else // Needs setting for fast (~16MHz) CPU clock.
  SPCR = _BV(SPR0) | ENABLE_MASTER; // 8x clock prescale for ~2MHz SPI clock from nominal ~16MHz CPU clock.
  SPSR = _BV(SPI2X);
#endif
  return(true);
  }

// Power down SPI.
void powerDownSPI()
  {
  SPCR &= ~_BV(SPE); // Disable SPI.
  PRR |= _BV(PRSPI); // Power down...

  pinMode(PIN_SPI_nSS, OUTPUT); // Ensure that nSS is an output to avoid forcing SPI to slave mode by accident.
  fastDigitalWrite(PIN_SPI_nSS, HIGH); // Ensure that nSS is HIGH and thus any slave deselected when powering up SPI.

  // Avoid pins from floating when SPI is disabled.
  // Try to preserve general I/O direction and restore previous output values for outputs.
  pinMode(PIN_SPI_SCK, OUTPUT);
  pinMode(PIN_SPI_MOSI, OUTPUT);
  pinMode(PIN_SPI_MISO, INPUT_PULLUP);

  // If sharing SPI SCK with LED indicator then return this pin to being an output (retaining previous value).
  //if(LED_HEATCALL == PIN_SPI_SCK) { pinMode(LED_HEATCALL, OUTPUT); }
  }

// Capture a little entropy from clock jitter between CPU and WDT clocks; possibly one bit of entropy captured.
// Expensive in terms of CPU time and thus energy.
// TODO: may be able to reduce clock speed to lower energy cost while still detecting useful jitter.
uint_fast8_t clockJitterWDT()
  {
  // Watchdog should be (already) be disabled on entry.
  watchdogFired = false;
  wdt_enable(WDTO_15MS); // Set watchdog for minimum time.
  WDTCSR |= (1 << WDIE);
  uint_fast8_t count = 0;
  while(!watchdogFired) { ++count; } // Effectively count CPU cycles until WDT fires.
  return(count);
  }

// Capture a little entropy from clock jitter between CPU and 32768Hz RTC clocks; possibly up to 2 bits of entropy captured.
// Expensive in terms of CPU time and thus energy.
// TODO: may be able to reduce clock speed at little to lower energy cost while still detecting useful jitter
//   (but not below 131072kHz since CPU clock must be >= 4x RTC clock to stay on data-sheet and access TCNT2).
uint_fast8_t clockJitterRTC()
  {
  const uint8_t t0 = TCNT2;
  while(t0 == TCNT2) { }
  uint_fast8_t count = 0;
  const uint8_t t1 = TCNT2;
  while(t1 == TCNT2) { ++count; } // Effectively count CPU cycles in one RTC sub-cycle tick.
  return(count);
  }

// Combined clock jitter techniques to generate approximately 8 bits (the entire result byte) of entropy efficiently on demand.
// Expensive in terms of CPU time and thus energy, though possibly more efficient than basic clockJitterXXX() routines.
// Internally this uses a CRC as a relatively fast and hopefully effective hash over intermediate values.
// Note the that rejection of repeat values will be less effective with two interleaved gathering mechanisms
// as the interaction while not necessarily adding genuine entropy, will make counts differ between runs.
// DHD20130519: measured as taking ~63ms to run, ie ~8ms per bit gathered.
uint_fast8_t clockJitterEntropyByte()
  {
  uint16_t hash = 0;

  uint_fast8_t result = 0;
  uint_fast8_t countR = 0, lastCountR = 0;
  uint_fast8_t countW = 0, lastCountW = 0;

  const uint8_t t0 = TCNT2; // Wait for sub-cycle timer to roll.
  while(t0 == TCNT2) { ++hash; } // Possibly capture some entropy from recent program activity/timing.
  uint8_t t1 = TCNT2;

  watchdogFired = false;
  wdt_enable(WDTO_15MS); // Start watchdog, with minimum timeout.
  WDTCSR |= (1 << WDIE);
  int_fast8_t bitsLeft = 8; // Decrement when a bit is harvested...
  for( ; ; )
    {
    // Extract watchdog jitter vs CPU.
    if(!watchdogFired) { ++countW; }
    else // Watchdog fired.
      {
      if(countW != lastCountW) // Got a different value from last; assume one bit of entropy.
        {
        hash = _crc_ccitt_update(hash, countW);
        result = (result << 1) ^ ((uint_fast8_t)hash); // Nominally capturing (at least) lsb of hash.
        if(--bitsLeft <= 0) { break; } // Got enough bits; stop now.
        lastCountW = countW;
        }
      countW = 0;
      watchdogFired = false;
      wdt_enable(WDTO_15MS); // Restart watchdog, with minimum timeout.
      WDTCSR |= (1 << WDIE);
      }

    // Extract RTC jitter vs CPU.
    if(t1 == TCNT2) { --countR; }
    else // Sub-cycle timer rolled.
      {
      if(countR != lastCountR) // Got a different value from last; assume one bit of entropy.
        {
        hash = _crc_ccitt_update(hash, countR);
        result = (result << 1) ^ ((uint_fast8_t)hash); // Nominally capturing (at least) lsb of hash.
        if(--bitsLeft <= 0) { break; } // Got enough bits; stop now.
        lastCountR = countR;
        }
      countR = 0;
      t1 = TCNT2; // Set to look for next roll.
      }
    }

  wdt_disable(); // Ensure no spurious WDT wakeup pending.
  return(result);
  }


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
 2013/05/22 19:17 ~3uA@2.6V min calcuated ~23uA mean (100R supply resistor) in FROST controlling FHT8V, post sync (& single TXes), LED off, USB disconnected.
 */
