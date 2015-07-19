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
  BlinkPower
  
  Based on Arduino Blink sketch, used to test some basic power-conservation techniques, with a LED blinking at ~1Hz.
  
  DHD20130416: hardware setup on 'minimal' breadboard.
    * 8MHz RC clock (initially w/ no prescaler) ATmega328P running at 5V via 10R or 100R resistor (with downstream 100uF cap) to help measure power consumption.
    * Fuse set for 65ms additional clock settle time, later reduced to 0ms.
    * LED on digital pin 13 to ground in series with 470R.
    * All unused pins unconnected and nominally floating.
    * 32768Hz xtal between pins 9 and 10 if WAKEUP_1HZ_XTAL is defined.
  
  Power consumption with different variants on loop() body:
  
  Reducing LED duty cycle, and leaving setup() empty except for pinMode(led, OUTPUT);
  11:34 ~13mA with digitalWrite(led, HIGH); delay(500); digitalWrite(led, LOW); delay(500);
  11:36 ~11mA with digitalWrite(led, HIGH); delay(100); digitalWrite(led, LOW); delay(900);
  11:37 ~11mA with digitalWrite(led, HIGH); delay(10); digitalWrite(led, LOW); delay(990);
  
  Avoiding floating inputs:
  12:35 ~9mA with additional code in setup(): for(int i = 2; i < 13; ++i) { pinMode(i, OUTPUT); }
  
  Turning off an unused module:
  13:21 ~8.8ma with additional code in setup(): power_adc_disable();
  
  Scaling the clock down while busy-idling:
  13:58 ~3.4mA with delay() replaced with sleepFor() and with code enabled by SLOW_DOWN_CLOCK_IN_DELAY to reduce clock speed during delay.

  Basic WAKEUP_1HZ_TIMER1 support with set_sleep_mode(SLEEP_MODE_IDLE):
  15:39 ~4.9mA with no power enable/disable around sleep...
  15:43 ~4.4mA with adc/spi/timer0/twi modules disabled before sleep.
  16:12 ~4.2mA with timer2/usart modules disabled before sleep (and some more analogue stuff disabled at start-up).
  16:32 ~4.1mA with most modules permanently disabled with just timer0 on again at wakeup.
  16:37 ~4.0mA with LED duty cycle reduced to 0.1% (1ms/1s).
  
  WAKEUP_1HZ_XTAL using external 32768Hz XTAL and SLEEP_MODE_PWR_SAVE.
  16:55 ~2.7mA with timer 0 still running (for delay()).
  16:58 ~2.0mA moving always-on power LED upstream of 10R resistor!
  
  DISABLING ALL MODULES unless explicitly needed.
  17:51 ~0.56mA with power_all_disable() in prologue.
  
  (TEMPORARY TEST: LEAKAGE TO/FROM USB APPARENTLY ~0.25mA)
  (18:02 ~0.34mA with ATmega RX (pin2) tied to Vcc via 10k rather than being held at 3.3V.)
  (18:04 ~0.31mA with ATmega TX (pin3) tied to 0V via 10k rather than driving ~3V RXD line via 10k.)

  DISABLE ENTIRE ADC SUBSYSTEM
  18:22 ~0.44mA with ADCSRA = 0
  
  REDUCED WAKEUP TIME FROM SLEEP to 0ms from default 65ms with boot flags
  21:46 ~0.44mA with USB wired up
  (21:47 ~0.18mA with USB disconnected and RX/TX tied as above)
  
  TX switched to weak pull-up to reduce leakage to USB.
  22:02 ~0.31mA with USB still connected.
  (22:03 ~0.18mA with USB disconnected, RX tied to Vcc via 10K, TX disconnected pulled up weakly internally.)
  
  23:03 ~0.08mA Actually using correct sleep for main loop!
  (23:05 ~0.02mA with USB disconnected, down to 2uA.)
  
  Used Optiboot 5 loader c/o tim7 w/ clock prescaled to 1MHz 20130417
  10:38 ~0.02mA (20uA) with USB disconnected, down to ~1uA minimum.
  13:47 ~3uA with USB disconnected, running from 2xAA NiMH (<1uA with LED removed).  Runs for >1min from 4700uF cap.
 */

#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay_basic.h>

#define WAKEUP_1HZ_XTAL // Use 1Hz wakeup from external 32768Hz xtal and timer 2.

#ifndef WAKEUP_1HZ_XTAL
#define WAKEUP_1HZ_TIMER1 // Use 1Hz wakeup from CPU idle using timer 1.
#endif

#ifndef WAKEUP_1HZ_TIMER1
#define SLOW_DOWN_CLOCK_IN_DELAY // Use dynamic clock scaling to reduce power: DO NOT COMBINE with TIMER1 interrupt stuff!
#endif

#define DONT_USE_TIMER0 // Avoid using timer 0 and this delay(), PWM pins 5&6, etc.
 
 
#ifndef sleep_bod_disable() // If not included in Arduino AVR toolset...
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
 
 

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
const int led = 13;

static volatile boolean ticked; // Set true on 1Hz interrupt.


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the LED digital pin as an output.
  pinMode(led, OUTPUT);

  wdt_disable(); // Disable (unused) watchdog timer.
  
  // Turn off analogue stuff that eats power.
  ADCSRA = 0; // Do before power_all_disable() to avoid freezing ADC in active state!
  ACSR = (1<<ACD); // Disable the analog comparator.
  DIDR0 = 0x3F; // Disable digital input buffers on all ADC0-ADC5 pins.
  DIDR1 = (1<<AIN1D)|(1<<AIN0D); // Disable digital input buffer on AIN1/0.

  // Turn off all modules; re-enable explicitly any still needed.
  power_all_disable(); 

  // Prevent digital I/O floating (sett all as outputs, and low).
  for(int i = 2; i <= 13; ++i) { pinMode(i, OUTPUT); digitalWrite(led, LOW); }

  pinMode(0, INPUT_PULLUP); // Switch RX to input with weak pull-up to reduce leakage.
  pinMode(1, INPUT_PULLUP); // Switch TX to input with weak pull-up to reduce leakage.

#ifndef DONT_USE_TIMER0
  power_timer0_enable(); // Turning timer0 off messes up some standard Arduino support such as delay().
#endif
  
#if defined(WAKEUP_1HZ_TIMER1)
  power_timer1_enable();
  timer1HzIntSetup();
#elif defined(WAKEUP_1HZ_XTAL)
  power_timer2_enable();
  timer1HzXtalIntSetup();
#endif
}

#ifdef WAKEUP_1HZ_XTAL
void timer1HzXtalIntSetup() {
  // Set up TIMER2.
  TCCR2A = 0x00;
  TCCR2B = (1<<CS22)|(1<<CS20); // Set CLK/128 for overflow interrupt every 1s.
  //TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20); // Set CLK/1024 for overflow interrupt every 8s.
  ASSR = (1<<AS2); // Enable asynchronous operation.
  TIMSK2 = (1<<TOIE2); // Enable the timer 2 interrupt.
}
#endif

#ifdef WAKEUP_1HZ_XTAL
ISR(TIMER2_OVF_vect) {
  sleep_disable();
  //power_all_enable(); // Optional power up some/all modules here.
#ifndef DONT_USE_TIMER0
  power_timer0_enable();
#endif
  ticked = true;
}
#endif

#ifdef WAKEUP_1HZ_TIMER1
// Set up 1Hz wakeup from (prescaled) CPU clock using timer1.
void timer1HzIntSetup() {
  cli(); // disable interrupts
  TCNT1 = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12); // CTC mode.
  TCCR1B |= (1 << CS10) | (1 << CS12); //  1024x prescaler.
  OCR1A = (F_CPU / 1024) - 1; // Get 1Hz interrupt given prescaling.
  TIMSK1 |= (1 << OCIE1A); // Enable timer compare int.
  sei(); // enable interrupts
}
#endif

#ifdef WAKEUP_1HZ_TIMER1
// Setup handler for 1Hz wakeup...
ISR(TIMER1_COMPA_vect) {
  sleep_disable();
  //power_all_enable(); // Optional power up some/all modules here.
#ifndef DONT_USE_TIMER0
  power_timer0_enable();
#endif
  ticked = true; 
  }
#endif

#if defined(WAKEUP_1HZ_TIMER1) || defined(WAKEUP_1HZ_XTAL)
void sleepUntilInt() {

#if defined(WAKEUP_1HZ_TIMER1)
  set_sleep_mode(SLEEP_MODE_IDLE);
#else
  set_sleep_mode(SLEEP_MODE_PWR_SAVE); // Deeper sleep when using external crystal.
#endif

  // Optionally disable modules here.
#ifndef DONT_USE_TIMER0
  power_timer0_disable();
#endif

  cli();
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  sleep_disable();
  sei();
}
#endif

// Sleep for specified number of milliseconds (approx), in as low-power mode as possible.
void sleepFor(int ms) {
#ifdef SLOW_DOWN_CLOCK_IN_DELAY
  const clock_div_t prescale = clock_prescale_get();
  // Reduce clock speed (increase prescale) as far as possible without losing too much precision...
  int newPrescale = prescale;
#ifdef DONT_USE_TIMER0
  uint32_t loops = (F_CPU / 4000) * ms;
  for( ; (loops > 4) && (newPrescale < clock_div_256); ++newPrescale, loops >>= 1) { }
#else
  for( ; (ms > 4) && (newPrescale < clock_div_256); ++newPrescale, ms >>= 1) { }
#endif
  if(newPrescale > prescale) { clock_prescale_set(newPrescale); }
#endif

#ifdef DONT_USE_TIMER0
  for( ; loops > 65536 ; loops -= 65536)  { _delay_loop_2(0); }
  _delay_loop_2(loops);
#else
  delay(ms); // Seems to use timer0.
#endif
  
#ifdef SLOW_DOWN_CLOCK_IN_DELAY
  if(newPrescale > prescale) { clock_prescale_set(prescale); } // Restore clock prescale.
#endif
}

#define CYCLE_MS 1000
#define LED_ON_MS 1

// the loop routine runs over and over again forever:
void loop() {
  digitalWrite(led, HIGH); sleepFor(LED_ON_MS); digitalWrite(led, LOW);
#if defined(WAKEUP_1HZ_TIMER1) || defined(WAKEUP_1HZ_XTAL)
  while(!ticked) { sleepUntilInt(); } // Sleep until interrupted.
  ticked = false;
#else
  sleepFor(CYCLE_MS - LED_ON_MS); // Busy sleep, burning CPU cycles.
#endif
}
