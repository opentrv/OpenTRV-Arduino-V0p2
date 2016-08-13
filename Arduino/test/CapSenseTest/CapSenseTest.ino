
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

Author(s) / Copyright (s): Damon Hart-Davis 2014
*/

/*
  CapSenseTest.ino
  
  Preminary test of bare capacitive proximity sensing (over ~10cm distance).
  
  Notes:
      20140906: Turns on on-board Uno LED when close object/hand detected; affected by laptop/Uno/USB combo mains powered or not (needs auto-recalibration).
      20140906: CapacitiveSensor all defaults, pins 4&2 on Uno, 4.7M resistor, THRESH=128, have to touch wire sensor or insulation to activate.
      20140906: CapacitiveSensor all defaults, pins 4&2 on Uno, 4.7M resistor, THRESH=32, within ~2cm of 10cm stiff wire sensor activates.
      20140906: CapacitiveSensor all defaults, pins 4&2 on Uno, 4.7M resistor, THRESH=32, 5cmX5cm PCB groundplane as sensor ~3cm detection.  (Laptop not connected to mains.)
      20140906: CapacitiveSensor all defaults, pins 4&2 on Uno, 4.7M resistor, THRESH=32, 30xmX30cm tin foil sheet as sensor >10cm detection.  (Laptop not connected to mains.)
 */


// See: http://playground.arduino.cc/Main/CapacitiveSensor
// CapSense 04 or newer
// https://github.com/arduino-libraries/CapacitiveSensor/zipball/master
#include <CapacitiveSensor.h>


#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay_basic.h>

 
 

// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
const int led = 13;

// Resistor (>=1Mohm) between pins 4 and 2, pin 2 is sensor pin, add wire/foil.
static CapacitiveSensor cs_4_2 = CapacitiveSensor(4, 2); 


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the LED digital pin as an output.
  pinMode(led, OUTPUT);
  Serial.begin(4800);
}


// Sleep for specified number of milliseconds (approx), in as low-power mode as possible.
void sleepFor(const int ms) {
  delay(ms);
}

#define CYCLE_MS 1000
#define LED_ON_MS 1 // Minimum blip on per cycle; just perceptible.

// THRES=128 requires touch with 10cm wire sensor, no false +ves
// THREH=32 activates from ~2cm away with 10cm wire sensor, no false +ves
#define THRESH 32


// the loop routine runs over and over again forever:
void loop() {
  
  const long total = cs_4_2.capacitiveSensor(30);
  Serial.println(total);

  digitalWrite(led, HIGH);
  if(total < THRESH) { delay(LED_ON_MS); digitalWrite(led, LOW); }
  delay(CYCLE_MS - LED_ON_MS); // Busy sleep, burning CPU cycles.
}
