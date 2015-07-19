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
 Simple debug output to the serial port at its default (bootloader BAUD) rate.

 Only enabled if DEBUG is defined, else does nothing, or at least as little as possible.
 
 See some other possibilites here: http://playground.arduino.cc/Main/Printf
 */

#include "Serial_Debug.h"

#include "Power_Management.h"
#include "RTC_Support.h"



#ifdef DEBUG // Don't emit debug-support code unless in DEBUG.

// Print timestamp with no newline in format: MinutesSinceMidnight:Seconds:SubCycleTime
void _debug_serial_timestamp()
  {
  // Grab time values ASAP, fastest-incrementing first.
  // TODO: could lock out interrupts to capture atomically.
  const uint8_t ss = getSubCycleTime();
  const uint8_t s = getSecondsLT();
  const uint16_t m = getMinutesSinceMidnightLT();
  Serial.print(m);
  Serial.print(':'); Serial.print(s);
  Serial.print(':'); Serial.print(ss);
  Serial.flush();
  }



#endif
