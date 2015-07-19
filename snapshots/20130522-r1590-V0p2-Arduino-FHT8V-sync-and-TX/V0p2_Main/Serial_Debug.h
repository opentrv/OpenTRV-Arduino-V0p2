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

#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

#include "V0p2_Main.h"



#ifndef DEBUG
#define DEBUG_SERIAL_PRINT(s) // Do nothing.
#define DEBUG_SERIAL_PRINTFMT(s, format) // Do nothing.
#define DEBUG_SERIAL_PRINT_FLASHSTRING(fs) // Do nothing.
#define DEBUG_SERIAL_PRINTLN_FLASHSTRING(fs) // Do nothing.
#define DEBUG_SERIAL_PRINTLN() // Do nothing.
#define DEBUG_SERIAL_TIMESTAMP() // Do nothing.
#else

// Send simple string or numeric to serial port and wait for it to have been sent.
// Make sure Serial.begin() has been invoked, etc.
#define DEBUG_SERIAL_PRINT(s) { Serial.print(s); Serial.flush(); }
#define DEBUG_SERIAL_PRINTFMT(s, fmt) { Serial.print((s), (fmt)); Serial.flush(); }
#define DEBUG_SERIAL_PRINT_FLASHSTRING(fs) { Serial.print(F(fs)); Serial.flush(); }
#define DEBUG_SERIAL_PRINTLN_FLASHSTRING(fs) { Serial.println(F(fs)); Serial.flush(); }
#define DEBUG_SERIAL_PRINTLN() { Serial.println(); Serial.flush(); }
// Print timestamp with no newline in format: MinutesSinceMidnight:Seconds:SubCycleTime
extern void _debug_serial_timestamp();
#define DEBUG_SERIAL_TIMESTAMP() _debug_serial_timestamp()

#endif













#endif
