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
						   Deniz Erbilgin 2015
*/

/*
 Serial (USB) I/O.

 Also, simple debug output to the serial port at its default (bootloader BAUD) rate.

 The debug support only enabled if DEBUG is defined, else does nothing, or at least as little as possible.
 */

#ifndef SERIAL_IO_H
#define SERIAL_IO_H

#include <OTV0p2Base.h>
#include "V0p2_Main.h"


// On serial output certain characters at the start of a line are reserved.
// These are used by remote software to trigger particular actions.
#define LINE_START_CHAR_CLI '>' // CLI prompt.
#define LINE_START_CHAR_ERROR '!' // Error log line.
#define LINE_START_CHAR_WARNING '?' // Warning log line.
#define LINE_START_CHAR_INFO '+' // Informational log line.
#define LINE_START_CHAR_RSTATS '@' // Remote (binary) stats log line.
#define LINE_START_CHAR_RJSTATS '{' // Remote (JSON) stats log line.
#define LINE_START_CHAR_STATS '=' // Local stats log line.


#ifndef DEBUG
#define DEBUG_SERIAL_PRINT(s) // Do nothing.
#define DEBUG_SERIAL_PRINTFMT(s, format) // Do nothing.
#define DEBUG_SERIAL_PRINT_FLASHSTRING(fs) // Do nothing.
#define DEBUG_SERIAL_PRINTLN_FLASHSTRING(fs) // Do nothing.
#define DEBUG_SERIAL_PRINTLN() // Do nothing.
#define DEBUG_SERIAL_TIMESTAMP() // Do nothing.
#else

// Send simple string or numeric to serial port and wait for it to have been sent.
// Make sure that Serial.begin() has been invoked, etc.
#define DEBUG_SERIAL_PRINT(s) { OTV0P2BASE::serialPrintAndFlush(s); }
#define DEBUG_SERIAL_PRINTFMT(s, fmt) { OTV0P2BASE::serialPrintAndFlush((s), (fmt)); }
#define DEBUG_SERIAL_PRINT_FLASHSTRING(fs) { OTV0P2BASE::serialPrintAndFlush(F(fs)); }
#define DEBUG_SERIAL_PRINTLN_FLASHSTRING(fs) { OTV0P2BASE::serialPrintlnAndFlush(F(fs)); }
#define DEBUG_SERIAL_PRINTLN() { OTV0P2BASE::serialPrintlnAndFlush(); }
// Print timestamp with no newline in format: MinutesSinceMidnight:Seconds:SubCycleTime
extern void _debug_serial_timestamp();
#define DEBUG_SERIAL_TIMESTAMP() _debug_serial_timestamp()

#endif // DEBUG

#endif // SERIAL_IO_H


