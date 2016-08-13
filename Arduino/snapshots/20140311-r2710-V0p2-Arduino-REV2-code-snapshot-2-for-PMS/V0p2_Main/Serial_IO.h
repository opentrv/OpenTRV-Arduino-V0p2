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
 Serial (USB) I/O.
 
 Also, simple debug output to the serial port at its default (bootloader BAUD) rate.

 The debug support only enabled if DEBUG is defined, else does nothing, or at least as little as possible.
 */

#ifndef SERIAL_IO_H
#define SERIAL_IO_H

#include "V0p2_Main.h"


// Write a single (Flash-resident) string to serial followed by line-end and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintlnAndFlush(__FlashStringHelper const *line);

// Write a single (Flash-resident) string to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(__FlashStringHelper const *text);

// Write a single (Flash-resident) string to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(char const *text);

// Write a single (Flash-resident) character to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(char c);

// Write a single (Flash-resident) number to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(int i, int fmt = DEC);

// Write a single (Flash-resident) number to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(unsigned u, int fmt = DEC);

// Write line-end to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintlnAndFlush();



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
#define DEBUG_SERIAL_PRINT(s) {serialPrintAndFlush(s); }
#define DEBUG_SERIAL_PRINTFMT(s, fmt) { serialPrintAndFlush((s), (fmt)); }
#define DEBUG_SERIAL_PRINT_FLASHSTRING(fs) { serialPrintAndFlush(F(fs)); }
#define DEBUG_SERIAL_PRINTLN_FLASHSTRING(fs) { serialPrintlnAndFlush(F(fs)); }
#define DEBUG_SERIAL_PRINTLN() { serialPrintlnAndFlush(); }
// Print timestamp with no newline in format: MinutesSinceMidnight:Seconds:SubCycleTime
extern void _debug_serial_timestamp();
#define DEBUG_SERIAL_TIMESTAMP() _debug_serial_timestamp()

#endif













#endif


