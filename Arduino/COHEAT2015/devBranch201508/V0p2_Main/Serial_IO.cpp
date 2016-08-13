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
 Simple debug output to the serial port at its default (bootloader BAUD) rate.

 Only enabled if DEBUG is defined, else does nothing, or at least as little as possible.
 
 See some other possibilities here: http://playground.arduino.cc/Main/Printf
 */

#include "Serial_IO.h"

#include "Power_Management.h"
#include "RTC_Support.h"


// Flush to use for all serialPrintXXX() and DEBUG_PRINTXXX routines.
#define _flush() flushSerialSCTSensitive()

// Write a single (Flash-resident) string to serial followed by line-end and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintlnAndFlush(__FlashStringHelper const * const line)
  {
  const bool neededWaking = powerUpSerialIfDisabled();
  // Send the line of text followed by line end.
  Serial.println(line);
  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  _flush();
  if(neededWaking) { powerDownSerial(); }
  }

// Write a single (Flash-resident) string to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(__FlashStringHelper const * const text)
  {
  const bool neededWaking = powerUpSerialIfDisabled();
  // Send the text.
  Serial.print(text);
  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  _flush();
  if(neededWaking) { powerDownSerial(); }
  }

// Write a single (read-only) string to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(const char * const text)
  {
  const bool neededWaking = powerUpSerialIfDisabled();
  // Send the text.
  Serial.print(text);
  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  _flush();
  if(neededWaking) { powerDownSerial(); }
  }

// Write a single (Flash-resident) character to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(const char c)
  {
  const bool neededWaking = powerUpSerialIfDisabled();
  // Send the character.
  Serial.print(c);
  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  _flush();
  if(neededWaking) { powerDownSerial(); }
  }

// Write a single (Flash-resident) number to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(const int i, const int fmt)
  {
  const bool neededWaking = powerUpSerialIfDisabled();
  // Send the character.
  Serial.print(i, fmt);
  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  _flush();
  if(neededWaking) { powerDownSerial(); }
  }

// Write a single (Flash-resident) number to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(const unsigned u, const int fmt)
  {
  const bool neededWaking = powerUpSerialIfDisabled();
  // Send the character.
  Serial.print(u, fmt);
  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  _flush();
  if(neededWaking) { powerDownSerial(); }
  }

// Write a single (Flash-resident) number to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintAndFlush(const unsigned long u, const int fmt)
  {
  const bool neededWaking = powerUpSerialIfDisabled();
  // Send the character.
  Serial.print(u, fmt);
  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  _flush();
  if(neededWaking) { powerDownSerial(); }
  }

// Write line-end to serial and wait for transmission to complete.
// This enables the serial if required and shuts it down afterwards if it wasn't enabled.
void serialPrintlnAndFlush()
  {
  const bool neededWaking = powerUpSerialIfDisabled();
  // Send the text.
  Serial.println();
  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  _flush();
  if(neededWaking) { powerDownSerial(); }
  }









#ifdef DEBUG // Don't emit debug-support code unless in DEBUG.

// Print timestamp with no newline in format: MinutesSinceMidnight:Seconds:SubCycleTime
void _debug_serial_timestamp()
  {
  const bool neededWaking = powerUpSerialIfDisabled();
  // Grab time values ASAP, fastest-incrementing first.
  // TODO: could lock out interrupts to capture atomically.
  const uint8_t ss = getSubCycleTime();
  const uint8_t s = getSecondsLT();
  const uint16_t m = getMinutesSinceMidnightLT();
  Serial.print(m);
  Serial.print(':'); Serial.print(s);
  Serial.print(':'); Serial.print(ss);
  _flush();
  if(neededWaking) { powerDownSerial(); }
  }

#endif


