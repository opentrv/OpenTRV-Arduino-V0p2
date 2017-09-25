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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2017
*/

/*
  V0p2 (V0.2) core/main header file for this project:
  all other project header files should #include this first
  (or at least immediately after std/AVR headers) for consistency,
  and project non-header files should include this via their own header files (or directly).
  */

#ifndef V0p2_MAIN_H
#define V0p2_MAIN_H


// GLOBAL flags that alter system build and behaviour.
//#define DEBUG // If defined, do extra checks and serial logging.  Will take more code space and power.
//#define EST_CPU_DUTYCYCLE // If defined, estimate CPU duty cycle and thus base power consumption.

#ifndef BAUD
// Ensure that OpenTRV 'standard' UART speed is set unless explicitly overridden.
#define BAUD 4800
#endif

#define V0p2_REV 7


#include <OTV0p2_Board_IO_Config.h> // I/O pin allocation and setup: include ahead of I/O module headers.

#include <Arduino.h>
#include <OTV0p2Base.h>
#include <OTRadValve.h>
#include <OTRadioLink.h>
#include <OTRFM23BLink.h>
#include <OTSIM900Link.h>
#include <OTRN2483Link.h>
#include <OTAESGCM.h>

///////// CONSTANTS
// Mask for Port D input change interrupts.
constexpr uint8_t SERIALRX_INT_MASK = 0b00000001; // Serial RX
constexpr uint8_t MASK_PD_BASIC = SERIALRX_INT_MASK; // Serial RX by default.
constexpr uint8_t MASK_PD1 = MASK_PD_BASIC; // Just serial RX, no voice.
#if BUTTON_MODE_L > 7
  #error BUTTON_MODE_L expected to be on port D
#endif
constexpr uint8_t MODE_INT_MASK = (1 << (BUTTON_MODE_L&7));
constexpr uint8_t MASK_PD = (MASK_PD1 | MODE_INT_MASK); // MODE button interrupt (et al).




#endif

