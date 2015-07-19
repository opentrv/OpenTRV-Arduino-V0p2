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
 Selects/defines I/O pins and other standard hardware config for 'standard' V0.2 build.
 
 May in some cases be adjusted by #includes/#defines ahead of this one.
 */

#ifndef V0P2_BOARD_IO_CONFIG_H
#define V0P2_BOARD_IO_CONFIG_H

#include "V0p2_Main.h"

//-----------------------------------------
// Force definitions for peripherals that should be present on every V0.09 board
// (though may be ignored or not added to the board)
// to enable safe I/O setup and (eg) avoid bus conflicts.
#define USE_MODULE_RFM22RADIOSIMPLE // Always fitted on V0.2 board.


// Note 'standard' allocations of (ATmega328P-PU) pins, to be nominally Arduino compatible,
// eg see here: http://www.practicalmaker.com/blog/arduino-shield-design-standards
//
// 32768Hz xtal between pins 9 and 10, async timer 2, for accurate timekeeping and low-power sleep.
//
// Serial (bootloader/general): RX (dpin 0), TX (dpin 1)
#define PIN_SERIAL_RX 0 // ATMega328P-PU PDIP pin 2.
#define PIN_SERIAL_TX 1 // ATMega328P-PU PDIP pin 2.
// SPI: SCK (dpin 13, also LED on Arduino boards that the bootloader may 'flash'), MISO (dpin 12), MOSI (dpin 11), nSS (dpin 10).
#define PIN_SPI_SCK 13 // ATMega328P-PU PDIP pin 19.
#define PIN_SPI_MISO 12 // ATMega328P-PU PDIP pin 18.
#define PIN_SPI_MOSI 11 // ATMega328P-PU PDIP pin 17.
#define PIN_SPI_nSS 10 // ATMega328P-PU PDIP pin 16.  Active low enable.
// I2C/TWI: SDA (ain 4), SCL (ain 5), interrupt (dpin3)
#define PIN_SDA_AIN 4 // ATMega328P-PU PDIP pin 27.
#define PIN_SCL_AIN 5 // ATMega328P-PU PDIP pin 28.
// OneWire: DQ (dpin2)
// PWM / general digital I/O: dpin 5, 6, 9, 10
// Interrupts: INT0 (dpin2, also OneWire), INT1 (dpin3)
// Analogue inputs (may need digital input buffers disabled to minimise power, so use as outputs): dpin 6, 7


// UI LED for 'heat call', digital out.
#define LED_HEATCALL 13 // ATMega328P-PU PDIP pin 19. SHARED WITH SPI DUTIES...

// Digital output for radiator node to call for heat by wire and/or for boiler node to activate boiler.
#define OUT_HEATCALL 6  // ATMega328P-PU PDIP pin 12, no usable analogue input.

// UI main 'mode' button (active/pulled low by button, pref using weak internal pull-up), digital in.
#define BUTTON_MODE_L 5 // ATMega328P-PU PDIP pin 11, no analogue input.

#ifdef LEARN_BUTTON_AVAILABLE
// OPTIONAL UI 'learn' button  (active/pulled low by button, pref using weak internal pull-up), digital in.
#define BUTTON_LEARN_L 8 // ATMega328P-PU PDIP pin 14, no analogue input.
#endif

// Pin to power-up I/O devices only intermittently enabled, when high, digital out.
// Pref connected via 330R+ current limit and 100nF+ decoupling).
#define IO_POWER_UP 7 // ATMega328P-PU PDIP pin 13, no usable analogue input.

// Ambient light sensor (eg LDR) analogue input: higher voltage means more light.
#define LDR_SENSOR_AIN 0 // ATMega328P-PU PDIP pin 23.



// Note: I/O budget for motor drive probably 4 pins minimum.
// 2D: To direct drive motor this will need 2 outputs for H-bridge.
// 1A: Then some sort of end-stop sensor (eg current draw) analogue input
// 1I: and/or pulse input/counter/interrupt
// ID: and some supply to pulse counter mechanism (eg LED for opto) maybe IO_POWER_UP.


// Call this ASAP in setup() to configure I/O safely for the board, avoid pins floating, etc.
inline void IOSetup()
  {
  // Initialise all digitial I/O to safe state ASAP and avoid floating.
  // In absence of a specific alternative, drive low as an output to minimise consumption (eg from floating input).
  for(int i = 14; --i >= 0; ) // For all digital pins from 0 to 13 inclusive...
    {
    switch(i)
      {
      // Low output is good low-power default.
      default: { digitalWrite(i, LOW); pinMode(i, OUTPUT); break; }

#ifdef PIN_SERIAL_RX
      // Weak pull-up empirically found to be lowest leakage current with TTL-232R-3V3 USB lead.
      case PIN_SERIAL_RX: case PIN_SERIAL_TX: { pinMode(i, INPUT_PULLUP); break; }
#endif

      // Switch main UI LED on for the rest of initialisation...
      case LED_HEATCALL: { pinMode(LED_HEATCALL, OUTPUT); digitalWrite(LED_HEATCALL, HIGH); break; }

      // Make button pins inputs with internal weak pull-ups (saving an external resistor on each case).
#ifdef BUTTON_LEARN_L
      case BUTTON_LEARN_L: // Learn button is optional.
#endif
      case BUTTON_MODE_L: { pinMode(i, INPUT_PULLUP); break; }

#ifdef PIN_SPI_nSS
      // Do not leave/set SPI nSS as low output (or floating) to avoid waking up SPI slave(s).
      case PIN_SPI_nSS: { pinMode(PIN_SPI_nSS, INPUT_PULLUP); break; }
#endif
#ifdef PIN_SPI_MISO
      // Do not leave/set SPI MISO as (low) output (or floating).
      case PIN_SPI_MISO: { pinMode(PIN_SPI_MISO, INPUT_PULLUP); break; }
#endif
      }
    }
  }



#endif
