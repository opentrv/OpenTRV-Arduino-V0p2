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

Author(s) / Copyright (s): Deniz Erbilgin 2016
*/
/**
 * Minimal REV7 config for testing power consumption.
 * Aim is to:
 *     - todo init GPIO pins to safe mode.
 *     - todo init peripherals to safe low power mode.
 *     - loop endlessly.
 */

// INCLUDES & DEFINES
// REV7 / DORM1 all-in-one valve unit, secure TX.
#define CONFIG_DORM1
// Get defaults for valve applications.
#include <OTV0p2_valve_ENABLE_defaults.h>
// All-in-one valve unit (DORM1).
#include <OTV0p2_CONFIG_REV7.h>
// OTV0p2Base Libraries
#include <OTV0p2Base.h>
// Radio Libraries
//#include <OTRadioLink.h>
#include <OTRFM23BLink.h>


// OBJECTS & VARIABLES
/**
 * Peripherals present on REV7
 * - Pot        Set input to Hi-Z?
 * - PHT        IO_POWER_UP LOW
 * - Encoder    IO_POWER_UP LOW
 * - LED        Set pin HIGH?
 * - Button     Set to INPUT
 * - H-Bridge   Control pins HIGH, Current sense set to input.
 * - SHT21      read() once.
 * - RFM23B     ??
 * - XTAL       Setup and leave running?
 * - UART       Disable
 */

/*
 * Radio instance
 */
static constexpr bool RFM23B_allowRX = false;
 
static constexpr uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
static constexpr int8_t RFM23B_IRQ_PIN = -1;// PIN_RFM_NIRQ;
// XXX Is it really worth having a separate primary radio in this case?
OTRFM23BLink::OTRFM23BLink<OTV0P2BASE::V0p2_PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> PrimaryRadio;//RFM23B;
///* constexpr */ OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B

/*
 * SHT21 instance
 */



// FUNCTIONS
/**
 * @brief   Set pins and on-board peripherals to safe low power state.
 */

/**
 * @brief   Run POST.
 */


/**
 * @brief   Initialise GPIO pins and peripherals to a safe and low power mode.
 */
void setup()
{

}


/**
 * @brief   Loop endlessly doing nothing.
 */
void loop() { }



/**
 * @note    Power consumption figures.
 */
