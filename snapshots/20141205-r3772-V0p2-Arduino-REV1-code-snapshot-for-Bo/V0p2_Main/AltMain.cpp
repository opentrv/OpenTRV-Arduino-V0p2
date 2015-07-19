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
  Alternate POST / setup and loop / main for non-OpenTRV code running on OpenTRV h/w platform.

  Also for rapid prototyping without dead-weight of OpenTRV intricate timing, etc!
 */


// Arduino libraries.
#include <Wire.h>

// From: https://github.com/bblanchon/ArduinoJson
#include <JsonGenerator.h>

#include "V0p2_Main.h"

#include "V0p2_Generic_Config.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.

#include "Control.h"
#include "EEPROM_Utils.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "RTC_Support.h"
#include "Power_Management.h"
#include "PRNG.h"
#include "RFM22_Radio.h"
#include "Security.h"
#include "Serial_IO.h"
#include "UI_Minimal.h"




// Link in support for alternate Power On Self-Test and main loop if required.
#if defined(ALT_MAIN_LOOP)

// Called from startup() after some initial setup has been done.
// Can abort with panic() if need be.
void POSTalt()
  {
#ifdef USE_MODULE_RFM22RADIOSIMPLE
#if !defined(RFM22_IS_ACTUALLY_RFM23) && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("(Using RFM22.)");
#endif
  // Initialise the radio, if configured, ASAP, because it can suck a lot of power until properly initialised.
  RFM22PowerOnInit();
  // Check that the radio is correctly connected; panic if not...
  if(!RFM22CheckConnected()) { panic(); }
  // Configure the radio.
  RFM22RegisterBlockSetup(FHT8V_RFM22_Reg_Values);
  // Put the radio in low-power standby mode.
  RFM22ModeStandbyAndClearState();
#endif
  }




using namespace ArduinoJson::Generator;

// Called from loop().
void loopAlt()
  {

  // Sleep in low-power mode (waiting for interrupts) until seconds roll.
  // NOTE: sleep at the top of the loop to minimise timing jitter/delay from Arduino background activity after loop() returns.
  // DHD20130425: waking up from sleep and getting to start processing below this block may take >10ms.
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*E"); // End-of-cycle sleep.
#endif
  powerDownSerial(); // Ensure that serial I/O is off.
  // Power down most stuff (except radio for hub RX).
  minimisePowerWithoutSleep();
  static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = getSecondsLT()))
    {
    sleepUntilInt(); // Normal long minimal-power sleep until wake-up interrupt.
    }
  TIME_LSD = newTLSD;
#if 0 && defined(DEBUG)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("*S"); // Start-of-cycle wake.
#endif


  // START LOOP BODY
  // ===============
 
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("tick...");
  
  const bool neededWaking = powerUpSerialIfDisabled();


  // Allow space in buffer for:
  //   * buffer offset/preamble
  //   * max binary length, or max JSON length + 1 for CRC + 1 to allow detection of oversize message
  //   * terminating 0xff
#define STATS_MSG_START_OFFSET (RFM22_PREAMBLE_BYTES + RFM22_SYNC_MIN_BYTES)
#define STATS_MSG_MAX_LEN (64 - STATS_MSG_START_OFFSET)
  uint8_t buf[STATS_MSG_START_OFFSET + max(FullStatsMessageCore_MAX_BYTES_ON_WIRE,  MSG_JSON_MAX_LENGTH+1) + 1];

  // Send JSON message.        
  uint8_t *bptr = buf + STATS_MSG_START_OFFSET;
  // Now append JSON text and closing 0xff...
  // Use letters that correspond to the values in ParsedRemoteStatsRecord and when displaying/parsing @ status records.
  int8_t wrote;

  static SimpleStatsRotation<4> ss1; // Configured for maximum different stats.
  bool maximise = false;
  if(ss1.isEmpty())
    {
#ifdef DEBUG
    ss1.enableCount(true); // For diagnostic purposes.
#endif
    // Try and get as much out on the first TX as possible.
    maximise = true;
    }
  ss1.put("T|C16", TemperatureC16.read());
#if defined(HUMIDITY_SENSOR_SUPPORT)
  ss1.put("H|%", RelHumidity.read());
#endif
  ss1.put("L", AmbLight.read()/4);
  ss1.put("B|cV", Supply_mV.read()/10);
  wrote = ss1.writeJSON(bptr, sizeof(buf) - (bptr-buf), getStatsTXLevel(), maximise);
  if(0 == wrote)
    {
DEBUG_SERIAL_PRINTLN_FLASHSTRING("JSON gen err!");
    return;
    }
    
  // Record stats as if local, and treat channel as secure.
  recordJSONStats(true, (const char *)bptr);

  DEBUG_SERIAL_PRINT_FLASHSTRING("JSON: ");
  DEBUG_SERIAL_PRINT((const char *)bptr);
  DEBUG_SERIAL_PRINTLN();

  // Force any pending output before return / possible UART power-down.
  flushSerialSCTSensitive();

  if(neededWaking) { powerDownSerial(); }
  }



#endif
