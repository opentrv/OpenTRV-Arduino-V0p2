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
 Implementation of minimal UI using single LED and one or two momentary push-buttons.
 */

#include <Arduino.h>
#include <string.h>

#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "Ambient_Light_Sensor.h"
#include "Control.h"
#include "EEPROM_Utils.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "Power_Management.h"
#include "PRNG.h"
#include "RTC_Support.h"
#include "Schedule.h"
#include "Serial_IO.h"
#include "Temperature_Sensor.h"
#include "UI_Minimal.h"

static bool isWarmMode; // Defaults to / starts at false/'frost'.
// If true then the unit is in 'warm' (heating) mode, else 'frost' protection mode.
bool inWarmMode() { return(isWarmMode); }

#ifdef SUPPORT_BAKE // IF DEFINED: this unit supports BAKE mode.
// Only relevant if isWarmMode is true,
static uint_least8_t bakeCountdownM;
// If true then the unit is in 'bake' mode, a subset of 'warm' mode which boosts the temperature target temporarily.
bool inBakeMode() { return(isWarmMode && (0 != bakeCountdownM)); }
// Cancel 'bake' mode if active; does not force to FROST mode.
void cancelBake() { bakeCountdownM = 0; }
// Start/restart 'bake' mode and timeout.
void startBake() { isWarmMode = true; bakeCountdownM = BAKE_MAX_M; }
#endif


// If true (the default) then the system has an 'Eco' energy-saving bias, else it has a 'comfort' bias.
// Several system parameters are adjusted depending on the bias,
// with 'eco' slanted toward saving energy, eg with lower target temperatures and shorter on-times.
// At the transition from one bias to the other user-settable values may be adjusted to match.
// ~0 (erased/default) means eco mode; any other value means comfort.
bool hasEcoBias() { return(eeprom_read_byte((uint8_t *)EE_START_ECO_BIAS) == (uint8_t)~0); }
// Set bias flag AND force any immediate related state changes such as adjusting thresholds.
// On setting eco mode:
//   * force target warm temperatures no higher than eco value.
// On setting confort mode:
//   * force target warm temperatures no lower than comfort value.
static bool setEcoBias(const bool eco)
  {
  if(eco)
    {
    eeprom_smart_erase_byte((uint8_t *)EE_START_ECO_BIAS);
    if(getWARMTargetC() > BIASECO_WARM) { setWARMTargetC(BIASECO_WARM); } // Allow no higher than eco default.
    if(getFROSTTargetC() > BIASECO_FROST) { setFROSTTargetC(BIASECO_FROST); } // Allow no higher than eco default.
    }
  else
    {
    eeprom_smart_clear_bits((uint8_t *)EE_START_ECO_BIAS, randRNG8() & ~1); // Gratuitously capture some slight entropy in EEPROM.
    if(getWARMTargetC() < BIASCOM_WARM) { setWARMTargetC(BIASCOM_WARM); } // Allow no lower than comfort default.
    if(getFROSTTargetC() < BIASCOM_FROST) { setFROSTTargetC(BIASCOM_FROST); } // Allow no lower than comfort default.
    }
  }

#ifdef OCCUPANCY_SUPPORT
// Number of minutes that room is regarded as occupied after markAsOccupied(); strictly positive.
// DHD20130528: no activity for 30 minutes usually enough to declare room empty in my experience.
// Should probably be at least as long as, or a little longer than, the BAKE timeout.
// Should probably be significantly shorter than normal 'learn' on time to allow savings from that in empty rooms.
#define OCCUPATION_TIMEOUT_M (min(max(SETBACK_FULL_M, 30), 255))

// Time until room regarded as unoccupied, in minutes; initially zero (ie treated as unoccupied at power-up).
// (Not volatile since not expected to be used from ISRs.)
static uint8_t occupationCountdownM;

// Returns true if the room appears to be likely occupied (with active users) now or recently.
// Operates on a timeout; calling markAsOccupied() restarts the timer.
// Defaults to false (and API still exists) when OCCUPANCY_SUPPORT not defined.
// Do not call from an ISR.
bool isLikelyOccupied() { return(0 != occupationCountdownM); }

// Returns true if the room appears to be likely occupied (with active users) recently.
// This uses the same timer as isOccupied() (restarted by markAsOccupied())
// but returns to false somewhat sooner for example to allow ramping up more costly occupancy detection methods
// and to allow some simple graduated occupancy responses.
// Do not call from an ISR.
bool isLikelyRecentlyOccupied() { return(occupationCountdownM > OCCUPATION_TIMEOUT_M/2); }

// Call when some strong evidence of room occupation has occurred.
// Such evidence may include operation of buttons (etc) on the unit or PIR.
// Do not call from (for example) 'on' schedule change.
// Do not call from an ISR.
void markAsOccupied() { occupationCountdownM = OCCUPATION_TIMEOUT_M; }
#endif



// Remaining seconds to keep CLI active; zero implies inactive.
// Starts up with full value to allow easy set of time, etc, without specially activating CLI.
static int8_t CLISecondsLeft = CLIActiveS;

// Reset CLI active timer to the full whack before it goes inactive again (ie makes CLI active for a while).
#define resetCLIActiveTimer() { CLISecondsLeft = CLIActiveS; }

// Returns true if the CLI is active, at least intermittently.
bool isCLIActive() { return(0 != CLISecondsLeft); }


#ifdef LEARN_BUTTON_AVAILABLE
// Period in minutes for simple learned on-time; strictly positive (and less than 1440).
#define LEARNED_ON_PERIOD_M 60
// Period in minutes for simple learned on-time with comfort bias; strictly positive (and less than 1440).
#define LEARNED_ON_PERIOD_COMFORT_M 120
// Handle learn button.
// In simple mode: if in frost mode clear simple schedule else set repeat for every 24h from now for 1h.
// May be called from pushbutton or CLI UI components.
static void handleLearnButton()
  {
  // Set simple schedule starting every 24h from a little before now and running 1h (eco) or 2h (comfort).  
  if(isWarmMode)
    {
    const uint8_t lengthM = hasEcoBias() ? LEARNED_ON_PERIOD_M : LEARNED_ON_PERIOD_COMFORT_M;
    const uint8_t windBackM = lengthM >> 2; // Wind back start by about 25%.
    const uint_least16_t msm = getMinutesSinceMidnightLT();
    const uint_least16_t start = (msm >= windBackM) ? (msm - windBackM) : (msm + 1440 - windBackM);
    setSimpleSchedule(start, lengthM);
    }
  // Clear simple schedule.
  else { clearSimpleSchedule(); }
  }
#endif

// Returns true if system is in 'learn'/smart mode.
// If in 'smart' mode can anticipate user demand to pre-warm rooms, maintain customary temperatures, etc.
// Currently true if any simple schedule is set.
// TODO: maybe only if schedule characteristic of having been set by the learn button.
bool inSmartMode()
  {
  return(isSimpleScheduleSet());
  }

// Pause between flashes to allow them to be distinguished (>100ms); was mediumPause() for PICAXE V0.09 impl.
static void inline offPause()
  {
  nap(WDTO_120MS); // 120ms, was V0.09 144ms mediumPause() for PICAXE V0.09 impl.
  pollIO(); // Slip in an I/O poll.
  }

// Call this on even numbered seconds (with current time in seconds) to allow the UI to operate.
// Should never be skipped, so as to allow the UI to remain responsive.
// Runs in 350ms or less; usually takes only a few milliseconds or microseconds.
// Returns true iff the user interacted with the system, and maybe caused a status change.
// NOTE: since this is on the minimum idle-loop code path, minimise CPU cycles, esp in frost mode.
// Also re-activates CLI on main button push.
bool tickUI(uint_fast8_t sec)
  {
  bool statusChange = false;

  const bool sec0 = (0 == sec);

#ifdef SUPPORT_BAKE
  // Run down BAKE mode timer if need be, one tick per minute.
  if(sec0 && (bakeCountdownM > 0)) { --bakeCountdownM; }
#endif

  // Run down occupation timer if need be.
  if(sec0 && (occupationCountdownM > 0)) { --occupationCountdownM; }

  // Time out CLI activation if need be, one tick per second.
  if(CLISecondsLeft != 0)
    {
    // Decrements two at a time (because routine called every two seconds) but avoids underflow.
    if((CLISecondsLeft -= 2) < 0) { CLISecondsLeft = 0; }
    }

  if(fastDigitalRead(BUTTON_MODE_L) == LOW)
    {
    // User has pressed button: cycle through frost | warm [ | bake ] states.
    statusChange = true;
    // Also keep CLI active for a while longer.
    resetCLIActiveTimer();
    // LED on...
    fastDigitalWrite(LED_HEATCALL, HIGH);
    if(!isWarmMode) // Was in frost mode; moving to warm mode.
      {
      isWarmMode = true;
      markAsOccupied(); // Mark room as currently occupied also.
#ifdef SUPPORT_BAKE
      cancelBake(); // Ensure no bake mode running.
#endif
      tinyPause(); // 2 x tiny flash 'heat call' to indicate now in WARM mode.
      fastDigitalWrite(LED_HEATCALL, LOW);
      offPause();
      fastDigitalWrite(LED_HEATCALL, HIGH);
      tinyPause();
      }
#ifdef SUPPORT_BAKE
    else if(!inBakeMode()) // Was in WARM mode, move to BAKE (with full timeout to run).
      {
      startBake();
      markAsOccupied(); // Mark room as currently occupied also.
      tinyPause(); // 3 x tiny flash 'heat call' to indicate now in BAKE mode.
      fastDigitalWrite(LED_HEATCALL, LOW);
      offPause();
      fastDigitalWrite(LED_HEATCALL, HIGH);
      tinyPause();
      fastDigitalWrite(LED_HEATCALL, LOW);
      offPause();
      fastDigitalWrite(LED_HEATCALL, HIGH);
      tinyPause();
      }
#endif
    else // Was in BAKE (if supported, else was in WARM), move to FROST.
      {
      isWarmMode = false;
      tinyPause(); // 1 x tiny flash 'heat call' to indicate now in FROST mode.
      }
    }
  else
    {
    const bool forthTick = !(sec & 6); // True on every 4th tick.

    // Mode button not pressed: indicate current mode with flash(es); more flashes if actually calling for heat.
    if(isWarmMode) // Generate flash(es) if in WARM mode.
      {
      // DHD20131223: do not flash if the room is dark so as to save energy and avoid disturbing sleep, etc.
      // In this case force resample of light level frequently in case user turns light on eg to operate unit.
      if(!isRoomDark() || (forthTick && (0 != readAmbientLight()) && !isRoomDark()))
        {
//#ifdef LEARN_BUTTON_AVAILABLE
//        if((!forthTick) || isSimpleScheduleSet()) // Omit every 4th set of flashes unless a schedule is set.
//#endif
          {
          // First flash to indicate WARM mode.
          fastDigitalWrite(LED_HEATCALL, HIGH);
          tinyPause();
  
          // Second flash to indicate calling for heat.
          if(getTRVPercentOpen() != 0)
            {
            fastDigitalWrite(LED_HEATCALL, LOW);
            offPause(); // V0.09 was mediumPause().
            fastDigitalWrite(LED_HEATCALL, HIGH); // flash
            tinyPause();
  
#ifdef SUPPORT_BAKE
            if(inBakeMode())
              {
              // Third flash to indicate BAKE mode.
              fastDigitalWrite(LED_HEATCALL, LOW);
              offPause(); // V0.09 was mediumPause().
              fastDigitalWrite(LED_HEATCALL, HIGH); // flash
              tinyPause();
              }
#endif
            }
          }
        }
#if 0 && defined(DEBUG)
      else { DEBUG_SERIAL_PRINTLN_FLASHSTRING("suppressed UI flash"); }
#endif
      }
 
    // Even in FROST mode, if calling for heat (eg opening the rad valve) emit tiny double flash on every 4th tick.
    // This call for heat may be frost protection or pre-warming / anticipating demand.
    // DHD20130528: new 4th-tick flash in FROST mode...
    // DHD20131223: do not flash if the room is dark so as to save energy and avoid disturbing sleep, etc.
    else if(forthTick && !isRoomDark() && (0 != getTRVPercentOpen()))
      {
      // Double flash every 4th tick indicates call for heat while in FROST MODE (matches call for heat in WARM mode).
      fastDigitalWrite(LED_HEATCALL, HIGH); // flash
      tinyPause();
      fastDigitalWrite(LED_HEATCALL, LOW);
      offPause(); // V0.09 was mediumPause().
      fastDigitalWrite(LED_HEATCALL, HIGH); // flash
      tinyPause();
      }
    }

  // Ensure LED forced off/LOW at least once each cycle.
  fastDigitalWrite(LED_HEATCALL, LOW);

#ifdef LEARN_BUTTON_AVAILABLE
  // Handle learn button if supported and if is currently pressed.
  if(fastDigitalRead(BUTTON_LEARN_L) == LOW)
    {
    statusChange = true;
    handleLearnButton();
    markAsOccupied(); // Mark room as currently occupied also.
    fastDigitalWrite(LED_HEATCALL, HIGH); // Leave heatcall LED on while learn button held down.
    }
#endif

  return(statusChange);
  }


// Check/apply the user's schedule, at least once each minute, and act on any timed events.
void checkUserSchedule()
  {
  // Get minutes since midnight local time [0,1439].
  const uint_least16_t msm = getMinutesSinceMidnightLT();

  // Check if now is the simple scheduled off time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
  // Programmed off/frost takes priority over on/warm if same to bias towards energy-saving.
  if(msm == getSimpleScheduleOff())
    { isWarmMode = false; }
  // Check if now is the simple scheduled on time.
  else if(msm == getSimpleScheduleOn())
    { isWarmMode = true; }
  }


// Prints a single space to Serial (which must be up and running).
static void Serial_print_space() { Serial.print(' '); }

// Sends a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
// Ideally should be similar to PICAXE V0.1 output to allow the same parser to handle either.
// Will turn on UART just for the duration of this call if powered off.
/*
Status output may look like this...
=F0%@18C;T16 36 W255 0 F255 0;C5 5 17
=W0%@18C;T16 38 W255 0 F255 0;C5 5 17
=W0%@18C;T16 39 W255 0 F255 0;C5 5 17
=W0%@18C;T16 40 W16 39 F17 39;C5 5 17
=W0%@18C;T16 41 W16 39 F17 39;C5 5 17
=W0%@17C;T16 42 W16 39 F17 39;C5 5 17
=W20%@17C;T16 43 W16 39 F17 39;C5 5 17
=W20%@17C;T16 44 W16 39 F17 39;C5 5 17
=F0%@17C;T16 45 W16 39 F17 39;C5 5 17

When driving an FHT8V wireless radiator valve it may look like this:
=F0%@18C;T2 30 W10 0 F12 0;C5 5 17 wf;HC255 255
=F0%@18C;T2 30 W10 0 F12 0;C5 5 17 wf;HC255 255
=W0%@18C;T2 31 W10 0 F12 0;C5 5 17 wf;HC255 255
=W10%@18C;T2 32 W10 0 F12 0;C5 5 17 wf;HC255 255
=W20%@18C;T2 33 W10 0 F12 0;C5 5 17 wfo;HC255 255

'=' starts the status line and CRLF ends it; sections are separated with ";".
The initial 'W' or 'F' is WARM or FROST mode indication.  (If BAKE mode is supported, 'B' may be shown instead of 'W' when in BAKE.)
The nn% is the target valve open percentage.
The @nnCh gives the current measured room temperature in (truncated, not rounded) degrees C, followed by hex digit for 16ths.
The ";" terminates this initial section.
Thh mm is the local current 24h time in hours and minutes.
Whh mm is the scheduled on/warm time in hours and minutes, or an invalid time if none.
Fhh mm is the scheduled off/frost time in hours and minutes, or an invalid time if none.
The ";" terminates this schedule section.
'C' introduces the current and settable-target temperatures in Celsius/centrigrade, if supported.
eg 'C5 5 17'
The first number is the current target in C, the second is the FROST target, the third is the WARM target.
The 'e' or 'c' indicates eco or comfort bias.
A 'w' indicates that this hour is predicted for smart warming ('f' indocates not), and another 'w' the hour ahead.
A trailing 'o' indicates room occupancy.
The ";" terminates this current/Celsius section.
'HC' introduces the FHT8V house codes, if supported.
eg 'HC255 255'
HChc1 hc2 are the house codes 1 and 2 for an FHT8V valve.
*/
void serialStatusReport()
  {
  const bool neededWaking = powerUpSerialIfDisabled();

  // Aim to overlap CPU usage with characters being TXed for throughput determined primarily by output size and baud.

  Serial.print('=');
#ifdef SUPPORT_BAKE
  Serial.print(isWarmMode ? (inBakeMode() ? 'B' : 'W') : 'F');
#else
  Serial.print(isWarmMode ? 'W' : 'F');
#endif
  Serial.print(getTRVPercentOpen()); Serial.print('%'); // Target valve position.
  const int temp = getTemperatureC16();
  Serial.print('@'); Serial.print(temp >> 4); Serial.print('C'); // Unrounded whole degrees C.
      Serial.print(temp & 0xf, HEX); // Show 16ths in hex.

  Serial.print(';'); // End of initial section.
  const uint_least8_t hh = getHoursLT();
  const uint_least8_t mm = getMinutesLT();
  Serial.print('T'); Serial.print(hh); Serial_print_space(); Serial.print(mm);
  Serial_print_space();
  uint_least16_t startMinutesSinceMidnightLT = getSimpleScheduleOn();
  const bool invalidStartTime = startMinutesSinceMidnightLT >= 1440;
  const int startH = invalidStartTime ? 255 : (startMinutesSinceMidnightLT / 60);
  const int startM = invalidStartTime ? 0 : (startMinutesSinceMidnightLT % 60);
  Serial.print('W'); Serial.print(startH); Serial_print_space(); Serial.print(startM);
  Serial_print_space();
  uint_least16_t endMinutesSinceMidnightLT = getSimpleScheduleOff();
  const bool invalidEndTime = endMinutesSinceMidnightLT >= 1440;
  const int endH = invalidEndTime ? 255 : (endMinutesSinceMidnightLT / 60);
  const int endM = invalidEndTime ? 0 : (endMinutesSinceMidnightLT % 60);
  Serial.print('F'); Serial.print(endH); Serial_print_space(); Serial.print(endM);

#ifdef SETTABLE_TARGET_TEMPERATURES // Show thresholds and current target since no longer so easily deduced.
  Serial.print(';'); // Terminate previous section.
  Serial.print('C'); // Current Celsius target, and FROST and WARM settings.
  Serial.print(getTargetTempC());
  Serial_print_space();
  Serial.print(getFROSTTargetC());
  Serial_print_space();
  Serial.print(getWARMTargetC());
  Serial_print_space();
  // Show bias.
  Serial.print(hasEcoBias() ? 'e' : 'c'); // Show eco/comfort bias.
  // Show warming predictions.
  Serial.print(shouldBeWarmedAtHour(hh) ? 'w' : 'f');
  Serial.print(shouldBeWarmedAtHour(hh < 23 ? (hh+1) : 0) ? 'w' : 'f');
  // Show occupancy if known.
  if(isLikelyOccupied()) { Serial.print('o'); } // Show room occupied.
#endif

#ifdef ENABLE_BOILER_HUB
  Serial.print(';'); // Terminate previous section.
  Serial.print('C'); // Indicate central hub mode available.
  Serial.print(getMinBoilerOnMinutes()); // Show min 'on' time, or zero if disabled.
#endif

#if defined(USE_MODULE_FHT8VSIMPLE)
  Serial.print(';'); // Terminate previous section.
  Serial.print(F("HC"));
  Serial.print(FHT8VGetHC1());
  Serial_print_space();
  Serial.print(FHT8VGetHC2());
  if(!isSyncedWithFHT8V())
    {
    Serial_print_space();
    Serial.print('s'); // Indicate syncing with trailing lower-case 's' in field...
    }
#endif
  Serial.println();

  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  flushSerialSCTSensitive();

  if(neededWaking) { powerDownSerial(); }
  }

#define SYNTAX_COL_WIDTH 11 // Width of 'syntax' column; strictly positive.
#define STOP_PRINTING_DESCRIPTION_AT (GSCT_MAX-(GSCT_MAX/8)) // Time into minor cycle after which the description should be skipped.
// Efficiently print a single line given the syntax element and the description, both non-null.
// NOTE: will skip the description if getting close to the end of a minor cycle to avoid overrun risk.
static void printCLILine(__FlashStringHelper const *syntax, __FlashStringHelper const *description)
  {
  Serial.print(syntax);
  Serial.flush();
  if(getSubCycleTime() >= STOP_PRINTING_DESCRIPTION_AT) { Serial.println(); return; }
  for(int8_t padding = SYNTAX_COL_WIDTH - strlen_P((const char *)syntax); --padding >= 0; ) { Serial_print_space(); }
  Serial.println(description);
  }
// Efficiently print a single line given a single-char syntax element and the description, both non-null.
// NOTE: will skip the description if getting close to the end of a minor cycle to avoid overrun risk.
static void printCLILine(char syntax, __FlashStringHelper const *description)
  {
  Serial.print(syntax);
  Serial.flush();
  if(getSubCycleTime() >= STOP_PRINTING_DESCRIPTION_AT) { Serial.println(); return; }
  for(int8_t padding = SYNTAX_COL_WIDTH - 1; --padding >= 0; ) { Serial_print_space(); }
  Serial.println(description);
  }

// Dump some brief CLI usage instructions to serial TX, which must be up and running.
// If this gets too big there is a risk of overrunning and missing the next tick...
static void dumpCLIUsage()
  {
  Serial.println();
  Serial.println(F("CLI usage:"));
  printCLILine('?', F("this help"));
  printCLILine('B X', F("Bias E (Eco) or C (Comfort)"));
#ifdef ENABLE_BOILER_HUB
  printCLILine(F("C M"), F("central hub: minimum M mins on, 0 disabled"));
#endif
  printCLILine(F("D N"), F("Dump stats set N"));
  printCLILine('E', F("Exit CLI"));
  printCLILine('F', F("Frost"));
#ifdef SETTABLE_TARGET_TEMPERATURES
  printCLILine(F("F CC"), F("set Frost temp CC"));
#endif
#if defined(USE_MODULE_FHT8VSIMPLE) && defined(LOCAL_TRV)
  printCLILine(F("H"), F("clear wireless FHT8V House codes"));
  printCLILine(F("H H1 H2"), F("set wireless FHT8V House codes 1&2"));
#endif
  printCLILine('L', F("Learn to warm every 24h from now, else cancel schedule if in frost mode"));
  printCLILine(F("P HH MM LL"), F("Program: warm daily starting at HH MM for LL hours"));
#ifdef SUPPORT_BAKE
  printCLILine('Q', F("Quick Heat (BAKE)"));
#endif
  printCLILine(F("R N"), F("dump Raw stats set N"));
  printCLILine('S', F("show Status and smart warming for next 24h"));
  printCLILine(F("T HH MM"), F("set 24h Time"));
  printCLILine('W', F("Warm"));
#ifdef SETTABLE_TARGET_TEMPERATURES
  printCLILine(F("W CC"), F("set Warm temp CC"));
#endif
  printCLILine('Z', F("Zap stats"));
  Serial.println();
  }

// Prints warning to serial (that must be up and running) that invalid (CLI) input has been ignored.
// Probably should not be inlined, to avoid creating duplicate strings in Flash.
static void InvalidIgnored() { Serial.println(F("Invalid, ignored.")); }

#define MAXIMUM_CLI_RESPONSE_CHARS 10 // Just enough for any valid command expected not including trailing CR.  (Note that Serial RX buffer is 64 bytes.)
#define IDLE_SLEEP_SCT (15/SUBCYCLE_TICK_MS_RD) // Approx sub-cycle ticks in idle sleep (15ms), erring on side of being too large; strictly positive.
#define BUF_FILL_TIME_MS (((MAXIMUM_CLI_RESPONSE_CHARS*10) * 1000 + (BAUD-1)) / BAUD) // Time to read full/maximal input command buffer; ms, strictly positive.
#define BUF_FILL_TIME_SCT (BUF_FILL_TIME_MS/SUBCYCLE_TICK_MS_RD) // Approx sub-cycle ticks to fill buf, erring on side of being too large; strictly positive.
#define MIN_POLL_SCT max(IDLE_SLEEP_SCT, BUF_FILL_TIME_SCT)
#define MIN_RX_BUFFER 16 // Minimum Arduino Serial RX buffer size.
// DHD20131213: CAN_IDLE_15MS true seemed to be causing intermittent crashes.
#ifdef ENABLE_AVR_IDLE_MODE
#define CAN_IDLE_15MS ((BAUD <= 4800) || (MAXIMUM_CLI_RESPONSE_CHARS < MIN_RX_BUFFER)) // If true, cannot get RX overrun during 30ms idle.
#else
#define CAN_IDLE_15MS (false)
#endif
// Used to poll user side for CLI input until specified sub-cycle time.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// NOT RENTRANT (eg uses static state for speed and code space).
void pollCLI(const uint8_t maxSCT)
  {
  // Compute safe limit time given granularity of sleep and buffer fill.
  const uint8_t targetMaxSCT = (maxSCT <= MIN_POLL_SCT) ? 0 : (maxSCT - 1 - MIN_POLL_SCT);
  if(getSubCycleTime() >= targetMaxSCT) { return; } // Too short to try.

  const bool neededWaking = powerUpSerialIfDisabled();

  // Purge any stray pending input, such as a trailing LF from previous input.
  while(Serial.available() > 0) { Serial.read(); }

  // Generate and flush prompt character to the user, after a CRLF to reduce ambiguity.
  Serial.println();
  Serial.print(CLIPromptChar);
  // Idle a short while to try to save energy, waiting for serial TX end and possible RX response start.
  flushSerialSCTSensitive();


  // Wait for input command line from the user (received characters may already have been queued)...
  // Read a line up to a terminating CR, either on its own or as part of CRLF.
  // (Note that command content and timing may be useful to fold into PRNG entropy pool.)
  static char buf[MAXIMUM_CLI_RESPONSE_CHARS+1]; // Note: static state, efficient for small command lines.  Space for terminating '\0'.
  //Serial.setTimeout(timeoutms); const int n = Serial.readBytesUntil('\r', buf, MAXIMUM_CLI_RESPONSE_CHARS);
  uint8_t n = 0;
  while(n < MAXIMUM_CLI_RESPONSE_CHARS)
    {
    // Read next character if immediately available.
    if(Serial.available() > 0)
      {
      const int ic = Serial.read();
      if(('\r' == ic) || ('\n' == ic)) { break; } // Stop at CR, eg from CRLF, or LF.
      buf[n++] = (char) ic;
      continue;
      }
    // Quit WITHOUT PROCESSING THE POSSIBLY-INCOMPLETE INPUT if time limit is hit (or very close).
    const uint8_t sct = getSubCycleTime();
    if(sct >= targetMaxSCT)
      {
      n = 0;
      break;
      }
    // Idle waiting for input, to save power, then/else do something useful with some CPU cycles...
#if CAN_IDLE_15MS
    idle30AndPoll(); // Minimise power consumption if safe for RX overrun.
#else
    burnHundredsOfCyclesProductivelyAndPoll(); // Use time time to poll for I/O, etc.
#endif
    }

  if(n > 0)
    {
    // Restart the CLI timer on receipt of plausible (ASCII) input (cf noise from UART floating or starting up),
    // Else print a very brief low-CPU-cost help message and give up as efficiently and quickly as possible.
    const char firstChar = buf[0];
    const bool plausibleCommand = ((firstChar >= '?') && (firstChar <= 'z'));
    if(plausibleCommand) { resetCLIActiveTimer(); }
    else { Serial.println(F("? for CLI help")); flushSerialSCTSensitive(); return; }

    // Null-terminate the command line.
    buf[n] = '\0';

    // Force to upper-case and echo the line received.
    strupr(buf);
    Serial.println(buf);

    // Process the input received, with action based on the first char...
    bool showStatus = true; // Default to showing status.
    switch(buf[0])
      {
      // Explicit request for help, or unrecognised first character.
      // Avoid showing status as may already be rather a lot of output.
      default: case '?': { dumpCLIUsage(); showStatus = false; break; }

#ifdef SUPPORT_BAKE
      // Set bias: B E or B C
      case 'B':
        {
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "B E".
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          // Anything other than 'C' (or 'c') forces eco mode.
          const bool isEco = 'C' != *tok1;
          setEcoBias(isEco);
          }

        break;
        }
#endif

#ifdef ENABLE_BOILER_HUB
      // C M
      // Set central-hub boiler minimum on (and off) time; 0 to disable.
      case 'C':
        {
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "C 0".
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t m = (uint8_t) atoi(tok1);
          setMinBoilerOnMinutes(m);
          }
        break;
        }
#endif

      // Exit/deactivate CLI immediately.
      case 'E': { CLISecondsLeft = 0; break; }

      // Raw stats: R N
      // Avoid showing status afterwards as may already be rather a lot of output.
      case 'R':
        {
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "R 0".
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t setN = (uint8_t) atoi(tok1);
          for(uint8_t hh = 0; hh < 24; ++hh)
            { Serial.print(getByHourStat(hh, setN)); Serial_print_space(); }
          Serial.println();
          }
        break;
        }

      // Dump (human-friendly) stats: D N
      // Avoid showing status afterwards as may already be rather a lot of output.
      case 'D':
        {
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "D 0".
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t setN = (uint8_t) atoi(tok1);
          const uint8_t thisHH = getHoursLT();
          const uint8_t lastHH = (thisHH > 0) ? (thisHH-1) : 23;
          for(uint8_t hh = 0; hh < 24; ++hh)
            {
            const uint8_t statRaw = getByHourStat(hh, setN);
            // For unset stat show '-'...
            if(STATS_UNSET_BYTE == statRaw) { Serial.print('-'); }
            // ...else print more human-friendly version of stat.
            else switch(setN) // Relationship between stats set and type should probably be centralised to avoid getting out of sync with usage.
              {
              case 0: case 1: { Serial.print((expandTempC16(statRaw)+8) >> 4); Serial.print('C'); break; } // Uncompanded temperature, rounded.
              case 2: case 3: { Serial.print(((int)statRaw) << 2); break; } // Uncompressed ambient light level.
              case 4: { Serial.print(statRaw, HEX); break; } // Warm mode usage over week.
              }
            if(hh == lastHH) { Serial.print('<'); } // Highlight most recent stat in this set.
            Serial_print_space();
            }
          Serial.println();
          }

        showStatus = false;
        break;
        }

      // Switch to FROST mode OR set FROST temperature.
      case 'F':
        {
#ifdef SETTABLE_TARGET_TEMPERATURES
        char *last; // Used by strtok_r().
        char *tok1;
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t tempC = (uint8_t) atoi(tok1);
          if(!setFROSTTargetC(tempC)) { InvalidIgnored(); }
          }
        else
#endif
          { isWarmMode = false; } // No parameter supplied; switch to FROST mode.
        break;
        }

#if defined(USE_MODULE_FHT8VSIMPLE) && defined(LOCAL_TRV)
      // Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
      // Missing values will clear the code entirely (and disable use of the valve).
      case 'H':
        {
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 5 character sequence makes sense and is safe to tokenise, eg "H 1 2".
        if((n >= 5) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          char *tok2 = strtok_r(NULL, " ", &last);
          if(NULL != tok2)
            {
            const int hc1 = atoi(tok1);
            const int hc2 = atoi(tok2);
            if((hc1 < 0) || (hc1 > 99) || (hc2 < 0) || (hc2 > 99)) { InvalidIgnored(); }
            else
              {
              FHT8VSetHC1(hc1);
              FHT8VSetHC2(hc2);
              FHT8VSyncAndTXReset(); // Force re-sync with FHT8V valve.
              }
            }
          }
        else if(n < 2) // Just 'H', possibly with trailing whitespace.
          {
          FHT8VClearHC();
          FHT8VSyncAndTXReset(); // Force into unsynchronized state.
          }
        break;
        }
#endif

      // Learn current settings, just as if LEARN button had been pressed.
      case 'L': { handleLearnButton(); break; }

      // Program simple schedule HH MM DD.
      case 'P':
        {
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 7 character sequence makes sense and is safe to tokenise, eg "P 1 2 3".
        if((n >= 7) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          char *tok2 = strtok_r(NULL, " ", &last);
          if(NULL != tok2)
            {
            char *tok3 = strtok_r(NULL, " ", &last);
            if(NULL != tok3)
              {
                const int hh = atoi(tok1);
                const int mm = atoi(tok2);
                const int dd = atoi(tok3);
                // Does not fully validate user inputs (eg for -ve values), but cannot set impossible values.
                if(!setSimpleSchedule((uint_least16_t) ((60 * hh) + mm), (uint_least16_t) (dd * 60))) { InvalidIgnored(); }
              }
            }
          }
        break;
        }

#ifdef SUPPORT_BAKE
      // Switch to (or restart) BAKE (Quick Heat) mode: Q
      case 'Q': { startBake(); break; }
#endif

      // Status line and smart/scheduled warming prediction request.
      case 'S':
        {
        Serial.print(F("Reset count: "));
        const uint8_t resetCount = eeprom_read_byte((uint8_t *)EE_START_RESET_COUNT);
        Serial.print(resetCount);
        Serial.println();
        uint_least8_t hh = getHoursLT();
        Serial.print(F("Smart warming: "));
        for(int i = 24; --i >= 0; )
          {
          Serial.print(shouldBeWarmedAtHour(hh) ? 'w' : 'f'); // TODO: show 'W' for scheduled WARM mode.
          if(++hh > 23) { hh = 0; }
          }
        Serial.println();
        break; // Note that status is by default printed after processing input line.
        }

      // Time set T HH MM.
      case 'T':
        {
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 5 character sequence makes sense and is safe to tokenise, eg "T 1 2".
        if((n >= 5) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          char *tok2 = strtok_r(NULL, " ", &last);
          if(NULL != tok2)
            {
            const int hh = atoi(tok1);
            const int mm = atoi(tok2);
            // TODO: zap collected stats if time change too large (eg >> 1h).
            if(!setHoursMinutesLT(hh, mm)) { InvalidIgnored(); }
            }
          }
        break;
        }

      // Switch to WARM (not BAKE) mode OR set WARM temperature.
      case 'W':
        {
#ifdef SETTABLE_TARGET_TEMPERATURES
        char *last; // Used by strtok_r().
        char *tok1;
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t tempC = (uint8_t) atoi(tok1);
          if(!setWARMTargetC(tempC)) { InvalidIgnored(); }
          }
        else
#endif
          {
          isWarmMode = true; // No parameter supplied; switch to WARM mode.
#ifdef SUPPORT_BAKE
          cancelBake(); // Ensure BAKE mode not entered.
#endif
          }
        break;
        }

      // Zap/erase learned statistics.
      case 'Z':
        {
        // Try to avoid causing an overrun if near the end of the minor cycle (even allowing for the warning message if unfinished!).
        if(zapStats((uint16_t) fnmax(1, ((int)msRemainingThisBasicCycle()/2) - 20)))
          { Serial.println(F("Zapped.")); }
        else
          { Serial.println(F("Not finished.")); }
        showStatus = false; // May be slow; avoid showing stats line which will in any case be unchanged.
        break;
        }
      }

    // Almost always show status line afterwards as feedback of command received and new state.
    if(showStatus) { serialStatusReport(); }
    }
  else { Serial.println(); } // Terminate empty CLI input line after timeout.

  // Force any pending output before return / possible UART power-down.
  flushSerialSCTSensitive();

  if(neededWaking) { powerDownSerial(); }
  }

