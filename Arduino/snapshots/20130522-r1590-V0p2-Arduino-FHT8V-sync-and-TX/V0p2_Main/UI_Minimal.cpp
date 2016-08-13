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
#include "Control.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "Power_Management.h"
#include "RTC_Support.h"
#include "Schedule.h"
#include "Serial_Debug.h"
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



// Remaining seconds to keep CLI active; zero implies inactive.
// Starts up with full value to allow easy set of time, etc, without specially activating CLI.
static int8_t CLISecondsLeft = CLIActiveS;

// Reset CLI active timer to the full whack before it goes inactive again (ie makes CLI active for a while).
#define resetCLIActiveTimer() { CLISecondsLeft = CLIActiveS; }

// Returns true if the CLI is active, at least intermittently.
bool isCLIActive() { return(0 != CLISecondsLeft); }


#ifdef LEARN_BUTTON_AVAILABLE
// Period in minutes for simple learned on-time; strictly positive (and less than 1440).
#define LEARNED_ON_PERIOD_M 120
// Handle learn button.
// In simple mode: if in frost mode clear simple schedule else set repeat for every 24h from now for 2h.
// May be called from pushbutton or CLI UI components.
static void handleLearnButton()
  {
  // Set simple schedule starting every 24h from now and running 120 minutes.  
  if(isWarmMode) { setSimpleSchedule(getMinutesSinceMidnightLT(), LEARNED_ON_PERIOD_M); }
  // Clear simple schedule.
  else { clearSimpleSchedule(); }
  }
#endif


// Call this on even numbered seconds (with current time in seconds) to allow the UI to operate.
// Should never be skipped, so as to allow the UI to remain responsive.
// Runs in 350ms or less; usually takes only a few milliseconds or microseconds.
// Returns true iff the user interacted with the system, and maybe caused a status change.
// NOTE: since this is on the minimum idle-loop code path, minimise CPU cycles, esp in frost mode.
// Also re-activates CLI on main button push.
bool tickUI(uint_fast8_t sec)
  {
  bool statusChange = false;

  // Run down BAKE mode timer if need be, one tick per minute.
#ifdef SUPPORT_BAKE
  if((0 == sec) && (bakeCountdownM > 0)) { --bakeCountdownM; }
#endif

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
    // Also re-activate CLI for a while.
    resetCLIActiveTimer();
    // LED on...
    fastDigitalWrite(LED_HEATCALL, HIGH);
    if(!isWarmMode) // Was in frost mode; moving to warm mode.
      {
      isWarmMode = true;
#ifdef SUPPORT_BAKE
      cancelBake(); // Ensure no bake mode running.
#endif
      bigPause(); // Long flash 'heat call' to indicate now in warm mode.   
      }
#ifdef SUPPORT_BAKE
    else if(!inBakeMode()) // Was in WARM mode, move to BAKE (with full timeout to run).
      {
      startBake();
      bigPause(); // Long then tiny flash 'heat call' to indicate now in bake mode.
      fastDigitalWrite(LED_HEATCALL, LOW);
      offPause(); // V0.09 was mediumPause().
      fastDigitalWrite(LED_HEATCALL, HIGH);
      tinyPause(); // V0.09 was mediumPause().
      }
#endif
    else // Was in BAKE (if supported, else was in WARM), move to FROST.
      {
      isWarmMode = false;
      mediumPause(); // Medium flash 'heat call' to indicate now in frost mode.
      }
    }
  else
    {
    // Button not pressed: quickly indicate current mode with flash(es), then optional further flash if actually calling for heat.

    if(isWarmMode) // Basically only generate any flash (and consume power) at all if in warm mode.
      {
#ifdef LEARN_BUTTON_AVAILABLE
      const bool learnActive = (!(sec & 6)) && isSimpleScheduleSet(); // Do something different every 4th time.
      if(learnActive) { pinMode(LED_HEATCALL, INPUT_PULLUP); } // Have LED dim: HIGH => weak pull-up, LOW means hi-Z.
#else     
      const bool learnActive = false; // No learn/schedule.
#endif

      fastDigitalWrite(LED_HEATCALL, HIGH); // Flash 'heat call' to indicate heating mode.

      // TODO: this tinyPause() could be topping up an underful entropy pool...
      tinyPause();

      // Display representation of internal heat-demand value iff in warm mode to avoid confusion.
      if(getTRVPercentOpen() != 0)
        {
        fastDigitalWrite(LED_HEATCALL, LOW);
        offPause(); // V0.09 was mediumPause().
        fastDigitalWrite(LED_HEATCALL, HIGH); // flash
        tinyPause();

#ifdef SUPPORT_BAKE
        if(inBakeMode()) // Third flash if in 'bake' mode.
          {
          fastDigitalWrite(LED_HEATCALL, LOW);
          offPause(); // V0.09 was mediumPause().
          fastDigitalWrite(LED_HEATCALL, HIGH); // flash
          tinyPause();
          }
#endif
        }

#ifdef LEARN_BUTTON_AVAILABLE
      if(learnActive) { pinMode(LED_HEATCALL, OUTPUT); } // Revert LED to full brightness subsequently.
#endif
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
    fastDigitalWrite(LED_HEATCALL, HIGH); // Leave heatcall LED on while learn button held down.
    }
#endif

  return(statusChange);
  }


// Check the user's schedule, at least once each minute, and act on any timed events.
void checkUserSchedule()
  {
  // Get minutes since midnight local time [0,1439].
  const uint_least16_t msm = getMinutesSinceMidnightLT();

  // Get the simple schedule off time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
  // Programmed off/frost takes priority over on/warm if same to bias towards energy-saving.
  if(msm == getSimpleScheduleOff())
    { isWarmMode = false; }
  else if(msm == getSimpleScheduleOn())
    { isWarmMode = true; }
  }


// Prints a single space to Serial (which must be up and running).
static void Serial_print_space() { Serial.print(' '); }

// Sends a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
// Should be similar to PICAXE V0.1 output to allow the same parser to handle either.
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
=F0%@18C;T2 30 W10 0 F12 0;C5 5 17;HC255 255
=F0%@18C;T2 30 W10 0 F12 0;C5 5 17;HC255 255
=W0%@18C;T2 31 W10 0 F12 0;C5 5 17;HC255 255
=W10%@18C;T2 32 W10 0 F12 0;C5 5 17;HC255 255
=W20%@18C;T2 33 W10 0 F12 0;C5 5 17;HC255 255

'=' starts the status line and CRLF ends it; sections are separated with ";".
The initial 'W' or 'F' is WARM or FROST mode indication.  (If BAKE mode is supported, 'B' may be shown instead of 'W' when in BAKE.)
The nn% is the target valve open percentage.
The @nnC gives the current measured room temperature in (truncated, not rounded) degrees C.
The ";" terminates this initial section.
Thh mm is the local current 24h time in hours and minutes.
Whh mm is the scheduled on/warm time in hours and minutes, or an invalid time if none.
Fhh mm is the scheduled off/frost time in hours and minutes, or an invalid time if none.
The ";" terminates this schedule section.
'C' introduces the current and settable-target temperatures in Celsius/centrigrade, if supported.
eg 'C5 5 17'
The first number is the current target in C, the second is the FROST target, the third is the WARM target.
The ";" terminates this current/Celsius section.
'HC' introduces the FHT8V house codes, if supported.
eg 'HC255 255'
HChc1 hc2 are the house codes 1 and 2 for an FHT8V valve.
*/
void serialStatusReport()
  {
  const bool neededWaking = powerUpSerialIfDisabled();

  Serial.print('=');
#ifdef SUPPORT_BAKE
  Serial.print(isWarmMode ? (inBakeMode() ? 'B' : 'W') : 'F');
#else
  Serial.print(isWarmMode ? 'W' : 'F');
#endif
  Serial.print(getTRVPercentOpen()); Serial.print('%'); // Target Valve position.
  Serial.print('@'); Serial.print((getTemperatureC16() + 8) >> 4); Serial.print('C'); // Round to nearest.
  Serial.print(';'); // End of initial section.
  Serial.print('T'); Serial.print(getHoursLT()); Serial_print_space(); Serial.print(getMinutesLT());
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

  // Ensure that all text is sent before this routine returns, in case a sleep follows that kills the UART.
  Serial.flush();

  if(neededWaking) { powerDownSerial(); }
  }

#define SYNTAX_COL_WIDTH 11 // Width of 'syntax' column; strictly positive.
#define STOP_PRINTING_DESCRIPTION_AT (GSCT_MAX-(GSCT_MAX/8)) // Time into minor cycle to drop description.
// Efficiently print a single line given the syntax element and the description, both non-null.
// NOTE: will skip the description if getting close to the end of a minor cycle to avoid overrun risk.
static void printCLILine(__FlashStringHelper const *syntax, __FlashStringHelper const *description)
  {
  Serial_print_space();
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
  Serial_print_space();
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
  Serial.println(F("CLI usage:"));
  printCLILine('?', F("this help"));
#ifdef SUPPORT_BAKE
  printCLILine('B', F("Bake"));
#endif
  printCLILine('E', F("Exit CLI"));
  printCLILine('F', F("Frost"));
#ifdef SETTABLE_TARGET_TEMPERATURES
  printCLILine(F("F CC"), F("set Frost temp CC"));
#endif
#if defined(USE_MODULE_FHT8VSIMPLE)
  printCLILine(F("H H1 H2"), F("set wireless FHT8V House codes 1&2"));
#endif
  printCLILine('L', F("Learn to warm every 24h from now, else cancel schedule if in frost mode"));
  printCLILine(F("P HH MM LL"), F("Program: warm daily starting at HH MM for LL hours"));
  printCLILine('S', F("show Status"));
  printCLILine(F("T HH MM"), F("set 24h Time"));
  printCLILine('W', F("Warm"));
#ifdef SETTABLE_TARGET_TEMPERATURES
  printCLILine(F("W CC"), F("set Warm temp CC"));
#endif
  Serial.println();
  }

// Prints warning to serial (that must be up and running) that invalid (CLI) input has been ignored.
// Probablu should nto be inlined, to avoid creating duplicate strings in Flash.
static void InvalidIgnored() { Serial.println(F("Invalid, ignored.")); }

// Used to poll user side for CLI input for at most approximately the number of milliseconds specified.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// NOT RENTRANT (eg uses static state for speed and code space).
#define MAXIMUM_CLI_RESPONSE_CHARS 10 // Just enough for any valid command expected not including trailing CR.  (Note that Serial RX buffer is 64 bytes.)
void pollCLI(int timeoutms)
  {
  const bool neededWaking = powerUpSerialIfDisabled();

  // Purge any stray pending input, such as a trailing LF from previous input.
  while(Serial.available() > 0) { Serial.read(); }

  // Generate and flush prompt character to the user, after a CRLF to reduce ambiguity.
  Serial.println();
  Serial.print(CLIPromptChar); 
  Serial.flush();

  // Wait for input from the user (received characters may already have been queued)...
  // Read a line up to a terminating CR, either on its own or as part of CRLF.
  // (Note that command content and timing may be useful to fold into PRNG entropy pool.)
  Serial.setTimeout(timeoutms);
  static char buf[MAXIMUM_CLI_RESPONSE_CHARS+1]; // Note: static state, efficient for small command lines.
  const int n = Serial.readBytesUntil('\r', buf, MAXIMUM_CLI_RESPONSE_CHARS);
  if(n > 0)
    {
    // Keep CLI active for a while.
    resetCLIActiveTimer();

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
      // Switch to (or restart) BAKE mode.
      case 'B': { startBake(); break; }
#endif

      // Exit/deactivate CLI immediately.
      case 'E': { CLISecondsLeft = 0; break; }

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

#if defined(USE_MODULE_FHT8VSIMPLE)
      // Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
      case 'H':
        {
        char *last; // Used by strtok_r().
        char *tok1 = strtok_r(buf+2, " ", &last);
        if(NULL != tok1)
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
        break;
        }
#endif

      // Learn current settings, just as if LEARN button had been pressed.
      case 'L': { handleLearnButton(); break; }

      // Program simple schedule HH MM DD.
      case 'P':
        {
        char *last; // Used by strtok_r().
        char *tok1 = strtok_r(buf+2, " ", &last);
        if(NULL != tok1)
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

      // Status line request; does nothing here as status is always printed after processing input line.
      case 'S': { break; }

      // Time set.
      case 'T':
        {
        char *last; // Used by strtok_r().
        char *tok1 = strtok_r(buf+2, " ", &last);
        if(NULL != tok1)
          {
          char *tok2 = strtok_r(NULL, " ", &last);
          if(NULL != tok2)
            {
            const int hh = atoi(tok1);
            const int mm = atoi(tok2);
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
      }
 
    // Almost always show status line afterwards as feedback of command received and new state.
    if(showStatus) { serialStatusReport(); }
    }

  // Force any pending output before return / possible UART power-down.
  Serial.flush();

  if(neededWaking) { powerDownSerial(); }
  }
