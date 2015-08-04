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
 Implementation of minimal UI using single LED and one or more momentary push-buttons, etc, plus CLI.
 */

#include <util/atomic.h>

#include <Arduino.h>
#include <string.h>

#include "V0p2_Main.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "Control.h"
#include "EEPROM_Utils.h"
#include "FHT8V_Wireless_Rad_Valve.h"
#include "Messaging.h"
#include "Power_Management.h"
#include "RTC_Support.h"
#include "Schedule.h"
#include "Serial_IO.h"
#include "UI_Minimal.h"



// Marked true if the physical UI controls are being used.
// Cleared at end of tickUI().
// Marked voilatile for thread-safe lock-free non-read-modify-write access to byte-wide value.
static volatile bool statusChange;

// If non-zero then UI controls have been recently manually/locally operated; counts down to zero.
// Marked volatile for thread-safe lock-free non-read-modify-write access to byte-wide value.
// Compound operations on this value must block interrupts.
#define UI_DEFAULT_RECENT_USE_TIMEOUT_M 31
#define UI_DEFAULT_VERY_RECENT_USE_TIMEOUT_M 2
static volatile uint8_t uiTimeoutM;

// Remaining minutes to keep CLI active; zero implies inactive.
// Starts up with full value to allow easy setting of time, etc, without specially activating CLI.
// Marked volatile for thread-safe lock-free non-read-modify-write access to byte-wide value.
// Compound operations on this value must block interrupts.
#define CLI_DEFAULT_TIMEOUT_M 2
static volatile uint8_t CLITimeoutM = CLI_DEFAULT_TIMEOUT_M;

// Reset CLI active timer to the full whack before it goes inactive again (ie makes CLI active for a while).
// Thread-safe.
void resetCLIActiveTimer() { CLITimeoutM = CLI_DEFAULT_TIMEOUT_M; }

// Returns true if the CLI is active, at least intermittently.
// Thread-safe.
bool isCLIActive() { return(0 != CLITimeoutM); }

// Record local manual operation of a local physical UI control, eg not remote or via CLI.
// To be thread-safe, everything that this touches or calls must be.
// Thread-safe.
void markUIControlUsed()
  {
  statusChange = true; // Note user interaction with the system.
  uiTimeoutM = UI_DEFAULT_RECENT_USE_TIMEOUT_M; // Ensure that UI controls are kept 'warm' for a little while.
  // Make CLI active for a while (at some slight possibly-significant energy cost).
  resetCLIActiveTimer(); // Thread-safe.
  // User operation of controls locally is strong indication of presence.
  Occupancy.markAsOccupied(); // Thread-safe.
  }

// True if a manual UI control has been very recently (seconds to minutes ago) operated.
// The user may still be interacting with the control and the UI etc should probably be extra responsive.
// Thread-safe.
bool veryRecentUIControlUse() { return(uiTimeoutM >= (UI_DEFAULT_RECENT_USE_TIMEOUT_M - UI_DEFAULT_VERY_RECENT_USE_TIMEOUT_M)); }

// True if a manual UI control has been recently (tens of minutes ago) operated.
// If true then local manual settings should 'win' in any conflict with programmed or remote ones.
// For example, remote requests to override settings may be ignored while this is true.
// Thread-safe....
bool recentUIControlUse() { return(0 != uiTimeoutM); }



#ifdef LEARN_BUTTON_AVAILABLE
// Handle learn button(s).
// First/primary button is 0, second is 1, etc.
// In simple mode: if in frost mode clear simple schedule else set repeat for every 24h from now.
// May be called from pushbutton or CLI UI components.
static void handleLEARN(const uint8_t which)
  {
  // Set simple schedule starting every 24h from a little before now and running for an hour or so.  
  if(inWarmMode()) { setSimpleSchedule(getMinutesSinceMidnightLT(), which); }
  // Clear simple schedule.
  else { clearSimpleSchedule(which); }
  }
#endif


// Pause between flashes to allow them to be distinguished (>100ms); was mediumPause() for PICAXE V0.09 impl.
static void inline offPause()
  {
  bigPause(); // 120ms, was V0.09 144ms mediumPause() for PICAXE V0.09 impl.
  pollIO(); // Slip in an I/O poll.
  }

// Counts calls to tickUI.
static uint8_t tickCount;

// Call this on even numbered seconds (with current time in seconds) to allow the UI to operate.
// Should never be skipped, so as to allow the UI to remain responsive.
// Runs in 350ms or less; usually takes only a few milliseconds or microseconds.
// Returns true iff the user interacted with the system, and maybe caused a status change.
// NOTE: since this is on the minimum idle-loop code path, minimise CPU cycles, esp in frost mode.
// Also re-activates CLI on main button push.
bool tickUI(const uint_fast8_t sec)
  {
  // Perform any once-per-minute operations.
  const bool sec0 = (0 == sec);
  if(sec0)
    {
    ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
      {
      // Run down UI iteraction timer if need be, one tick per minute.
      if(uiTimeoutM > 0) { --uiTimeoutM; }
//      // Run down CLI timer if need be.
//      if(CLITimeoutM > 0) { --CLITimeoutM; }
      }
    }

#ifdef OCCUPANCY_SUPPORT
  const bool reportedRecently = Occupancy.reportedRecently();
#else
  const bool reportedRecently = false;
#endif
// Drive second UI LED if available.
#if defined(LED_UI2_ON)
  // Flash 2nd UI LED very briefly every 'tick' while activity has recently been reported.
  if(reportedRecently) { LED_UI2_ON(); veryTinyPause(); }
  LED_UI2_OFF(); // Generally force 2nd LED off.
#endif

  // True on every 4th tick/call, ie about once every 8 seconds.
  const bool forthTick = !((++tickCount) & 3); // True on every 4th tick.

#ifdef TEMP_POT_AVAILABLE
  const bool rUIcu = veryRecentUIControlUse();
  if(rUIcu || forthTick) // If recent UI activity, and periodically.
    {
    // Force relatively-frequent re-read of temp pot UI device.
    TempPot.read();
    }
#endif

  // If true then is in WARM (or BAKE) mode; defaults to (starts as) false/FROST.
  // Should be only be set when 'debounced'.
  // Defaults to (starts as) false/FROST.
  static bool isWarmModePutative;
#ifdef SUPPORT_BAKE
  static bool isBakeModePutative;
#endif

  static bool modeButtonWasPressed;
  if(fastDigitalRead(BUTTON_MODE_L) == LOW)
    {
    if(!modeButtonWasPressed)
      {
      // Capture real mode variable as button is pressed.
      isWarmModePutative = inWarmMode();
#ifdef SUPPORT_BAKE
      isBakeModePutative = inBakeMode();
#endif      
      modeButtonWasPressed = true;
      }

    // User is pressing the mode button: cycle through FROST | WARM [ | BAKE ].
    // Mark controls used and room as currently occupied given button press.
    markUIControlUsed();
    // LED on...
    LED_HEATCALL_ON();
    tinyPause(); // Leading tiny pause...
    if(!isWarmModePutative) // Was in FROST mode; moving to WARM mode.
      {
      isWarmModePutative = true;
#ifdef SUPPORT_BAKE
      isBakeModePutative = false;
#endif
      // 2 x flash 'heat call' to indicate now in WARM mode.
      LED_HEATCALL_OFF();
      offPause();
      LED_HEATCALL_ON();
      tinyPause();
      }
#ifdef SUPPORT_BAKE
    else if(!isBakeModePutative) // Was in WARM mode, move to BAKE (with full timeout to run).
      {
      isBakeModePutative = true;
      // 2 x flash + one longer flash 'heat call' to indicate now in BAKE mode.
      LED_HEATCALL_OFF();
      offPause();
      LED_HEATCALL_ON();
      tinyPause();
      LED_HEATCALL_OFF();
      mediumPause(); // Note different flash on/off duty cycle to try to distinguish this last flash.
      LED_HEATCALL_ON();
      mediumPause();
      }
#endif
    else // Was in BAKE (if supported, else was in WARM), move to FROST.
      {
      isWarmModePutative = false;
#ifdef SUPPORT_BAKE
      isBakeModePutative = false;
#endif
      // 1 x flash 'heat call' to indicate now in FROST mode.
      }
    }
  else
    {
    // Update real control variables for mode when button is released.
    if(modeButtonWasPressed)
      {
      // Don't update the debounced WARM mode while button held down.
      // Will also capture programmatic changes to isWarmMode, eg from schedules.
      const bool isWarmModeDebounced = isWarmModePutative;
      setWarmModeDebounced(isWarmModeDebounced);
#ifdef SUPPORT_BAKE
      if(isBakeModePutative) { startBakeDebounced(); } else { cancelBakeDebounced(); }
#endif

      markUIControlUsed(); // Note activity on release of MODE button...
      modeButtonWasPressed = false;
      }

    // Keep reporting UI status if the user has just touched the unit in some way.
    // (Or if occupancy/activity was just detected, to give the use some feedback for indirectly interacting.)
    const bool justTouched = statusChange || veryRecentUIControlUse()
#ifdef OCCUPANCY_SUPPORT
        || Occupancy.reportedRecently()
#endif
        ;

    // Mode button not pressed: indicate current mode with flash(es); more flashes if actually calling for heat.
    // Force display while UI controls are being used, eg to indicate temp pot position.
    if(justTouched || inWarmMode()) // Generate flash(es) if in WARM mode or fiddling with UI other than Mode button.
      {
      // DHD20131223: do not flash if the room is dark so as to save energy and avoid disturbing sleep, etc.
      // In this case force resample of light level frequently in case user turns light on eg to operate unit.
      // Do show LED flash if user has recently operated controls (other than mode button) manually.
      // Flash infrequently if no recently operated controls and not in BAKE mode and not actually calling for heat;
      // this is to conserve batteries for those people who leave the valves in WARM mode all the time.
      if(justTouched ||
         ((forthTick
#ifdef ENABLE_NOMINAL_RAD_VALVE
             || NominalRadValve.isCallingForHeat()
#endif
             || inBakeMode()) && !AmbLight.isRoomDark()))
        {
        // First flash to indicate WARM mode (or pot being twiddled).
        LED_HEATCALL_ON();
        // LED on stepwise proportional to temp pot setting.
        // Small number of steps (3) should help make positioning more obvious.
        const uint8_t wt = getWARMTargetC();
        // Makes vtiny|tiny|medium flash for cool|OK|warm temperature target.
        if(isEcoTemperature(wt)) { veryTinyPause(); }
        else if(!isComfortTemperature(wt)) { tinyPause(); }
        else { mediumPause(); }

#ifdef ENABLE_NOMINAL_RAD_VALVE
        // Second flash to indicate actually calling for heat.
        if(NominalRadValve.isCallingForHeat())
          {
          LED_HEATCALL_OFF();
          offPause(); // V0.09 was mediumPause().
          LED_HEATCALL_ON(); // flash
          if(isEcoTemperature(wt)) { veryTinyPause(); }
          else if(!isComfortTemperature(wt)) { sleepLowPowerMs((VERYTINY_PAUSE_MS + TINY_PAUSE_MS) / 2); }
          else { tinyPause(); }

#ifdef SUPPORT_BAKE
          if(inBakeMode())
            {
            // Third (lengthened) flash to indicate BAKE mode.
            LED_HEATCALL_OFF();
            mediumPause(); // Note different flash off time to try to distinguish this last flash.
            LED_HEATCALL_ON();
            // Makes tiny|small|medium flash for eco|OK|comfort temperature target.
            if(isEcoTemperature(wt)) { tinyPause(); }
            else if(!isComfortTemperature(wt)) { smallPause(); }
            else { mediumPause(); }
            }
#endif
          }
#endif
        }
      }
 
#ifdef ENABLE_NOMINAL_RAD_VALVE
    // Even in FROST mode, and if actually calling for heat (eg opening the rad valve significantly, etc)
    // then emit a tiny double flash on every 4th tick.
    // This call for heat may be frost protection or pre-warming / anticipating demand.
    // DHD20130528: new 4th-tick flash in FROST mode...
    // DHD20131223: do not flash if the room is dark so as to save energy and avoid disturbing sleep, etc.
    else if(forthTick &&
            !AmbLight.isRoomDark() &&
            NominalRadValve.isCallingForHeat() &&
            NominalRadValve.isControlledValveReallyOpen())
      {
      // Double flash every 4th tick indicates call for heat while in FROST MODE (matches call for heat in WARM mode).
      LED_HEATCALL_ON(); // flash
      veryTinyPause();
      LED_HEATCALL_OFF();
      offPause();
      LED_HEATCALL_ON(); // flash
      veryTinyPause();
      }
#endif

    // Enforce any changes that may have been driven by other UI components (ie other than MODE button).
    // Eg adjustment of temp pot / eco bias changing scheduled state.
    if(statusChange)
      {
      static bool prevScheduleStatus;
      const bool currentScheduleStatus = isAnyScheduleOnWARMNow();
      if(currentScheduleStatus != prevScheduleStatus)
        {
        prevScheduleStatus = currentScheduleStatus;
        setWarmModeDebounced(currentScheduleStatus);
        }
      }
    }

  // Ensure LED forced off unconditionally at least once each cycle.
  LED_HEATCALL_OFF();

#ifdef LEARN_BUTTON_AVAILABLE
  // Handle learn button if supported and if is currently pressed.
  if(fastDigitalRead(BUTTON_LEARN_L) == LOW)
    {
    handleLEARN(0);
    markUIControlUsed(); // Mark controls used and room as currently occupied given button press.
    LED_HEATCALL_ON(); // Leave heatcall LED on while learn button held down.
    }

#if defined(BUTTON_LEARN2_L)
  // Handle second learn button if supported and currently pressed and primary learn button not pressed.
  else if(fastDigitalRead(BUTTON_LEARN2_L) == LOW)
    {
    handleLEARN(1);
    markUIControlUsed(); // Mark controls used and room as currently occupied given button press.
    LED_HEATCALL_ON(); // Leave heatcall LED on while learn button held down.
    }
#endif
#endif

  const bool statusChanged = statusChange;
  statusChange = false; // Potential race.
  return(statusChanged);
  }


// Check/apply the user's schedule, at least once each minute, and act on any timed events.
void checkUserSchedule()
  {
  // Get minutes since midnight local time [0,1439].
  const uint_least16_t msm = getMinutesSinceMidnightLT();

  // Check all available schedules.
  // FIXME: probably will NOT work as expected for overlapping schedules (ie will got to FROST at end of first one).
  for(uint8_t which = 0; which < MAX_SIMPLE_SCHEDULES; ++which)
    {
    // Check if now is the simple scheduled off time, as minutes after midnight [0,1439]; invalid (eg ~0) if none set.
    // Programmed off/frost takes priority over on/warm if same to bias towards energy-saving.
    // Note that in the presence of multiple overlapping schedules only the last 'off' applies however.
    if(((MAX_SIMPLE_SCHEDULES < 1) || !isAnyScheduleOnWARMNow()) &&
       (msm == getSimpleScheduleOff(which)))
      { setWarmModeDebounced(false); }
    // Check if now is the simple scheduled on time.
    else if(msm == getSimpleScheduleOn(which))
      { setWarmModeDebounced(true); }
    }
  }





#ifdef ENABLE_EXTENDED_CLI
// Handle CLI extension commands.
// Commands of form:
//   +EXT .....
// where EXT is the name of the extension, usually 3 letters.

#include <OTProtocolCC.h>
#include "RFM22_Radio.h"

// It is acceptable for extCLIHandler() to alter the buffer passed,
// eg with strtok_t().
static bool extCLIHandler(Print *const p, char *const buf, const uint8_t n)
  {

#ifdef ALLOW_CC1_SUPPORT_RELAY
  // If CC1 replay then allow +CC1 ! command to send an alert to the hub.
  // Full command is:
  //    +CC1 !
  // This unit's housecode is used in the frame sent.
  const uint8_t CC1_A_PREFIX_LEN = 6;
  // Falling through rather than return(true) indicates failure.
  if((n >= CC1_A_PREFIX_LEN) && (0 == strncmp("+CC1 !", buf, CC1_A_PREFIX_LEN)))
    {
    OTProtocolCC::CC1Alert a = OTProtocolCC::CC1Alert::make(FHT8VGetHC1(), FHT8VGetHC2());
    if(a.isValid())
      {
      uint8_t txbuf[STATS_MSG_START_OFFSET + OTProtocolCC::CC1Alert::primary_frame_bytes+1]; // More than large enough for preamble + sync + alert message.
      uint8_t *const bptr = RFM22RXPreambleAdd(txbuf);
      const uint8_t bodylen = a.encodeSimple(bptr, sizeof(txbuf) - STATS_MSG_START_OFFSET, true);
      const uint8_t buflen = STATS_MSG_START_OFFSET + bodylen;
#if 0 && defined(DEBUG)
OTRadioLink::printRXMsg(p, txbuf, buflen);
#endif
      if(RFM23B.sendRaw(txbuf, buflen)) // Send at default volume...  One going missing won't hurt that much.
        { return(true); } // Done it!
      }

    return(false); // FAILED if fallen through from above.
    }
#endif

#ifdef ALLOW_CC1_SUPPORT_HUB
  // If CC1 hub then allow +CC1 ? command to poll a remote relay.
  // Full command is:
  //    +CC1 ? hc1 hc2 rp lc lt lf
  // ie six numeric arguments, see below, with out-of-range values coerced (other than housecodes):
//            // Factory method to create instance.
//            // Invalid parameters (except house codes) will be coerced into range.
//            //   * House code (hc1, hc2) of valve controller that the poll/command is being sent to.
//            //   * rad-open-percent     [0,100] 0-100 in 1% steps, percent open approx to set rad valve (rp)
//            //   * light-colour         [0,3] bit flags 1==red 2==green (lc) 0 => stop everything
//            //   * light-on-time        [1,15] (0 not allowed) 30-450s in units of 30s (lt) ???
//            //   * light-flash          [1,3] (0 not allowed) 1==single 2==double 3==on (lf)
//            // Returns instance; check isValid().
//            static CC1PollAndCommand make(uint8_t hc1, uint8_t hc2,
//                                          uint8_t rp,
//                                          uint8_t lc, uint8_t lt, uint8_t lf);
  const uint8_t CC1_Q_PREFIX_LEN = 7;
  const uint8_t CC1_Q_PARAMS = 6;
  // Falling through rather than return(true) indicates failure.
  if((n >= CC1_Q_PREFIX_LEN) && (0 == strncmp("+CC1 ? ", buf, CC1_Q_PREFIX_LEN)))
    {
    char *last; // Used by strtok_r().
    char *tok1;
    // Attempt to parse the parameters.
    if((n-CC1_Q_PREFIX_LEN >= CC1_Q_PARAMS*2-1) && (NULL != (tok1 = strtok_r(buf+CC1_Q_PREFIX_LEN, " ", &last))))
      {
      char *tok2 = strtok_r(NULL, " ", &last);
      char *tok3 = (NULL == tok2) ? NULL : strtok_r(NULL, " ", &last);
      char *tok4 = (NULL == tok3) ? NULL : strtok_r(NULL, " ", &last);
      char *tok5 = (NULL == tok4) ? NULL : strtok_r(NULL, " ", &last);
      char *tok6 = (NULL == tok5) ? NULL : strtok_r(NULL, " ", &last);
      if(NULL != tok6)
        {
        OTProtocolCC::CC1PollAndCommand q = OTProtocolCC::CC1PollAndCommand::make(
            atoi(tok1),
            atoi(tok2),
            atoi(tok3),
            atoi(tok4),
            atoi(tok5),
            atoi(tok6));
        if(q.isValid())
          {
          uint8_t txbuf[STATS_MSG_START_OFFSET + OTProtocolCC::CC1PollAndCommand::primary_frame_bytes+1]; // More than large enough for preamble + sync + alert message.
          uint8_t *const bptr = RFM22RXPreambleAdd(txbuf);
          const uint8_t bodylen = q.encodeSimple(bptr, sizeof(txbuf) - STATS_MSG_START_OFFSET, true);
          const uint8_t buflen = STATS_MSG_START_OFFSET + bodylen;
#if 0 && defined(DEBUG)
    OTRadioLink::printRXMsg(p, txbuf, buflen);
#endif
          const bool doubleTX = false;
          if(RFM23B.sendRaw(txbuf, buflen, 0, (doubleTX ? OTRadioLink::OTRadioLink::TXmax : OTRadioLink::OTRadioLink::TXloud)))
            { return(true); } // Done it!
          }
        }
      }
    return(false); // FAILED if fallen through from above.
    }
#endif

  return(false); // FAILED if not otherwise handled.
  }
#endif 






// Prints a single space to Serial (which must be up and running).
static void Serial_print_space() { Serial.print(' '); }

// Sends a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
// Ideally should be similar to PICAXE V0.1 output to allow the same parser to handle either.
// Will turn on UART just for the duration of this call if powered off.
// Has multiple sections, some optional, starting with a unique letter and separated with ';'.
/*
Status output may look like this...
=F0%@18C;T16 36 W255 0 F255 0;S5 5 17
=W0%@18C;T16 38 W255 0 F255 0;S5 5 17
=W0%@18C;T16 39 W255 0 F255 0;S5 5 17
=W0%@18C;T16 40 W16 39 F17 39;S5 5 17
=W0%@18C;T16 41 W16 39 F17 39;S5 5 17
=W0%@17C;T16 42 W16 39 F17 39;S5 5 17
=W20%@17C;T16 43 W16 39 F17 39;S5 5 17
=W20%@17C;T16 44 W16 39 F17 39;S5 5 17
=F0%@17C;T16 45 W16 39 F17 39;S5 5 17

When driving an FHT8V wireless radiator valve it may look like this:
=F0%@18C;T2 30 W10 0 F12 0;S5 5 17 wf;HC255 255
=F0%@18C;T2 30 W10 0 F12 0;S5 5 17 wf;HC255 255
=W0%@18C;T2 31 W10 0 F12 0;S5 5 17 wf;HC255 255
=W10%@18C;T2 32 W10 0 F12 0;S5 5 17 wf;HC255 255
=W20%@18C;T2 33 W10 0 F12 0;S5 5 17 wfo;HC255 255

'=' starts the status line and CRLF ends it; sections are separated with ";".
The initial 'W' or 'F' is WARM or FROST mode indication.  (If BAKE mode is supported, 'B' may be shown instead of 'W' when in BAKE.)
The nn% is the target valve open percentage.
The @nnCh gives the current measured room temperature in (truncated, not rounded) degrees C, followed by hex digit for 16ths.
The ";" terminates this initial section.
Thh mm is the local current 24h time in hours and minutes.
Whh mm is the scheduled on/warm time in hours and minutes, or an invalid time if none.
Fhh mm is the scheduled off/frost time in hours and minutes, or an invalid time if none.
The ";" terminates this schedule section.
'S' introduces the current and settable-target temperatures in Celsius/centrigrade, if supported.
eg 'S5 5 17'
The first number is the current target in C, the second is the FROST target, the third is the WARM target.
The 'e' or 'c' indicates eco or comfort bias.
A 'w' indicates that this hour is predicted for smart warming ('f' indocates not), and another 'w' the hour ahead.
A trailing 'o' indicates room occupancy.
The ";" terminates this 'settable' section.

'HC' introduces the optional FHT8V house codes section, if supported and codes are set.
eg 'HC99 99'
HChc1 hc2 are the house codes 1 and 2 for an FHT8V valve.
*/
void serialStatusReport()
  {
  const bool neededWaking = powerUpSerialIfDisabled();

  // Aim to overlap CPU usage with characters being TXed for throughput determined primarily by output size and baud.

  // Stats line starts with distingushed marker character.
  // Initial '=' section with common essentials.
  Serial.print(LINE_START_CHAR_STATS);
#ifdef SUPPORT_BAKE
  Serial.print(inWarmMode() ? (inBakeMode() ? 'B' : 'W') : 'F');
#else
  Serial.print(inWarmMode() ? 'W' : 'F');
#endif
#ifdef ENABLE_NOMINAL_RAD_VALVE
  Serial.print(NominalRadValve.get()); Serial.print('%'); // Target valve position.
#endif
  const int temp = TemperatureC16.get();
  Serial.print('@'); Serial.print(temp >> 4); Serial.print('C'); // Unrounded whole degrees C.
      Serial.print(temp & 0xf, HEX); // Show 16ths in hex.

//#if 0
//  // *P* section: low power flag only shown iff (battery) low.
//  if(Supply_mV.isSupplyVoltageLow()) { Serial.print(F(";Plow")); }
//#endif

#if 1
  // *X* section: Xmit security level shown only if some non-essential TX potentially allowed.
  const stats_TX_level xmitLevel = getStatsTXLevel();
  if(xmitLevel < stTXnever) { Serial.print(F(";X")); Serial.print(xmitLevel); }
#endif

  // *T* section: time and schedules.
  const uint_least8_t hh = getHoursLT();
  const uint_least8_t mm = getMinutesLT();
  Serial.print(';'); // End previous section.
  Serial.print('T'); Serial.print(hh); Serial_print_space(); Serial.print(mm);
  // Show all schedules set.
  for(uint8_t scheduleNumber = 0; scheduleNumber < MAX_SIMPLE_SCHEDULES; ++scheduleNumber)
    {
    Serial_print_space();
    uint_least16_t startMinutesSinceMidnightLT = getSimpleScheduleOn(scheduleNumber);
    const bool invalidStartTime = startMinutesSinceMidnightLT >= MINS_PER_DAY;
    const int startH = invalidStartTime ? 255 : (startMinutesSinceMidnightLT / 60);
    const int startM = invalidStartTime ? 0 : (startMinutesSinceMidnightLT % 60);
    Serial.print('W'); Serial.print(startH); Serial_print_space(); Serial.print(startM);
    Serial_print_space();
    uint_least16_t endMinutesSinceMidnightLT = getSimpleScheduleOff(scheduleNumber);
    const bool invalidEndTime = endMinutesSinceMidnightLT >= MINS_PER_DAY;
    const int endH = invalidEndTime ? 255 : (endMinutesSinceMidnightLT / 60);
    const int endM = invalidEndTime ? 0 : (endMinutesSinceMidnightLT % 60);
    Serial.print('F'); Serial.print(endH); Serial_print_space(); Serial.print(endM);
    }
  if(isAnyScheduleOnWARMNow()) { Serial.print('*'); } // Indicate that at least one schedule is active now.

  // *S* section: settable target/threshold temperatures, current target, and eco/smart/occupied flags.
#ifdef SETTABLE_TARGET_TEMPERATURES // Show thresholds and current target since no longer so easily deduced.
  Serial.print(';'); // Terminate previous section.
  Serial.print('S'); // Current settable temperature target, and FROST and WARM settings.
#ifdef ENABLE_NOMINAL_RAD_VALVE
  Serial.print(NominalRadValve.getTargetTempC());
#endif
  Serial_print_space();
  Serial.print(getFROSTTargetC());
  Serial_print_space();
  Serial.print(getWARMTargetC());
#if 0
  // Show bias.
  Serial_print_space();
  Serial.print(hasEcoBias() ? 'e' : 'c'); // Show eco/comfort bias.
#endif
#ifdef ENABLE_ANTICIPATION
  // Show warming predictions.
  Serial.print(shouldBeWarmedAtHour(hh) ? 'w' : 'f');
  Serial.print(shouldBeWarmedAtHour(hh < 23 ? (hh+1) : 0) ? 'w' : 'f');
#endif
#endif

  // *C* section: central hub values.
#if defined(ENABLE_BOILER_HUB) || defined(ALLOW_STATS_RX)
  // Print optional hub boiler-on-time section if apparently set (non-zero) and thus in hub mode.
  const uint8_t boilerOnMinutes = getMinBoilerOnMinutes();
  if(boilerOnMinutes != 0)
    {
    Serial.print(';'); // Terminate previous section.
    Serial.print('C'); // Indicate central hub mode available.
    Serial.print(boilerOnMinutes); // Show min 'on' time, or zero if disabled.
    }
#endif

  // *H* section: house codes for local FHT8V valve and if syncing, iff set.
#if defined(USE_MODULE_FHT8VSIMPLE)
  // Print optional house code section if codes set.
  const uint8_t hc1 = FHT8VGetHC1();
  if(hc1 != 255)
    {
    Serial.print(F(";HC"));
    Serial.print(hc1);
    Serial_print_space();
    Serial.print(FHT8VGetHC2());
    if(!isSyncedWithFHT8V())
      {
      Serial_print_space();
      Serial.print('s'); // Indicate syncing with trailing lower-case 's' in field...
      }
    }
#endif

#ifdef ENABLE_NOMINAL_RAD_VALVE
  // *M* section: min-valve-percentage open section, iff not at default value.
  const uint8_t minValvePcOpen = NominalRadValve.getMinValvePcReallyOpen();
  if(DEFAULT_MIN_VALVE_PC_REALLY_OPEN != minValvePcOpen) { Serial.print(F(";M")); Serial.print(minValvePcOpen); }
#endif

#if 1 && defined(ALLOW_JSON_OUTPUT)
  Serial.print(';'); // Terminate previous section.
  char buf[80];
  static SimpleStatsRotation<5> ss1; // Configured for maximum different stats.
//  ss1.put(TemperatureC16);
#if defined(HUMIDITY_SENSOR_SUPPORT)
  ss1.put(RelHumidity);
#endif
  ss1.put(AmbLight);
  ss1.put(Supply_mV);
#if defined(OCCUPANCY_SUPPORT)
  ss1.put(Occupancy);
//  ss1.put(Occupancy.vacHTag(), Occupancy.getVacancyH()); // EXPERIMENTAL
#endif
#ifdef ENABLE_MODELLED_RAD_VALVE
    ss1.put(NominalRadValve.tagCMPC(), NominalRadValve.getCumulativeMovementPC()); // EXPERIMENTAL
#endif
  const uint8_t wrote = ss1.writeJSON((uint8_t *)buf, sizeof(buf), 0, true);
  if(0 != wrote) { Serial.print(buf); }
#endif

  // Terminate line.
  Serial.println();

  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  flushSerialSCTSensitive();

  if(neededWaking) { powerDownSerial(); }
  }

#define SYNTAX_COL_WIDTH 10 // Width of 'syntax' column; strictly positive.
// Estimated maximum overhead in sub-cycle ticks to print full line and all trailing CLI summary info.
#define CLI_PRINT_OH_SCT ((uint8_t)(GSCT_MAX/4))
// Deadline in minor cycle by which to stop printing description.
#define STOP_PRINTING_DESCRIPTION_AT ((uint8_t)(GSCT_MAX-CLI_PRINT_OH_SCT))
// Efficiently print a single line given the syntax element and the description, both non-null.
// NOTE: will skip the description if getting close to the end of the time deadline, in order to avoid overrun.
static void printCLILine(const uint8_t deadline, __FlashStringHelper const *syntax, __FlashStringHelper const *description)
  {
  Serial.print(syntax);
  flushSerialProductive(); // Ensure all pending output is flushed before sampling current position in minor cycle.
  if(getSubCycleTime() >= deadline) { Serial.println(); return; }
  for(int8_t padding = SYNTAX_COL_WIDTH - strlen_P((const char *)syntax); --padding >= 0; ) { Serial_print_space(); }
  Serial.println(description);
  }
// Efficiently print a single line given a single-char syntax element and the description, both non-null.
// NOTE: will skip the description if getting close to the end of the time deadline to avoid overrun.
static void printCLILine(const uint8_t deadline, const char syntax, __FlashStringHelper const *description)
  {
  Serial.print(syntax);
  flushSerialProductive(); // Ensure all pending output is flushed before sampling current position in minor cycle.
  if(getSubCycleTime() >= deadline) { Serial.println(); return; }
  for(int8_t padding = SYNTAX_COL_WIDTH - 1; --padding >= 0; ) { Serial_print_space(); }
  Serial.println(description);
  }

// Dump some brief CLI usage instructions to serial TX, which must be up and running.
// If this gets too big there is a risk of overrunning and missing the next tick...
static void dumpCLIUsage(const uint8_t stopBy)
  {
  const uint8_t deadline = fnmin((uint8_t)(stopBy - fnmin(stopBy,CLI_PRINT_OH_SCT)), STOP_PRINTING_DESCRIPTION_AT);
  Serial.println();
  //Serial.println(F("CLI usage:"));
  printCLILine(deadline, '?', F("this help"));
  
  // Core CLI features first... (E, [H], I, S V)
  printCLILine(deadline, 'E', F("Exit CLI"));
#if defined(USE_MODULE_FHT8VSIMPLE) && defined(LOCAL_TRV)
  printCLILine(deadline, F("H H1 H2"), F("set FHT8V House codes 1&2"));
  printCLILine(deadline, 'H', F("clear House codes"));
#endif
  printCLILine(deadline, 'I', F("new ID"));
  printCLILine(deadline, 'S', F("show Status"));
  printCLILine(deadline, 'V', F("sys Version"));

#ifdef ENABLE_FULL_OT_CLI
  // Optional CLI features...
  Serial.println(F("-"));
#if defined(ENABLE_BOILER_HUB) || defined(ALLOW_STATS_RX)
  printCLILine(deadline, F("C M"), F("Central hub >=M mins on, 0 off"));
#endif
  printCLILine(deadline, F("D N"), F("Dump stats set N"));
  printCLILine(deadline, 'F', F("Frost"));
#if defined(SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
  printCLILine(deadline, F("F CC"), F("set Frost/setback temp CC"));
#endif

  //printCLILine(deadline, 'L', F("Learn to warm every 24h from now, clear if in frost mode, schedule 0"));
  printCLILine(deadline, F("L S"), F("Learn daily warm now, clear if in frost mode, schedule S"));
  //printCLILine(deadline, F("P HH MM"), F("Program: warm daily starting at HH MM schedule 0"));
  printCLILine(deadline, F("P HH MM S"), F("Program: warm daily starting at HH MM schedule S"));
  printCLILine(deadline, F("O PP"), F("min % for valve to be Open"));
#if defined(ENABLE_NOMINAL_RAD_VALVE)
  printCLILine(deadline, 'O', F("reset Open %"));
#endif
#ifdef SUPPORT_BAKE
  printCLILine(deadline, 'Q', F("Quick Heat"));
#endif
//  printCLILine(deadline, F("R N"), F("dump Raw stats set N"));

  printCLILine(deadline, F("T HH MM"), F("set 24h Time"));
  printCLILine(deadline, 'W', F("Warm"));
#if defined(SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
  printCLILine(deadline, F("W CC"), F("set Warm temp CC"));
#endif
  printCLILine(deadline, 'X', F("Xmit security level; 0 always, 255 never"));
  printCLILine(deadline, 'Z', F("Zap stats"));
#endif // ENABLE_FULL_OT_CLI
  Serial.println();
  }

// Prints warning to serial (that must be up and running) that invalid (CLI) input has been ignored.
// Probably should not be inlined, to avoid creating duplicate strings in Flash.
static void InvalidIgnored() { Serial.println(F("Invalid, ignored.")); }

// If INTERACTIVE_ECHO defined then immediately echo received characters, not at end of line.
#define CLI_INTERACTIVE_ECHO

#define MAXIMUM_CLI_OT_RESPONSE_CHARS 9 // Just enough for any valid core/OT command expected not including trailing LF.  (Note that Serial RX buffer is 64 bytes.)
#ifdef ENABLE_EXTENDED_CLI // Allow for much longer input commands.
#define MAXIMUM_CLI_RESPONSE_CHARS (max(64, MAXIMUM_CLI_OT_RESPONSE_CHARS))
#else
#define MAXIMUM_CLI_RESPONSE_CHARS MAXIMUM_CLI_OT_RESPONSE_CHARS
#endif
#define IDLE_SLEEP_SCT (15/SUBCYCLE_TICK_MS_RD) // Approx sub-cycle ticks in idle sleep (15ms), erring on side of being too large; strictly positive.
#define BUF_FILL_TIME_MS (((MAXIMUM_CLI_RESPONSE_CHARS*10) * 1000 + (BAUD-1)) / BAUD) // Time to read full/maximal input command buffer; ms, strictly positive.
#define BUF_FILL_TIME_SCT (BUF_FILL_TIME_MS/SUBCYCLE_TICK_MS_RD) // Approx sub-cycle ticks to fill buf, erring on side of being too large; strictly positive.
#define MIN_POLL_SCT max(IDLE_SLEEP_SCT, BUF_FILL_TIME_SCT)
#if MIN_POLL_SCT > CLI_POLL_MIN_SCT
#error "MIN_POLL_SCT > CLI_POLL_MIN_SCT" 
#endif
#define MIN_RX_BUFFER 16 // Minimum Arduino Serial RX buffer size.
// DHD20131213: CAN_IDLE_15MS true seemed to be causing intermittent crashes.
#ifdef ENABLE_AVR_IDLE_MODE
#define CAN_IDLE_30MS ((BAUD <= 4800) || (MAXIMUM_CLI_RESPONSE_CHARS < MIN_RX_BUFFER)) // If true, cannot get RX overrun during 15--30ms idle.
#else
#define CAN_IDLE_30MS (false)
#endif
// Used to poll user side for CLI input until specified sub-cycle time.
// Commands should be sent terminated by CR *or* LF; both may prevent 'E' (exit) from working properly.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// Times itself out after at least a minute or two of inactivity. 
// NOT RENTRANT (eg uses static state for speed and code space).
void pollCLI(const uint8_t maxSCT, const bool startOfMinute)
  {
  // Perform any once-per-minute operations.
  if(startOfMinute)
    {
    ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
      {
      // Run down CLI timer if need be.
      if(CLITimeoutM > 0) { --CLITimeoutM; }
      }
    }

  // Compute safe limit time given granularity of sleep and buffer fill.
  const uint8_t targetMaxSCT = (maxSCT <= MIN_POLL_SCT) ? ((uint8_t) 0) : ((uint8_t) (maxSCT - 1 - MIN_POLL_SCT));
  if(getSubCycleTime() >= targetMaxSCT) { return; } // Too short to try.

  const bool neededWaking = powerUpSerialIfDisabled();

  // Purge any stray pending input, such as a trailing LF from previous input.
  while(Serial.available() > 0) { Serial.read(); }

  // Generate and flush prompt character to the user, after a CRLF to reduce ambiguity.
  // Do this AFTER flushing the input so that sending command immediately after prompt should work.
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
      int ic = Serial.read();
      if(('\r' == ic) || ('\n' == ic)) { break; } // Stop at CR, eg from CRLF, or LF.
#ifdef CLI_INTERACTIVE_ECHO
      if(('\b' == ic) || (127 == ic))
        {
        // Handle backspace or delete as delete...
        if(n > 0) // Ignore unless something to delete...
          {
          Serial.print('\b');
          Serial.print(' ');
          Serial.print('\b');
          --n;
          }
        continue;
        }
#endif
      if((ic < 32) || (ic > 126)) { continue; } // Drop bogus non-printable characters.
      // Ignore any leading char that is not a letter (or '?' or '+'),
      // and force leading (command) char to upper case.
      if(0 == n)
        {
        ic = toupper(ic);
        if(('+' != ic) && ('?' != ic) && ((ic < 'A') || (ic > 'Z'))) { continue; }
        }
      // Store the incoming char.
      buf[n++] = (char) ic;
#ifdef CLI_INTERACTIVE_ECHO
      Serial.print((char) ic); // Echo immediately.
#endif
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
#if CAN_IDLE_30MS
    // Minimise power consumption leaving CPU/UART clock running, if no danger of RX overrun.
    // Don't do this too close to end of target end time to avoid missing it.
    // Note: may get woken on timer0 interrupts as well as RX and watchdog.
    if(sct < targetMaxSCT-2) { idle15AndPoll(); continue; }
#endif
    burnHundredsOfCyclesProductivelyAndPoll(); // Use time time to poll for I/O, etc.
    }

  if(n > 0)
    {
    // Restart the CLI timer on receipt of plausible (ASCII) input (cf noise from UART floating or starting up),
    // Else print a very brief low-CPU-cost help message and give up as efficiently and safely and quickly as possible.
    const char firstChar = buf[0];
    const bool plausibleCommand = ((firstChar > ' ') && (firstChar <= 'z'));
    if(plausibleCommand) { resetCLIActiveTimer(); }
    else
      {
      Serial.println(F("? for CLI help"));
      // Force any pending output before return / possible UART power-down.
      flushSerialSCTSensitive();
      if(neededWaking) { powerDownSerial(); }
      return;
      }

    // Null-terminate the received command line.
    buf[n] = '\0';

    // strupr(buf); // Force to upper-case
#ifdef CLI_INTERACTIVE_ECHO
    Serial.println(); // ACK user's end-of-line.
#else
    Serial.println(buf); // Echo the line received (asynchronously).
#endif

    // Process the input received, with action based on the first char...
    bool showStatus = true; // Default to showing status.
    switch(buf[0])
      {
      // Explicit request for help, or unrecognised first character.
      // Avoid showing status as may already be rather a lot of output.
      default: case '?': { dumpCLIUsage(maxSCT); showStatus = false; break; }


      // CORE CLI FEATURES: keep small and low-impact.
      //     E, [H], I, S V
      // ---
      // Exit/deactivate CLI immediately.
      // This should be followed by JUST CR ('\r') OR LF ('\n')
      // else the second will wake the CLI up again.
      case 'E': { CLITimeoutM = 0; break; }
#if defined(USE_MODULE_FHT8VSIMPLE) && (defined(LOCAL_TRV) || defined(SLAVE_TRV))
      // H nn nn
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
      // Set new random ID.
      // Should possibly restart the system afterwards.
      case 'I':
        {
        ensureIDCreated(true); // Force ID change.
        break;
        }
      // Status line and optional smart/scheduled warming prediction request.
      case 'S':
        {
        Serial.print(F("Resets: "));
        const uint8_t resetCount = eeprom_read_byte((uint8_t *)EE_START_RESET_COUNT);
        Serial.print(resetCount);
        Serial.println();
        Serial.print(F("Overruns: "));
        const uint8_t overrunCount = (~eeprom_read_byte((uint8_t *)EE_START_OVERRUN_COUNTER)) & 0xff;
        Serial.print(overrunCount);
        Serial.println();
        break; // Note that status is by default printed after processing input line.
        }
      // Version information printed as one line to serial, machine- and human- parseable.
      case 'V':
        {
        serialPrintlnBuildVersion();
#ifdef ENABLE_EXTENDED_CLI // Allow for much longer input commands for extended CLI.
        Serial.print(F("Ext CLI max chars: ")); Serial.println(MAXIMUM_CLI_RESPONSE_CHARS);
#endif
        break;
        }


#ifdef ENABLE_EXTENDED_CLI
      // Handle CLI extension commands.
      // Command of form:
      //   +EXT .....
      // where EXT is the name of the extension, usually 3 letters.
      //
      // It is acceptable for extCLIHandler() to alter the buffer passed,
      // eg with strtok_t().
      case '+':
        {
        const bool success = extCLIHandler(&Serial, buf, n);
        Serial.println(success ? F("OK") : F("FAILED"));
        break;
        }
#endif  


#ifdef ENABLE_FULL_OT_CLI // NON-CORE CLI FEATURES

#if defined(ENABLE_BOILER_HUB) || defined(ALLOW_STATS_RX)
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

//      // Raw stats: R N
//      // Avoid showing status afterwards as may already be rather a lot of output.
//      case 'R':
//        {
//        char *last; // Used by strtok_r().
//        char *tok1;
//        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "R 0".
//        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
//          {
//          const uint8_t setN = (uint8_t) atoi(tok1);
//          for(uint8_t hh = 0; hh < 24; ++hh)
//            { Serial.print(getByHourStat(hh, setN)); Serial_print_space(); }
//          Serial.println();
//          }
//        break;
//        }

      // Dump (human-friendly) stats: D N
      // DEBUG only: "D?" to force partial stats sample and "D!" to force an immediate full stats sample; use with care.
      // Avoid showing status afterwards as may already be rather a lot of output.
      case 'D':
        {
#if 0 && defined(DEBUG)
        if(n == 2) // Sneaky way of forcing stats samples.
          {
          if('?' == buf[1]) { sampleStats(false); Serial.println(F("Part sample")); }
          else if('!' == buf[1]) { sampleStats(true); Serial.println(F("Full sample")); }
          break;
          }
#endif
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "D 0".
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t setN = (uint8_t) atoi(tok1);
          const uint8_t thisHH = getHoursLT();
//          const uint8_t lastHH = (thisHH > 0) ? (thisHH-1) : 23;
          // Print label.
          switch(setN)
            {
            default: { Serial.print('?'); break; }
            case EE_STATS_SET_TEMP_BY_HOUR: case EE_STATS_SET_TEMP_BY_HOUR_SMOOTHED: { Serial.print('C'); break; }
            case EE_STATS_SET_AMBLIGHT_BY_HOUR: case EE_STATS_SET_AMBLIGHT_BY_HOUR_SMOOTHED: { Serial.print(F("ambl")); break; }
            case EE_STATS_SET_OCCPC_BY_HOUR: case EE_STATS_SET_OCCPC_BY_HOUR_SMOOTHED: { Serial.print(F("occ%")); break; }
            case EE_STATS_SET_RHPC_BY_HOUR: case EE_STATS_SET_RHPC_BY_HOUR_SMOOTHED: { Serial.print(F("RH%")); break; }
            case EE_STATS_SET_USER1_BY_HOUR: case EE_STATS_SET_USER1_BY_HOUR_SMOOTHED: { Serial.print('u'); break; }
#if defined(EE_STATS_SET_WARMMODE_BY_HOUR_OF_WK)
            case EE_STATS_SET_WARMMODE_BY_HOUR_OF_WK: { Serial.print('W'); break; }
#endif
            }
          Serial_print_space();
          if(setN & 1) { Serial.print(F("smoothed")); } else { Serial.print(F("last")); }
          Serial_print_space();
          // Now print values.
          for(uint8_t hh = 0; hh < 24; ++hh)
            {
            const uint8_t statRaw = getByHourStat(hh, setN);
            // For unset stat show '-'...
            if(STATS_UNSET_BYTE == statRaw) { Serial.print('-'); }
            // ...else print more human-friendly version of stat.
            else switch(setN)
              {
              default: { Serial.print(statRaw); break; } // Generic decimal stats.

              // Special formatting cases.
              case EE_STATS_SET_TEMP_BY_HOUR: case EE_STATS_SET_TEMP_BY_HOUR_SMOOTHED:
                // Uncompanded temperature, rounded.
                { Serial.print((expandTempC16(statRaw)+8) >> 4); break; }
#if defined(EE_STATS_SET_WARMMODE_BY_HOUR_OF_WK)
              case EE_STATS_SET_WARMMODE_BY_HOUR_OF_WK:
                // Warm mode usage bitmap by hour over week.
                { Serial.print(statRaw, HEX); break; }
#endif
              }
            if(hh == thisHH) { Serial.print('<'); } // Highlight current stat in this set.
#if 0 && defined(DEBUG)
            if(inOutlierQuartile(false, setN, hh)) { Serial.print('B'); } // In bottom quartile.
            if(inOutlierQuartile(true, setN, hh)) { Serial.print('T'); } // In top quartile.
#endif
            Serial_print_space();
            }
          Serial.println();
          }

        showStatus = false;
        break;
        }

      // Switch to FROST mode OR set FROST/setback temperature (even with temp pot available).
      // With F! force to frost and holiday (long-vacant) mode.  Useful for testing and for remote CLI use.
      case 'F':
        {
        if(n == 2)
          {
          if('!' == buf[1]) { Serial.println(F("hols")); }
#ifdef OCCUPANCY_SUPPORT
          Occupancy.setHolidayMode();
#endif
          setWarmModeDebounced(false);
          break;
          }
#if defined(SETTABLE_TARGET_TEMPERATURES)
        char *last; // Used by strtok_r().
        char *tok1;
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t tempC = (uint8_t) atoi(tok1);
          if(!setFROSTTargetC(tempC)) { InvalidIgnored(); }
          }
        else
#endif
          { setWarmModeDebounced(false); } // No parameter supplied; switch to FROST mode.
        break;
        }

      // Learn current settings, just as if primary/specified LEARN button had been pressed.
      case 'L':
        {
        int s = 0;
//#if MAX_SIMPLE_SCHEDULES > 1
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "L 0".
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          s = atoi(tok1);
          }
//#endif
        handleLEARN((uint8_t) s); break;
        break;
        }

#if defined(ENABLE_NOMINAL_RAD_VALVE)
      // Set/clear min-valve-open-% threshold override.
      case 'O':
        {
        uint8_t minPcOpen = 0; // Will clear override and use default threshold.
        char *last; // Used by strtok_r().
        char *tok1;
        if((n > 1) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          { minPcOpen = (uint8_t) atoi(tok1); }
        NominalRadValve.setMinValvePcReallyOpen(minPcOpen);
        break;
        }
#endif

      // Program simple schedule HH MM [N].
      case 'P':
        {
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 5 character sequence makes sense and is safe to tokenise, eg "P 1 2".
        if((n >= 5) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          char *tok2 = strtok_r(NULL, " ", &last);
          if(NULL != tok2)
            {
            const int hh = atoi(tok1);
            const int mm = atoi(tok2);
            int s = 0;
//#if MAX_SIMPLE_SCHEDULES > 1         
            char *tok3 = strtok_r(NULL, " ", &last);
            if(NULL != tok3)
              {
              s = atoi(tok3);
              }
//#endif
            // Does not fully validate user inputs (eg for -ve values), but cannot set impossible values.
            if(!setSimpleSchedule((uint_least16_t) ((60 * hh) + mm), (uint8_t)s)) { InvalidIgnored(); }
            }
          }
        break;
        }

#ifdef SUPPORT_BAKE
      // Switch to (or restart) BAKE (Quick Heat) mode: Q
      case 'Q': { startBakeDebounced(); break; }
#endif

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
#if defined(SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
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
#ifdef SUPPORT_BAKE
          cancelBakeDebounced(); // Ensure BAKE mode not entered.
#endif
          setWarmModeDebounced(true); // No parameter supplied; switch to WARM mode.
          }
        break;
        }

      // tX security level: X NN
      // Avoid showing status afterwards as may already be rather a lot of output.
      case 'X':
        {
        char *last; // Used by strtok_r().
        char *tok1;
        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "X 0".
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t nn = (uint8_t) atoi(tok1);
          eeprom_smart_update_byte((uint8_t *)EE_START_STATS_TX_ENABLE, nn);
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
#endif // ENABLE_FULL_OT_CLI // NON-CORE FEATURES
      }

    // Almost always show status line afterwards as feedback of command received and new state.
    if(showStatus) { serialStatusReport(); }
    // Else show ack of command received.
    else { Serial.println(F("OK")); }
    }
  else { Serial.println(); } // Terminate empty/partial CLI input line after timeout.

  // Force any pending output before return / possible UART power-down.
  flushSerialSCTSensitive();

  if(neededWaking) { powerDownSerial(); }
  }



// CUSTOM IO FOR SPECIAL DEPLOYMENTS
#ifdef ALLOW_CC1_SUPPORT_RELAY_IO // REV9 CC1 relay...
// Do basic static LED setting.
static void setLEDs(const uint8_t lc)
    {
    // Assume primary UI LED is the red one (at least fot REV9 boards)...
    if(lc & 1) { LED_HEATCALL_ON(); } else { LED_HEATCALL_OFF(); }
    // Assume secondary UI LED is the green one (at least fot REV9 boards)...
    if(lc & 2) { LED_UI2_ON(); } else { LED_UI2_OFF(); }
    }

// Count down in 2s ticks until LEDs go out.
static uint8_t countDownLEDSforCO;

// Call this on even numbered seconds (with current time in seconds) to allow the CO UI to operate.
// Should never be skipped, so as to allow the UI to remain responsive.
bool tickUICO(const uint_fast8_t sec)
  {
  // All LEDs off when count-down timer hits zero.
  if((0 == countDownLEDSforCO) || (0 == --countDownLEDSforCO)) { setLEDs(0); }
  return(false); // No human interaction this tick...
  }

// Directly adjust LEDs.
//   * light-colour         [0,3] bit flags 1==red 2==green (lc) 0 => stop everything
//   * light-on-time        [1,15] (0 not allowed) 30-450s in units of 30s (lt) ???
//   * light-flash          [1,3] (0 not allowed) 1==single 2==double 3==on (lf)
void setLEDsCO(const uint8_t lc, const uint8_t lt, const uint8_t lf)
    {
    countDownLEDSforCO = lt * 15; // Units are 30s, ticks are 2s.
    setLEDs(lc);
    }
#endif
