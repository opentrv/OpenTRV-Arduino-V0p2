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
 
 Author(s) / Copyright (s): Damon Hart-Davis 2013--2016
 */

/*
 Implementation of minimal UI using single LED and one or two momentary push-buttons.
 
 ; UI DESCRIPTION (derived from V0.09 PICAXE code)
 ; Button causes cycling through 'off'/'frost' target of 5C, 'warm' target of ~18C,
 ; and an optional 'bake' mode that raises the target temperature to up to ~24C
 ; for up to ~30 minutes or until the target is hit then reverts to 'warm' automatically.
 ; (Button may have to be held down for up to a few seconds to get the unit's attention.)
 ; As of 2013/12/15 acknowledgment single/double/triple flash in new mode.
 ;; Up to 2013/12/14, acknowledgment was medium/long/double flash in new mode
 ;; (medium is frost, long is 'warm', long + second flash is 'bake').
 
 ; Without the button pressed,
 ; the unit generates one to three short flashes on a two-second cycle if in heat mode.
 ; A first flash indicates "warm mode".  (V0.2: every 4th set of flashes will be dim or omitted if a schedule is set.) 
 ; A second flash if present indicates "calling for heat".
 ; A third flash if present indicates "bake mode" (which is automatically cancelled after a short time, or if the high target is hit).
 
 ; This may optionally support an interactive CLI over the serial connection,
 ; with reprogramming initiation permitted (instead of CLI) while the UI button is held down.
 
 ; If target is not being met then aim to turn TRV on/up and call for heat from the boiler too,
 ; else if target is being met then turn TRV off/down and stop calling for heat from the boiler.
 ; Has a small amount of hysteresis to reduce short-cycling of the boiler.
 ; Does some proportional TRV control as target temperature is neared to reduce overshoot.
 
 ; This can use a simple setback (drops the 'warm' target a little to save energy)
 ; eg using an LDR, ie reasonable ambient light, as a proxy for occupancy.


 NOTE: when communicating to a host over serial, leading punctuation characters are significant,
 and output is line-oriented:
 
  '!' introduces an error.
  '?' introduces a warning.
  '=' introduces a local status message.
  '>' is a CLI prompt.
  '@' introduces a translated (to ASCII7) binary status messages.
  '{' introduces a raw JSON (map) message.
  '+<msgtype> ' introduces a relayed/decoded message of the givens message type.  Note the space. 
 */

#ifndef UI_MINIMAL_H
#define UI_MINIMAL_H

#include <stdint.h>

#include "V0p2_Main.h"
#include "Control.h" // To get ENABLE_NOMINAL_RAD_VALVE etc.


// WIP: valve physical UI controller.
#if defined(ENABLE_LOCAL_TRV) && !defined(NO_UI_SUPPORT)
#define valveUI_DEFINED
  #if defined(ENABLE_SIMPLIFIED_MODE_BAKE)
  typedef OTRadValve::ModeButtonAndPotActuatorPhysicalUI valveUI_t;
  #else
  typedef OTRadValve::CycleModeAndLearnButtonsAndPotActuatorPhysicalUI<BUTTON_MODE_L> valveUI_t;
  #endif
extern valveUI_t valveUI;
#endif // ENABLE_LOCAL_TRV && !NO_UI_SUPPORT


// Call this on even numbered seconds (with current time in seconds) to allow the UI to operate.
// Should never be skipped, so as to allow the UI to remain responsive.
// Runs in 350ms or less; usually takes only a few milliseconds or microseconds.
// Returns true iff the user interacted with the system, and maybe caused a status change.
// NOTE: since this is on the minimum idle-loop code path, minimise CPU cycles, esp in frost mode.
// Also re-activates CLI on main button push.
//
#if !defined(BUTTON_MODE_L) || (!defined(ENABLE_LOCAL_TRV) && !defined(ENABLE_SLAVE_TRV))
// If the appropriate button input is not available
// or this is not driving a local TRV (eg because this is a sensor module)
// then disable the usual interactive UI entirely
// (except to ensure the main LED is turned off once per minor cycle).
#define NO_UI_SUPPORT
// Ensure LED forced off unconditionally at least once each cycle.
inline bool tickUI(uint_fast8_t) { OTV0P2BASE::LED_HEATCALL_OFF(); return(false); } // Always false.
#else
bool tickUI(uint_fast8_t sec);
#endif

// Record local manual operation of a physical UI control, eg not remote or via CLI.
// Marks room as occupied amongst other things.
// Thread-safe.
void markUIControlUsed();

// Record significant local manual operation of a physical UI control, eg not remote or via CLI.
// Marks room as occupied amongst other things.
// As markUIControlUsed() but likely to generate some feedback to the user, ASAP.
// Thread-safe.
void markUIControlUsedSignificant();


// True if a manual UI control has been very recently (minutes ago) operated.
// The user may still be interacting with the control and the UI etc should probably be extra responsive.
// Thread-safe.
bool veryRecentUIControlUse();

// True if a manual UI control has been recently (tens of minutes ago) operated.
// If true then local manual settings should 'win' in any conflict with programmed or remote ones.
// For example, remote requests to override settings may be ignored while this is true.
// Thread-safe.
bool recentUIControlUse();

// Check/apply the user's schedule, at least once each minute, and act on any timed events.
#if defined(SCHEDULER_AVAILABLE)
void checkUserSchedule();
#else
#define checkUserSchedule() // Reduce to a no-op.
#endif // defined(SCHEDULER_AVAILABLE)

// UI feedback.
// Provide low-key visual / audio / tactile feedback on a significant user action.
// May take hundreds of milliseconds and noticeable energy.
// By default includes visual feedback,
// but that can be prevented if other visual feedback already in progress.
// Marks the UI as used.
// Not thread-/ISR- safe.
void userOpFeedback(bool includeVisual = true);


// Sends a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
// Should be similar to PICAXE V0.1 output to allow the same parser to handle either.
#ifdef ENABLE_SERIAL_STATUS_REPORT
void serialStatusReport();
#else
#define serialStatusReport() { }
#endif

// Reset CLI active timer to the full whack before it goes inactive again (ie makes CLI active for a while).
// Thread-safe.
void resetCLIActiveTimer();

// Returns true if the CLI is (or should currently be) active, at least intermittently.
// Thread-safe.
bool isCLIActive();

// Used to poll user side for CLI input until specified sub-cycle time.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// Times itself out after at least a minute or two of inactivity. 
// NOT RENTRANT (eg uses static state for speed and code space).
void pollCLI(uint8_t maxSCT, bool startOfMinute);

// Use WDT-based timer for xxxPause() routines.
// Very tiny low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define VERYTINY_PAUSE_MS 5
static void inline veryTinyPause() { OTV0P2BASE::sleepLowPowerMs(VERYTINY_PAUSE_MS); }
// Tiny low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define TINY_PAUSE_MS 15
static void inline tinyPause() { OTV0P2BASE::nap(WDTO_15MS); } // 15ms vs 18ms nominal for PICAXE V0.09 impl.
// Small low-power sleep.
#define SMALL_PAUSE_MS 30
static void inline smallPause() { OTV0P2BASE::nap(WDTO_30MS); }
// Medium low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
// Premature wakeups MAY be allowed to avoid blocking I/O polling for too long.
#define MEDIUM_PAUSE_MS 60
static void inline mediumPause() { OTV0P2BASE::nap(WDTO_60MS); } // 60ms vs 144ms nominal for PICAXE V0.09 impl.
// Big low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
// Premature wakeups MAY be allowed to avoid blocking I/O polling for too long.
#define BIG_PAUSE_MS 120
static void inline bigPause() { OTV0P2BASE::nap(WDTO_120MS); } // 120ms vs 288ms nominal for PICAXE V0.09 impl.


#endif

