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



// Call this on even numbered seconds (with current time in seconds) to allow the UI to operate.
// Should never be skipped, so as to allow the UI to remain responsive.
// Runs in 350ms or less; usually takes only a few milliseconds or microseconds.
// Returns true iff the user interacted with the system, and maybe caused a status change.
// NOTE: since this is on the minimum idle-loop code path, minimise CPU cycles, esp in frost mode.
// Also re-activates CLI on main button push.
bool tickUI(uint_fast8_t sec);




// Record local manual operation of a local physical UI control, eg not remote or via CLI.
// Thread-safe.
void markUIControlUsed();

// True if a manual UI control has been very recently (seconds to minutes ago) operated.
// The user may still be interacting with the control and the UI etc should probably be extra responsive.
// Thread-safe.
bool veryRecentUIControlUse();

// True if a manual UI control has been recently (tens of minutes ago) operated.
// If true then local manual settings should 'win' in any conflict with programmed or remote ones.
// For example, remote requests to override settings may be ignored while this is true.
// Thread-safe....
bool recentUIControlUse();

// Check/apply the user's schedule, at least once each minute, and act on any timed events.
void checkUserSchedule();


// Sends a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
// Should be similar to PICAXE V0.1 output to allow the same parser to handle either.
void serialStatusReport();

// Character that should trigger any pending command from user to be sent.
#define CLIPromptChar (LINE_START_CHAR_CLI) // Printable ASCII char that should be avoided in status output.

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

// Minimum recommended poll time in sub-cycle ticks...
#define CLI_POLL_MIN_SCT (200/SUBCYCLE_TICK_MS_RN)

#endif

