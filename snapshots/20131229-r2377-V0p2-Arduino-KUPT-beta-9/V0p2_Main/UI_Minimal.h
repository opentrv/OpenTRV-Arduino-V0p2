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
 
 ; UI DESCRIPTION (derived from V0.09 PICAXE code)
 ; Button causes cycling through 'off'/'frost' target of 5C, 'warm' target of ~18C,
 ; and an optional 'bake' mode that raises the target temperature to up to ~24C
 ; for up to ~30 minutes or until the target is hit then reverts to 'warm' automatically.
 ; (Button may have to be held down for up to a few seconds to get the unit's attention.)
 ; As of 2013/12/15 acknowledgement single/double/triple flash in new mode.
 ;; Up to 2013/12/14, acknowledgement was medium/long/double flash in new mode
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


// If true then the unit is in 'warm' (heating) mode, else 'frost' protection mode.
bool inWarmMode();

#ifdef SUPPORT_BAKE // IF DEFINED: this unit supports BAKE mode.
// If true then the unit is in 'bake' mode, a subset of 'warm' mode which boosts the temperature target temporarily.
bool inBakeMode();

// Cancel 'bake' mode if active.
void cancelBake();
#else
#define inBakeMode() (false)
#define cancelBake() {}
#endif


// If true (the default) then the system has an 'Eco' energy-saving bias, else it has a 'comfort' bias.
// Several system parameters are adjusted depending on the bias,
// with 'eco' slanted toward saving energy, eg with lower target temperatures and shorter on-times.
// At the transition from one bias to the other user-settable values may be adjusted to match.
bool hasEcoBias();


// Check/apply the user's schedule, at least once each minute, and act on any timed events.
void checkUserSchedule();


// Sends a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
// Should be similar to PICAXE V0.1 output to allow the same parser to handle either.
void serialStatusReport();

// How long the CLI stays listening for input after startup, button push, or last command; seconds, strictly positive.
// May have to fit in a signed byte, so no larger than 127.
// Keeping this high makes CLI interaction easier but wastes energy
// and may also disrupt RX polling at the hub and so on.
#define CLIActiveS 60

// Character that should trigger any pending command from user to be sent.
#define CLIPromptChar ('>') // Printable ASCII char that should be avoided in status output.

// Returns true if the CLI is (or should currently be) active, at least intermittently.
bool isCLIActive();

// Used to poll user side for CLI input until specified sub-cycle time.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// NOT RENTRANT (eg uses static state for speed and code space).
void pollCLI(const uint8_t maxSCT);
#define CLI_POLL_MIN_SCT (100/SUBCYCLE_TICK_MS_RN) // Minimum recommended poll time in sub-cycle ticks...



// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#ifdef OCCUPANCY_SUPPORT
// Returns true if the room appears to be likely occupied (with active users) recently.
// This uses the same timer as isOccupied() (restarted by markAsOccupied())
// but returns to false somewhat sooner for example to allow ramping up more costly occupancy detection methods
// and to allow some simple graduated occupancy responses.
// Do not call from an ISR.
bool isLikelyRecentlyOccupied();

// Returns true if the estimated likelyhood of occupancy is deminishing
// and expending effort above a basic level to check for continuing occupancy is worthwhile.
#define increaseCheckForOccupancy() (!isLikelyRecentlyOccupied() && isLikelyOccupied())

// Returns true if the room appears to be likely occupied (with active users) now or recently.
// Operates on a timeout; calling markAsOccupied() restarts the timer.
// Defaults to false (and API still exists) when OCCUPANCY_SUPPORT not defined.
// Do not call from an ISR.
bool isLikelyOccupied();

// False if room likely currently unoccupied (no active users).
// Defaults to false (and API still exists) when OCCUPANCY_SUPPORT not defined.
// This may require a substantial timeout (many hours) of inactivity to become true.
// This and isLikelyOccupied() cannot be true together; it is possible for neither to be true.
// Do not call from an ISR.
#define isLikelyUnoccupied() (!isLikelyOccupied())

// Call when some strong evidence of room occupation and human activity has occurred.
// Such evidence may include operation of buttons (etc) on the unit or PIR.
// Do not call from (for example) 'on' schedule change.
// Do not call from an ISR.
void markAsOccupied();
#else
#define markAsOccupied() // Defined as NO-OP for convenience when no general occupancy support.
#define isLikelyOccupied() (false) // Always false without OCCUPANCY_SUPPORT
#define isLikelyUnoccupied() (false) // Always false without OCCUPANCY_SUPPORT
#endif


// Returns true if system is in 'learn'/smart mode.
// If in 'smart' mode can anticipate user demand to pre-warm rooms, maintain customary temperatures, etc.
bool inSmartMode();


#endif


