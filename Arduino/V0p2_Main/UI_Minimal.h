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

#ifndef UI_MINIMAL_H
#define UI_MINIMAL_H

#include <stdint.h>

#include "V0p2_Main.h"
#include "Control.h"


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


// Check/apply the user's schedule, at least once each minute, and act on any timed events.
#if defined(SCHEDULER_AVAILABLE)
void checkUserSchedule();
#else
#define checkUserSchedule() // Reduce to a no-op.
#endif // defined(SCHEDULER_AVAILABLE)


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

#endif

