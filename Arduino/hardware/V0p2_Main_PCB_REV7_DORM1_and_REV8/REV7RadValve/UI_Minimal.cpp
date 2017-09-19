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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2017
*/

/*
 Implementation of minimal UI using single LED
 and one or more momentary push-buttons, etc, plus CLI.
 */

#include "V0p2_Main.h"

// Valve physical UI controller.
valveUI_t valveUI(
  &valveMode,
  &tempControl,
  &NominalRadValve,
  &Occupancy,
  &AmbLight,
#if defined(TEMP_POT_AVAILABLE) // Eg REV2/REV7.
  &TempPot,
#else
  NULL,
#endif
  &Supply_cV,
  OTV0P2BASE::LED_HEATCALL_ON,
  OTV0P2BASE::LED_HEATCALL_OFF,
  OTV0P2BASE::LED_HEATCALL_ON_ISR_SAFE);


// Dump some brief CLI usage instructions to serial TX, which must be up and running.
// If this gets too big there is a risk of overrunning and missing the next tick...
static void dumpCLIUsage(const uint8_t stopBy)
  {
  OTV0P2BASE::CLI::InvalidIgnored(); // Minimal placeholder.
  Serial.println();
  }

// Used to poll user side for CLI input until specified sub-cycle time.
// Commands should be sent terminated by CR *or* LF; both may prevent 'E' (exit) from working properly.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// Times itself out after at least a minute or two of inactivity. 
// NOT RENTRANT (eg uses static state for speed and code space).
void pollCLI(const uint8_t maxSCT, const bool startOfMinute, const OTV0P2BASE::ScratchSpace &s)
  {
  // Perform any once-per-minute operations.
  if(startOfMinute)
    { OTV0P2BASE::CLI::countDownCLI(); }

  const bool neededWaking = OTV0P2BASE::powerUpSerialIfDisabled<V0P2_UART_BAUD>();

  // Wait for input command line from the user (received characters may already have been queued)...
  // Read a line up to a terminating CR, either on its own or as part of CRLF.
  // (Note that command content and timing may be useful to fold into PRNG entropy pool.)
  // A static buffer generates better code but permanently consumes previous SRAM.
  const uint8_t n = OTV0P2BASE::CLI::promptAndReadCommandLine(maxSCT, s, [](){pollIO();});
  char *buf = (char *)s.buf;
//  const uint8_t bufsize = s.bufsize;

  if(n > 0)
    {
    // Got plausible input so keep the CLI awake a little longer.
    OTV0P2BASE::CLI::resetCLIActiveTimer();

    // Process the input received, with action based on the first char...
    bool showStatus = true; // Default to showing status.
    switch(buf[0])
      {
      // Explicit request for help, or unrecognised first character.
      // Avoid showing status as may already be rather a lot of output.
      default: case '?': { dumpCLIUsage(maxSCT); showStatus = false; break; }

      // Exit/deactivate CLI immediately.
      // This should be followed by JUST CR ('\r') OR LF ('\n')
      // else the second will wake the CLI up again.
      case 'E': { OTV0P2BASE::CLI::makeCLIInactive(); break; }

#if defined(ENABLE_GENERIC_PARAM_CLI_ACCESS)
      // Show/set generic parameter values (eg "G N [M]").
      case 'G': { showStatus = OTV0P2BASE::CLI::GenericParam().doCommand(buf, n); break; }
#endif

      // Reset or display ID.
      case 'I': { showStatus = OTV0P2BASE::CLI::NodeIDWithSet().doCommand(buf, n); break; }

      // Status line stats print and TX.
      case 'S':
        {
        Serial.print(F("Resets: "));
        const uint8_t resetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
        Serial.print(resetCount);
        Serial.println();
        // Show stack headroom.
        OTV0P2BASE::serialPrintAndFlush(F("SH ")); OTV0P2BASE::serialPrintAndFlush(OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd()); OTV0P2BASE::serialPrintlnAndFlush();
        // Default light-weight print and TX of stats.
        bareStatsTX();
        break; // Note that status is by default printed after processing input line.
        }

      // Switch to FROST mode OR set FROST/setback temperature (even with temp pot available).
      // With F! force to frost and holiday (long-vacant) mode.  Useful for testing and for remote CLI use.
      case 'F':
        {
#if defined(ENABLE_SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
        char *last; // Used by strtok_r().
        char *tok1;
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t tempC = (uint8_t) atoi(tok1);
          if(!tempControl.setFROSTTargetC(tempC)) { OTV0P2BASE::CLI::InvalidIgnored(); }
          }
        else
#endif
          { valveMode.setWarmModeDebounced(false); } // No parameter supplied; switch to FROST mode.
        break;
        }
      // Set secret key.
      /**
       * @note  The OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond
       *        function pointer MUST be passed here to ensure safe handling of the key and the Tx message
       *        counter.
       */
      case 'K': { showStatus = OTV0P2BASE::CLI::SetSecretKey(OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond).doCommand(buf, n); break; }

      // Switch to WARM (not BAKE) mode OR set WARM temperature.
      case 'W':
        {
#if defined(ENABLE_SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
        char *last; // Used by strtok_r().
        char *tok1;
        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
          {
          const uint8_t tempC = (uint8_t) atoi(tok1);
          if(!tempControl.setWARMTargetC(tempC)) { OTV0P2BASE::CLI::InvalidIgnored(); }
          }
        else
#endif
          {
          valveMode.cancelBakeDebounced(); // Ensure BAKE mode not entered.
          valveMode.setWarmModeDebounced(true); // No parameter supplied; switch to WARM mode.
          }
        break;
        }

#if !defined(ENABLE_ALWAYS_TX_ALL_STATS)
      // TX security/privacy level: X NN
      // Avoid showing status afterwards as may already be rather a lot of output.
      case 'X': { showStatus = OTV0P2BASE::CLI::SetTXPrivacy().doCommand(buf, n); break; }
#endif

      // Zap/erase learned statistics.
      case 'Z': { showStatus = OTV0P2BASE::CLI::ZapStats().doCommand(buf, n); break; }
      }

    // Almost always show status line afterwards as feedback of command received and new state.
    if(showStatus) { serialStatusReport(); }
    // Else show ack of command received.
    else { Serial.println(F("OK")); }
    }
  else { Serial.println(); } // Terminate empty/partial CLI input line after timeout.

  // Force any pending output before return / possible UART power-down.
  OTV0P2BASE::flushSerialSCTSensitive();

  if(neededWaking) { OTV0P2BASE::powerDownSerial(); }
  }
