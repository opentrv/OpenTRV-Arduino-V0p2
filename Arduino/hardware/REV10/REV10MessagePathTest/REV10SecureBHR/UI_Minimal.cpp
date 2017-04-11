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

#include "REV10SecureBHR.h"

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

      // Reset or display ID.
      case 'I': { showStatus = OTV0P2BASE::CLI::NodeID().doCommand(buf, n); break; }

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


      // Set new node association (nodes to accept frames from).
      // Only needed if able to RX and/or some sort of hub.
     case 'A': { showStatus = OTV0P2BASE::CLI::SetNodeAssoc().doCommand(buf, n); break; }

// XXX
//#if defined(ENABLE_RADIO_RX) && (defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)) && !defined(ENABLE_DEFAULT_ALWAYS_RX)
//      // C M
//      // Set central-hub boiler minimum on (and off) time; 0 to disable.
//      case 'C':
//        {
//        char *last; // Used by strtok_r().
//        char *tok1;
//        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "C 0".
//        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
//          {
//          const uint8_t m = (uint8_t) atoi(tok1);
//          setMinBoilerOnMinutes(m);
//          }
//        break;
//        }
//#endif
 
      // Set secret key.
      /**
       * @note  The OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond
       *        function pointer MUST be passed here to ensure safe handling of the key and the Tx message
       *        counter.
       */
      case 'K': { showStatus = OTV0P2BASE::CLI::SetSecretKey(OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond).doCommand(buf, n); break; }
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
