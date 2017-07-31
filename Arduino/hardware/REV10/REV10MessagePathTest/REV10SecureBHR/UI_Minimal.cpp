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

#if defined(valveUI_DEFINED)
// Valve physical UI controller.
valveUI_t valveUI(
  &valveMode,
  &tempControl,
  &NominalRadValve,
#ifdef ENABLE_OCCUPANCY_SUPPORT
  &Occupancy,
#else
  (OTV0P2BASE::PseudoSensorOccupancyTracker*)NULL,
#endif
#ifdef ENABLE_AMBLIGHT_SENSOR
  &AmbLight,
#else
  (OTV0P2BASE::SensorAmbientLight *)NULL,
#endif
#if defined(TEMP_POT_AVAILABLE) // Eg REV2/REV7.
  &TempPot,
#else
  NULL,
#endif
  &Supply_cV,
  OTV0P2BASE::LED_HEATCALL_ON,
  OTV0P2BASE::LED_HEATCALL_OFF,
  OTV0P2BASE::LED_HEATCALL_ON_ISR_SAFE);
#endif // valveUI_DEFINED


#ifdef ENABLE_EXTENDED_CLI
// Handle CLI extension commands.
// Commands of form:
//   +EXT .....
// where EXT is the name of the extension, usually 3 letters.

// It is acceptable for extCLIHandler() to alter the buffer passed,
// eg with strtok_t().
static bool extCLIHandler(Print *const p, char *const buf, const uint8_t n)
  {
  return(false); // FAILED if not otherwise handled.
  }
#endif 


#if defined(ENABLE_CLI_HELP) && !defined(ENABLE_TRIMMED_MEMORY)
#define _CLI_HELP_
static constexpr uint8_t SYNTAX_COL_WIDTH = 10; // Width of 'syntax' column; strictly positive.
// Estimated maximum overhead in sub-cycle ticks to print full line and all trailing CLI summary info.
static constexpr uint8_t CLI_PRINT_OH_SCT = ((uint8_t)(OTV0P2BASE::GSCT_MAX/4));
// Deadline in minor cycle by which to stop printing description.
static constexpr uint8_t STOP_PRINTING_DESCRIPTION_AT = ((uint8_t)(OTV0P2BASE::GSCT_MAX-CLI_PRINT_OH_SCT));
// Efficiently print a single line given the syntax element and the description, both non-null.
// NOTE: will skip the description if getting close to the end of the time deadline, in order to avoid overrun.
static void printCLILine(const uint8_t deadline, __FlashStringHelper const *syntax, __FlashStringHelper const *description)
  {
  Serial.print(syntax);
  OTV0P2BASE::flushSerialProductive(); // Ensure all pending output is flushed before sampling current position in minor cycle.
  if(OTV0P2BASE::getSubCycleTime() >= deadline) { Serial.println(); return; }
  for(int8_t padding = SYNTAX_COL_WIDTH - strlen_P((const char *)syntax); --padding >= 0; ) { OTV0P2BASE::Serial_print_space(); }
  Serial.println(description);
  }
// Efficiently print a single line given a single-char syntax element and the description, both non-null.
// NOTE: will skip the description if getting close to the end of the time deadline to avoid overrun.
static void printCLILine(const uint8_t deadline, const char syntax, __FlashStringHelper const *description)
  {
  Serial.print(syntax);
  OTV0P2BASE::flushSerialProductive(); // Ensure all pending output is flushed before sampling current position in minor cycle.
  if(OTV0P2BASE::getSubCycleTime() >= deadline) { Serial.println(); return; }
  for(int8_t padding = SYNTAX_COL_WIDTH - 1; --padding >= 0; ) { OTV0P2BASE::Serial_print_space(); }
  Serial.println(description);
  }
#endif // defined(ENABLE_CLI_HELP) && !defined(ENABLE_TRIMMED_MEMORY)

// Dump some brief CLI usage instructions to serial TX, which must be up and running.
// If this gets too big there is a risk of overrunning and missing the next tick...
static void dumpCLIUsage(const uint8_t stopBy)
  {
#ifndef _CLI_HELP_
  OTV0P2BASE::CLI::InvalidIgnored(); // Minimal placeholder.
#else
  const uint8_t deadline = OTV0P2BASE::fnmin((uint8_t)(stopBy - OTV0P2BASE::fnmin(stopBy,CLI_PRINT_OH_SCT)), STOP_PRINTING_DESCRIPTION_AT);
  Serial.println();
  //Serial.println(F("CLI usage:"));
  printCLILine(deadline, '?', F("this help"));
  
  // Core CLI features first... (E, [H], I, S V)
  printCLILine(deadline, 'E', F("Exit CLI"));
#if defined(ENABLE_FHT8VSIMPLE) && defined(ENABLE_LOCAL_TRV)
  printCLILine(deadline, F("H H1 H2"), F("set FHT8V House codes 1&2"));
  printCLILine(deadline, 'H', F("clear House codes"));
#endif
  printCLILine(deadline, F("I *"), F("create new ID"));
  printCLILine(deadline, 'S', F("show Status"));
  printCLILine(deadline, 'V', F("sys Version"));
#ifdef ENABLE_GENERIC_PARAM_CLI_ACCESS
  printCLILine(deadline, F("G N [M]"), F("Show [set] generic param N [to M]")); // *******
#endif

#ifdef ENABLE_FULL_OT_CLI
  // Optional CLI features...
  Serial.println(F("-"));
#if defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)
  printCLILine(deadline, F("C M"), F("Central hub >=M mins on, 0 off"));
#endif
  printCLILine(deadline, F("D N"), F("Dump stats set N"));
  printCLILine(deadline, 'F', F("Frost"));
#if defined(ENABLE_SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
  printCLILine(deadline, F("F CC"), F("set Frost/setback temp CC"));
#endif

  //printCLILine(deadline, 'L', F("Learn to warm every 24h from now, clear if in frost mode, schedule 0"));
#if defined(SCHEDULER_AVAILABLE)
  printCLILine(deadline, F("L S"), F("Learn daily warm now, clear if in frost mode, schedule S"));
  //printCLILine(deadline, F("P HH MM"), F("Program: warm daily starting at HH MM schedule 0"));
  printCLILine(deadline, F("P HH MM S"), F("Program: warm daily starting at HH MM schedule S"));
#endif
  printCLILine(deadline, F("O PP"), F("min % for valve to be Open"));
#if defined(ENABLE_NOMINAL_RAD_VALVE)
  printCLILine(deadline, 'O', F("reset Open %"));
#endif
  printCLILine(deadline, 'Q', F("Quick Heat"));
//  printCLILine(deadline, F("R N"), F("dump Raw stats set N"));

  printCLILine(deadline, F("T HH MM"), F("set 24h Time"));
  printCLILine(deadline, 'W', F("Warm"));
#if defined(ENABLE_SETTABLE_TARGET_TEMPERATURES) && !defined(TEMP_POT_AVAILABLE)
  printCLILine(deadline, F("W CC"), F("set Warm temp CC"));
#endif
#if !defined(ENABLE_ALWAYS_TX_ALL_STATS)
  printCLILine(deadline, 'X', F("Xmit security level; 0 always, 255 never"));
#endif
  printCLILine(deadline, 'Z', F("Zap stats"));
#endif // ENABLE_FULL_OT_CLI
#endif // ENABLE_CLI_HELP
  Serial.println();
  }

//#if defined(ENABLE_EXTENDED_CLI) || defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
//static const uint8_t MAXIMUM_CLI_RESPONSE_CHARS = 1 + OTV0P2BASE::CLI::MAX_TYPICAL_CLI_BUFFER;
//#else
//static const uint8_t MAXIMUM_CLI_RESPONSE_CHARS = 1 + OTV0P2BASE::CLI::MIN_TYPICAL_CLI_BUFFER;
//#endif
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

#if defined(ENABLE_FHT8VSIMPLE) && (defined(ENABLE_LOCAL_TRV) || defined(ENABLE_SLAVE_TRV))
      // H [nn nn]
      // Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
      // Missing values will clear the code entirely (and disable use of the valve).
      case 'H': { showStatus = OTRadValve::FHT8VRadValveBase::SetHouseCode(&FHT8V).doCommand(buf, n); break; }
#endif

#if defined(ENABLE_GENERIC_PARAM_CLI_ACCESS)
      // Show/set generic parameter values (eg "G N [M]").
      case 'G': { showStatus = OTV0P2BASE::CLI::GenericParam().doCommand(buf, n); break; }
#endif

      // Reset or display ID.
#ifdef ENABLE_ID_SET_FROM_CLI
      case 'I': { showStatus = OTV0P2BASE::CLI::NodeIDWithSet().doCommand(buf, n); break; }
#else
      case 'I': { showStatus = OTV0P2BASE::CLI::NodeID().doCommand(buf, n); break; }
#endif

      // Status line stats print and TX.
      case 'S':
        {
#if !defined(ENABLE_WATCHDOG_SLOW)
        Serial.print(F("Resets/overruns: "));
#else
        Serial.print(F("Resets: "));
#endif // !defined(ENABLE_WATCHDOG_SLOW) 
        const uint8_t resetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
        Serial.print(resetCount);
#if !defined(ENABLE_WATCHDOG_SLOW)
        Serial.print(' ');
        const uint8_t overrunCount = (~eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_OVERRUN_COUNTER)) & 0xff;
        Serial.print(overrunCount);
#endif // !defined(ENABLE_WATCHDOG_SLOW)
        Serial.println();
        // Show stack headroom.
        OTV0P2BASE::serialPrintAndFlush(F("SH ")); OTV0P2BASE::serialPrintAndFlush(OTV0P2BASE::MemoryChecks::getMinSPSpaceBelowStackToEnd()); OTV0P2BASE::serialPrintlnAndFlush();
#if defined(ENABLE_STATS_TX)
        // Default light-weight print and TX of stats.
        bareStatsTX();
#endif
        break; // Note that status is by default printed after processing input line.
        }

#if !defined(ENABLE_TRIMMED_MEMORY)
      // Version information printed as one line to serial, machine- and human- parseable.
      case 'V':
        {
        V0p2Base_serialPrintlnBuildVersion();
#if defined(DEBUG) && defined(ENABLE_EXTENDED_CLI) // && !defined(ENABLE_TRIMMED_MEMORY)
        // Allow for much longer input commands for extended CLI.
        Serial.print(F("Ext CLI max chars: ")); Serial.println(MAXIMUM_CLI_RESPONSE_CHARS);
#endif
        break;
        }
#endif // !defined(ENABLE_TRIMMED_MEMORY)

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

#ifdef ENABLE_FULL_OT_CLI // *******  NON-CORE CLI FEATURES

#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT) && (defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)) && defined(ENABLE_RADIO_RX)
      // Set new node association (nodes to accept frames from).
      // Only needed if able to RX and/or some sort of hub.
      case 'A': { showStatus = OTV0P2BASE::CLI::SetNodeAssoc().doCommand(buf, n); break; }
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT

#if defined(ENABLE_RADIO_RX) && (defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)) && !defined(ENABLE_DEFAULT_ALWAYS_RX)
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
          hubManager.setMinBoilerOnMinutes(m);
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
//            { Serial.print(getByHourStat(setN, hh)); OTV0P2BASE::Serial_print_space(); }
//          Serial.println();
//          }
//        break;
//        }

#if !defined(ENABLE_TRIMMED_MEMORY)
      // Dump (human-friendly) stats: D N
      case 'D': { showStatus = OTV0P2BASE::CLI::DumpStats().doCommand(buf, n); break; }
#endif

#if defined(ENABLE_LOCAL_TRV)
      // Switch to FROST mode OR set FROST/setback temperature (even with temp pot available).
      // With F! force to frost and holiday (long-vacant) mode.  Useful for testing and for remote CLI use.
      case 'F':
        {
#if defined(ENABLE_OCCUPANCY_SUPPORT) && !defined(ENABLE_TRIMMED_MEMORY)
        if((n == 2) && ('!' == buf[1]))
          {
          Serial.println(F("hols"));
          Occupancy.setHolidayMode();
          }
#endif
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
#endif // defined(ENABLE_LOCAL_TRV)
 
#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      // Set secret key.
      /**
       * @note  The OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond
       *        function pointer MUST be passed here to ensure safe handling of the key and the Tx message
       *        counter.
       */
      case 'K': { showStatus = OTV0P2BASE::CLI::SetSecretKey(OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond).doCommand(buf, n); break; }
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT

// FIXME
//#ifdef ENABLE_LEARN_BUTTON
//      // Learn current settings, just as if primary/specified LEARN button had been pressed.
//      case 'L':
//        {
//        int s = 0;
////#if MAX_SIMPLE_SCHEDULES > 1
//        char *last; // Used by strtok_r().
//        char *tok1;
//        // Minimum 3 character sequence makes sense and is safe to tokenise, eg "L 0".
//        if((n >= 3) && (NULL != (tok1 = strtok_r(buf+2, " ", &last))))
//          {
//          s = atoi(tok1);
//          }
////#endif
//        valveUI.handleLEARN((uint8_t) s); break;
//        break;
//        }
//#endif // ENABLE_LEARN_BUTTON

#if defined(ENABLE_NOMINAL_RAD_VALVE) && !defined(ENABLE_TRIMMED_MEMORY)
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

#ifdef ENABLE_LEARN_BUTTON
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
            if(!Scheduler.setSimpleSchedule((uint_least16_t) ((60 * hh) + mm), (uint8_t)s)) { OTV0P2BASE::CLI::InvalidIgnored(); }
            }
          }
        break;
        }
#endif // ENABLE_LEARN_BUTTON

#if defined(ENABLE_LOCAL_TRV) && !defined(ENABLE_TRIMMED_MEMORY)
      // Switch to (or restart) BAKE (Quick Heat) mode: Q
      // We can live without this if very short of memory.
      case 'Q': { valveMode.startBake(); break; }
#endif

#if !defined(ENABLE_TRIMMED_MEMORY)
      // Time set T HH MM.
      case 'T': { showStatus = OTV0P2BASE::CLI::SetTime().doCommand(buf, n); break; }
#endif // !defined(ENABLE_TRIMMED_MEMORY)

#if defined(ENABLE_LOCAL_TRV)
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
#endif // defined(ENABLE_LOCAL_TRV)

#if !defined(ENABLE_ALWAYS_TX_ALL_STATS)
      // TX security/privacy level: X NN
      // Avoid showing status afterwards as may already be rather a lot of output.
      case 'X': { showStatus = OTV0P2BASE::CLI::SetTXPrivacy().doCommand(buf, n); break; }
#endif

#if defined(ENABLE_LOCAL_TRV)
      // Zap/erase learned statistics.
      case 'Z': { showStatus = OTV0P2BASE::CLI::ZapStats().doCommand(buf, n); break; }
#endif // defined(ENABLE_LOCAL_TRV)

#endif // ENABLE_FULL_OT_CLI // NON-CORE FEATURES
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
