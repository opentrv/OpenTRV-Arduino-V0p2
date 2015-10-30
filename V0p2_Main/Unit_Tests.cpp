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
 Development-time unit tests (NOT part of production code).
 
 Tests code and some I/O and sensors.
 
 It should be possible to auto-detect success by looking for a line starting "%%%".

 It should be possible to auto-detect failure by looking for a line starting "***Test FAILED".

 Soak testing is possible by simply letting the tests repeat as is the default;
 the first failure will stop the tests and continue reporting in a loop.

 None of these tests should write to EEPROM or FLASH
 (or perform any other unbounded life-limited operation)
 to avoid wear during soak testing, and thus allow soak testing to run without concern.
 */


#include "V0p2_Main.h"


#ifdef UNIT_TESTS // Exclude unit test code from production systems.

#include <util/atomic.h>

#include "Control.h"

#include "FHT8V_Wireless_Rad_Valve.h"
#include "Messaging.h"
#include "Power_Management.h"
#include "RFM22_Radio.h"
#include "Schedule.h"
#include "Security.h"
#include "Serial_IO.h"
#include "UI_Minimal.h"


// Error exit from failed unit test, one int parameter and the failing line number to print...
// Expects to terminate like panic() with flashing light can be detected by eye or in hardware if required.
static void error(int err, int line)
  {
  for( ; ; )
    {
    OTV0P2BASE::serialPrintAndFlush(F("***Test FAILED*** val="));
    OTV0P2BASE::serialPrintAndFlush(err, DEC);
    OTV0P2BASE::serialPrintAndFlush(F(" =0x"));
    OTV0P2BASE::serialPrintAndFlush(err, HEX);
    if(0 != line)
      {
      OTV0P2BASE::serialPrintAndFlush(F(" at line "));
      OTV0P2BASE::serialPrintAndFlush(line);
      }
    OTV0P2BASE::serialPrintlnAndFlush();
    LED_HEATCALL_ON();
    tinyPause();
    LED_HEATCALL_OFF();
    OTV0P2BASE::sleepLowPowerMs(1000);
    }
  }

// Deal with common equality test.
static inline void errorIfNotEqual(int expected, int actual, int line) { if(expected != actual) { error(actual, line); } }
// Allowing a delta.
static inline void errorIfNotEqual(int expected, int actual, int delta, int line) { if(abs(expected - actual) > delta) { error(actual, line); } }

// Test expression and bucket out with error if false, else continue, including line number.
// Macros allow __LINE__ to work correctly.
#define AssertIsTrueWithErr(x, err) { if(!(x)) { error((err), __LINE__); } }
#define AssertIsTrue(x) AssertIsTrueWithErr((x), 0)
#define AssertIsEqual(expected, x) { errorIfNotEqual((expected), (x), __LINE__); }
#define AssertIsEqualWithDelta(expected, x, delta) { errorIfNotEqual((expected), (x), (delta), __LINE__); }



// Check that correct versions of underlying libraries are in use.
static void testLibVersions()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("LibVersions");
#if !(0 == ARDUINO_LIB_OTV0P2BASE_VERSION_MAJOR) || !(8 <= ARDUINO_LIB_OTV0P2BASE_VERSION_MINOR)
#error Wrong OTV0p2Base library version!
#endif
#if !(0 == ARDUINO_LIB_OTRADIOLINK_VERSION_MAJOR) || !(9 <= ARDUINO_LIB_OTRADIOLINK_VERSION_MINOR)
#error Wrong OTRadioLink library version!
#endif
//  AssertIsEqual(0, ARDUINO_LIB_OTRADIOLINK_VERSION_MAJOR);
//  AssertIsTrue(1 <= ARDUINO_LIB_OTRADIOLINK_VERSION_MINOR); // Minimum acceptable minor version.
#if !(0 == ARDUINO_LIB_OTRFM23BLINK_VERSION_MAJOR) || !(9 <= ARDUINO_LIB_OTRFM23BLINK_VERSION_MINOR)
#error Wrong OTRFM23BLink library version!
#endif
#ifdef ALLOW_CC1_SUPPORT
#if !(0 == ARDUINO_LIB_OTPROTOCOLCC_VERSION_MAJOR) || !(3 <= ARDUINO_LIB_OTPROTOCOLCC_VERSION_MINOR)
#error Wrong OTProtocolCC library version!
#endif
#endif
  }



#ifdef ENABLE_BOILER_HUB
// Test simple on/off boiler-driver behaviour.
static void testOnOffBoilerDriverLogic()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("OnOffBoilerDriver");
  // Ensure status structure is a reasonable size.
  AssertIsTrue(sizeof(OnOffBoilerDriverLogic::PerIDStatus) <= 4);
  OnOffBoilerDriverLogic oobdl1;
  // Verify that power-up state is boiler off.
  AssertIsTrue(!oobdl1.isCallingForHeat());
  // Calling tick one or more times makes no difference by itself...
  for(uint8_t i = 2 + (OTV0P2BASE::randRNG8() & 0x1fu); --i > 0; ) { oobdl1.tick2s(); }
  AssertIsTrue(!oobdl1.isCallingForHeat());
  // Ensure bogus update/signal is rejected.
  AssertIsTrue(!oobdl1.receiveSignal(0xffffu, OTV0P2BASE::randRNG8()));
  AssertIsTrue(!oobdl1.isCallingForHeat());
  // Ensure no 'live' or other records created.
  OnOffBoilerDriverLogic::PerIDStatus valves1[1];
  AssertIsEqual(0, oobdl1.valvesStatus(valves1, 1, OTV0P2BASE::randRNG8NextBoolean()));
  }
#endif



// Test for general sanity of computation of desired valve position.
static void testComputeRequiredTRVPercentOpen()
  {
#ifdef ENABLE_MODELLED_RAD_VALVE
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("ComputeRequiredTRVPercentOpen");
  // Test that if the real temperature is zero
  // and the initial valve position is anything less than 100%
  // that after one tick (with mainly defaults)
  // that the valve is being opened (and more than glacially),
  // ie that when below any possible legal target FROST/WARM/BAKE temperature the valve will open monotonically,
  // and also test that the fully-open state is reached in a bounded number of ticks ie bounded time.
  static const int maxFullTravelMins = 25;
//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("open...");
  ModelledRadValveInputState is0(0);
  is0.targetTempC = OTV0P2BASE::randRNG8NextBoolean() ? FROST : WARM;
  ModelledRadValveState rs0;
  const uint8_t valvePCOpenInitial0 = OTV0P2BASE::randRNG8() % 100;
  volatile uint8_t valvePCOpen = valvePCOpenInitial0;
  for(int i = maxFullTravelMins; --i >= 0; ) // Must fully open in reasonable time.
    {
    // Simulates one minute on each iteration.
    // Futz some input parameters that should not matter.
    is0.widenDeadband = OTV0P2BASE::randRNG8NextBoolean();
    is0.hasEcoBias = OTV0P2BASE::randRNG8NextBoolean();
    const uint8_t oldValvePos = valvePCOpen;
    rs0.tick(valvePCOpen, is0);
    const uint8_t newValvePos = valvePCOpen;
    AssertIsTrue(newValvePos > 0);
    AssertIsTrue(newValvePos <= 100);
    AssertIsTrue(newValvePos > oldValvePos);
    if(oldValvePos < is0.minPCOpen) { AssertIsTrue(is0.minPCOpen <= newValvePos); } // Should open to at least minimum-really-open-% on first step.
    AssertIsTrue(rs0.valveMoved == (oldValvePos != newValvePos));
    if(100 == newValvePos) { break; }
    }
  AssertIsEqual(100, valvePCOpen);
  AssertIsEqual(100 - valvePCOpenInitial0, rs0.cumulativeMovementPC);
  // Equally test that if the temperature is much higher than any legit target
  // the valve will monotonically close to 0% in bounded time.
  // Check for superficially correct linger behaviour:
  //   * minPCOpen-1 % must be hit (lingering close) if starting anywhere above that.
  //   * Once in linger all reductions should be by 1% until possible final jump to 0.
  //   * Check that linger was long enough (if linger threshold is higher enough to allow it).
  // Also check for some correct initialisation and 'velocity'/smoothing behaviour.
//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("close...");
  ModelledRadValveInputState is1(100<<4);
  is1.targetTempC = OTV0P2BASE::randRNG8NextBoolean() ? FROST : WARM;
  ModelledRadValveState rs1;
  AssertIsTrue(!rs1.initialised); // Initialisation not yet complete.
  const uint8_t valvePCOpenInitial1 = 1 + (OTV0P2BASE::randRNG8() % 100);
  valvePCOpen = valvePCOpenInitial1;
  const bool lookForLinger = (valvePCOpenInitial1 >= is1.minPCOpen);
  bool hitLinger = false; // True if the linger value was hit.
  uint8_t lingerMins = 0; // Approx mins spent in linger.
  for(int i = maxFullTravelMins; --i >= 0; ) // Must fully close in reasonable time.
    {
    // Simulates one minute on each iteration.
    // Futz some input parameters that should not matter.
    is1.widenDeadband = OTV0P2BASE::randRNG8NextBoolean();
    is1.hasEcoBias = OTV0P2BASE::randRNG8NextBoolean();
    const uint8_t oldValvePos = valvePCOpen;
    rs1.tick(valvePCOpen, is1);
    const uint8_t newValvePos = valvePCOpen;
    AssertIsTrue(rs1.initialised); // Initialisation must have completed.
    AssertIsTrue(newValvePos < 100);
//    AssertIsTrue(newValvePos >= 0);
    AssertIsTrue(newValvePos < oldValvePos);
    if(hitLinger) { ++lingerMins; }
    if(hitLinger && (0 != newValvePos)) { AssertIsEqual(oldValvePos - 1, newValvePos); }
    if(newValvePos == is1.minPCOpen-1) { hitLinger = true; }
    AssertIsTrue(rs1.valveMoved == (oldValvePos != newValvePos));
    if(0 == newValvePos) { break; }
    }
  AssertIsEqual(0, valvePCOpen);
  AssertIsEqual(valvePCOpenInitial1, rs1.cumulativeMovementPC);
  AssertIsTrue(hitLinger == lookForLinger);
  if(lookForLinger) { AssertIsTrue(lingerMins >= min(is1.minPCOpen, DEFAULT_MAX_RUN_ON_TIME_M)); }
  // Filtering should not have been engaged and velocity should be zero (temperature is flat).
  for(int i = ModelledRadValveState::filterLength; --i >= 0; ) { AssertIsEqual(100<<4, rs1.prevRawTempC16[i]); }
  AssertIsEqual(100<<4, rs1.getSmoothedRecent());
//  AssertIsEqual(0, rs1.getVelocityC16PerTick());
  AssertIsTrue(!rs1.isFiltering);
  // Some tests of basic velocity computation.
//  ModelledRadValveState rs2;
//  // Test with steady rising/falling value.
//  const int step2C16 = (randRNG8() & 0x1f) - 16;
//DEBUG_SERIAL_PRINT(step2C16);
//DEBUG_SERIAL_PRINTLN();
//  const int base2C16 = (FROST + (randRNG8() % (WARM - FROST))) << 16;
//  rs2.prevRawTempC16[0] = base2C16;
//  for(int i = 1; i < ModelledRadValveState::filterLength; ++i)
//    { rs2.prevRawTempC16[i] = rs2.prevRawTempC16[i-1] - step2C16; }
////DEBUG_SERIAL_PRINT(rs2.getVelocityC16PerTick());
////DEBUG_SERIAL_PRINTLN();
//  AssertIsEqualWithDelta(step2C16, rs2.getVelocityC16PerTick(), 2);
  // Test that soft setback works as expected to support dark-based quick setback.
  // ENERGY SAVING RULE TEST (TODO-442 2a: "Setback in WARM mode must happen in dark (quick response) or long vacant room.")
#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION
      //AmbLight._TEST_set_multi_((j != 0) ? 1023 : 0, j != 0);
    // ENERGY SAVING RULE TEST (TODO-442 2a: "Setback in WARM mode must happen in dark (quick response) or long vacant room.")
    ModelledRadValveInputState is3(100<<4);
    is3.targetTempC = WARM;
    // Try a range of (whole-degree) offsets...
    for(int offset = -2; offset <= +2; ++offset)
      {
      // Try soft setback off and on.
      for(int s = 0; s < 2; ++s) 
        {
#if defined(ALLOW_SOFT_SETBACK)
        is3.softSetback = (0 != s);
#endif
        // Other than in the proportional range, valve should unconditionally be driven off/on by gross temperature error.
        if(0 != offset)
          {
          is3.refTempC16 = (is3.targetTempC + offset) << 4;
          // Where adjusted reference temperature is (well) below target, valve should be driven on.
          ModelledRadValveState rs3a;
          valvePCOpen = 0;
          rs3a.tick(valvePCOpen, is3);
//DEBUG_SERIAL_PRINT('@');
//DEBUG_SERIAL_PRINT(offset);
//DEBUG_SERIAL_PRINT(' ');
//DEBUG_SERIAL_PRINT(valvePCOpen);
//DEBUG_SERIAL_PRINTLN();
          AssertIsTrue((offset < 0) ? (valvePCOpen > 0) : (0 == valvePCOpen));
          // Where adjusted reference temperature is (well) above target, valve should be driven off.
          ModelledRadValveState rs3b;
          valvePCOpen = 100;
          rs3b.tick(valvePCOpen, is3);
          AssertIsTrue((offset < 0) ? (100 == valvePCOpen) : (valvePCOpen < 100));
          }
        else
          {
          // Below the half way mark the valve should always be opened (from off), soft setback or not.
          is3.refTempC16 = (is3.targetTempC << 4) + 0x4;
          ModelledRadValveState rs3c;
          valvePCOpen = 0;
          rs3c.tick(valvePCOpen, is3);
          AssertIsTrue(valvePCOpen > 0);
          // Above the half way mark the valve should only be opened without soft setback.
          is3.refTempC16 = (is3.targetTempC << 4) + 0xc;
          ModelledRadValveState rs3d;
          valvePCOpen = 0;
          rs3d.tick(valvePCOpen, is3);
#if 1 /* TODO-453: drift down soft by default */
          AssertIsTrue(0 == valvePCOpen);
#elif defined(ALLOW_SOFT_SETBACK)
          AssertIsTrue(is3.softSetback ? (0 == valvePCOpen) : (valvePCOpen > 0));
#else
          AssertIsTrue(valvePCOpen > 0);
#endif
          }
        }
      }

#if defined(QUICK_DARK_SETBACK_IS_SOFT)

    // TODO
#endif
#endif
#endif // ENABLE_MODELLED_RAD_VALVE
  }


// Test set derived from following status lines from a hard-to-regulate-smoothly unit DHD20141230
// (poor static balancing, direct radiative heat, low thermal mass, insufficiently insulated?):
/*
=F0%@9CC;X0;T12 30 W255 0 F255 0 W18 51 F20 36;S7 7 18;HC65 74;{"@":"414a","L":142,"B|mV":3315,"occ|%":0,"vC|%":0}
>W
=W0%@9CC;X0;T12 30 W255 0 F255 0 W18 51 F20 36;S7 7 18;HC65 74;{"@":"414a","L":142,"B|mV":3315,"occ|%":0,"vC|%":0}
=W0%@9CC;X0;T12 30 W255 0 F255 0 W18 51 F20 36;S7 7 18;HC65 74;{"@":"414a","L":135,"B|mV":3315,"occ|%":0,"vC|%":0}
=W10%@9CC;X0;T12 30 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":135,"B|mV":3315,"occ|%":0,"vC|%":10}
=W20%@9CC;X0;T12 31 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":20,"L":132,"B|mV":3315,"occ|%":0}
=W30%@10C0;X0;T12 32 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":30,"L":129,"B|mV":3315,"occ|%":0}
=W40%@10CB;X0;T12 33 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":131,"vC|%":40,"B|mV":3315,"occ|%":0}
=W45%@11C5;X0;T12 34 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":45,"L":131,"B|mV":3315,"occ|%":0}
=W50%@11CC;X0;T12 35 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":50,"L":139,"B|mV":3315,"occ|%":0}
=W55%@12C2;X0;T12 36 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":55,"L":132,"B|mV":3315,"occ|%":0}
=W60%@12C7;X0;T12 37 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":133,"vC|%":60,"B|mV":3315,"occ|%":0}
=W65%@12CB;X0;T12 38 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":65,"L":130,"B|mV":3315,"occ|%":0}
=W70%@12CF;X0;T12 39 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":70,"L":127,"B|mV":3315,"occ|%":0}
=W75%@13C2;X0;T12 40 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":75,"L":127,"B|mV":3315,"occ|%":0}
=W80%@13C5;X0;T12 41 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":124,"vC|%":80,"B|mV":3315,"occ|%":0}
=W85%@13C8;X0;T12 42 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":85,"L":121,"B|mV":3315,"occ|%":0}
=W90%@13CB;X0;T12 43 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":90,"L":120,"B|mV":3315,"occ|%":0}
=W95%@13CD;X0;T12 44 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":95,"L":120,"B|mV":3315,"occ|%":0}
=W100%@14C0;X0;T12 45 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":120,"B|mV":3315,"occ|%":0}
=W100%@14C2;X0;T12 46 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":120,"B|mV":3315,"occ|%":0}
=W100%@14C4;X0;T12 47 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":120,"B|mV":3315,"occ|%":0}
=W100%@14C6;X0;T12 48 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":120,"B|mV":3315,"occ|%":0}
=W100%@14C8;X0;T12 49 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":119,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@14CA;X0;T12 50 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":120,"B|mV":3315,"occ|%":0}
=W100%@14CC;X0;T12 51 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":120,"B|mV":3315,"occ|%":0}
=W100%@14CE;X0;T12 52 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":124,"B|mV":3315,"occ|%":0}
=W100%@14CF;X0;T12 53 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":121,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@15C1;X0;T12 54 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":123,"B|mV":3315,"occ|%":0}
=W100%@15C3;X0;T12 55 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":125,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@15C4;X0;T12 56 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":125,"B|mV":3315,"occ|%":0}
=W100%@15C6;X0;T12 57 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":126,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@15C7;X0;T12 58 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":127,"B|mV":3315,"occ|%":0}
=W100%@15C9;X0;T12 59 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":128,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@15CA;X0;T13 0 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":129,"B|mV":3315,"occ|%":0}
=W100%@15CB;X0;T13 1 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":129,"B|mV":3315,"occ|%":0}
=W100%@15CD;X0;T13 2 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":129,"B|mV":3315,"occ|%":0}
=W100%@15CE;X0;T13 3 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":130,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@15CF;X0;T13 4 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":130,"B|mV":3315,"occ|%":0}
=W100%@16C1;X0;T13 5 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":130,"B|mV":3315,"occ|%":0}
=W100%@16C2;X0;T13 6 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":130,"B|mV":3315,"occ|%":0}
=W100%@16C3;X0;T13 7 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":131,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@16C4;X0;T13 8 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":131,"B|mV":3315,"occ|%":0}
=W100%@16C6;X0;T13 9 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":132,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@16C7;X0;T13 10 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":132,"B|mV":3315,"occ|%":0}
=W100%@16C8;X0;T13 11 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":132,"B|mV":3315,"occ|%":0}
=W100%@16C9;X0;T13 12 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":132,"B|mV":3315,"occ|%":0}
=W100%@16CA;X0;T13 13 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":133,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@16CB;X0;T13 14 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":134,"B|mV":3315,"occ|%":0}
=W100%@16CC;X0;T13 15 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":135,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@16CD;X0;T13 16 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":136,"B|mV":3315,"occ|%":0}
=W100%@16CE;X0;T13 17 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":137,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@16CF;X0;T13 18 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":137,"B|mV":3315,"occ|%":0}
=W100%@17C0;X0;T13 19 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":140,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@17C1;X0;T13 20 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":140,"B|mV":3315,"occ|%":0}
=W100%@17C2;X0;T13 21 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":139,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@17C3;X0;T13 22 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":133,"B|mV":3315,"occ|%":0}
=W100%@17C4;X0;T13 23 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":131,"vC|%":100,"B|mV":3315,"occ|%":0}
=W100%@17C5;X0;T13 24 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":130,"B|mV":3315,"occ|%":0}
=W100%@17C5;X0;T13 25 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":130,"B|mV":3315,"occ|%":0}
=W100%@17C6;X0;T13 26 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":128,"B|mV":3315,"occ|%":0}
=W100%@17C7;X0;T13 27 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":128,"B|mV":3315,"occ|%":0}
=W100%@17C8;X0;T13 28 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":100,"L":127,"B|mV":3315,"occ|%":0}
=W95%@17C9;X0;T13 29 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":105,"L":127,"B|mV":3315,"occ|%":0}
=W90%@17CA;X0;T13 30 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":110,"L":127,"B|mV":3315,"occ|%":0}
=W85%@17CB;X0;T13 31 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":125,"vC|%":115,"B|mV":3315,"occ|%":0}
=W80%@17CC;X0;T13 32 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":120,"L":125,"B|mV":3315,"occ|%":0}
=W75%@17CD;X0;T13 33 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":125,"L":125,"B|mV":3315,"occ|%":0}
=W70%@17CD;X0;T13 34 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":130,"L":126,"B|mV":3315,"occ|%":0}
=W65%@17CF;X0;T13 35 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":135,"L":126,"B|mV":3315,"occ|%":0}
=W60%@18C0;X0;T13 36 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":140,"L":126,"B|mV":3315,"occ|%":0}
=W55%@18C0;X0;T13 37 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":124,"vC|%":145,"B|mV":3315,"occ|%":0}
=W50%@18C1;X0;T13 38 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":150,"L":127,"B|mV":3315,"occ|%":0}
=W45%@18C2;X0;T13 39 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":155,"L":127,"B|mV":3315,"occ|%":0}
=W40%@18C3;X0;T13 40 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":160,"L":127,"B|mV":3315,"occ|%":0}
=W35%@18C3;X0;T13 41 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":165,"L":127,"B|mV":3315,"occ|%":0}
=W30%@18C4;X0;T13 42 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":170,"L":128,"B|mV":3315,"occ|%":0}
=W25%@18C5;X0;T13 43 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":130,"vC|%":175,"B|mV":3315,"occ|%":0}
=W20%@18C5;X0;T13 44 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":180,"L":131,"B|mV":3315,"occ|%":0}
=W15%@18C6;X0;T13 45 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":185,"L":131,"B|mV":3315,"occ|%":0}
=W15%@18C7;X0;T13 46 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":185,"L":132,"B|mV":3315,"occ|%":0}
=W9%@18C8;X0;T13 47 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":191,"L":132,"B|mV":3315,"occ|%":0}
=W9%@18C3;X0;T13 48 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":191,"L":134,"B|mV":3315,"occ|%":0}
=W9%@17C9;X0;T13 49 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":191,"L":134,"B|mV":3315,"occ|%":0}
=W9%@17C1;X0;T13 50 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":191,"L":135,"B|mV":3315,"occ|%":0}
=W9%@16CB;X0;T13 51 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":134,"vC|%":191,"B|mV":3315,"occ|%":0}
=W9%@16C6;X0;T13 52 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":191,"L":132,"B|mV":3315,"occ|%":0}
=W9%@16C3;X0;T13 53 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":130,"vC|%":191,"B|mV":3315,"occ|%":0}
=W9%@16C0;X0;T13 54 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":191,"L":127,"B|mV":3315,"occ|%":0}
=W9%@15CD;X0;T13 55 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":125,"vC|%":191,"B|mV":3315,"occ|%":0}
=W10%@15CB;X0;T13 56 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":192,"L":123,"B|mV":3315,"occ|%":0}
=W20%@15CC;X0;T13 57 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":202,"L":119,"B|mV":3315,"occ|%":0}
=W30%@16C5;X0;T13 58 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":212,"L":118,"B|mV":3315,"occ|%":0}
=W40%@16CD;X0;T13 59 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":115,"vC|%":222,"B|mV":3315,"occ|%":0}
=W45%@17C4;X0;T14 0 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":227,"L":113,"B|mV":3315,"occ|%":0}
=W50%@17C8;X0;T14 1 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":232,"L":110,"B|mV":3315,"occ|%":0}
=W55%@17CC;X0;T14 2 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":237,"L":108,"B|mV":3315,"occ|%":0}
=W55%@17CF;X0;T14 3 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":105,"vC|%":237,"B|mV":3315,"occ|%":0}
=W55%@18C1;X0;T14 4 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":237,"L":102,"B|mV":3315,"occ|%":0}
=W50%@18C4;X0;T14 5 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":242,"L":100,"B|mV":3315,"occ|%":0}
=W45%@18C6;X0;T14 6 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":247,"L":98,"B|mV":3315,"occ|%":0}
=W40%@18C7;X0;T14 7 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":252,"L":98,"B|mV":3315,"occ|%":0}
=W9%@18C9;X0;T14 8 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":283,"L":96,"B|mV":3315,"occ|%":0}
=W9%@18C9;X0;T14 8 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":283,"L":96,"B|mV":3315,"occ|%":0}
=W9%@17CC;X0;T14 10 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":283,"L":96,"B|mV":3315,"occ|%":0}
=W9%@17C4;X0;T14 11 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":94,"vC|%":283,"B|mV":3315,"occ|%":0}
=W9%@16CF;X0;T14 12 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":283,"L":95,"B|mV":3315,"occ|%":0}
=W9%@16CB;X0;T14 13 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":91,"vC|%":283,"B|mV":3315,"occ|%":0}
=W9%@16C7;X0;T14 14 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":283,"L":92,"B|mV":3315,"occ|%":0}
=W9%@16C5;X0;T14 15 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":95,"vC|%":283,"B|mV":3315,"occ|%":0}
=W9%@16C3;X0;T14 16 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":283,"L":98,"B|mV":3315,"occ|%":0}
=W10%@16C1;X0;T14 17 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":284,"L":101,"B|mV":3315,"occ|%":0}
=W20%@16C0;X0;T14 18 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":294,"L":104,"B|mV":3315,"occ|%":0}
=W30%@16C9;X0;T14 19 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":108,"vC|%":304,"B|mV":3315,"occ|%":0}
=W40%@17C2;X0;T14 20 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":314,"L":112,"B|mV":3315,"occ|%":0}
=W45%@17C8;X0;T14 21 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":319,"L":116,"B|mV":3315,"occ|%":0}
=W50%@17CE;X0;T14 22 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":324,"L":118,"B|mV":3315,"occ|%":0}
=W50%@18C2;X0;T14 23 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":121,"vC|%":324,"B|mV":3315,"occ|%":0}
=W50%@18C5;X0;T14 24 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":324,"L":125,"B|mV":3315,"occ|%":0}
=W45%@18C8;X0;T14 25 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":329,"L":127,"B|mV":3315,"occ|%":0}
=W40%@18CB;X0;T14 26 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":334,"L":127,"B|mV":3315,"occ|%":0}
=W9%@18CD;X0;T14 27 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":365,"L":127,"B|mV":3315,"occ|%":0}
=W8%@18C9;X0;T14 28 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":366,"L":130,"B|mV":3315,"occ|%":0}
=W7%@18C0;X0;T14 29 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":168,"vC|%":367,"B|mV":3315,"occ|%":0}
=W7%@17CA;X0;T14 30 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":367,"L":191,"B|mV":3315,"occ|%":0}
=W7%@17C4;X0;T14 31 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":367,"L":191,"B|mV":3315,"occ|%":0}
=W7%@17C0;X0;T14 32 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":367,"L":137,"B|mV":3315,"occ|%":0}
=W7%@16CD;X0;T14 33 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":163,"vC|%":367,"B|mV":3315,"occ|%":0}
=W7%@16CA;X0;T14 34 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":367,"L":140,"B|mV":3315,"occ|%":0}
=W7%@16C8;X0;T14 35 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":133,"vC|%":367,"B|mV":3315,"occ|%":0}
=W7%@16C6;X0;T14 36 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":367,"L":162,"B|mV":3315,"occ|%":0}
=W7%@16C5;X0;T14 37 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":126,"vC|%":367,"B|mV":3315,"occ|%":0}
=W10%@16C3;X0;T14 38 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":370,"L":118,"B|mV":3315,"occ|%":0}
=W20%@16C2;X0;T14 39 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":380,"L":111,"B|mV":3315,"occ|%":0}
=W30%@16C9;X0;T14 40 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":390,"L":108,"B|mV":3315,"occ|%":0}
=W40%@17C2;X0;T14 41 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":107,"vC|%":400,"B|mV":3315,"occ|%":0}
=W45%@17CA;X0;T14 42 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":405,"L":104,"B|mV":3315,"occ|%":0}
=W50%@17CF;X0;T14 43 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":410,"L":102,"B|mV":3315,"occ|%":0}
=W50%@18C4;X0;T14 44 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":410,"L":100,"B|mV":3315,"occ|%":0}
=W50%@18C7;X0;T14 45 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":410,"L":100,"B|mV":3315,"occ|%":0}
=W45%@18CA;X0;T14 46 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":415,"L":100,"B|mV":3315,"occ|%":0}
=W9%@18CD;X0;T14 47 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":97,"vC|%":451,"B|mV":3315,"occ|%":0}
=W8%@18CA;X0;T14 48 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":452,"L":103,"B|mV":3315,"occ|%":0}
=W7%@18C1;X0;T14 49 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":453,"L":103,"B|mV":3315,"occ|%":0}
=W7%@17CB;X0;T14 50 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":453,"L":101,"B|mV":3315,"occ|%":0}
=W7%@17C6;X0;T14 51 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":453,"L":101,"B|mV":3315,"occ|%":0}
=W7%@17C2;X0;T14 52 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":453,"L":97,"B|mV":3315,"occ|%":0}
=W7%@16CF;X0;T14 53 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":93,"vC|%":453,"B|mV":3315,"occ|%":0}
=W7%@16CD;X0;T14 54 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":453,"L":93,"B|mV":3315,"occ|%":0}
=W7%@16CB;X0;T14 55 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":453,"L":93,"B|mV":3315,"occ|%":0}
=W7%@16C9;X0;T14 56 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":453,"L":90,"B|mV":3315,"occ|%":0}
=W7%@16C8;X0;T14 57 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":88,"vC|%":453,"B|mV":3315,"occ|%":0}
=W10%@16C7;X0;T14 58 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":456,"L":86,"B|mV":3315,"occ|%":0}
=W20%@16CB;X0;T14 59 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":466,"L":83,"B|mV":3315,"occ|%":0}
=W30%@17C5;X0;T15 0 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":476,"L":81,"B|mV":3315,"occ|%":0}
=W40%@17CD;X0;T15 1 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":486,"L":81,"B|mV":3315,"occ|%":0}
=W40%@18C3;X0;T15 2 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":486,"L":81,"B|mV":3315,"occ|%":0}
=W40%@18C8;X0;T15 3 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":486,"L":81,"B|mV":3315,"occ|%":0}
=W35%@18CC;X0;T15 4 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":491,"L":78,"B|mV":3315,"occ|%":0}
=W9%@19C0;X0;T15 5 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":517,"L":78,"B|mV":3315,"occ|%":0}
=W8%@18CD;X0;T15 6 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":518,"L":78,"B|mV":3315,"occ|%":0}
=W7%@18C5;X0;T15 7 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":519,"L":78,"B|mV":3315,"occ|%":0}
=W6%@17CE;X0;T15 8 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":520,"L":80,"B|mV":3315,"occ|%":0}
=W6%@17CA;X0;T15 9 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":81,"vC|%":520,"B|mV":3315,"occ|%":0}
=W6%@17C6;X0;T15 10 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":520,"L":81,"B|mV":3315,"occ|%":0}
=W6%@17C1;X0;T15 12 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":520,"L":77,"B|mV":3315,"occ|%":0}
=W6%@16CF;X0;T15 13 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":75,"vC|%":520,"B|mV":3315,"occ|%":0}
=W6%@16CD;X0;T15 14 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":520,"L":75,"B|mV":3315,"occ|%":0}
=W6%@16CC;X0;T15 15 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":73,"vC|%":520,"B|mV":3315,"occ|%":0}
=W6%@16CB;X0;T15 16 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":520,"L":71,"B|mV":3315,"occ|%":0}
=W10%@16CA;X0;T15 17 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":524,"L":71,"B|mV":3315,"occ|%":0}
=W20%@16CA;X0;T15 18 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":534,"L":67,"B|mV":3315,"occ|%":0}
=W30%@17C4;X0;T15 19 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","L":64,"vC|%":544,"B|mV":3315,"occ|%":0}
=W40%@17CC;X0;T15 20 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":554,"L":63,"B|mV":3315,"occ|%":0}
=W45%@18C3;X0;T15 21 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":559,"L":61,"B|mV":3315,"occ|%":0}
=W45%@18C9;X0;T15 22 W255 0 F255 0 W18 51 F20 36;S18 7 18;HC65 74;{"@":"414a","vC|%":559,"L":59,"B|mV":3315,"occ|%":0}
...
*/



// Test basic computation of target temperature and the associated energy saving rules.
// This ensures that basic energy efficiency techniques are functional.
/*
TODO-442:
1a) *No prewarm (eg 'smart' extra heating in FROST mode) in a long-vacant room.
1b) *Never a higher pre-warm/FROST-mode target temperature than WARM-mode target.
1c) *Prewarm temperature must be set back from normal WARM target.

2a) *Setback in WARM mode must happen in dark (quick response) or long vacant room.
2b) *Setbacks of up to FULL (3C) must be possible in full eco mode.
2c) *Setbacks are at most 2C in comfort mode (but there is a setback).
2d) Bigger setbacks are possible after a room has been vacant longer (eg for weekends).
2e) Setbacks should be targeted at times of expected low occupancy.
2f) Some setbacks should be possible in office environments with lights mainly or always on.

Starred items are tested.
*/
static void testTargetComputation()
  {
#ifdef ENABLE_MODELLED_RAD_VALVE
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("TargetComputation");
  // For most tests cycle through combinations of:
  //   * base temperature
  //   * light
  //   * schedule activation
  uint8_t maxEcoSetback = 0;
  uint8_t maxComSetback = 0;
  // Systematically work through multiple base temperatures, ending at no override.
  for(int i = _TEST_basetemp_override_MAX+1; --i >= 0; )
    {
    _TEST_set_basetemp_override((_TEST_basetemp_override)i);
#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION
     // Test at high and low light levels; j==0 implies dark (for a little while), j==1 implies light (for a little while).
     for(int j = 2; --j >= 0; )
#endif
      {
#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION
      AmbLight._TEST_set_multi_((j != 0) ? 1023 : 0, j != 0, 15 + (OTV0P2BASE::randRNG8() & 0x7f));
#endif
      // Systematically work through all schedule states, ending at 0 (no override).
      for(int k = _TEST_schedule_override_MAX+1; --k >= 0; )
        {
        _TEST_set_schedule_override((_TEST_schedule_override)k);
        // Some basic invariants should always be met:
        //   * Both FROST and WARM target values legal.
        //   * WARM target never lower than FROST target.
        AssertIsTrue((getFROSTTargetC() >= MIN_TARGET_C) && (getFROSTTargetC() <= MAX_TARGET_C));
        AssertIsTrue((getWARMTargetC() >= MIN_TARGET_C) && (getWARMTargetC() <= MAX_TARGET_C));
        AssertIsTrue(getFROSTTargetC() <= getWARMTargetC());
#ifdef OCCUPANCY_SUPPORT
        // ENERGY SAVING RULE TEST (TODO-442 1a: "No prewarm (eg 'smart' extra heating in FROST mode) in a long-vacant room.")
        // In FROST mode target temperature must stay at frost level once the room has been vacant for a while.
        // IE: pre-warming is disabled in long-vacant rooms as an energy-saving strategy.
        // Set to long long vacant and FROST mode.
//        Occupancy._TEST_set_vacH_(OccupancyTracker::longLongVacantHThrH+1);
        Occupancy.setHolidayMode();
        setWarmModeDebounced(false);
        NominalRadValve.computeTargetTemperature();
        AssertIsEqual(getFROSTTargetC(), NominalRadValve.getTargetTempC());
#endif
        // ENERGY SAVING RULE TEST (TODO-442 1b: "Never a higher pre-warm than WARM target.")
        // Check that in the the target temperature is never higher in FROST than WARM.
        // Perturb the other implicit parameters.
        Occupancy._TEST_set_(OTV0P2BASE::randRNG8NextBoolean());
        setWarmModeDebounced(false);
        NominalRadValve.computeTargetTemperature();
        const uint8_t tTF = NominalRadValve.getTargetTempC();
        setWarmModeDebounced(true);
        NominalRadValve.computeTargetTemperature();
        const uint8_t tTW = NominalRadValve.getTargetTempC();
        AssertIsTrue(tTF <= tTW);
        // ENERGY SAVING RULE TEST (TODO-442 2a: "Setback in WARM mode must happen in dark (quick response) or long vacant room.")
        if(_soUT_off == k)
          {
          setWarmModeDebounced(true);
          if(AmbLight.isRoomDark())
            {
            Occupancy.markAsOccupied();
            NominalRadValve.computeTargetTemperature();
            AssertIsTrue(NominalRadValve.getTargetTempC() < getWARMTargetC()); // Temp must be set back (assumes FROST < WARM).
            }
#if defined(OCCUPANCY_SUPPORT)
          if(!AmbLight.isRoomDark())
            {
            Occupancy.setHolidayMode();
            NominalRadValve.computeTargetTemperature();
            AssertIsTrue(NominalRadValve.getTargetTempC() < getWARMTargetC()); // Temp must be set back (assumes FROST < WARM).
            }
#endif
          }

        // Try to discover/force maximum WARM-mode setback with dark and long vacancy.
        setWarmModeDebounced(true);
#ifdef OCCUPANCY_SUPPORT
//        Occupancy._TEST_set_vacH_(OccupancyTracker::longLongVacantHThrH+1);
        Occupancy.setHolidayMode();
#endif
        NominalRadValve.computeTargetTemperature();
        const int8_t setback = getWARMTargetC() - NominalRadValve.getTargetTempC();
        if(setback > 0)
          {
#if 0 && defined(DEBUG)
DEBUG_SERIAL_PRINT_FLASHSTRING("setback: ");
DEBUG_SERIAL_PRINT(setback);
DEBUG_SERIAL_PRINT_FLASHSTRING(" tW: ");
DEBUG_SERIAL_PRINT(getWARMTargetC());
DEBUG_SERIAL_PRINT(hasEcoBias() ? F(" eco") : F(" com"));
if(AmbLight.isRoomDark()) { DEBUG_SERIAL_PRINT(F(" isDark")); }
DEBUG_SERIAL_PRINTLN();
#endif
          if(hasEcoBias()) { maxEcoSetback = max(maxEcoSetback, setback); }
          else { maxComSetback = max(maxComSetback, setback); }
          }
        }

      // ENERGY SAVING RULE TEST (TODO-442 1c: "Prewarm temperature must be set back from normal WARM target.")
#ifdef OCCUPANCY_SUPPORT
//      Occupancy._TEST_set_vacH_(randRNG8() % OccupancyTracker::longVacantHThrH); // Mark occupied or at least not long vacant.
      Occupancy.markAsOccupied(); // Mark occupied or at least not long vacant.
#endif
      setWarmModeDebounced(false);
      _TEST_set_schedule_override(_soUT_soon);
      NominalRadValve.computeTargetTemperature();
      const uint8_t tTpw = NominalRadValve.getTargetTempC();
      _TEST_set_schedule_override(_soUT_now);
      setWarmModeDebounced(true);
      NominalRadValve.computeTargetTemperature();
      const uint8_t tTw = NominalRadValve.getTargetTempC();
      AssertIsTrue(tTpw < tTw);
      _TEST_set_schedule_override(_soUT_normal); // Override off...
      }
    }

  // ENERGY SAVING RULE TEST (TODO-442 2b: "Setbacks of up to FULL (3C) must be possible in full eco mode.")
  AssertIsTrue(maxEcoSetback >= SETBACK_FULL);
  // ENERGY SAVING RULE TEST (TODO-442 2c: "Setbacks are at most 2C in comfort mode (but there is a setback).")
  AssertIsTrue(maxComSetback > 0);
  AssertIsTrue(maxComSetback <= 2);
#endif // ENABLE_MODELLED_RAD_VALVE
  }

// Test self-mocking of sensor modules (and others) to facilitate other unit tests. 
static void testSensorMocking()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("SensorMocking");
#ifndef OMIT_MODULE_LDROCCUPANCYDETECTION
  // Ambient light
  for(uint8_t i = 0; i < 2; ++i)
    {
    const uint8_t nal = OTV0P2BASE::randRNG8();
    const bool nil = OTV0P2BASE::randRNG8NextBoolean();
    AmbLight._TEST_set_multi_(((uint16_t)nal)<<2, nil, OTV0P2BASE::randRNG8());
    AssertIsEqual(nal, AmbLight.get());
    AssertIsTrue(nil == AmbLight.isRoomLit());
// DHD20151017: temporarily disabled.
//    const uint8_t nal2 = OTV0P2BASE::randRNG8();
//    AmbLight._TEST_set_(nal2);
//    AssertIsEqual(nal2, AmbLight.get());
    }
#endif
#ifdef OCCUPANCY_SUPPORT
  // Occupancy
//  const uint8_t vacH = randRNG8() | 1; // Ensure non-zero.
//  Occupancy._TEST_set_vacH_(vacH);
  Occupancy.setHolidayMode();
  AssertIsEqual(0, Occupancy.get());
  AssertIsEqual(255, Occupancy.getVacancyH());
  AssertIsTrue(Occupancy.isLikelyUnoccupied());
//    Occupancy._TEST_set_vacH_(0);
  Occupancy._TEST_set_(true);
  AssertIsEqual(0, Occupancy.getVacancyH());
  AssertIsTrue(0 != Occupancy.get());
  AssertIsTrue(Occupancy.isLikelyOccupied());
#endif
  // Schedule
  _TEST_set_schedule_override(_soUT_now);
  AssertIsTrue(isAnySimpleScheduleSet());
  AssertIsTrue(isAnyScheduleOnWARMNow());
  AssertIsTrue(!isAnyScheduleOnWARMSoon());
  _TEST_set_schedule_override(_soUT_soon);
  AssertIsTrue(isAnySimpleScheduleSet());
  AssertIsTrue(!isAnyScheduleOnWARMNow());
  AssertIsTrue(isAnyScheduleOnWARMSoon());
  _TEST_set_schedule_override(_soUT_off);
  AssertIsTrue(!isAnySimpleScheduleSet());
  AssertIsTrue(!isAnyScheduleOnWARMNow());
  AssertIsTrue(!isAnyScheduleOnWARMSoon());
  _TEST_set_schedule_override(_soUT_normal); // Override off.
  // Base temperature
  _TEST_set_basetemp_override(_btoUT_min);
  AssertIsTrue(hasEcoBias());
  AssertIsTrue(getWARMTargetC() <= BIASECO_WARM);
  _TEST_set_basetemp_override(_btoUT_mid);
  AssertIsTrue(hasEcoBias());
  AssertIsTrue(getWARMTargetC() > BIASECO_WARM);
  AssertIsTrue(getWARMTargetC() < BIASCOM_WARM);
  _TEST_set_basetemp_override(_btoUT_max);
  AssertIsTrue(!hasEcoBias());
  AssertIsTrue(getWARMTargetC() >= BIASCOM_WARM);
  _TEST_set_basetemp_override(_btoUT_normal); // Override off.
  }
  
// Test basic behaviour of system mode setting and some basic parameters.
static void testModeControls()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("ModeControls");
  AssertIsTrue(!(inBakeMode() && !inWarmMode())); // Check not in initial illegal combination.
  setWarmModeDebounced(false);
  AssertIsTrue(!inWarmMode());
  AssertIsTrue(!inBakeMode());
  setWarmModeDebounced(true);
  AssertIsTrue(inWarmMode());
  AssertIsTrue(!inBakeMode());
  setWarmModeDebounced(false);
  AssertIsTrue(!inWarmMode());
  AssertIsTrue(!inBakeMode());
  startBakeDebounced();
  AssertIsTrue(inWarmMode());
  AssertIsTrue(inBakeMode());
  cancelBakeDebounced();
  AssertIsTrue(inWarmMode());
  AssertIsTrue(!inBakeMode());
  setWarmModeDebounced(false);
  AssertIsTrue(!inWarmMode());
  AssertIsTrue(!inBakeMode());
  }

// Test basic behaviour of stats quartile routines.
static void testQuartiles()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("Quartiles");
  // For whatever happens to be in EEPROM at the moment, test for sanity for all stats sets.
  // This does not write to EEPROM, so will not wear it out.
  // Make sure that nothing can be seen as top and bottom quartile at same time.
  // Make sure that there cannot be too many items reported in each quartile
  for(uint8_t i = 0; i < V0P2BASE_EE_STATS_SETS; ++i)
    {
    int bQ = 0, tQ = 0;
    for(uint8_t j = 0; j < 24; ++j)
      {
      const bool inTopQ = inOutlierQuartile(true, i, j);
      if(inTopQ) { ++tQ; }
      const bool inBotQ = inOutlierQuartile(false, i, j);
      if(inBotQ) { ++bQ; }
      AssertIsTrue(!inTopQ || !inBotQ);
      }
    AssertIsTrue(bQ <= 6);
    AssertIsTrue(tQ <= 6);
    }
  }

// Test handling of JSON stats.
static void testJSONStats()
  {
#if defined(ALLOW_JSON_OUTPUT)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("JSONStats");
  SimpleStatsRotation<2> ss1;
  ss1.setID("1234");
  AssertIsEqual(0, ss1.size());
  //AssertIsTrue(0 == ss1.writeJSON(NULL, randRNG8(), randRNG8(), randRNG8NextBoolean()));
  char buf[MSG_JSON_MAX_LENGTH + 2]; // Allow for trailing '\0' and spare byte.
  // Create minimal JSON message with no data content. just the (supplied) ID.
  const uint8_t l1 = ss1.writeJSON((uint8_t*)buf, sizeof(buf), OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean());
//OTV0P2BASE::serialPrintAndFlush(buf); OTV0P2BASE::serialPrintlnAndFlush();
  AssertIsEqual(12, l1);
  const char PROGMEM *t1 = (const char PROGMEM *)F("{\"@\":\"1234\"}");
  AssertIsTrue(0 == strcmp_P(buf, t1));
  ss1.enableCount(false);
  AssertIsEqual(12, ss1.writeJSON((uint8_t*)buf, sizeof(buf), OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean()));
  AssertIsTrue(0 == strcmp_P(buf, t1));
  // Check that count works.
  ss1.enableCount(true);
  AssertIsEqual(0, ss1.size());
  AssertIsEqual(18, ss1.writeJSON((uint8_t*)buf, sizeof(buf), OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean()));
//OTV0P2BASE::serialPrintAndFlush(buf); OTV0P2BASE::serialPrintlnAndFlush();
  AssertIsTrue(0 == strcmp_P(buf, (const char PROGMEM *)F("{\"@\":\"1234\",\"+\":2}")));
  // Turn count off for rest of tests.
  ss1.enableCount(false);
  AssertIsEqual(12, ss1.writeJSON((uint8_t*)buf, sizeof(buf), OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean()));
  // Check that removal of absent entry does nothing.
  AssertIsTrue(!ss1.remove("bogus"));
  AssertIsEqual(0, ss1.size());
  // Check that new item can be added/put (with no/default properties).
  ss1.put("f1", 42);
  AssertIsEqual(1, ss1.size());
  AssertIsEqual(20, ss1.writeJSON((uint8_t*)buf, sizeof(buf), 0, OTV0P2BASE::randRNG8NextBoolean()));
#if 0 // Short of Flash space!
//OTV0P2BASE::serialPrintAndFlush(buf); OTV0P2BASE::serialPrintlnAndFlush();
  AssertIsTrue(0 == strcmp_P(buf, (const char PROGMEM *)F("{\"@\":\"1234\",\"f1\":42}")));
#endif
  ss1.put("f1", -111);
  AssertIsEqual(1, ss1.size());
  AssertIsEqual(22, ss1.writeJSON((uint8_t*)buf, sizeof(buf), 0, OTV0P2BASE::randRNG8NextBoolean()));
  AssertIsTrue(0 == strcmp_P(buf, (const char PROGMEM *)F("{\"@\":\"1234\",\"f1\":-111}")));
#endif
  }

// Test handling of JSON messages for transmission and reception.
// Includes bit-twiddling, CRC computation, and other error checking.
static void testJSONForTX()
  {
#if defined(ALLOW_JSON_OUTPUT)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("JSONForTX");
  char buf[MSG_JSON_MAX_LENGTH + 2]; // Allow for trailing '\0' or CRC + 0xff terminator.
  // Clear the buffer.
  memset(buf, 0, sizeof(buf));
  // Fail sanity check on a completely empty buffer (zero-length string).
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // Fail sanity check on a few initially-plausible length-1 values.
  buf[0] = '{';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  buf[0] = '}';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  buf[0] = '[';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  buf[0] = ']';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  buf[0] = ' ';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // Fail sanity check with already-adjusted (minimal) nessage.
  buf[0] = '{';
  buf[1] = ('}' | 0x80);
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // Minimal correct messaage should pass.
  buf[0] = '{';
  buf[1] = '}';
  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
  // Try a longer valid trivial message.
  strcpy_P(buf, (const char PROGMEM *)F("{  }"));
  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
  // Invalidate it with a non-printable char and check that it is rejected.
  buf[2] = '\1';
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // Try a longer valid non-trivial message.
  __FlashStringHelper const* longJSONMsg1 = F("{\"@\":\"cdfb\",\"T|C16\":299,\"H|%\":83,\"L\":255,\"B|cV\":256}");
  memset(buf, 0, sizeof(buf));
  strcpy_P(buf, (const char PROGMEM *)longJSONMsg1);
  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
  // Invalidate it with a high-bit set and check that it is rejected.
  buf[5] |= 0x80;
  AssertIsTrue(!quickValidateRawSimpleJSONMessage(buf));
  // CRC fun!  
  memset(buf, 0, sizeof(buf));
  buf[0] = '{';
  buf[1] = '}';
  const uint8_t crc1 = adjustJSONMsgForTXAndComputeCRC(buf);
  // Check that top bit is not set (ie CRC was computed OK).
  AssertIsTrueWithErr(!(crc1 & 0x80), crc1);
  // Check for expected CRC value.
  AssertIsTrueWithErr((0x38 == crc1), crc1);
  // Check that initial part unaltered.
  AssertIsTrueWithErr(('{' == buf[0]), buf[0]);
  // Check that top bit has been set in trailing brace.
  AssertIsTrueWithErr(((char)('}' | 0x80) == buf[1]), buf[1]);
  // Check that trailing '\0' still present.
  AssertIsTrueWithErr((0 == buf[2]), buf[2]);
  // Check that TX-format can be converted for RX.
  buf[2] = crc1;
  buf[3] = 0xff; // As for normal TX...
// FIXME
//  const int8_t l1 = adjustJSONMsgForRXAndCheckCRC(buf, sizeof(buf));
//  AssertIsTrueWithErr(2 == l1, l1);
//  AssertIsTrueWithErr(2 == strlen(buf), strlen(buf));
//  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
  // Now a longer message...
  memset(buf, 0, sizeof(buf));
  strcpy_P(buf, (const char PROGMEM *)longJSONMsg1);
  const int8_t l2o = strlen(buf);
  const uint8_t crc2 = adjustJSONMsgForTXAndComputeCRC(buf);
  // Check that top bit is not set (ie CRC was computed OK).
  AssertIsTrueWithErr(!(crc2 & 0x80), crc2);
  // Check for expected CRC value.
  AssertIsTrueWithErr((0x77 == crc2), crc2);
// FIXME
//  // Check that TX-format can be converted for RX.
//  buf[l2o] = crc2;
//  buf[l2o+1] = 0xff;
// FIXME
//  const int8_t l2 = adjustJSONMsgForRXAndCheckCRC(buf, sizeof(buf));
//  AssertIsTrueWithErr(l2o == l2, l2);
//  AssertIsTrue(quickValidateRawSimpleJSONMessage(buf));
#endif
  }


//// Self-test of EEPROM functioning (and smart/split erase/write).
//// Will not usually perform any wear-inducing activity (is idempotent).
//// Aborts with panic() upon failure.
//static void testEEPROM()
//  {
//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("EEPROM");
//
//  if((uint8_t) 0xff != eeprom_read_byte((uint8_t*)EE_START_TEST_LOC))
//    {
//    if(!eeprom_smart_erase_byte((uint8_t*)EE_START_TEST_LOC)) { panic(); } // Should have attempted erase.
//    if((uint8_t) 0xff != eeprom_read_byte((uint8_t*)EE_START_TEST_LOC)) { panic(); } // Should have erased.
//    }
//  if(eeprom_smart_erase_byte((uint8_t*)EE_START_TEST_LOC)) { panic(); } // Should not need erase nor attempt one.
//
//  const uint8_t eaTestPattern = 0xa5; // Test pattern for masking (selective bit clearing).
//  if(0 != ((~eaTestPattern) & eeprom_read_byte((uint8_t*)EE_START_TEST_LOC2))) // Will need to clear some bits.
//    {
//      if(!eeprom_smart_clear_bits((uint8_t*)EE_START_TEST_LOC2, eaTestPattern)) { panic(); } // Should have attempted write.
//      if(0 != ((~eaTestPattern) & eeprom_read_byte((uint8_t*)EE_START_TEST_LOC2))) { panic(); } // Should have written.
//    }
//  if(eeprom_smart_clear_bits((uint8_t*)EE_START_TEST_LOC2, eaTestPattern)) { panic(); } // Should not need write nor attempt one.
//  }

// Test of FHT8V bitstream encoding and decoding.
static void testFHTEncoding()
  {
#ifdef USE_MODULE_FHT8VSIMPLE_RX
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("FHTEncoding");
  
  uint8_t buf[FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE];
  fht8v_msg_t command; // For encoding.
  fht8v_msg_t commandDecoded; // For decoding.

  // Encode an example message for a real house code and command (close valve).
  command.hc1 = 13;
  command.hc2 = 73;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  command.command = 0x26;
  command.extension = 0;
  memset(buf, 0xff, sizeof(buf));
  uint8_t *result1 = FHT8VCreate200usBitStreamBptr(buf, &command);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == 38), result1-buf); // Check correct length.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xe3) == buf[6], buf[6]); // Check end of preamble.
  AssertIsTrueWithErr(((uint8_t)0xce) == buf[34], buf[34]); // Check part of checksum.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(13 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(73 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);

  // Encode shortest-possible (all-zero-bits) FHT8V command as 200us-bit-stream...
  command.hc1 = 0;
  command.hc2 = 0;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  command.command = 0;
  command.extension = 0;
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreate200usBitStreamBptr(buf, &command);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == 35), result1-buf); // Check correct length.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(0 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(0 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);

  // Encode longest-possible (as many 1-bits as possible) FHT8V command as 200us-bit-stream...
  command.hc1 = 0xff;
  command.hc2 = 0xff;
#ifdef FHT8V_ADR_USED
  address = 0xff;
#endif
  command.command = 0xff;
  command.extension = 0xff;
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreate200usBitStreamBptr(buf, &command);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  AssertIsTrueWithErr((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(0xff == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(0xff == commandDecoded.hc2, commandDecoded.hc2);
#ifdef FHT8V_ADR_USED
  AssertIsTrueWithErr(0xff == commandDecoded.address, commandDecoded.address);
#endif
  AssertIsTrueWithErr(0xff == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0xff == commandDecoded.extension, commandDecoded.extension);
#endif
  }

// Test of heat and tail of FHT8V bitstream encoding and decoding.
static void testFHTEncodingHeadAndTail()
  {
#ifdef USE_MODULE_FHT8VSIMPLE_RX
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("FHTEncodingHeadAndTail");

//// Create FHT8V TRV outgoing valve-setting command frame (terminated with 0xff) at bptr with optional headers and trailers.
////   * TRVPercentOpen value is used to generate the frame
////   * doHeader  if true then an extra RFM22/23-friendly 0xaaaaaaaa sync header is preprended
////   * trailer  if not null then a (3-byte) trailer is appented, build from that info plus a CRC
////   * command  on entry hc1, hc2 (and addresss if used) must be set correctly, this sets the command and extension; never NULL
//// The generated command frame can be resent indefinitely.
//// The output buffer used must be (at least) FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE bytes.
//// Returns pointer to the terminating 0xff on exit.
//uint8_t *FHT8VCreateValveSetCmdFrameHT_r(uint8_t *const bptrInitial, const bool doHeader, fht8v_msg_t *const command, const uint8_t TRVPercentOpen, const trailingMinimalStatsPayload_t *trailer)

  uint8_t buf[FHT8V_200US_BIT_STREAM_FRAME_BUF_SIZE];
  fht8v_msg_t command; // For encoding.
  fht8v_msg_t commandDecoded; // For decoding.

  // Encode a basic message to set a valve to 0%, without headers or trailers.
  command.hc1 = 13;
  command.hc2 = 73;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  memset(buf, 0xff, sizeof(buf));
  uint8_t *result1 = FHT8VCreateValveSetCmdFrameHT_r(buf, false, &command, 0, NULL);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == 38), result1-buf); // Check correct length: 38-byte body.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xe3) == buf[6], buf[6]); // Check end of preamble.
  AssertIsTrueWithErr(((uint8_t)0xce) == buf[34], buf[34]); // Check part of checksum.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(13 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(73 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);
  // Verify that trailer NOT present.
  AssertIsTrue(!verifyHeaderAndCRCForTrailingMinimalStatsPayload(result1));

 // Encode a basic message to set a valve to 0%, with header but without trailer.
  command.hc1 = 13;
  command.hc2 = 73;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreateValveSetCmdFrameHT_r(buf, true, &command, 0, NULL);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
  AssertIsTrueWithErr((result1 - buf == RFM22_PREAMBLE_BYTES + 38), result1-buf); // Check correct length: preamble + 38-byte body.
  AssertIsTrueWithErr(((uint8_t)0xaa) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[RFM22_PREAMBLE_BYTES], buf[RFM22_PREAMBLE_BYTES]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xe3) == buf[6+RFM22_PREAMBLE_BYTES], buf[6+RFM22_PREAMBLE_BYTES]); // Check end of preamble.
  AssertIsTrueWithErr(((uint8_t)0xce) == buf[34+RFM22_PREAMBLE_BYTES], buf[34+RFM22_PREAMBLE_BYTES]); // Check part of checksum.
  // Attempt to decode.
  AssertIsTrue(FHT8VDecodeBitStream(buf + RFM22_PREAMBLE_BYTES, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded));
  AssertIsTrueWithErr(13 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(73 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);
  // Verify that trailer NOT present.
  AssertIsTrue(!verifyHeaderAndCRCForTrailingMinimalStatsPayload(result1));

  // Encode a basic message to set a valve to 0%, with header and trailer.
  command.hc1 = 13;
  command.hc2 = 73;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  FullStatsMessageCore_t fullStats;
  clearFullStatsMessageCore(&fullStats);
  OTV0P2BASE::captureEntropy1(); // Try to stir a little noise into the PRNG before using it.
  const bool powerLow = !(OTV0P2BASE::randRNG8() & 0x40); // Random value.
  fullStats.containsTempAndPower = true;
  fullStats.tempAndPower.powerLow = powerLow;
  const int tempC16 = (OTV0P2BASE::randRNG8()&0xff) + (10 << 4); // Random value in range [10C, 25C[.
  fullStats.tempAndPower.tempC16 = tempC16;
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreateValveSetCmdFrameHT_r(buf, true, &command, 0, &fullStats);
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
#if defined(ALLOW_MINIMAL_STATS_TXRX)
  AssertIsTrueWithErr((result1 - buf == 41 + RFM22_PREAMBLE_BYTES), result1-buf); // Check correct length:preamble + 38-byte body + 3-byte trailer.
#else // Expect longer encoding in this case...
  AssertIsTrueWithErr((result1 - buf == 43 + RFM22_PREAMBLE_BYTES), result1-buf); // Check correct length:preamble + 38-byte body + 5-byte trailer.
#endif
  AssertIsTrueWithErr(((uint8_t)0xaa) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[RFM22_PREAMBLE_BYTES], buf[RFM22_PREAMBLE_BYTES]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xe3) == buf[6+RFM22_PREAMBLE_BYTES], buf[6+RFM22_PREAMBLE_BYTES]); // Check end of preamble.
  AssertIsTrueWithErr(((uint8_t)0xce) == buf[34+RFM22_PREAMBLE_BYTES], buf[34+RFM22_PREAMBLE_BYTES]); // Check part of checksum.
  // Attempt to decode.
  const uint8_t *afterBody = FHT8VDecodeBitStream(buf + RFM22_PREAMBLE_BYTES, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded);
  AssertIsTrue(afterBody);
  AssertIsTrueWithErr(13 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(73 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);
#if 0 && defined(ALLOW_MINIMAL_STATS_TXRX)
  OTV0P2BASE::serialPrintAndFlush(F("  Minimal trailer bytes: "));
  OTV0P2BASE::serialPrintAndFlush(afterBody[0], HEX);
  OTV0P2BASE::serialPrintAndFlush(' ');
  OTV0P2BASE::serialPrintAndFlush(afterBody[1], HEX);
  OTV0P2BASE::serialPrintAndFlush(' ');
  OTV0P2BASE::serialPrintAndFlush(afterBody[2], HEX);
  OTV0P2BASE::serialPrintlnAndFlush();
#endif
  // Verify (start of) trailer is OK.
  for(uint8_t i = 0; i < 3; ++i)
    {
    AssertIsTrueWithErr(0xff != afterBody[i], i); // No trailer byte should be 0xff (so 0xff can be terminator).
    AssertIsTrueWithErr(0 == (0x80 & afterBody[i]), i); // No trailer byte should have its high bit set.
    }
#if defined(ALLOW_MINIMAL_STATS_TXRX)
  AssertIsTrueWithErr(verifyHeaderAndCRCForTrailingMinimalStatsPayload(afterBody), *afterBody);
#endif
  // Decode values...
#if defined(ALLOW_MINIMAL_STATS_TXRX)
  trailingMinimalStatsPayload_t statsDecoded;
  extractTrailingMinimalStatsPayload(afterBody, &statsDecoded);
  AssertIsEqual(powerLow, statsDecoded.powerLow);
  AssertIsEqual(tempC16, statsDecoded.tempC16);
#else
  FullStatsMessageCore_t statsDecoded;
  AssertIsTrue(NULL != decodeFullStatsMessageCore(afterBody, sizeof(buf) - (afterBody - buf), (stats_TX_level)OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean(), &statsDecoded));
  AssertIsEqual(powerLow, statsDecoded.tempAndPower.powerLow);
  AssertIsEqual(tempC16, statsDecoded.tempAndPower.tempC16);
#endif

  // Encode a basic message to set a different valve to 0%, with header and trailer.
  // This one was apparently impossible to TX or RX...
  command.hc1 = 65;
  command.hc2 = 74;
#ifdef FHT8V_ADR_USED
  address = 0;
#endif
  memset(buf, 0xff, sizeof(buf));
  result1 = FHT8VCreateValveSetCmdFrameHT_r(buf, true, &command, 0, &fullStats);
//OTV0P2BASE::serialPrintAndFlush(result1 - buf); OTV0P2BASE::serialPrintlnAndFlush();
  AssertIsTrueWithErr((result1 - buf) < sizeof(buf), (result1 - buf) - sizeof(buf)); // result1 points to the terminating 0xff, not just after it.
  AssertIsTrueWithErr(((uint8_t)~0U) == *result1, *result1); // Check that result points at terminator value 0xff/~0.
  //AssertIsTrue((result1 - buf < MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE), result1-buf); // Check not overflowing the buffer.
#if defined(ALLOW_MINIMAL_STATS_TXRX)
  AssertIsTrueWithErr((result1 - buf == 42 + RFM22_PREAMBLE_BYTES), result1-buf); // Check correct length.
#else
  AssertIsTrueWithErr((result1 - buf == 44 + RFM22_PREAMBLE_BYTES), result1-buf); // Check correct length.
#endif
  AssertIsTrueWithErr(((uint8_t)0xaa) == buf[0], buf[0]); // Check that result starts with FHT8V 0xcc preamble.
  AssertIsTrueWithErr(((uint8_t)0xcc) == buf[RFM22_PREAMBLE_BYTES], buf[RFM22_PREAMBLE_BYTES]); // Check that result starts with FHT8V 0xcc preamble.
  // Attempt to decode.
  afterBody = FHT8VDecodeBitStream(buf + RFM22_PREAMBLE_BYTES, buf + MIN_FHT8V_200US_BIT_STREAM_BUF_SIZE - 1, &commandDecoded);
  AssertIsTrue(0 != afterBody);
//OTV0P2BASE::serialPrintAndFlush(afterBody - buf); OTV0P2BASE::serialPrintlnAndFlush();
  AssertIsEqual(5, (result1 - buf) - (afterBody - buf));
  AssertIsTrueWithErr(65 == commandDecoded.hc1, commandDecoded.hc1);
  AssertIsTrueWithErr(74 == commandDecoded.hc2, commandDecoded.hc2);
  AssertIsTrueWithErr(0x26 == commandDecoded.command, commandDecoded.command);
  AssertIsTrueWithErr(0 == commandDecoded.extension, commandDecoded.extension);
  // Verify trailer start is OK.
  for(uint8_t i = 0; i < 3; ++i)
    {
    AssertIsTrueWithErr(0xff != afterBody[i], i); // No trailer byte should be 0xff (so 0xff can be terminator).
    AssertIsTrueWithErr(0 == (0x80 & afterBody[i]), i); // No trailer byte should have its high bit set.
    }
#if defined(ALLOW_MINIMAL_STATS_TXRX)
  AssertIsTrueWithErr(verifyHeaderAndCRCForTrailingMinimalStatsPayload(afterBody), *afterBody);
#endif
  // Decode values...
  memset(&statsDecoded, 0xff, sizeof(statsDecoded)); // Clear structure...
#if defined(ALLOW_MINIMAL_STATS_TXRX)
  extractTrailingMinimalStatsPayload(afterBody, &statsDecoded);
  AssertIsEqual(powerLow, statsDecoded.powerLow);
  AssertIsEqual(tempC16, statsDecoded.tempC16);
#else
  AssertIsTrue(NULL != decodeFullStatsMessageCore(afterBody, sizeof(buf) - (afterBody - buf), (stats_TX_level)OTV0P2BASE::randRNG8(), OTV0P2BASE::randRNG8NextBoolean(), &statsDecoded));
  AssertIsEqual(powerLow, statsDecoded.tempAndPower.powerLow);
  AssertIsEqual(tempC16, statsDecoded.tempAndPower.tempC16);
#endif
#endif
  }

// Test elements of encoding and decoding FullStatsMessageCore_t.
// These are the routines primarily under test:
//     uint8_t *encodeFullStatsMessageCore(uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel, const FullStatsMessageCore_t *content)
//     const uint8_t *decodeFullStatsMessageCore(const uint8_t *buf, uint8_t buflen, stats_TX_level secLevel, bool secureChannel, FullStatsMessageCore_t *content)
static void testFullStatsMessageCoreEncDec()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("FullStatsMessageCoreEncDec");

  // Ensure that with null buffer/content encode and decode fail regardless of other arguments.
  uint8_t buf[FullStatsMessageCore_MAX_BYTES_ON_WIRE + 1];
  FullStatsMessageCore_t content;
  AssertIsTrue(NULL == encodeFullStatsMessageCore(NULL, OTV0P2BASE::randRNG8(), stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), NULL));
  AssertIsTrue(NULL == decodeFullStatsMessageCore(NULL, OTV0P2BASE::randRNG8(), stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), NULL));
  AssertIsTrue(NULL == encodeFullStatsMessageCore(NULL, FullStatsMessageCore_MAX_BYTES_ON_WIRE+1, stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), &content));
  AssertIsTrue(NULL == encodeFullStatsMessageCore(buf, FullStatsMessageCore_MAX_BYTES_ON_WIRE+1, stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), NULL));
  AssertIsTrue(NULL == decodeFullStatsMessageCore(NULL, FullStatsMessageCore_MIN_BYTES_ON_WIRE+1, stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), &content));
  AssertIsTrue(NULL == decodeFullStatsMessageCore(buf, FullStatsMessageCore_MIN_BYTES_ON_WIRE+1, stTXalwaysAll, OTV0P2BASE::randRNG8NextBoolean(), NULL));

  // Prepare a minimal (empty) non-secure message.
  memset(buf, 0, sizeof(buf));
  clearFullStatsMessageCore(&content);
  const uint8_t *emptyMsg = encodeFullStatsMessageCore(buf, sizeof(buf), stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != emptyMsg); // Must succeed.
  AssertIsTrueWithErr(emptyMsg - buf == FullStatsMessageCore_MIN_BYTES_ON_WIRE, emptyMsg - buf); // Must correspond to minimum size.
  AssertIsTrueWithErr(MESSAGING_FULL_STATS_HEADER_MSBS == buf[0], buf[0]); // Header byte.
  AssertIsTrueWithErr(MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS == buf[1], buf[1]); // Flags header byte.
  AssertIsTrueWithErr(0x65 == buf[2], buf[2]); // CRC.
  AssertIsTrue(((uint8_t)0xff) == *emptyMsg); // Must be correctly terminated.
  // Decode the message just generated into a freshly-scrubbed content structure.
  clearFullStatsMessageCore(&content);
  const uint8_t *emptyMsgDE = decodeFullStatsMessageCore(buf, emptyMsg-buf, stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != emptyMsgDE); // Must succeed.
  AssertIsTrue(emptyMsg == emptyMsgDE); // Must return correct end of message.
  // Verify that there is no content.
  AssertIsTrue(!content.containsID);
  AssertIsTrue(!content.containsTempAndPower);
  AssertIsTrue(!content.containsAmbL);

  // Prepare a non-secure message with ID.
  memset(buf, 0, sizeof(buf));
  clearFullStatsMessageCore(&content);
  content.id0 = 0x80;
  content.id1 = 0x00;
  content.containsID = true;
  AssertIsTrue(NULL == encodeFullStatsMessageCore(buf, sizeof(buf), stTXalwaysAll, false, &content)); // Should reject ID bytes with differring msbits.
  content.id1 = 0x81;
  const uint8_t *onlyIDMsg = encodeFullStatsMessageCore(buf, sizeof(buf), stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != onlyIDMsg); // Must succeed.
  AssertIsTrueWithErr(onlyIDMsg - buf == FullStatsMessageCore_MIN_BYTES_ON_WIRE + 2, onlyIDMsg - buf); // Must correspond to minimum size + 2 ID bytes.
  AssertIsTrueWithErr((MESSAGING_FULL_STATS_HEADER_MSBS | MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT | MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH) == buf[0], buf[0]); // Header byte.
  AssertIsTrueWithErr(0x00 == buf[1], buf[1]); // ID0 without msbit.
  AssertIsTrueWithErr(0x01 == buf[2], buf[2]); // ID1 without msbit.
  AssertIsTrueWithErr(MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS == buf[3], buf[3]); // Flags header byte.
  AssertIsTrueWithErr(0x01 == buf[4], buf[4]); // CRC.
  AssertIsTrue(((uint8_t)0xff) == *onlyIDMsg); // Must be correctly terminated.
  // Decode the message just generated into a freshly-scrubbed content structure.
  clearFullStatsMessageCore(&content);
  const uint8_t *onlyIDMsgDE = decodeFullStatsMessageCore(buf, onlyIDMsg-buf, stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != onlyIDMsgDE); // Must succeed.
  AssertIsTrue(onlyIDMsg == onlyIDMsgDE); // Must return correct end of message.
  // Verify that there is only ID.
  AssertIsTrue(content.containsID);
  AssertIsTrueWithErr(content.id0 == (uint8_t)0x80, content.id0);
  AssertIsTrueWithErr(content.id1 == (uint8_t)0x81, content.id1);
  AssertIsTrue(!content.containsTempAndPower);
  AssertIsTrue(!content.containsAmbL);

  // Prepare a non-secure message with ID, temp/power, ambient light level and occupancy.
  memset(buf, 0, sizeof(buf));
  clearFullStatsMessageCore(&content);
  content.id0 = 0x83;
  content.id1 = 0x98;
  content.containsID = true;
  content.tempAndPower.tempC16 = (19 << 4) + 1; // (19 + 1/16)C.
  content.tempAndPower.powerLow = false; // Normal power.
  content.containsTempAndPower = true;
  content.ambL = 42; // Allowed value in range [1,254].
  content.containsAmbL = true;
  content.occ = 3; // Not occupied recently.
  const uint8_t *msg1 = encodeFullStatsMessageCore(buf, sizeof(buf), stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != msg1); // Must succeed.
  AssertIsTrueWithErr(msg1 - buf == FullStatsMessageCore_MAX_BYTES_ON_WIRE, msg1 - buf); // Must correspond to minimum size + 2 ID bytes.
  AssertIsTrueWithErr((MESSAGING_FULL_STATS_HEADER_MSBS | MESSAGING_FULL_STATS_HEADER_BITS_ID_PRESENT | MESSAGING_FULL_STATS_HEADER_BITS_ID_HIGH) == buf[0], buf[0]); // Header byte.
  AssertIsTrueWithErr(0x03 == buf[1], buf[1]); // ID0 without msbit.
  AssertIsTrueWithErr(0x18 == buf[2], buf[2]); // ID1 without msbit.
  AssertIsTrueWithErr((MESSAGING_TRAILING_MINIMAL_STATS_HEADER_MSBS + 1) == buf[3], buf[3]); // Temp/power first byte.
  AssertIsTrueWithErr((19 + 20) == buf[4], buf[4]); // Temp second byte.
  AssertIsTrueWithErr((MESSAGING_FULL_STATS_FLAGS_HEADER_MSBS | MESSAGING_FULL_STATS_FLAGS_HEADER_AMBL | 3) == buf[5], buf[5]); // Flags header (no extension byte follows).
  AssertIsTrueWithErr(42 == buf[6], buf[6]); // Ambient light.
  AssertIsTrueWithErr(0x44 == buf[7], buf[7]); // CRC.
  AssertIsTrue(((uint8_t)0xff) == *msg1); // Must be correctly terminated.
  // Decode the message just generated into a freshly-scrubbed content structure.
  clearFullStatsMessageCore(&content);
  const uint8_t *msg1DE = decodeFullStatsMessageCore(buf, msg1-buf, stTXalwaysAll, false, &content);
  AssertIsTrue(NULL != msg1DE); // Must succeed.
  AssertIsTrue(msg1 == msg1DE); // Must return correct end of message.
  AssertIsTrue(content.containsID);
  AssertIsTrueWithErr(content.id0 == (uint8_t)0x83, content.id0);
  AssertIsTrueWithErr(content.id1 == (uint8_t)0x98, content.id1);
  AssertIsTrue(content.containsTempAndPower);
  AssertIsTrue(!content.tempAndPower.powerLow);
  AssertIsTrue((19 << 4) + 1 == content.tempAndPower.tempC16);
  AssertIsTrue(content.containsAmbL);
  AssertIsTrue(42 == content.ambL);
  }






// Test elements of RTC time persist/restore (without causing more EEPROM wear, if working correctly).
static void testRTCPersist()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("RTCPersist");
  // Perform with interrupts shut out to avoid RTC ISR interferring.
  // This will effectively stall the RTC.
  bool minutesPersistOK;
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    const uint_least16_t mb = OTV0P2BASE::getMinutesSinceMidnightLT();
    OTV0P2BASE::persistRTC();
    OTV0P2BASE::restoreRTC();
    const uint_least16_t ma = OTV0P2BASE::getMinutesSinceMidnightLT();
    // Check that persist/restore did not change live minutes value at least, within the 15-minute quantum used.
    minutesPersistOK = (mb/15 == ma/15);
    }
    AssertIsTrue(minutesPersistOK);
  }


//// Tests of entropy gathering routines.
////
//// Maximum number of identical nominally random bits (or values with approx one bit of entropy) in a row tolerated.
//// Set large enough that even soak testing for many hours should not trigger a failure if behaviour is plausibly correct.
//#define MAX_IDENTICAL_BITS_SEQUENTIALLY 32
//void testEntropyGathering()
//  {
//  DEBUG_SERIAL_PRINTLN_FLASHSTRING("EntropyGathering");
//
////  // Test WDT jitter: assumed about 1 bit of entropy per call/result.
////  //DEBUG_SERIAL_PRINT_FLASHSTRING("jWDT... ");
////  const uint8_t jWDT = clockJitterWDT();
////  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY; --i >= 0; )
////    {
////    if(jWDT != clockJitterWDT()) { break; } // Stop as soon as a different value is obtained.
////    AssertIsTrueWithErr(0 != i, i); // Generated too many identical values in a row.
////    }
////  //DEBUG_SERIAL_PRINT_FLASHSTRING(" 1st=");
////  //DEBUG_SERIAL_PRINTFMT(jWDT, BIN);
////  //DEBUG_SERIAL_PRINTLN();
////
////#ifndef NO_clockJitterRTC
////  // Test RTC jitter: assumed about 1 bit of entropy per call/result.
////  //DEBUG_SERIAL_PRINT_FLASHSTRING("jRTC... ");
////  for(const uint8_t t0 = getSubCycleTime(); t0 == getSubCycleTime(); ) { } // Wait for sub-cycle time to roll to toughen test.
////  const uint8_t jRTC = clockJitterRTC();
////  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY; --i >= 0; )
////    {
////    if(jRTC != clockJitterRTC()) { break; } // Stop as soon as a different value is obtained.
////    AssertIsTrue(0 != i); // Generated too many identical values in a row.
////    }
////  //DEBUG_SERIAL_PRINT_FLASHSTRING(" 1st=");
////  //DEBUG_SERIAL_PRINTFMT(jRTC, BIN);
////  //DEBUG_SERIAL_PRINTLN();
////#endif
////
////#ifndef NO_clockJitterEntropyByte
////  // Test full-byte jitter: assumed about 8 bits of entropy per call/result.
////  //DEBUG_SERIAL_PRINT_FLASHSTRING("jByte... ");
////  const uint8_t t0j = getSubCycleTime();
////  while(t0j == getSubCycleTime()) { } // Wait for sub-cycle time to roll to toughen test.
////  const uint8_t jByte = clockJitterEntropyByte();
////
////  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY/8; --i >= 0; )
////    {
////    if(jByte != clockJitterEntropyByte()) { break; } // Stop as soon as a different value is obtained.
////    AssertIsTrue(0 != i); // Generated too many identical values in a row.
////    }
////  //DEBUG_SERIAL_PRINT_FLASHSTRING(" 1st=");
////  //DEBUG_SERIAL_PRINTFMT(jByte, BIN);
////  //DEBUG_SERIAL_PRINT_FLASHSTRING(", ticks=");
////  //DEBUG_SERIAL_PRINT((uint8_t)(t1j - t0j - 1));
////  //DEBUG_SERIAL_PRINTLN();
////#endif
////
////  // Test noisy ADC read: assumed at least one bit of noise per call/result.
////  const uint8_t nar1 = noisyADCRead(true);
////#if 0
////  DEBUG_SERIAL_PRINT_FLASHSTRING("nar1 ");
////  DEBUG_SERIAL_PRINTFMT(nar1, BIN);
////  DEBUG_SERIAL_PRINTLN();
////#endif
////  for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY; --i >= 0; )
////    {
////    const uint8_t nar = noisyADCRead(true);
////    if(nar1 != nar) { break; } // Stop as soon as a different value is obtained.
////#if 0
////    DEBUG_SERIAL_PRINT_FLASHSTRING("repeat nar ");
////    DEBUG_SERIAL_PRINTFMT(nar, BIN);
////    DEBUG_SERIAL_PRINTLN();
////#endif
////    AssertIsTrue(0 != i); // Generated too many identical values in a row.
////    }
////
////  for(int w = 0; w < 2; ++w)
////    {
////    const bool whiten = (w != 0);
////    // Test secure random byte generation with and without whitening
////    // to try to ensure that the underlying generation is sound.
////    const uint8_t srb1 = getSecureRandomByte(whiten);
////#if 0
////    DEBUG_SERIAL_PRINT_FLASHSTRING("srb1 ");
////    DEBUG_SERIAL_PRINTFMT(srb1, BIN);
////    if(whiten) { DEBUG_SERIAL_PRINT_FLASHSTRING(" whitened"); }
////    DEBUG_SERIAL_PRINTLN();
////#endif
////    for(int i = MAX_IDENTICAL_BITS_SEQUENTIALLY/8; --i >= 0; )
////      {
////      if(srb1 != getSecureRandomByte(whiten)) { break; } // Stop as soon as a different value is obtained.
////      AssertIsTrue(0 != i); // Generated too many identical values in a row.
////      }
////    }
//  }

// Test sleepUntilSubCycleTime() routine.
void testSleepUntilSubCycleTime()
  {
#ifdef WAKEUP_32768HZ_XTAL
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("SleepUntilSubCycleTime");

  const uint8_t start = getSubCycleTime();

  // Check that this correctly notices/vetoes attempt to sleep until time already past.
  if(start > 0) { AssertIsTrue(!sleepUntilSubCycleTime(start-1)); }

  // Don't attempt rest of test if near the end of the current minor cycle...
  if(start > (GSCT_MAX/2)) { return; }

  // Set a random target significantly before the end of the current minor cycle.
#if 0x3f > GSCT_MAX/4
#error
#endif
  const uint8_t sleepTicks = 2 + (OTV0P2BASE::randRNG8() & 0x3f);
  const uint8_t target = start + sleepTicks;
  AssertIsTrue(target > start);
  AssertIsTrue(target < GSCT_MAX);

  // Call should succeed.
  AssertIsTrue(sleepUntilSubCycleTime(target));

  // Call should return with some of specified target tick still to run...
  const uint8_t end = getSubCycleTime();
  AssertIsTrueWithErr(target == end, end); // FIXME: DHD2014020: getting occasional failures.

#if 0
  DEBUG_SERIAL_PRINT_FLASHSTRING("Sleep ticks: ");
  DEBUG_SERIAL_PRINT(sleepTicks);
  DEBUG_SERIAL_PRINTLN();
#endif
#endif
  }

// Test that the simple smoothing function never generates an out of range value.
// In particular, with a legitimate value range of [0,254]
// smoothStatsValue() must never generate 255 (0xff) which looks like an uninitialised EEPROM value,
// nor wrap around in either direction.
static void testSmoothStatsValue()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("SmoothStatsValue");
  // Covers the key cases 0 and 254 in particular.
  for(int i = 256; --i >= 0; ) { AssertIsTrue((uint8_t)i == smoothStatsValue((uint8_t)i, (uint8_t)i)); }
  }


// Test temperature companding.
static void testTempCompand()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("TempCompand");
  // Ensure that all (whole) temperatures from 0C to 100C are correctly compressed and expanded.
  for(int i = 0; i <= 100; ++i)
    {
    //DEBUG_SERIAL_PRINT(i<<4); DEBUG_SERIAL_PRINT(" => "); DEBUG_SERIAL_PRINT(compressTempC16(i<<4)); DEBUG_SERIAL_PRINT(" => "); DEBUG_SERIAL_PRINT(expandTempC16(compressTempC16(i<<4))); DEBUG_SERIAL_PRINTLN();
    AssertIsTrueWithErr(i<<4 == expandTempC16(compressTempC16(i<<4)), i);
    }
  // Ensure that out-of-range inputs are coerced to the limits.
  AssertIsTrueWithErr(0 == expandTempC16(compressTempC16(-1)), -1);
  AssertIsTrueWithErr((100<<4) == expandTempC16(compressTempC16(101<<4)), 101);
  AssertIsTrueWithErr(COMPRESSION_C16_CEIL_VAL_AFTER == compressTempC16(102<<4), COMPRESSION_C16_CEIL_VAL_AFTER); // Verify ceiling.
  AssertIsTrue(COMPRESSION_C16_CEIL_VAL_AFTER < 0xff);
  // Ensure that 'unset' compressed value expands to 'unset' uncompressed value.
  AssertIsTrue(STATS_UNSET_INT == expandTempC16(STATS_UNSET_BYTE));
  }

// Test some of the mask/port calculations.
static void testFastDigitalIOCalcs()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("FastDigitalIOCalcs");
  AssertIsEqual(digitalPinToBitMask(0), fastDigitalMask(0));
  AssertIsEqual(digitalPinToBitMask(2), fastDigitalMask(2));
  AssertIsEqual(digitalPinToBitMask(13), fastDigitalMask(13));
  AssertIsEqual(digitalPinToBitMask(19), fastDigitalMask(19));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(0)), (intptr_t)fastDigitalInputRegister(0));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(2)), (intptr_t)fastDigitalInputRegister(2));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(7)), (intptr_t)fastDigitalInputRegister(7));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(8)), (intptr_t)fastDigitalInputRegister(8));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(14)), (intptr_t)fastDigitalInputRegister(14));
  AssertIsEqual((intptr_t)portInputRegister(digitalPinToPort(19)), (intptr_t)fastDigitalInputRegister(19));
  }







#if !defined(DISABLE_SENSOR_UNIT_TESTS)
// Test temperature sensor returns value in reasonable bounds for a test environment.
// Attempts to test that the sensor is actually present.
static void testTempSensor()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("TempSensor");
  const int temp = TemperatureC16.read();
#if 0
  OTV0P2BASE::serialPrintAndFlush("  temp: ");
  OTV0P2BASE::serialPrintAndFlush(temp >> 4, DEC);
  OTV0P2BASE::serialPrintAndFlush('C');
  OTV0P2BASE::serialPrintAndFlush(temp & 0xf, HEX);
  OTV0P2BASE::serialPrintlnAndFlush();
#endif
  // During testing temp should be above 0C (0C might indicate a missing/broken sensor) and below 50C.
  AssertIsTrueWithErr((temp > 0) && (temp < (50 << 4)), temp);
  }
#endif

#if !defined(DISABLE_SENSOR_UNIT_TESTS)
// Test that on-chip temperature sensor returns value in half-reasonable bounds for a test environment.
// Internal sensor may be +/- 10C out.
static void testInternalTempSensor()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("InternalTempSensor");
  const int temp = readInternalTemperatureC16();
#if 0
  OTV0P2BASE::serialPrintAndFlush("  int temp: ");
  OTV0P2BASE::serialPrintAndFlush(temp >> 4, DEC);
  OTV0P2BASE::serialPrintAndFlush('C');
  OTV0P2BASE::serialPrintAndFlush(temp & 0xf, HEX);
  OTV0P2BASE::serialPrintlnAndFlush();
#endif
  // During testing temp should be above 0C (0C might indicate a missing/broken sensor) and below 50C.
  // Internal sensor may be +/- 10C out.
  // DHD20141223: Just has a reading of ~17C from an otherwise-OK AVR with room temp ~20C.
  AssertIsTrueWithErr((temp > (-20 << 4)) && (temp < (60 << 4)), temp);
  }
#endif

#if !defined(DISABLE_SENSOR_UNIT_TESTS)
static void testSupplyVoltageMonitor()
  {
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("SupplyVoltageMonitor");
  const int mv = Supply_mV.read();
#if 0
  OTV0P2BASE::serialPrintAndFlush("  Battery mv: ");
  OTV0P2BASE::serialPrintAndFlush(mv, DEC);
  OTV0P2BASE::serialPrintlnAndFlush();
#endif
  // During testing power supply voltage should be above ~1.7V BOD limit,
  // and no higher than 3.6V for V0p2 boards which is RFM22 Vss limit.
  // Note that REV9 first boards are running at 3.6V nominal!
  AssertIsTrueWithErr((mv >= 1700) && (mv < 3700), mv);
  }
#endif




// To be called from loop() instead of main code when running unit tests.
// Tests generally flag an error and stop the test cycle with a call to panic() or error().
void loopUnitTest()
  {
  static int loopCount = 0;

  // Allow the terminal console to be brought up.
  for(int i = 3; i > 0; --i)
    {
    OTV0P2BASE::serialPrintAndFlush(F("Tests starting... "));
    OTV0P2BASE::serialPrintAndFlush(i);
    OTV0P2BASE::serialPrintlnAndFlush();
    OTV0P2BASE::sleepLowPowerMs(1000);
    }
  OTV0P2BASE::serialPrintlnAndFlush();


  // Run the tests, fastest / newest / most-fragile / most-interesting first...
  testLibVersions();
  testComputeRequiredTRVPercentOpen();
  testFastDigitalIOCalcs();
  testTargetComputation();
  testModeControls();
  testJSONStats();
  testJSONForTX();
  testFullStatsMessageCoreEncDec();
  testTempCompand();
  testRTCPersist();
  testQuartiles();
  testSmoothStatsValue();
  testSleepUntilSubCycleTime();
  testFHTEncoding();
  testFHTEncodingHeadAndTail();
  testSensorMocking();

  // Boiler-hub tests.
#ifdef ENABLE_BOILER_HUB
  testOnOffBoilerDriverLogic();
#endif

  // Sensor tests.
  // May need to be disabled if, for example, running in a simulator or on a partial board.
  // Should not involve anything too complex from the normal run-time, such as interrupts.
#if !defined(DISABLE_SENSOR_UNIT_TESTS)
  testTempSensor();
  testInternalTempSensor();
  testSupplyVoltageMonitor();
#endif


  // Announce successful loop completion and count.
  ++loopCount;
  OTV0P2BASE::serialPrintlnAndFlush();
  OTV0P2BASE::serialPrintAndFlush(F("%%% All tests completed OK, round "));
  OTV0P2BASE::serialPrintAndFlush(loopCount);
  OTV0P2BASE::serialPrintlnAndFlush();
  OTV0P2BASE::serialPrintlnAndFlush();
  OTV0P2BASE::serialPrintlnAndFlush();
  // Briefly flash the LED once to indicate successful completion of the tests.
  // (Panic/failure causes repeated rapid flash by contrast, and a hang may result in no flashes.)
  LED_HEATCALL_ON();
  tinyPause();
  LED_HEATCALL_OFF();
  // Help avoid tests spinning too fast even to see!
  // Also make panic() state flash clearly different to (faster than) this loop success/repeat.
  OTV0P2BASE::sleepLowPowerMs(2000);
  }


#endif



