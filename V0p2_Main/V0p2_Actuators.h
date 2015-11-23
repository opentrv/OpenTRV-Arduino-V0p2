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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2015
*/

/*
 * Header for main on-board sensors and actuators for V0p2 variants.
 */

#ifndef V0P2_ACTUATORS_H
#define V0P2_ACTUATORS_H

#include "V0p2_Main.h"

#include <OTRadioLink.h>
#include <OTRadValve.h>

#include "Messaging.h"


// DORM1/REV7 direct drive actuator.
#if /* defined(LOCAL_TRV) && */ defined(DIRECT_MOTOR_DRIVE_V1)
#define HAS_DORM1_VALVE_DRIVE
// Singleton implementation/instance.
extern OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_ML, MOTOR_DRIVE_MR, MOTOR_DRIVE_MI_AIN> ValveDirect;
#endif


// FHT8V radio-controlled actuator.
#ifdef USE_MODULE_FHT8VSIMPLE
// Singleton FHT8V valve instance (to control remote FHT8V valve by radio).
static const uint8_t _FHT8V_MAX_EXTRA_TRAILER_BYTES = (1 + max(MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES, FullStatsMessageCore_MAX_BYTES_ON_WIRE));
extern OTRadValve::FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V;
// This unit may control a local TRV.
// Returns TRV if valve/radiator is to be controlled by this unit.
// Usually the case, but may not be for (a) a hub or (b) a not-yet-configured unit.
// Returns false if house code parts are set to invalid or uninitialised values (>99).
#if defined(LOCAL_TRV) || defined(SLAVE_TRV)
inline bool localFHT8VTRVEnabled() { return(!FHT8V.isUnavailable()); }
#else
#define localFHT8VTRVEnabled() (false) // Local FHT8V TRV disabled.
#endif
// Clear both housecode parts (and thus disable local valve).
void FHT8VClearHC();
// Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
// Will cache in FHT8V instance for speed.
void FHT8VSetHC1(uint8_t hc);
void FHT8VSetHC2(uint8_t hc);
// Get (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control (will be 0xff until set).
// Used FHT8V instance as a transparent cache of the values for speed.
uint8_t FHT8VGetHC1();
uint8_t FHT8VGetHC2();
inline uint16_t FHT8VGetHC() { return(FHT8VGetHC2() | (((uint16_t) FHT8VGetHC1()) << 8)); }
// Load EEPROM house codes into primary FHT8V instance at start-up or once cleared in FHT8V instance.
void FHT8VLoadHCFromEEPROM();
#endif // USE_MODULE_FHT8VSIMPLE















#if defined(ENABLE_BOILER_HUB)
// Internal logic for simple on/off boiler output, fully testable.
class OnOffBoilerDriverLogic
  {
  public:
    // Maximum distinct radiators tracked by this system.
    // The algorithms uses will assume that this is a smallish number,
    // and may work best of a power of two.
    // Reasonable candidates are 8 or 16.
    static const uint8_t maxRadiators = 8;

    // Per-radiator data status.
    struct PerIDStatus
      {
      // ID of remote device; an empty slot is marked with the (invalid) ID of 0xffffu.
      uint16_t id;
      // Ticks until we expect new update from given valve else its data has expired when non-negative.
      // A zero or negative value means no active call for heat from the matching ID.
      // A negative value indicates that a signal is overdue by that number of ticks
      // or units greater than ticks to allow old entries to linger longer for tracking or performance reasons.
      // An empty slot is given up for an incoming new ID calling for heat.
      int8_t ticksUntilOff;
      // Last percent open for given valve.
      // A zero value means no active call for heat from the matching ID.
      // An empty slot is given up for an incoming new ID calling for heat.
      uint8_t percentOpen;
      };

  private:
    // True to call for heat from the boiler.
    bool callForHeat;

    // Number of ticks that boiler has been in current state, on or off, to avoid short-cycling.
    // The state cannot be changed until the specified minimum off/on ticks have been passed.
    // (This rule may be ignored when the boiler is currently on if all valves seem to have been turned off prematurely.)
    // This value does not roll back round to zero, ie will stop at maximum until reset.
    // The max representable value allows for several hours at 1 or 2 seconds per tick.
    uint8_t ticksInCurrentState;

    // Ticks minimum for boiler to stay in each state to avoid short-cycling; should be significantly positive but won't fail if otherwise.
    // Typically the equivalent of 2--10 minutes.
    uint8_t minTicksInEitherState;

    // Minimum individual valve percentage to be considered open [1,100].
    uint8_t minIndividualPC;

    // Minimum aggrigate valve percentage to be considered open, no lower than minIndividualPC; [1,100].
    uint8_t minAggregatePC;

    // 'Bad' (never valid as housecode or OpenTRV code) ID.
    static const uint16_t badID = 0xffffu;

#if defined(BOILER_RESPOND_TO_SPECIFIED_IDS_ONLY)
    // List of authorised IDs.
    // Entries beyond the last valid authorised ID are set to 0xffffu (never a valid ID);
    // if there are none then [0] is set to 0xffffu.
    // If no authorised IDs then no authorisation is done and all IDs are accepted
    // and kicked out expiry-first if there is a shortage of slots.
    uint16_t authedIDs[maxRadiators];
#endif

    // List of recently-heard IDs and ticks until their last call for heat will expire (matched by index).
    // If there any authorised IDs then only such IDs are admitted to the list.
    // The first empty empty slot is marked with the (invalid) ID of 0xffffu.
    // Marked volatile since may be updated by ISR.
    volatile PerIDStatus status[maxRadiators];

#if defined(OnOffBoilerDriverLogic_CLEANUP)
    // Do some incremental clean-up to speed up future operations.
    // Aim to free up at least one status slot if possible.
    void cleanup();
#endif

  public:
    OnOffBoilerDriverLogic() : callForHeat(false), ticksInCurrentState(0), minTicksInEitherState(60), minIndividualPC(OTRadValve::DEFAULT_VALVE_PC_MIN_REALLY_OPEN), minAggregatePC(OTRadValve::DEFAULT_VALVE_PC_MODERATELY_OPEN)
      {
#if defined(BOILER_RESPOND_TO_SPECIFIED_IDS_ONLY)
      authedIDs[0] = badID;
#endif
      status[0].id = badID;
      }

    // Set thresholds for per-value and minimum-aggregate percentages to fire the boiler.
    // Coerces values to be valid:
    // minIndividual in range [1,100] and minAggregate in range [minIndividual,100].
    void setThresholds(uint8_t minIndividual, uint8_t minAggregate);

    // Set minimum ticks for boiler to stay in each state to avoid short-cycling; should be significantly positive but won't fail if otherwise.
    // Typically the equivalent of 2--10 minutes (eg ~2+ for gas, ~8 for oil).
    void setMinTicksInEitherState(uint8_t minTicks) { minTicksInEitherState = minTicks; }

    // Called upon incoming notification of status or call for heat from given (valid) ID.
    // ISR-/thread- safe to allow for interrupt-driven comms, and as quick as possible.
    // Returns false if the signal is rejected, eg from an unauthorised ID.
    // The basic behaviour is that a signal with sufficient percent open
    // is good for 2 minutes (120s, 60 ticks) unless explicitly cancelled earlier,
    // for all valve types including FS20/FHT8V-style.
    // That may be slightly adjusted for IDs that indicate FS20 housecodes, etc.
    //   * id  is the two-byte ID or house code; 0xffffu is never valid
    //   * percentOpen  percentage open that the remote valve is reporting
    bool receiveSignal(uint16_t id, uint8_t percentOpen);

    // Iff true then call for heat from the boiler.
    bool isCallingForHeat() const { return(callForHeat); }

    // Poll every 2 seconds in real/virtual time to update state in particular the callForHeat value.
    // Not to be called from ISRs,
    // in part because this may perform occasional expensive-ish operations
    // such as incremental clean-up.
    // Because this does not assume a tick is in real time
    // this remains entirely unit testable,
    // and no use of wall-clock time is made within this or sibling class methods.
    void tick2s();

    // Fetches statuses of valves recently heard from and returns the count; 0 if none.
    // Optionally filters to return only those still live and apparently calling for heat.
    //   * valves  array to copy status to the start of; never null
    //   * size  size of valves[] in entries (not bytes), no more entries than that are used,
    //     and no more than maxRadiators entries are ever needed
    //   * onlyLiveAndCallingForHeat  if true retrieves only current entries
    //     'calling for heat' by percentage
    uint8_t valvesStatus(PerIDStatus valves[], uint8_t size, bool onlyLiveAndCallingForHeat) const;
  };


// Boiler output control (call-for-heat driver).
// Nominally drives on scale of [0,100]%
// but any non-zero value should be regarded as calling for heat from an on/off boiler,
// and only values of 0 and 100 may be produced.
// Implementations require poll() called at a fixed rate (every 2s).
class BoilerDriver : public OTV0P2BASE::SimpleTSUint8Actuator
  {
  private:
    OnOffBoilerDriverLogic logic;

  public:
    // Regular poll/update.
    virtual uint8_t read();

    // Preferred poll interval (2 seconds).
    virtual uint8_t preferredPollInterval_s() const { return(2); }

  };
// Singleton implementation/instance.
extern BoilerDriver BoilerControl;
#endif


#endif
