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

Author(s) / Copyright (s): Damon Hart-Davis 2014--2016
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
#ifdef ENABLE_DORM1_MOTOR_REVERSED // Reversed vs sample 2015/12
extern OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_ML, MOTOR_DRIVE_MR, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#else
extern OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_MR, MOTOR_DRIVE_ML, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#endif // HAS_DORM1_MOTOR_REVERSED
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



//#if defined(ENABLE_BOILER_HUB)
//// Singleton implementation/instance.
//extern BoilerDriver BoilerControl;
//#endif


#endif
