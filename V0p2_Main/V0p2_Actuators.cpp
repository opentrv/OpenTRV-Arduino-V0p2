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
 V0p2 boards physical actuator support.
 */
#include <stdint.h>
#include <limits.h>
#include <util/atomic.h>

#include <Wire.h> // Arduino I2C library.

#include "V0p2_Main.h"
#include "V0p2_Board_IO_Config.h" // I/O pin allocation: include ahead of I/O module headers.
#include "V0p2_Actuators.h" // I/O code access.
#include "Control.h"


// DORM1/REV7 direct drive actuator.
#ifdef HAS_DORM1_VALVE_DRIVE
//#ifdef DIRECT_MOTOR_DRIVE_V1
// Singleton implementation/instance.
#ifdef ENABLE_DORM1_MOTOR_REVERSED // Reversed vs sample 2015/12
OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_ML, MOTOR_DRIVE_MR, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#else
OTRadValve::ValveMotorDirectV1<MOTOR_DRIVE_MR, MOTOR_DRIVE_ML, MOTOR_DRIVE_MI_AIN, MOTOR_DRIVE_MC_AIN> ValveDirect;
#endif // HAS_DORM1_MOTOR_REVERSED
#endif


// FHT8V radio-controlled actuator.
#ifdef USE_MODULE_FHT8VSIMPLE
// Function to append stats trailer (and 0xff) to FHT8V/FS20 TX buffer.
// Assume enough space in buffer for largest possible stats message.
#if defined(ALLOW_STATS_TX)
uint8_t *appendStatsToTXBufferWithFF(uint8_t *bptr, const uint8_t bufSize)
  {
  FullStatsMessageCore_t trailer;
  populateCoreStats(&trailer);
  // Ensure that no ID is encoded in the message sent on the air since it would be a repeat from the FHT8V frame.
  trailer.containsID = false;

#if defined(ENABLE_MINIMAL_STATS_TXRX)
  // As bandwidth optimisation just write minimal trailer if only temp&power available.
  if(trailer.containsTempAndPower &&
     !trailer.containsID && !trailer.containsAmbL)
    {
    writeTrailingMinimalStatsPayload(bptr, &(trailer.tempAndPower));
    bptr += 3;
    *bptr = (uint8_t)0xff; // Terminate TX bytes.
    }
  else
#endif
    {
    // Assume enough space in buffer for largest possible stats message.
    bptr = encodeFullStatsMessageCore(bptr, bufSize, OTV0P2BASE::getStatsTXLevel(), false, &trailer);
    }
  return(bptr);
  }
#else
#define appendStatsToTXBufferWithFF NULL // Do not append stats.
#endif
#endif // USE_MODULE_FHT8VSIMPLE

#ifdef USE_MODULE_FHT8VSIMPLE
OTRadValve::FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V(appendStatsToTXBufferWithFF);
#endif

// Clear both housecode parts (and thus disable local valve).
// Does nothing if FHT8V not in use.
#ifdef USE_MODULE_FHT8VSIMPLE
void FHT8VClearHC()
  {
  FHT8V.clearHC();
  OTV0P2BASE::eeprom_smart_erase_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC1);
  OTV0P2BASE::eeprom_smart_erase_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC2);
  }
#endif

// Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
// Also set value in FHT8V rad valve model.
// Does nothing if FHT8V not in use.
#ifdef USE_MODULE_FHT8VSIMPLE
void FHT8VSetHC1(uint8_t hc)
  {
  FHT8V.setHC1(hc);
  OTV0P2BASE::eeprom_smart_update_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC1, hc);
  }
void FHT8VSetHC2(uint8_t hc)
  {
  FHT8V.setHC2(hc);
  OTV0P2BASE::eeprom_smart_update_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC2, hc);
  }
#endif

// Get (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control (will be 0xff until set).
// FHT8V instance values are used as a cache.
// Does nothing if FHT8V not in use.
#ifdef USE_MODULE_FHT8VSIMPLE
uint8_t FHT8VGetHC1()
  {
  const uint8_t vv = FHT8V.getHC1();
  // If cached value in FHT8V instance is valid, return it.
  if(OTRadValve::FHT8VRadValveBase::isValidFHTV8HouseCode(vv))
    { return(vv); }
  // Else if EEPROM value is valid, then cache it in the FHT8V instance and return it.
  const uint8_t ev = eeprom_read_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC1);
  if(OTRadValve::FHT8VRadValveBase::isValidFHTV8HouseCode(ev))
      { FHT8V.setHC1(ev); }
  return(ev);
  }
uint8_t FHT8VGetHC2()
  {
  const uint8_t vv = FHT8V.getHC2();
  // If cached value in FHT8V instance is valid, return it.
  if(OTRadValve::FHT8VRadValveBase::isValidFHTV8HouseCode(vv))
    { return(vv); }
  // Else if EEPROM value is valid, then cache it in the FHT8V instance and return it.
  const uint8_t ev = eeprom_read_byte((uint8_t*)V0P2BASE_EE_START_FHT8V_HC2);
  if(OTRadValve::FHT8VRadValveBase::isValidFHTV8HouseCode(ev))
      { FHT8V.setHC2(ev); }
  return(ev);
  }
#endif // USE_MODULE_FHT8VSIMPLE

#ifdef USE_MODULE_FHT8VSIMPLE
// Load EEPROM house codes into primary FHT8V instance at start-up or once cleared in FHT8V instance.
void FHT8VLoadHCFromEEPROM()
  {
  // Uses side-effect to cache/save in FHT8V instance.
  FHT8VGetHC1();
  FHT8VGetHC2();
  }
#endif // USE_MODULE_FHT8VSIMPLE















//#if defined(ENABLE_BOILER_HUB)
//// Boiler output control.
//// Singleton implementation/instance.
//extern BoilerDriver BoilerControl;
//#endif
