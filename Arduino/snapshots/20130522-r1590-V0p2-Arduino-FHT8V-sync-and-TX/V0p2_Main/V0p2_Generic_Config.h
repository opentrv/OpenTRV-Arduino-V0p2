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
 TRV (and boiler-node) global configuration parameters for V0.09 PCB1 hardware.
 The top part should contain one #define CONFIG_... and nothing else (uncommented).
 */

#ifndef V0P2_GENERIC_CONFIG_H
#define V0P2_GENERIC_CONFIG_H


// Define/uncomment exactly one of the CONFIG_XXX labels to enable a configuration set below.
// Some can be specific to particular locations and boards,
// others can be vanilla ready to be configured by the end-user one way or another.

//#define CONFIG_GENERIC_ROOM_NODE
//#define CONFIG_GENERIC_BOILER_NODE
//#define CONFIG_GENERIC_RANDB_NODE
//#define CONFIG_GENERIC_DHW_NODE


// Some specific/example configs.
//#define CONFIG_DHD_STUDY
//#define CONFIG_DHD_KITCHEN
//#define CONFIG_DHD_LIVINGROOM
//#define CONFIG_DHD_BEDROOM1
#define CONFIG_DHD_TESTLAB
//#define CONFIG_BH_DHW ; Bo's hot water.







// ------------------------------------------------------
// PRE-DEFINED CONFIG_... BUNDLE IMPLEMENTATION/EXPANSION

// Defaults for V0.2; have to be undefined if not required.
//#define HALF_SECOND_RTC_SUPPORT // Wake at 2Hz for accurate 0.5-second timing for some possible extra power draw.
#define SUPPLY_VOLTAGE_LOW_2AA // May require limiting clock speed and using some alternative peripherals/sensors.
#define USE_RTC_INTERNAL_SIMPLE // Provide software RTC support by default.


#ifdef CONFIG_DHD_TESTLAB // DHD's test lab with TRV.
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define SUPPORT_CLI
// IF DEFINED: this unit supports BAKE mode.
#define SUPPORT_BAKE
// IF DEFINED: this unit may run on 2xAA cells, preferably rechargeable eg NiMH, ~2V--2.4V, and should monitor supply voltage with calibadc.
#define SUPPLY_VOLTAGE_LOW_2AA // May require limiting clock speed and using some alternative peripherals/sensors...
// IF DEFINED: this unit will act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
//#define BOILER_HUB
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler).
#define LOCAL_TRV
// IF DEFINED: Minimise TRV slew rates to minimise flow rates to minimise m^3 charges.
//#define TRV_SLEW_GLACIAL 
// IF DEFINED: basic FROST/WARM temperatures are settable.
#define SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: use FHT8V wireless radio module/valve.
#define USE_MODULE_FHT8VSIMPLE
// If DEFINED: ignore the FHT8V sync procedure and send every second (which is wasteful of resources but may help with debugging).
//#define IGNORE_FHT_SYNC
// IF DEFINED: RFM23 is in use in place of RFM22.
//#define RFM22_IS_ACTUALLY_RFM23 // Note: RFM23 used on V0.09 PCB1.
// IF DEFINED: good RF environment means that TX power level can be reduced.
//#define RFM22_GOOD_RF_ENV ; Good ground-plane and antenna on V0.09 PCB: drop TX level.
// IF DEFINED: use simple LDR-based detection of room use/occupancy; brings in getRoomInUseFromLDR subroutine.
#define USE_MODULE_LDROCCUPANCYDETECTION
// If LDR is not to be used then specifically define OMIT_... as below.
//#define OMIT_MODULE_LDROCCUPANCYDETECTION ; LDR 'occupancy' sensing irrelevant for DHW.
// IF DEFINED: use software RTC.
#define USE_RTC_INTERNAL_SIMPLE // Provide software RTC support by default.
// IF DEFINED: produce regular status reports on sertxd.
//#define SERTXD_STATUS_REPORTS
// IF DEFINED: use active-low LEARN button.  Needs SUPPORT_SINGLETON_SCHEDULE.
#define LEARN_BUTTON_AVAILABLE // OPTIONAL ON V0.09 PCB1
// IF DEFINED: support one on and one off time per day (possibly in conjunction with 'learn' button).
#define SUPPORT_SINGLETON_SCHEDULE
//#define DEFAULT_SINGLETON_SCHEDULE_ON ~0 // No default on.
//#define DEFAULT_SINGLETON_SCHEDULE_OFF ~0 // No default off.
#endif







// --------------------------------------------
// CONSEQUENTIAL DEFINITIONS ARISING FROM ABOVE
// (Don't fiddle with these unless you are sure of module dependencies, etc!)


// If LEARN_BUTTON_AVAILABLE then in the absence of anything better SUPPORT_SINGLETON_SCHEDULE should be supported.
#ifdef LEARN_BUTTON_AVAILABLE
#ifndef SUPPORT_SINGLETON_SCHEDULE
#define SUPPORT_SINGLETON_SCHEDULE
#endif
#endif

// By default, use the RFM22/RFM23 module to talk to an FHT8V wireless radiator valve.
#ifdef USE_MODULE_FHT8VSIMPLE
#define USE_MODULE_RFM22RADIOSIMPLE
#endif


#endif
