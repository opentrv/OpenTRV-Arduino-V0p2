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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2014
*/

/*
 TRV (and boiler-node) global configuration parameters for V0.2 PCB hardware.
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
//#define CONFIG_DHD_TESTLAB_REV0
//#define CONFIG_Trial2013Winter_Round1
//#define CONFIG_BH_DHW // Bo's hot water.
//#define CONFIG_BH_TESTLAB
//#define CONFIG_DHD_TESTLAB_REV2
#define CONFIG_Trial2013Winter_Round2







// ------------------------------------------------------
// PRE-DEFINED CONFIG_... BUNDLE IMPLEMENTATION/EXPANSION

// Defaults for V0.2; have to be undefined if not required.
//#define HALF_SECOND_RTC_SUPPORT // Wake at 2Hz for accurate 0.5-second timing for some possible extra power draw.
#define SUPPLY_VOLTAGE_LOW_2AA // May require limiting clock speed and using some alternative peripherals/sensors.
#define USE_RTC_INTERNAL_SIMPLE // Provide software RTC support by default.


// -------------------------

#ifdef CONFIG_DHD_TESTLAB_REV0 // DHD's test lab with TRV.
// Revision of V0.2 board.
#define V0p2_REV 0 // REV0 covers initial breadboard and first V0.2 PCB.
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------

#ifdef CONFIG_BH_TESTLAB // BH's test lab with TRV.
// Revision of V0.2 board.
#define V0p2_REV 1
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------

#ifdef CONFIG_Trial2013Winter_Round1 // For trial over winter of 2013--4, first round (REV1).
// Revision REV1 of V0.2 board.
#define V0p2_REV 1
// Use common settings.
#define COMMON_SETTINGS
#endif

#ifdef CONFIG_Trial2013Winter_Round2 // For trial over winter of 2013--4, second round (REV2).
// Revision REV2 (cut4+) of V0.2 board.
#define V0p2_REV 2
// IF DEFINED: allow for less light on sideways-pointing LDR on cut4 2014/03/17 REV2 boards (TODO-209).
#define LDR_EXTRA_SENSITIVE
// IF DEFINED: reverse direction of REV2 (nominally cut4) temperature put usage.
#define TEMP_POT_REVERSE
// Use common settings.
#define COMMON_SETTINGS
#endif


// -------------------------

#ifdef CONFIG_DHD_TESTLAB_REV2 // DHD's test lab with TRV.
// Revision of V0.2 board.
#define V0p2_REV 2 // Updated breadboard.
// IF DEFINED: reverse direction of REV2 (nominally cut4) temperature put usage.
#define TEMP_POT_REVERSE
// Use common settings.
#define COMMON_SETTINGS
#endif





// --------------------------------------------
// CONSEQUENTIAL DEFINITIONS ARISING FROM ABOVE
// (Don't fiddle with these unless you are sure of module dependencies, etc!)


#ifdef COMMON_SETTINGS // FOR REV0 and REV1.
#if (V0p2_REV >= 1) && (V0p2_REV <= 2) && !defined(CONFIG_DHD_TESTLAB_REV2) // All REV 1--2 PCBs use RFM23.
// IF DEFINED: RFM23 is in use in place of RFM22.
#define RFM22_IS_ACTUALLY_RFM23 // Note: RFM23 used on V0.2 PCB.
#endif
#ifdef RFM22_IS_ACTUALLY_RFM23 // Note: All RFM23s on PCBs with good ground place.
// IF DEFINED: good RF environment means that TX power level can be reduced.
#define RFM22_GOOD_RF_ENV // Good ground-plane and antenna on V0.2 PCB: drop TX level.
#endif
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define SUPPORT_CLI
// IF DEFINED: this unit supports BAKE mode.
#define SUPPORT_BAKE
// IF DEFINED: this unit may run on 2xAA cells, preferably rechargeable eg NiMH, ~2V--2.4V, and should monitor supply voltage.
#define SUPPLY_VOLTAGE_LOW_2AA // May require limiting clock speed and using some alternative peripherals/sensors...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler).
#define LOCAL_TRV
// IF DEFINED: basic FROST/WARM temperatures are settable.
#define SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: use FHT8V wireless radio module/valve.
#define USE_MODULE_FHT8VSIMPLE
// IF DEFINED: use simple LDR-based detection of room use/occupancy; brings in getRoomInUseFromLDR subroutine.
#define USE_MODULE_LDROCCUPANCYDETECTION
// If LDR is not to be used then specifically define OMIT_... as below.
//#define OMIT_MODULE_LDROCCUPANCYDETECTION ; LDR 'occupancy' sensing irrelevant for DHW.
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#define OCCUPANCY_SUPPORT
// IF DEFINED: use software RTC.
#define USE_RTC_INTERNAL_SIMPLE // Provide software RTC support by default.
// IF DEFINED: use active-low LEARN button(s).  Needs SUPPORT_SINGLETON_SCHEDULE.
#define LEARN_BUTTON_AVAILABLE // OPTIONAL ON V0.09 PCB1
// IF DEFINED: support one on and one off time per day (possibly in conjunction with 'learn' button).
#define SUPPORT_SINGLETON_SCHEDULE
#endif


// If LEARN_BUTTON_AVAILABLE then in the absence of anything better SUPPORT_SINGLETON_SCHEDULE should be supported.
#ifdef LEARN_BUTTON_AVAILABLE
#ifndef SUPPORT_SINGLETON_SCHEDULE
#define SUPPORT_SINGLETON_SCHEDULE
#endif
#endif

// By default, use the RFM22/RFM23 module to talk to an FHT8V wireless radiator valve.
#ifdef USE_MODULE_FHT8VSIMPLE
#define USE_MODULE_RFM22RADIOSIMPLE
// If this can be a hub, enable extra RX code.
#ifdef ENABLE_BOILER_HUB
#define USE_MODULE_FHT8VSIMPLE_RX
#endif
#endif



#endif


