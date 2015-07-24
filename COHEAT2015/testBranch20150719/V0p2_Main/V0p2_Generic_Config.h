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


// Production configs.
//#define CONFIG_Trial2013Winter_Round1 // REV1
//#define CONFIG_Trial2013Winter_Round2 // REV2 cut4
//#define CONFIG_Trial2013Winter_Round2_BOILERHUB // REV2 cut4 as boiler hub.
//#define CONFIG_Trial2013Winter_Round2_STATSHUB // REV2 cut4 as stats hub.
#define CONFIG_Trial2013Winter_Round2_NOHUB // REV2 cut4 as TX-only leaf node.
//#define CONFIG_DORM1 // REV7 / DORM1 Winter 2014/2015 all-in-one valve unit.
//#define CONFIG_DORM1_BOILER // REV8 / DORM1 Winter 2014/2015 boiler-control unit.


// One-offs and special cases.
//#define CONFIG_DHD_TESTLAB_REV0 // REV0 / breadboard.
//#define CONFIG_DHD_TESTLAB_REV1 // REV1.
//#define CONFIG_DHD_TESTLAB_REV4 // REV4 cut2
//#define CONFIG_DHD_TESTLAB_REV4_NOHUB // REV4 cut2, no hub.
//#define CONFIG_BH_DHW // Bo's hot water.
//#define CONFIG_BH_TESTLAB
//#define CONFIG_DORM1_SANS32K // REV7 / DORM1 without working 32768Hz clock.
//#define CONFIG_REV7N // REV7 with external "Model N" valve.
//#define CONFIG_REV9 // REV9







// ------------------------------------------------------
// PRE-DEFINED CONFIG_... BUNDLE IMPLEMENTATION/EXPANSION
// These features can be turned off if not required in particular implementations.

// Defaults for V0.2; have to be undefined if not required.
// Use sleep wakeup (2Hz by default) from external 32768Hz xtal and timer 2.
#define WAKEUP_32768HZ_XTAL
// May require limiting clock speed and using some alternative peripherals/sensors.
#define SUPPLY_VOLTAGE_LOW_2AA
// Provide software RTC support by default.
#define USE_RTC_INTERNAL_SIMPLE
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#define LOCAL_TRV
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#define ALLOW_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ALLOW_STATS_TX
// IF DEFINED: allow JSON stats frames alongside binary ones.
#define ALLOW_JSON_OUTPUT
// IF DEFINED: use active-low LEARN button(s).  Needs SUPPORT_SINGLETON_SCHEDULE.
#define LEARN_BUTTON_AVAILABLE // OPTIONAL ON V0.09 PCB1
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.
#undef MIN_ENERGY_BOOT






// -------------------------

#ifdef CONFIG_Trial2013Winter_Round1 // For trial over winter of 2013--4, first round (REV1).
// Revision REV1 of V0.2 board.
#define V0p2_REV 1
// TODO-264: Find out why IDLE seems to crash some REV1 boards.
#define DISABLE_AVR_IDLE_MODE
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: don't allow RX of stats frames (since there is no easy way to plug in a serial connection to relay them!)
#undef ALLOW_STATS_RX
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------

#ifdef CONFIG_Trial2013Winter_Round2_BOILERHUB // For trial over winter of 2013--4, second round (REV2), as pure boiler hub.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ALLOW_STATS_RX
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
//#undef LOCAL_TRV // FOR NOW *disable* local TRV support.
#endif

#ifdef CONFIG_Trial2013Winter_Round2_STATSHUB // For trial over winter of 2013--4, second round (REV2), as stats hub.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#define ALLOW_STATS_RX
#endif

#ifdef CONFIG_Trial2013Winter_Round2_NOHUB // REV2 cut4 as TX-only leaf node.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ALLOW_STATS_RX
#endif

#ifdef CONFIG_Trial2013Winter_Round2 // For trial over winter of 2013--4, second round (REV2).
// Revision REV2 (cut4+) of V0.2 board.
#define V0p2_REV 2
// IF DEFINED: allow for less light on sideways-pointing LDR on cut4 2014/03/17 REV2 boards (TODO-209).
#define LDR_EXTRA_SENSITIVE
// Use common settings.
#define COMMON_SETTINGS
#endif




// -------------------------
// NON-STANDARD DEFINITIONS
// -------------------------

#ifdef CONFIG_DHD_TESTLAB_REV0 // DHD's test lab breadboard with TRV.
// Revision of V0.2 board.
#define V0p2_REV 0 // REV0 covers DHD's breadboard (was first V0.2 PCB).
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.
#define MIN_ENERGY_BOOT
//// Enable use of DS18B20 temp sensor.
//#define SENSOR_DS18B20_ENABLE
// Enable use of SHT21 RH and temp sensor.
//#define SENSOR_SHT21_ENABLE
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
//// Enable experimental voice detection.
//#define ENABLE_VOICE_SENSOR
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: don't allow RX of stats frames (since there is no easy way to plug in a serial connection to relay them!)
#undef ALLOW_STATS_RX
// IF DEFINED: initial direct motor drive design.
//#define DIRECT_MOTOR_DRIVE_V1
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------

#ifdef CONFIG_DHD_TESTLAB_REV1 // DHD's test lab on REV1.
// Revision of V0.2 board.
#define V0p2_REV 1
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.
#define MIN_ENERGY_BOOT
//// Enable use of DS18B20 temp sensor.
//#define SENSOR_DS18B20_ENABLE
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
//// Enable experimental voice detection.
//#define ENABLE_VOICE_SENSOR
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: don't allow RX of stats frames (since there is no easy way to plug in a serial connection to relay them!)
#undef ALLOW_STATS_RX
// IF DEFINED: initial direct motor drive design.
//#define DIRECT_MOTOR_DRIVE_V1
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------

#ifdef CONFIG_DHD_TESTLAB_REV4_NOHUB // REV4 board but no listening...
#define CONFIG_DHD_TESTLAB_REV4
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ALLOW_STATS_RX
#endif

#ifdef CONFIG_DHD_TESTLAB_REV4 // DHD's test lab with TRV on REV4 (cut2) board.
// Revision of V0.2 board.
#define V0p2_REV 4 // REV0 covers DHD's breadboard and first V0.2 PCB.
// Enable use of SHT21 RH and temp sensor.
#define SENSOR_SHT21_ENABLE
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
//#undef ENABLE_BOILER_HUB
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------

#ifdef CONFIG_DORM1_SANS32K // REV7 / DORM1 without working 32768Hz clock.
#define CONFIG_DORM1
#undef WAKEUP_32768HZ_XTAL
#endif

#ifdef CONFIG_DORM1 // For trial over winter of 2014--5, all-in-one (REV7).
// Revision REV7 of V0.2 board, all-in-one valve unit with local motor drive.
// Does not ever need to act as a boiler hub nor to receive stats.
#define V0p2_REV 7
// IF DEFINED: initial direct motor drive design.
#define DIRECT_MOTOR_DRIVE_V1
// Enable use of SHT21 RH and temp sensor.
#define SENSOR_SHT21_ENABLE
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: do not allow RX of stats frames.
#undef ALLOW_STATS_RX
// IF UNDEFINED: do not allow JSON stats frames (to save ~3kB Flash at 20141210).
//#undef ALLOW_JSON_OUTPUT
// Use common settings.
#define COMMON_SETTINGS
#endif

#ifdef CONFIG_DORM1_BOILER // For trial over winter of 2014--5, REV8 boiler-control counterpart to REV7.
// Revision REV8 of V0.2 board, boiler control unit.
#define V0p2_REV 8
// No working xtal on initial batch.
#undef WAKEUP_32768HZ_XTAL
// TMP112 to save a few pennies?
//#define SENSOR_SHT21_ENABLE
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF UNDEFINED: do not allow JSON stats frames (to save ~3kB Flash at 20141210).
//#undef ALLOW_JSON_OUTPUT
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------

#ifdef CONFIG_REV9 // REV9 (initial board release) derived from REV4.
// Revision of V0.2 board.
#define V0p2_REV 9 // REV0 covers DHD's breadboard and first V0.2 PCB.
//// Enable use of OneWire devices.
//#define SUPPORT_ONEWIRE
//// Enable use of DS18B20 temp sensor.
//#define SENSOR_DS18B20_ENABLE
// Enable use of SHT21 RH and temp sensor.
#define SENSOR_SHT21_ENABLE
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// For 1st-cust REV9 boards phototransistor was accidentally pulling down not up.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400_WRONG_WAY
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: no LEARN mode for REV9 boards (window sensor(s) instead).
//#undef LEARN_BUTTON_AVAILABLE
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------








#ifdef CONFIG_BH_DHW // DHW on REV1 board.
// Revision of V0.2 board.
#define V0p2_REV 1
// Enable use of OneWire devices.
#define SUPPORT_ONEWIRE
// Enable use of DS18B20 temp sensor.
#define SENSOR_DS18B20_ENABLE
// Select DHW temperatures by default.
#define DHW_TEMPERATURES
// Must minimise water flow.
#define TRV_SLEW_GLACIAL
// Set max percentage open: BH reports 30% to be optimal 2015/03.
#define TRV_MAX_PC_OPEN 30
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: don't allow RX of stats frames (since there is no easy way to plug in a serial connection to relay them!)
#undef ALLOW_STATS_RX
// Use common settings.
#define COMMON_SETTINGS
#endif





















// --------------------------------------------
// CONSEQUENTIAL DEFINITIONS ARISING FROM ABOVE
// (Don't fiddle with these unless you are sure of module interdependencies, etc!)


#ifdef COMMON_SETTINGS // FOR REV0 and REV1.
#if (V0p2_REV >= 1) // && (V0p2_REV <= 8) // && !defined(CONFIG_DHD_TESTLAB_REV2) // All REV 1--8 PCBs use RFM23B.
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
// IF DEFINED: basic FROST/WARM temperatures are settable.
#define SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: use FHT8V wireless radio module/valve.
#define USE_MODULE_FHT8VSIMPLE
// IF DEFINED: use simple LDR-based detection of room use/occupancy; brings in getRoomInUseFromLDR subroutine.
#define USE_MODULE_LDROCCUPANCYDETECTION
// If LDR is not to be used then specifically define OMIT_... as below.
//#define OMIT_MODULE_LDROCCUPANCYDETECTION //  LDR 'occupancy' sensing irrelevant for DHW.
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#define OCCUPANCY_SUPPORT
// IF DEFINED: use software RTC.
#define USE_RTC_INTERNAL_SIMPLE // Provide software RTC support by default.
// IF DEFINED: support one on and one off time per day (possibly in conjunction with 'learn' button).
#define SUPPORT_SINGLETON_SCHEDULE
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
//#define ENABLE_BOILER_HUB // NOT defined by default to allow omission for pure leaf nodes.
//// IF DEFINED: allow TX of stats frames.
//#define ALLOW_STATS_TX
// IF DEFINED: allow RX of stats frames.
//#define ALLOW_STATS_RX
// IF DEFINED: allow minimal binary format in addition to more generic one: ~400 bytes.
//#define ALLOW_MINIMAL_STATS_TXRX
#endif


// If LEARN_BUTTON_AVAILABLE then in the absence of anything better SUPPORT_SINGLETON_SCHEDULE should be supported.
#ifdef LEARN_BUTTON_AVAILABLE
#ifndef SUPPORT_SINGLETON_SCHEDULE
#define SUPPORT_SINGLETON_SCHEDULE
#endif
#endif



// If (potentially) needing to run in some sort of continuous RX mode, define a flag true (else false).
#if defined(ENABLE_BOILER_HUB) || defined(ALLOW_STATS_RX)
#define CONFIG_IMPLIES_MAY_NEED_CONTINUOUS_RX true
#else
#define CONFIG_IMPLIES_MAY_NEED_CONTINUOUS_RX false
#endif

// By default, use the RFM22/RFM23 module to talk to an FHT8V wireless radiator valve.
#ifdef USE_MODULE_FHT8VSIMPLE
#define USE_MODULE_RFM22RADIOSIMPLE
// If this can be a hub, enable extra RX code.
#ifdef ENABLE_BOILER_HUB
#define USE_MODULE_FHT8VSIMPLE_RX
#endif
#endif

//// Supporting library required for OneWire-connected sensors.
//// Will also need a #include in the .ino file because of Arduino's pre-preprocessing logic (IDE 1.0.x).
//#if defined(SENSOR_DS18B20_ENABLE)
//#define REQUIRES_ONEWIRE22_LIB // Requires V2.2 of OneWire lib.
//#include <OneWire.h>
//#endif


#endif


