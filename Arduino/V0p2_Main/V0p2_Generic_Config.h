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
                           Deniz Erbilgin 2015--2016
*/

/*
 Global configuration parameters for V0.2 PCB hardware.
 The top part should contain one #define CONFIG_XXX and nothing else (uncommented).
 Below a set of ENABLE_XXX flags should be defined or undefined as a result.
 */

#ifndef V0P2_GENERIC_CONFIG_H
#define V0P2_GENERIC_CONFIG_H


// ------------------------------------------------------
// PRE-DEFINED CONFIG_... BUNDLE IMPLEMENTATION/EXPANSION
// These features can be turned off if not required in particular implementations.
// These flag names are all of the form ENABLE_XXX.

// Get defaults for valve applications.
#include <OTV0p2_valve_ENABLE_defaults.h>


// Define/uncomment exactly one of the CONFIG_XXX flags below
// to enable a standard configuration set.
// Some can be specific to particular locations and boards,
// others can be vanilla ready to be configured by the end-user one way or another.
// As far as possible the configs should simply #define/#undef a set of ENABLE_XXX flags.
// There is a set of 'default' ENABLE_XXX flags which will evolve over time,
// requiring alterations to them to be tracked in individual configs.

//#define CONFIG_GENERIC_ROOM_NODE
//#define CONFIG_GENERIC_BOILER_NODE
//#define CONFIG_GENERIC_RANDB_NODE
//#define CONFIG_GENERIC_DHW_NODE


// Production configs.
//#define CONFIG_Trial2013Winter_Round1 // REV1 default config.
//#define CONFIG_Trial2013Winter_Round1_LVBHSH // REV1: local valve control, boiler hub, stats hub & TX.
//#define CONFIG_Trial2013Winter_Round1_BOILERHUB // REV1 as plain boiler node.
//#define CONFIG_Trial2013Winter_Round1_NOHUB // REV1 as TX-only leaf node.
//#define CONFIG_Trial2013Winter_Round2 // REV2 cut4 default config.
//#define CONFIG_Trial2013Winter_Round2_LVBHSH // REV2 cut4: local valve control, boiler hub, stats hub & TX.
//#define CONFIG_Trial2013Winter_Round2_LVBH // REV2 cut4 local valve control and boiler hub.
//#define CONFIG_Trial2013Winter_Round2_BOILERHUB // REV2 cut4 as plain boiler hub.
//#define CONFIG_Trial2013Winter_Round2_STATSHUB // REV2 cut4 as stats hub.
//#define CONFIG_Trial2013Winter_Round2_NOHUB // REV2 cut4 as TX-only leaf node.
#define CONFIG_DORM1 // REV7 / DORM1 / TRV1.x all-in-one valve unit, secure TX.
//#define CONFIG_DORM1_BOILER // REV8 / DORM1 boiler-control unit.
//#define CONFIG_REV8_SECURE_BHR // REV8 secure boiler controller and stats hub.
//#define CONFIG_REV10_AS_GSM_RELAY_ONLY // REV10: stats relay only.
//#define CONFIG_REV10_SECURE_BHR // REV10: secure stats relay and boiler hub.
//#define CONFIG_REV11_RAW_JSON // REV11 as raw JSON-only stats/sensor leaf.
//#define CONFIG_REV14_WORKSHOP // REV14 w/ light sensor, SHT21, for Launchpad workshop.


// One-offs and special cases.
//#define CONFIG_DHD_TESTLAB_REV0 // REV0 / breadboard.
//#define CONFIG_Trial2013Winter_Round1_STATSHUB // REV1 as stats hub.
//#define CONFIG_Trial2013Winter_Round2_BHR // REV2 cut4: boiler hub and stats relay.
//#define CONFIG_Trial2013Winter_Round1_SECURE_STATSHUB // REV1 as secure stats hub.
//#define CONFIG_Trial2013Winter_Round1_SECURE_SENSOR // REV1 as secure sensor node.
//#define CONFIG_Trial2013Winter_Round2_SECURE_NOHUB // REV2 cut4 leaf (valve/sensor) 2015/12 secure protocol.
//#define CONFIG_Trial2013Winter_Round2_SECURE_STATSHUB // REV2 cut4 hub (boiler/stats) 2015/12 secure protocol.
//#define CONFIG_DHD_TESTLAB_REV4 // REV4 cut2.
//#define CONFIG_DHD_TESTLAB_REV4_NOHUB // REV4 cut2, no hub.
//#define CONFIG_BH_DHW // Bo's hot water.
//#define CONFIG_BH_TESTLAB // Bo's test environment.
//#define CONFIG_DORM1_SECURE_AND_FS20 // REV7 / DORM1 / TRV1.x all-in-one valve unit, secure + FS20 non-secure comms.
//#define CONFIG_REV7N // REV7 with external "Model N" valve.
//#define CONFIG_REV7_AS_SENSOR // REV7 as JSON-only stats/sensor leaf.
//#define CONFIG_REV7_AS_SECURE_SENSOR // REV7 as JSON-only stats/sensor leaf with secure comms.
//#define CONFIG_REV9_STATS // REV9 as stats node, cut 2 of the board.
//#define CONFIG_DE_TESTLAB // Deniz's test environment.
//#define CONFIG_REV10_STRIPBOARD // REV10-based stripboard precursor for bus shelters.
//#define CONFIG_REV10 // Generic REV10 config.
//#define CONFIG_REV10_BHR // REV10: boiler hub and stats relay.
//#define CONFIG_REV10_SECURE_BOILERHUB_GSM_SECURE // REV10 PCB boiler hub, relay to GSM, 2015/12 secure protocol.
//#define CONFIG_REV10_SECURE_BHR_NULLRADIO// REV10: boiler hub and stats relay with AESGCM and a null secondary radio.
//#define CONFIG_REV11_RFM23BTEST // Basic test to see if stats send.
//#define CONFIG_REV11_SECURE_SENSOR
//#define CONFIG_REV11_SENSOR
//#define CONFIG_REV11_SECURE_STATSHUB
//#define CONFIG_REV11_STATSHUB
//#define CONFIG_REV14_PROTO  // Prototype REV14 w/ LoRa, TMP, SHT and QM-1.
//#define CONFIG_REV14 // REV14 w/ light sensor, SHT21 and voice sensor.
//#define CONFIG_BAREBONES // No peripherals / on breadboard.

// --------------------------------------------

// Breadboard/stripboard/minimal designs.
#include <OTV0p2_CONFIG_REV0.h>

// For trial over winter of 2013--4, first round (REV1).
#include <OTV0p2_CONFIG_REV1.h>

// For trial over winter of 2013--4, second round (REV2).
#include <OTV0p2_CONFIG_REV2.h>

//// All-in-one valve unit.
//#include <OTV0p2_CONFIG_REV4.h>

// All-in-one valve unit (DORM1).
#include <OTV0p2_CONFIG_REV7.h>

// Boiler controller for all-in-one valve unit (DORM1).
#include <OTV0p2_CONFIG_REV8.h>

//// COHEAT radio relay.
//#include <OTV0p2_CONFIG_REV9.h>

// REV8 + GSM Arduino shield + I2CEXT, see TODO-551.
#include <OTV0p2_CONFIG_REV10.h>

// REV4 (ie SHT21 sensor and phototransistor) + PCB antenna + PCB battery pack (probably AAA), see TODO-566
#include <OTV0p2_CONFIG_REV11.h>

// REV14 w/ LoRa, TMP, SHT and QM-1.
#include <OTV0p2_CONFIG_REV14.h>

// --------------------------------------------

// Fixups to apply after loading the target config.
#include <OTV0p2_valve_ENABLE_fixups.h>


// --------------------------------------------
// Trailing one-off local fixups.

// Define to force data out once-per-minute for debugging and algorithm development.
//#define ENABLE_FREQUENT_STATS_TX


#endif

