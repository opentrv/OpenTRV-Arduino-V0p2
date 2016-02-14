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

Author(s) / Copyright (s): Damon Hart-Davis 2013--2016
                           Deniz Erbilgin 2015--2016
*/

/*
 Global configuration parameters for V0.2 PCB hardware.
 The top part should contain one #define CONFIG_XXX and nothing else (uncommented).
 Below a set of ENABLE_XXX flags should be defined or undefined as a result.
 */

#ifndef V0P2_GENERIC_CONFIG_H
#define V0P2_GENERIC_CONFIG_H

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
#define CONFIG_Trial2013Winter_Round2_NOHUB // REV2 cut4 as TX-only leaf node.
//#define CONFIG_DORM1 // REV7 / DORM1 all-in-one valve unit.
//#define CONFIG_DORM1_BOILER // REV8 / DORM1 boiler-control unit.
//#define CONFIG_REV11_RAW_JSON // REV11 as raw JSON-only stats/sensor leaf.


// One-offs and special cases.
//#define CONFIG_DHD_TESTLAB_REV0 // REV0 / breadboard.
//#define CONFIG_DHD_TESTLAB_REV1 // REV1.
//#define CONFIG_Trial2013Winter_Round1_STATSHUB // REV1 as stats hub.
//#define CONFIG_Trial2013Winter_Round2_CC1HUB // REV2 cut4 as CC1 hub.
//#define CONFIG_Trial2013Winter_Round2_BHR // REV2 cut4: boiler hub and stats relay.
//#define CONFIG_Trial2013Winter_Round2_SECURE_NOHUB // REV2 cut4 leaf (valve/sensor) 2015/12 secure protocol.
//#define CONFIG_Trial2013Winter_Round2_SECURE_HUB // REV2 cut4 hub (boiler/stats) 2015/12 secure protocol.
//#define CONFIG_DHD_TESTLAB_REV4 // REV4 cut2.
//#define CONFIG_DHD_TESTLAB_REV4_NOHUB // REV4 cut2, no hub.
//#define CONFIG_BH_DHW // Bo's hot water.
//#define CONFIG_BH_TESTLAB // Bo's test environment.
//#define CONFIG_DORM1_SANS32K // REV7 / DORM1 without working 32768Hz clock.
//#define CONFIG_DORM1_MUT // REV7 / DORM1 Winter 2014/2015 minimal for unit testing.
//#define CONFIG_REV7N // REV7 with external "Model N" valve.
//#define CONFIG_REV7_AS_SENSOR // REV7 as JSON-only stats/sensor leaf.
//#define CONFIG_REV9 // REV9 as CC1 relay, cut 2 of the board.
//#define CONFIG_REV9_STATS // REV9 as stats node, cut 2 of the board.
//#define CONFIG_REV9_cut1 // REV9 as CC1 relay, cut1 of board.
//#define CONFIG_DE_TESTLAB // Deniz's test environment.
//#define CONFIG_REV10_STRIPBOARD // REV10-based stripboard precursor for bus shelters
//#define CONFIG_REV10 // Generic REV10 config
//#define CONFIG_REV10_ASRELAY // REV10: stats relay only.
//#define CONFIG_REV10_BHR // REV10: boiler hub and stats relay.
// TODO //#define CONFIG_REV10_SECURE_BOILERHUB_GSM_SECURE // REV10 PCB boiler hub, relay to GSM, 2015/12 secure protocol.
//#define CONFIG_REV11_RFM23BTEST // Basic test to see if stats send
//#define CONFIG_REV14_PROTO  // Prototype REV14 w/ LoRa, TMP, SHT and QM-1
//#define CONFIG_BAREBONES // No peripherals / on breadboard.








// ------------------------------------------------------
// PRE-DEFINED CONFIG_... BUNDLE IMPLEMENTATION/EXPANSION
// These features can be turned off if not required in particular implementations.
// These flag names are all of the form ENABLE_XXX.

// Defaults for V0.2; should be '#undef'ined if not required.
//
// Use sleep wakeup (2Hz by default) from external 32768Hz xtal and timer 2.
#define ENABLE_WAKEUP_32768HZ_XTAL
// IF DEFINED: this unit may run on 2xAA cells, preferably rechargeable eg NiMH, ~2V--2.4V, and should monitor supply voltage.
#define ENABLE_SUPPLY_VOLTAGE_LOW_2AA // May require limiting clock speed and using some alternative peripherals/sensors...
// IF DEFINED: enable use AVR's 'idle' mode to stop the CPU but leave I/O clocls (eg Serial) running to save power.
// DHD20150920: NOT RECOMMENDED AS SEEMS TO CAUSE SOME BOARDS (REV1,REV9) TO CRASH.
#undef ENABLE_USE_OF_AVR_IDLE_MODE
// Provide software RTC support by default.
#define ENABLE_RTC_INTERNAL_SIMPLE
// IF DEFINED: try to trim memory (primarily RAM, also code/Flash) space used.
#undef ENABLE_TRIMMED_MEMORY
// IF DEFINED: try to trim bandwidth as may be especially expensive/scarce.
#undef ENABLE_TRIMMED_BANDWIDTH
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.
#undef ENABLE_MIN_ENERGY_BOOT
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#define ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: this unit *can* act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
//////////////////////////////////////// DEV/MAINT UI OPTIONS (and support for them)
// IF DEFINED: allow JSON stats frames alongside binary ones.
#define ENABLE_JSON_OUTPUT
// IF DEFINED: allow periodic machine- and human- readable status report to serial, starting with "=".
#define ENABLE_SERIAL_STATUS_REPORT
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: there is run-time help available for the CLI.
#define ENABLE_CLI_HELP
// IF DEFINED: enable a full OpenTRV CLI.
#define ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc.
#define ENABLE_FULL_OT_UI
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#undef ENABLE_EXTENDED_CLI
// IF DEFINED: physical UI use wakes CLI (not needed when CLI can auto-wake from serial).
#undef ENABLE_UI_WAKES_CLI
//////////////////////////////////////// DEVICE UI OPTIONS (and support for them)
// IF DEFINED: basic FROST/WARM temperatures are settable.
#define ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: fast temp pot/dial sampling to partly compensate for less good mechanics (at some energy cost).
#undef ENABLE_FAST_TEMP_POT_SAMPLING
// IF DEFINED: support one on and one off time per day (possibly in conjunction with 'learn' button).
#define ENABLE_SINGLETON_SCHEDULE
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.
#define ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1
// IF DEFINED: enable use of second UI LED if available.
#define ENABLE_UI_LED_2_IF_AVAILABLE
// IF DEFINED: simplified mode button behaviour: tapping button invokes BAKE, not mode cycling.
#undef ENABLE_SIMPLIFIED_MODE_BAKE 
//////////////////////////////////////// SENSOR OPTIONS (and support for them)
// IF DEFINED: allow use of ambient light sensor.
#define ENABLE_AMBLIGHT_SENSOR
// IF DEFINED: allow for less light on sideways-pointing ambient light sensor, eg on cut4 2014/03/17 REV2 boards (TODO-209).
#undef ENABLE_AMBLIGHT_EXTRA_SENSITIVE
// IF DEFINED: use the temperature-setting potentiometer/dial if present.
#define ENABLE_TEMP_POT_IF_PRESENT
// Enable use of OneWire devices.
#undef ENABLE_MINIMAL_ONEWIRE_SUPPORT
// IF DEFINED: enable use of on-board SHT21 primary temperature and RH% sensor (in lieu of default TMP112).
#undef ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Enable use of DS18B20 as primary temp sensor.
#undef ENABLE_PRIMARY_TEMP_SENSOR_DS18B20
// IF DEFINED: enable use of additional (ie external) DS18B20 temp sensor(s).
#undef ENABLE_EXTERNAL_TEMP_SENSOR_DS18B20
//////////////////////////////////////// OCCUPANCY OPTIONS
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#define ENABLE_OCCUPANCY_SUPPORT
// IF DEFINED: detect occupancy based on ambient light, if available.
#define ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT
// IF DEFINED: detect occupancy based on relative humidity, if available.
#define ENABLE_OCCUPANCY_DETECTION_FROM_RH
// IF DEFINED: detect occupancy based on voice detection, if available. This undefines learn button 2 to use GPIO as input.
#undef ENABLE_OCCUPANCY_DETECTION_FROM_VOICE
//////////////////////////////////////// RADIO OPTIONS
// IF DEFINED: enable (at least) a primary radio module; without, this unit has no radio comms.
#define ENABLE_RADIO_PRIMARY_MODULE
// IF DEFINED: enable a 'null' radio module; can be used to simplify code for a radio-less unit.
#undef ENABLE_RADIO_NULL
// IF DEFINED: had RFM23B as the primary radio module: default from REV1 to REV11.
#define ENABLE_RADIO_RFM23B
// IF DEFINED: make RFM23B the primary radio.
#define ENABLE_RADIO_PRIMARY_RFM23B
// IF DEFINED: enable a secondary (typically WAN-relay) radio module.
#undef ENABLE_RADIO_SECONDARY_MODULE
// IF DEFINED: enable a WAN-relay radio module, primarily to relay stats outbound.
#undef ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
// IF DEFINED: enable periodic secure beacon broadcast.
#undef ENABLE_SECURE_RADIO_BEACON
// IF DEFINED: enable support for fast (>50kbps) packet-handling carrier (leading length byte).
#undef ENABLE_FAST_FRAMED_CARRIER_SUPPORT
// IF DEFINED: enable support for FS20 carrier for RX or TX.
#define ENABLE_FS20_CARRIER_SUPPORT
// IF DEFINED: use FHT8V wireless radio module/valve, eg to control FHT8V local valve.
#define ENABLE_FHT8VSIMPLE
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#define ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// IF DEFINED: enable support for FS20 encoding/decoding, eg to send to FHT8V.
#define ENABLE_FS20_ENCODING_SUPPORT
// IF DEFINED: enable OpenTRV secure frame encoding/decoding (as of 2015/12).
#undef ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
// IF DEFINED: allow non-secure OpenTRV secure frame RX (as of 2015/12): DISABLED BY DEFAULT.
#undef ENABLE_OTSECUREFRAME_INSECURE_RX_PERMITTED
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: always allow some kind of stats TX, whatever the privacy settings.
// HAS HUGE PRIVACY IMPLICATIONS: DO NOT ENABLE UNNECESSARILY!
#undef ENABLE_ALWAYS_TX_ALL_STATS
// IF DEFINED: allow minimal binary format in addition to more generic one: ~400 bytes code cost.
#undef ENABLE_MINIMAL_STATS_TXRX
// IF DEFINED: allow binary stats to be TXed.
#define ENABLE_BINARY_STATS_TX
// IF DEFINED: allow radio listen/RX.
#define ENABLE_RADIO_RX
// IF DEFINED: forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#undef ENABLE_DEFAULT_ALWAYS_RX








// ------------------------- REV1

#ifdef CONFIG_Trial2013Winter_Round1_LVBHSH // REV1: local valve control, boiler hub, stats hub & TX.
#define CONFIG_Trial2013Winter_Round1 // Just like normal REV1 except...
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#define ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.
#define ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#define ENABLE_OCCUPANCY_SUPPORT
#endif

#ifdef CONFIG_Trial2013Winter_Round1_BOILERHUB // REV1 as plain boiler hub + can TX stats.
#define CONFIG_Trial2013Winter_Round1 // Just like normal REV1 except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
#endif

#ifdef CONFIG_Trial2013Winter_Round1_STATSHUB // REV1 as stats hub.
#define CONFIG_Trial2013Winter_Round1 // Just like normal REV1 except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#undef ENABLE_STATS_TX // Don't allow it to TX its own...
#endif

#ifdef CONFIG_Trial2013Winter_Round1_NOHUB // REV1 as TX-only leaf node.
#define CONFIG_Trial2013Winter_Round1 // Just like normal REV2 except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
#endif

#ifdef CONFIG_Trial2013Winter_Round1 // For trial over winter of 2013--4, first round (REV1).
// Revision REV1 of V0.2 board.
#define V0p2_REV 1
// TODO-264: Find out why IDLE seems to crash some REV1 boards.
#undef ENABLE_USE_OF_AVR_IDLE_MODE
// Use common settings.
#define COMMON_SETTINGS
#endif


// ------------------------- REV2

#ifdef CONFIG_Trial2013Winter_Round2_LVBHSH // REV2 cut4: local valve control, boiler hub, stats hub & TX.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV2 except...
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#define ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.
#define ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#define ENABLE_OCCUPANCY_SUPPORT
#endif

#ifdef CONFIG_Trial2013Winter_Round2_LVBH // REV2 cut4: local valve control, boiler hub & TX.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV2 except...
// IF DEFINED: try to trim memory (primarily RAM, also code/Flash) space used.
#define ENABLE_TRIMMED_MEMORY
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#define ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.
#define ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#define ENABLE_OCCUPANCY_SUPPORT
#endif

#ifdef CONFIG_Trial2013Winter_Round2_BOILERHUB // For trial over winter of 2013--4, second round (REV2), as pure boiler hub + can TX stats.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV2 except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
#endif

#ifdef CONFIG_Trial2013Winter_Round2_STATSHUB // For trial over winter of 2013--4, second round (REV2), as stats hub.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV2 except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#undef ENABLE_STATS_TX // Don't allow it to TX its own...
#endif

#ifdef CONFIG_Trial2013Winter_Round2_NOHUB // REV2 cut4 as TX-only leaf node.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV2 except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
#endif


#ifdef CONFIG_Trial2013Winter_Round2_CC1HUB // REV2 cut4 as CC1 hub.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV2 except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX // Not needed for CC1 frames.
// IF DEFINED: allow TX of stats frames.
#undef ENABLE_STATS_TX
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#undef ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// IF DEFINED: enable support for FS20 encoding/decoding, eg to send to FHT8V.
#undef ENABLE_FS20_ENCODING_SUPPORT
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#define ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV // THESE HUB UNITS DO NOT manage a local TRV.
// IF DEFINED: allow JSON stats frames alongside binary ones.
#undef ENABLE_JSON_OUTPUT
// IF DEFINED: enable a full OpenTRV CLI.
#undef ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc.
#undef ENABLE_FULL_OT_UI
// IF DEFINED: basic FROST/WARM temperatures are settable.
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#define ENABLE_EXTENDED_CLI
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#undef ENABLE_OCCUPANCY_SUPPORT // None of that logic required at hub.
// IF DEFINED: act as CC1 simple hub node.
#define ALLOW_CC1_SUPPORT
#define ALLOW_CC1_SUPPORT_HUB
#endif

#ifdef CONFIG_Trial2013Winter_Round2_BHR // REV2 cut4: boiler hub and stats relay.
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV2 except...
// IF DEFINED: basic FROST/WARM temperatures are settable.
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#define ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#undef ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.
#undef ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#undef ENABLE_OCCUPANCY_SUPPORT
// IF DEFINED: enable a secondary (typically WAN-relay) radio module.
#define ENABLE_RADIO_SECONDARY_MODULE
// IF DEFINED: enable a WAN-relay radio module, primarily to relay stats outbound.
#define ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
// Chose NullRadio as secondary.
#define RADIO_SECONDARY_NULL
// IF DEFINED: allow periodic machine- and human- readable status report to serial, starting with "="/
//#undef ENABLE_SERIAL_STATUS_REPORT
// Use common settings.
#define COMMON_SETTINGS
#endif

#ifdef CONFIG_Trial2013Winter_Round2_SECURE_NOHUB
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV2 except...
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: allow JSON stats frames alongside binary ones.
#define ENABLE_JSON_OUTPUT
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: enable support for FS20 carrier for RX or TX.
#undef ENABLE_FS20_CARRIER_SUPPORT
// IF DEFINED: use FHT8V wireless radio module/valve.
#undef ENABLE_FHT8VSIMPLE
// IF DEFINED: enable support for fast (>50kbps) packet-handling carrier (leading length byte).
#define ENABLE_FAST_FRAMED_CARRIER_SUPPORT
// IF DEFINED: enable support for FS20 carrier for TX specifically (to allow RX-only).
#undef ENABLE_FS20_CARRIER_SUPPORT_TX
// IF DEFINED: enable support for FS20 encoding/decoding, eg to send to FHT8V.
#undef ENABLE_FS20_ENCODING_SUPPORT
// IF DEFINED: enable OpenTRV secure frame encoding/decoding (as of 2015/12).
#define ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
// IF DEFINED: allow non-secure OpenTRV secure frame RX (as of 2015/12): DISABLED BY DEFAULT.
#undef ENABLE_OTSECUREFRAME_INSECURE_RX_PERMITTED
#endif


#ifdef CONFIG_Trial2013Winter_Round2_SECURE_HUB
#define CONFIG_Trial2013Winter_Round2 // Just like normal REV2 except...
// IF DEFINED: there is run-time help available for the CLI.
#undef ENABLE_CLI_HELP
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#define ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: allow JSON stats frames alongside binary ones.
#define ENABLE_JSON_OUTPUT
// IF DEFINED: enable periodic secure beacon broadcast.
#undef ENABLE_SECURE_RADIO_BEACON
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: enable support for fast (>50kbps) packet-handling carrier (leading length byte).
#define ENABLE_FAST_FRAMED_CARRIER_SUPPORT
// IF DEFINED: enable support for FS20 carrier for RX or TX.
#undef ENABLE_FS20_CARRIER_SUPPORT
// IF DEFINED: use FHT8V wireless radio module/valve.
#undef ENABLE_FHT8VSIMPLE
// IF DEFINED: enable support for FS20 carrier for TX specifically (to allow RX-only).
#undef ENABLE_FS20_CARRIER_SUPPORT_TX
// IF DEFINED: enable support for FS20 encoding/decoding, eg to send to FHT8V.
#undef ENABLE_FS20_ENCODING_SUPPORT
// IF DEFINED: enable OpenTRV secure frame encoding/decoding (as of 2015/12).
#define ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
// IF DEFINED: allow non-secure OpenTRV secure frame RX (as of 2015/12): DISABLED BY DEFAULT.
#undef ENABLE_OTSECUREFRAME_INSECURE_RX_PERMITTED
#endif

#ifdef CONFIG_Trial2013Winter_Round2 // For trial over winter of 2013--4, second round (REV2).
// Revision REV2 (cut4+) of V0.2 board.
#define V0p2_REV 2
// IF DEFINED: allow for less light on sideways-pointing LDR on cut4 2014/03/17 REV2 boards (TODO-209).
#define ENABLE_AMBLIGHT_EXTRA_SENSITIVE
// Use common settings.
#define COMMON_SETTINGS
#endif

// ------------------------- REV7

#ifdef CONFIG_DORM1 // All-in-one (REV7).
// Revision REV7 of V0.2 board, all-in-one valve unit with local motor drive.
// Does not ever need to act as a boiler hub nor to receive stats.
// Although LEARN buttons are provided, by default they are disabled as is the scheduler.
#define V0p2_REV 7
// IF DEFINED: simplified mode button behaviour: tapping button invokes BAKE, not mode cycling.
#define ENABLE_SIMPLIFIED_MODE_BAKE 
// IF DEFINED: fast temp pot/dial sampling to partly compensate for less good mechanics (at some energy cost).
#define ENABLE_FAST_TEMP_POT_SAMPLING
// IF DEFINED: support one on and one off time per day (possibly in conjunction with 'learn' button).
#undef ENABLE_SINGLETON_SCHEDULE
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.
#undef ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1
// IF DEFINED: try to trim memory (primarily RAM, also code/Flash) space used.
#define ENABLE_TRIMMED_MEMORY
// IF DEFINED: initial direct motor drive design.
#define ENABLE_V1_DIRECT_MOTOR_DRIVE
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF DEFINED: detect occupancy based on relative humidity, if available.
// DHD20160101: seems to still be set off spuriously by fast drop in temp when rad turns off (TODO-696).
#undef ENABLE_OCCUPANCY_DETECTION_FROM_RH
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: do not allow TX of stats frames.
#define ENABLE_STATS_TX
// IF UNDEFINED: do not allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow radio listen/RX.
#undef ENABLE_RADIO_RX
// IF DEFINED: allow JSON stats frames.
#define ENABLE_JSON_OUTPUT
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#define ENABLE_LOCAL_TRV
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: there is run-time help available for the CLI.
#undef ENABLE_CLI_HELP
// IF DEFINED: enable a full OpenTRV CLI.
#define ENABLE_FULL_OT_CLI
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#undef ENABLE_EXTENDED_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc.
#define ENABLE_FULL_OT_UI
// IF DEFINED: enable use of second UI LED if available.
#undef ENABLE_UI_LED_2_IF_AVAILABLE
// IF DEFINED: reverse DORM1 motor with respect to very first samples.
#define ENABLE_DORM1_MOTOR_REVERSED
// Use common settings.
#define COMMON_SETTINGS
#endif




// -------------------------
// NON-STANDARD DEFINITIONS
// -------------------------

// ------------------------- REV0 / breadboard

#ifdef CONFIG_DHD_TESTLAB_REV0 // DHD's test lab breadboard with TRV.
// Revision of V0.2 board.
#define V0p2_REV 0 // REV0 covers DHD's breadboard (was first V0.2 PCB).
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.
#define ENABLE_MIN_ENERGY_BOOT
//// Enable use of DS18B20 temp sensor.
//#define ENABLE_PRIMARY_TEMP_SENSOR_DS18B20
//// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
//#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
//// Enable experimental voice detection.
//#define ENABLE_VOICE_SENSOR
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: don't allow RX of stats frames (since there is no easy way to plug in a serial connection to relay them!)
#undef ENABLE_STATS_RX
// IF DEFINED: initial direct motor drive design.
//#define ENABLE_V1_DIRECT_MOTOR_DRIVE
// Use common settings.
#define COMMON_SETTINGS
#endif

#ifdef CONFIG_BAREBONES
// use alternative loop
#define ALT_MAIN_LOOP
#define V0p2_REV 0
// Defaults for V0.2; have to be undefined if not required.  ***
// May require limiting clock speed and using some alternative peripherals/sensors.
#define ENABLE_SUPPLY_VOLTAGE_LOW_2AA
// Provide software RTC support by default.
#define ENABLE_RTC_INTERNAL_SIMPLE
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit *can* act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.  ***
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#undef ENABLE_STATS_TX
// IF DEFINED: allow minimal binary format in addition to more generic one: ~400 bytes code cost.
#undef ENABLE_MINIMAL_STATS_TXRX
// IF DEFINED: allow JSON stats frames alongside binary ones.
#undef ENABLE_JSON_OUTPUT
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable. ***
#undef ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: enable a full OpenTRV CLI.
#define ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc. ***
//#define ENABLE_FULL_OT_UI
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#undef ENABLE_EXTENDED_CLI
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.  ***
#undef ENABLE_MIN_ENERGY_BOOT
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).   ***
#undef ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// IF DEFINED: enable use AVR's 'idle' mode to stop the CPU but leave I/O (eg Serial) running to save power.
// DHD20150920: CURRENTLY NOT RECOMMENDED AS STILL SEEMS TO CAUSE SOME BOARDS TO CRASH.
#define ENABLE_USE_OF_AVR_IDLE_MODE
// IF DEFINED: Use OTNullRadioLink instead of a radio module
// Undefine other radio //FIXME make this a part of the automatic stuff
#define USE_NULLRADIO
//#define USE_MODULE_SIM900
// things that break
// IF DEFINED: basic FROST/WARM temperatures are settable.
//#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.  ***
//#undef ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1  UI_Minimal.cpp:1180:32: error: 'handleLEARN' was not declared in this scope
#define ENABLE_FHT8VSIMPLE //Control.cpp:1322:27: error: 'localFHT8VTRVEnabled' was not declared in this scope
// If LDR is not to be used then specifically define OMIT_... as below.
//#undef ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT //  LDR 'occupancy' sensing irrelevant for DHW. Messaging.cpp:232:87: error: 'class AmbientLight' has no member named 'getRaw
#endif


// -------------------------
#ifdef CONFIG_DE_TESTLAB
// use alternative loop
#define ALT_MAIN_LOOP
#define V0p2_REV 0
// Defaults for V0.2; have to be undefined if not required.  ***
// May require limiting clock speed and using some alternative peripherals/sensors.
#define ENABLE_SUPPLY_VOLTAGE_LOW_2AA
// Provide software RTC support by default.
#define ENABLE_RTC_INTERNAL_SIMPLE
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit *can* act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.  ***
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: allow minimal binary format in addition to more generic one: ~400 bytes code cost.
#undef ENABLE_MINIMAL_STATS_TXRX
// IF DEFINED: allow JSON stats frames alongside binary ones.
//#undef ENABLE_JSON_OUTPUT
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable. ***
#undef ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: enable a full OpenTRV CLI.
#define ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc. ***
//#define ENABLE_FULL_OT_UI
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#undef ENABLE_EXTENDED_CLI
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.  ***
#undef ENABLE_MIN_ENERGY_BOOT
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).   ***
#undef ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// IF DEFINED: enable use AVR's 'idle' mode to stop the CPU but leave I/O (eg Serial) running to save power.
// DHD20150920: CURRENTLY NOT RECOMMENDED AS STILL SEEMS TO CAUSE SOME BOARDS TO CRASH.
#define ENABLE_USE_OF_AVR_IDLE_MODE
// IF DEFINED: Use OTNullRadioLink instead of a radio module
// Undefine other radio //FIXME make this a part of the automatic stuff
//#define USE_NULLRADIO
#define USE_MODULE_SIM900
// Define voice module
#define ENABLE_VOICE_SENSOR
// Enable use of OneWire devices.
#define ENABLE_MINIMAL_ONEWIRE_SUPPORT
// Enable use of DS18B20 temp sensor.
#define ENABLE_PRIMARY_TEMP_SENSOR_DS18B20

// things that break
// IF DEFINED: basic FROST/WARM temperatures are settable.
//#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.  ***
//#undef ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1  UI_Minimal.cpp:1180:32: error: 'handleLEARN' was not declared in this scope
#define ENABLE_FHT8VSIMPLE //Control.cpp:1322:27: error: 'localFHT8VTRVEnabled' was not declared in this scope
// If LDR is not to be used then specifically define OMIT_... as below.
//#undef ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT //  LDR 'occupancy' sensing irrelevant for DHW. Messaging.cpp:232:87: error: 'class AmbientLight' has no member named 'getRaw

#endif // CONFIG_DE_TESTLAB


// ------------------------- REV1

#ifdef CONFIG_DHD_TESTLAB_REV1 // DHD's test lab on REV1.
// Revision of V0.2 board.
#define V0p2_REV 1
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.
#define ENABLE_MIN_ENERGY_BOOT
//// Enable use of DS18B20 temp sensor.
//#define ENABLE_PRIMARY_TEMP_SENSOR_DS18B20
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
//// Enable experimental voice detection.
//#define ENABLE_VOICE_SENSOR
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: don't allow RX of stats frames (since there is no easy way to plug in a serial connection to relay them!)
#undef ENABLE_STATS_RX
// IF DEFINED: initial direct motor drive design.
//#define ENABLE_V1_DIRECT_MOTOR_DRIVE
// Use common settings.
#define COMMON_SETTINGS
#endif

// ------------------------- REV4

#ifdef CONFIG_DHD_TESTLAB_REV4_NOHUB // REV4 board but no listening...
#define CONFIG_DHD_TESTLAB_REV4
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
#endif

#ifdef CONFIG_DHD_TESTLAB_REV4 // DHD's test lab with TRV on REV4 (cut2) board.
// Revision of V0.2 board.
#define V0p2_REV 4 // REV0 covers DHD's breadboard and first V0.2 PCB.
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
//#undef ENABLE_BOILER_HUB
// Use common settings.
#define COMMON_SETTINGS
#endif


// ------------------------- REV7

#ifdef CONFIG_REV7_AS_SENSOR // REV7 as JSON-only stats/sensor leaf.
// Revision REV7 of V0.2 board, all-in-one valve unit with local motor drive.
// In this off-label mode being used as stats gatherers or simple hubs.
#define V0p2_REV 7
// IF DEFINED: initial direct motor drive design.  Doesn't imply it gets used, but I/O can be set up safely.
#define ENABLE_V1_DIRECT_MOTOR_DRIVE
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF DEFINED: use the temperature-setting potentiometer/dial if present.
#undef ENABLE_TEMP_POT_IF_PRESENT
// IF DEFINED: basic FROST/WARM temperatures are settable.
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB // NO BOILER CODE
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: always allow some kind of stats TX, whatever the privacy settings.
#define ENABLE_ALWAYS_TX_ALL_STATS
// IF DEFINED: allow JSON stats frames.
#define ENABLE_JSON_OUTPUT
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#undef ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// Use common settings.
#define COMMON_SETTINGS
#endif


//#ifdef CONFIG_DORM1_SANS32K // REV7 / DORM1 without working 32768Hz clock.
//#define CONFIG_DORM1
//#undef ENABLE_WAKEUP_32768HZ_XTAL
//#endif

#ifdef CONFIG_DORM1_MUT // REV7 / DORM1 Winter 2014/2015 minimal for unit testing.
// Revision REV7 of V0.2 board, all-in-one valve unit with local motor drive.
// Does not ever need to act as a boiler hub nor to receive stats.
#define V0p2_REV 7
// IF DEFINED: initial direct motor drive design.
#undef ENABLE_V1_DIRECT_MOTOR_DRIVE
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: do not allow TX of stats frames.
#undef ENABLE_STATS_TX
// IF UNDEFINED: do not allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow JSON stats frames.
#undef ENABLE_JSON_OUTPUT
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#undef ENABLE_CLI
// IF DEFINED: enable a full OpenTRV CLI.
#undef ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc.
#undef ENABLE_FULL_OT_UI
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#undef ENABLE_EXTENDED_CLI
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#define ENABLE_LOCAL_TRV
// Use common settings.
#define COMMON_SETTINGS
#endif

// ------------------------- REV8

#ifdef CONFIG_DORM1_BOILER // REV8 boiler-control counterpart to REV7.
// Revision REV8.B of V0.2 board, boiler control unit.
// NO LIGHT SENSOR FITTED ON REV8.B BOARDS.
// BOTH TMP112 AND SHT21 FITTED on REV8.B BOARDS.
#define V0p2_REV 8
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
//#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// Use common settings.
#define COMMON_SETTINGS
#endif

// ------------------------- REV9

#ifdef CONFIG_REV9_cut1
#define V0p2_REV 9 // Just like cut2 but with some bugs...
// For 1st-cut REV9 boards phototransistor was accidentally pulling down not up.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400_WRONG_WAY
#endif

#ifdef CONFIG_REV9 // REV9 cut2, derived from REV4.
// Revision of V0.2 board.
#define V0p2_REV 9
// Enable use of OneWire devices.
#define ENABLE_MINIMAL_ONEWIRE_SUPPORT
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// IF DEFINED: enable use of additional (eg external) DS18B20 temp sensor(s).
#define ENABLE_EXTERNAL_TEMP_SENSOR_DS18B20
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#undef ENABLE_STATS_TX
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#define ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: allow JSON stats frames alongside binary ones.
#undef ENABLE_JSON_OUTPUT
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#define ENABLE_SLAVE_TRV
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#define ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: enable a full OpenTRV CLI.
#undef ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc.
#undef ENABLE_FULL_OT_UI
// IF DEFINED: basic FROST/WARM temperatures are settable.
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#define ENABLE_EXTENDED_CLI
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#undef ENABLE_OCCUPANCY_SUPPORT // No direct occupancy tracking at relay unit itself.
// IF UNDEFINED: no LEARN mode for REV9 boards (window sensor(s) instead).
//#undef ENABLE_LEARN_BUTTON
// IF DEFINED: act as CC1 simple relay node.
#define ALLOW_CC1_SUPPORT
#define ALLOW_CC1_SUPPORT_RELAY
#define ALLOW_CC1_SUPPORT_RELAY_IO // Direct addressing of LEDs, use of buttons, etc.
// Use common settings.
#define COMMON_SETTINGS
#endif

#ifdef CONFIG_REV9_STATS // REV9 cut2, derived from REV4, as stats node, for testing.
#define V0p2_REV 9
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: allow JSON stats frames alongside binary ones.
#define ENABLE_JSON_OUTPUT
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
//#undef ENABLE_BOILER_HUB
// Use common settings.
#define COMMON_SETTINGS
#endif

// ------------------------- REV10

// REV8 + GSM Arduino shield + I2CEXT, see TODO-551

#ifdef CONFIG_REV10 // REV10 base config
#define V0p2_REV 10
#define COMMON_SETTINGS
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit *can* act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.  ***
#define ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: allow minimal binary format in addition to more generic one: ~400 bytes code cost.
#undef ENABLE_MINIMAL_STATS_TXRX
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: enable a full OpenTRV CLI.
#undef ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc. ***
#undef ENABLE_FULL_OT_UI
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#undef ENABLE_EXTENDED_CLI
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.  ***
#undef ENABLE_MIN_ENERGY_BOOT
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).   ***
#undef ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// IF DEFINED: enable use AVR's 'idle' mode to stop the CPU but leave I/O (eg Serial) running to save power.
// DHD20150920: CURRENTLY NOT RECOMMENDED AS STILL SEEMS TO CAUSE SOME BOARDS TO CRASH.
#define ENABLE_USE_OF_AVR_IDLE_MODE
// Secondary radio
// IF DEFINED: enable a secondary (typically WAN-relay) radio module.
#define ENABLE_RADIO_SECONDARY_MODULE
#define ENABLE_RADIO_SIM900   // Enable SIM900
#define RADIO_SECONDARY_SIM900  // Assign SIM900
#endif // CONFIG_REV10

#ifdef CONFIG_REV10_STRIPBOARD // REV10-based stripboard precursor for bus shelters
#define V0p2_REV 10
#define COMMON_SETTINGS
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit *can* act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.  ***
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable. ***
#undef ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: allow minimal binary format in addition to more generic one: ~400 bytes code cost.
#undef ENABLE_MINIMAL_STATS_TXRX
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: enable a full OpenTRV CLI.
#define ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc. ***
#undef ENABLE_FULL_OT_UI
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#undef ENABLE_EXTENDED_CLI
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.  ***
#undef ENABLE_MIN_ENERGY_BOOT
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).   ***
#undef ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// IF DEFINED: enable use AVR's 'idle' mode to stop the CPU but leave I/O (eg Serial) running to save power.
// DHD20150920: CURRENTLY NOT RECOMMENDED AS STILL SEEMS TO CAUSE SOME BOARDS TO CRASH.
#define ENABLE_USE_OF_AVR_IDLE_MODE
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#define ENABLE_OCCUPANCY_SUPPORT
// IF DEFINED: enable a 'null' radio module; without this unit is stand-alone.
#define ENABLE_RADIO_NULL
// Secondary radio
#undef ENABLE_RADIO_RFM23B
#undef ENABLE_RADIO_PRIMARY_RFM23B
// IF DEFINED: enable a secondary (typically WAN-relay) radio module.
#define ENABLE_RADIO_SECONDARY_MODULE
#define ENABLE_RADIO_SIM900   // Enable SIM900
//#define RADIO_PRIMARY_SIM900  // Assign SIM900
#define RADIO_SECONDARY_SIM900  // Assign SIM900
// Enable use of OneWire devices.
#define ENABLE_MINIMAL_ONEWIRE_SUPPORT
// Enable use of DS18B20 temp sensor.
#define ENABLE_PRIMARY_TEMP_SENSOR_DS18B20
// Define voice module
#define ENABLE_VOICE_SENSOR
#define ENABLE_OCCUPANCY_DETECTION_FROM_VOICE
#define ENABLE_VOICE_STATS
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.  ***
#undef ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1  UI_Minimal.cpp:1180:32: error: 'handleLEARN' was not declared in this scope
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
#endif // CONFIG_REV10_BUSSHELTER


#ifdef CONFIG_REV10_ASRELAY // REV10: stats relay.
#define V0p2_REV 10
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF DEFINED: basic FROST/WARM temperatures are settable.
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#define ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#undef ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.
#undef ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#undef ENABLE_OCCUPANCY_SUPPORT
// IF DEFINED: enable a secondary (typically WAN-relay) radio module.
#define ENABLE_RADIO_SECONDARY_MODULE
// IF DEFINED: enable a WAN-relay radio module, primarily to relay stats outbound.
#define ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
// SIM900 relay.
#define ENABLE_RADIO_SIM900   // Enable SIM900
#define RADIO_SECONDARY_SIM900  // Assign SIM900
// Use common settings.
#define COMMON_SETTINGS
#endif // REV10_ASRELAY


#ifdef CONFIG_REV10_BHR // REV10: boiler hub and stats relay.
#define V0p2_REV 10
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF DEFINED: basic FROST/WARM temperatures are settable.
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#define ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#undef ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.
#undef ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#undef ENABLE_OCCUPANCY_SUPPORT
// IF DEFINED: enable a secondary (typically WAN-relay) radio module.
#define ENABLE_RADIO_SECONDARY_MODULE
// IF DEFINED: enable a WAN-relay radio module, primarily to relay stats outbound.
#define ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
// SIM900 relay.
#define ENABLE_RADIO_SIM900   // Enable SIM900
#define RADIO_SECONDARY_SIM900  // Assign SIM900
// Use common settings.
#define COMMON_SETTINGS
#endif



// ------------------------- REV11

// REV4 (ie SHT21 sensor and phototransistor) + PCB antenna + PCB battery back (probably AAA), see TODO-566
#ifdef CONFIG_REV11_RFM23BTEST
// Revision of V0.2 board.
#define V0p2_REV 11 // REV11 covers first sensor only board.
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// Anticipation logic not yet ready for prime-time.
//#define ENABLE_ANTICIPATION
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
//#undef ENABLE_BOILER_HUB
// Use common settings.
#define COMMON_SETTINGS
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
#undef ENABLE_LOCAL_TRV
#endif // CONFIG_REV11_RFM23BTEST

#ifdef CONFIG_REV11_RAW_JSON // REV11 as raw JSON-only stats/sensor leaf.
// Revision of V0.2 board.
#define V0p2_REV 11 // REV11 covers first sensor only board.
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// Using RoHS-compliant phototransistor in place of LDR.
#define AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF DEFINED: basic FROST/WARM temperatures are settable.
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: this unit can act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB // NO BOILER CODE
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: allow JSON stats frames.
#define ENABLE_JSON_OUTPUT
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#undef ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------

#ifdef CONFIG_REV14_PROTO // Prototype REV14 w/ LoRa, TMP, SHT and QM-1

#define V0p2_REV 14
#define COMMON_SETTINGS
// IF DEFINED: basic FROST/WARM temperatures are settable.
#undef ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.  ***
#undef ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1 
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// IF DEFINED: this unit *can* act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.  ***
#undef ENABLE_BOILER_HUB
// IF DEFINED: allow RX of stats frames.
#undef ENABLE_STATS_RX
// IF DEFINED: (default) forced always-on radio listen/RX, eg not requiring setup to explicitly enable. ***
#undef ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// IF DEFINED: allow minimal binary format in addition to more generic one: ~400 bytes code cost.
#undef ENABLE_MINIMAL_STATS_TXRX
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#undef ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// IF DEFINED: this unit supports CLI over the USB/serial connection, eg for run-time reconfig.
#define ENABLE_CLI
// IF DEFINED: enable a full OpenTRV CLI.
#define ENABLE_FULL_OT_CLI
// IF DEFINED: enable a full OpenTRV UI with normal LEDs etc. ***
#undef ENABLE_FULL_OT_UI
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#undef ENABLE_EXTENDED_CLI
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.  ***
#undef ENABLE_MIN_ENERGY_BOOT
// IF DEFINED: enable use of on-board SHT21 RH and temp sensor (in lieu of TMP112).   ***
#undef ENABLE_PRIMARY_TEMP_SENSOR_SHT21
//#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
// IF DEFINED: enable use AVR's 'idle' mode to stop the CPU but leave I/O (eg Serial) running to save power.
// DHD20150920: CURRENTLY NOT RECOMMENDED AS STILL SEEMS TO CAUSE SOME BOARDS TO CRASH.
#define ENABLE_USE_OF_AVR_IDLE_MODE

// IF DEFINED: enable a 'null' radio module; without this unit is stand-alone.
#define ENABLE_RADIO_NULL

// Secondary radio
#undef ENABLE_RADIO_RFM23B
#undef ENABLE_RADIO_PRIMARY_RFM23B

// IF DEFINED: enable a secondary (typically WAN-relay) radio module.
#define ENABLE_RADIO_SECONDARY_MODULE
#define ENABLE_RADIO_RN2483   // Enable RN2483
//#define RADIO_PRIMARY_RN2483 // Must be secondary to avoid sending preamble etc
#define RADIO_SECONDARY_RN2483

// Define voice module
#define ENABLE_VOICE_SENSOR
#define ENABLE_OCCUPANCY_DETECTION_FROM_VOICE
#define ENABLE_VOICE_STATS

#endif // CONFIG_REV14_PROTO

//___________________________________

#ifdef CONFIG_BH_DHW // DHW on REV1 board.
// Revision of V0.2 board.
#define V0p2_REV 1
// Enable use of OneWire devices.
#define ENABLE_MINIMAL_ONEWIRE_SUPPORT
// Enable use of DS18B20 temp sensor.
#define ENABLE_PRIMARY_TEMP_SENSOR_DS18B20
// Select DHW temperatures by default.
#define DHW_TEMPERATURES
// Must minimise water flow.
#define TRV_SLEW_GLACIAL
// Set max percentage open: BH reports 30% to be (near) optimal 2015/03; BH requested 20% at 2015/10/15, 13% at 2016/01/19.
#define TRV_MAX_PC_OPEN 13
// IF UNDEFINED: this unit cannot act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#undef ENABLE_BOILER_HUB
// IF UNDEFINED: don't allow RX of stats frames (since there is no easy way to plug in a serial connection to relay them!)
#undef ENABLE_STATS_RX
// IF DEFINED: allow TX of stats frames.
#define ENABLE_STATS_TX
// TODO-264: Find out why IDLE seems to crash some REV1 boards.
#undef ENABLE_USE_OF_AVR_IDLE_MODE
// Override schedule on time to simple fixed value of 2h per BH request 2015/10/15.
#define LEARNED_ON_PERIOD_M 120 // Must be <= 255.
#define LEARNED_ON_PERIOD_COMFORT_M LEARNED_ON_PERIOD_M
// Bo just wants the timing for his DHW; no occupancy sensing.
// IF DEFINED: support for general timed and multi-input occupancy detection / use.
#undef ENABLE_OCCUPANCY_SUPPORT
// IF DEFINED: detect occupancy based on ambient light, if available.
#undef ENABLE_OCCUPANCY_DETECTION_FROM_AMBLIGHT
// Use common settings.
#define COMMON_SETTINGS
#endif

// -------------------------













// --------------------------------------------
// CONSEQUENTIAL DEFINITIONS ARISING FROM ABOVE
// (Don't fiddle with these unless you are sure of module interdependencies, etc!)


#ifdef COMMON_SETTINGS // FOR REV0 onwards...
#if (V0p2_REV >= 1) // && (V0p2_REV <= 8) // && !defined(CONFIG_DHD_TESTLAB_REV2) // All REV 1--8 PCBs use RFM23B.
// IF DEFINED: RFM23 is in use in place of RFM22.
#define RFM22_IS_ACTUALLY_RFM23 // Note: RFM23 used on V0.2 PCB.
#endif // V0p2_REV >= 1
#endif // COMMON_SETTINGS

// If ENABLE_LEARN_BUTTON then in the absence of anything better ENABLE_SINGLETON_SCHEDULE should be supported.
#ifdef ENABLE_LEARN_BUTTON
#ifndef ENABLE_SINGLETON_SCHEDULE
#define ENABLE_SINGLETON_SCHEDULE
#endif
#endif

// For now (DHD20150927) allowing stats TX forces JSON to allos JSON stats.
// IF DEFINED: allow TX of stats frames.
#ifdef ENABLE_STATS_TX
// IF DEFINED: allow JSON stats frames alongside binary ones.
#define ENABLE_JSON_OUTPUT
#endif

// If (potentially) needing to run in some sort of continuous RX mode, define a flag.
#if defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX) || defined(ENABLE_DEFAULT_ALWAYS_RX)
#define ENABLE_CONTINUOUS_RX // was #define CONFIG_IMPLIES_MAY_NEED_CONTINUOUS_RX true
#endif

// By default (up to 2015), use the RFM22/RFM23 module to talk to an FHT8V wireless radiator valve.
#ifdef ENABLE_FHT8VSIMPLE
#define ENABLE_RADIO_RFM23B
#define ENABLE_FS20_CARRIER_SUPPORT
#define ENABLE_FS20_ENCODING_SUPPORT
// If this can be a hub, enable extra RX code.
#if defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)
#define ENABLE_FHT8VSIMPLE_RX
#define LISTEN_FOR_FTp2_FS20_native
#endif // defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)
#if defined(ENABLE_STATS_RX)
#define ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
#endif // defined(ENABLE_STATS_RX)
#endif // ENABLE_FHT8VSIMPLE

// If in stats or boiler hub mode, and with an FS20 OOK carrier, then apply a trailing-zeros RX filter.
#if (defined(ENABLE_BOILER_HUB) || defined(ENABLE_STATS_RX)) && defined(LISTEN_FOR_FTp2_FS20_native)
#define CONFIG_TRAILING_ZEROS_FILTER_RX
#endif


#endif

