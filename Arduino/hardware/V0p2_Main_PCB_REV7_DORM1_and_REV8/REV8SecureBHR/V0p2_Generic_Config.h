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
 
 Note: the format of this needs to be managed carefully and restricted (see below)
 to allow dynamic adjustment in Continuous Integration (CI) tests
 that all primary/production configurations remain viable.

 This should only be "#include"d once by code
 since normal guards against re-inclusion may be defeated during CI processing.
 */

#ifndef V0P2_GENERIC_CONFIG_H
#define V0P2_GENERIC_CONFIG_H

// Defaults for V0p2 / V0.2; should be '#undef'ined if not required.
//
// Use sleep wakeup (2Hz by default) from external 32768Hz xtal and timer 2.
#define ENABLE_WAKEUP_32768HZ_XTAL
// IF DEFINED: this unit may run on 2xAA cells, preferably rechargeable eg NiMH, ~2V--2.4V, and should monitor supply voltage.
#define ENABLE_SUPPLY_VOLTAGE_LOW_2AA // May require limiting clock speed and using some alternative peripherals/sensors...
// IF DEFINED: enable use AVR's 'idle' mode to stop the CPU but leave I/O clocls (eg Serial) running to save power.
// DHD20150920: NOT RECOMMENDED AS SEEMS TO CAUSE SOME BOARDS (REV1,REV9) TO CRASH.
#undef ENABLE_USE_OF_AVR_IDLE_MODE
// IF DEFINED: use (slow, low energy) 32768Hz-clock-based watchdog to recover from some software hangups.
#define ENABLE_WATCHDOG_SLOW
// IF DEFINED: attempt to tune the internal fast (RC/resonator) clock from the RTC source.
#define ENABLE_TUNE_FAST_OSC_TO_RTC_SOURCE
// IF DEFINED: provide software RTC support by default.
#define ENABLE_RTC_INTERNAL_SIMPLE
// IF DEFINED: try to trim memory (primarily RAM, also code/Flash) space used.
#define ENABLE_TRIMMED_MEMORY
// IF DEFINED: try to trim bandwidth as may be especially expensive/scarce.
#undef ENABLE_TRIMMED_BANDWIDTH
// IF DEFINED: minimise boot effort and energy eg for intermittently-powered energy-harvesting applications.
#undef ENABLE_MIN_ENERGY_BOOT
// IF DEFINED: this unit will act as a thermostat controlling a local TRV (and calling for heat from the boiler), else is a sensor/hub unit.
#undef ENABLE_LOCAL_TRV
// IF DEFINED: this unit controls a valve, but provides slave valve control only.
#undef ENABLE_SLAVE_TRV
// If DEFINED: attempt proportional (rather than cruder, eg, on/off, control of TRV or other heat source).
#define ENABLE_PROPORTIONAL_VALVE_CONTROL
// IF DEFINED: this unit *can* act as boiler-control hub listening to remote thermostats, possibly in addition to controlling a local TRV.
#define ENABLE_BOILER_HUB
//////////////////////////////////////// DEV/MAINT UI OPTIONS (and support for them)
// IF DEFINED: allow local generation of JSON stats frames (this may not affect relaying).
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
// IF DEFINED: provide CLI read/write access to generic parameter block.
#define ENABLE_GENERIC_PARAM_CLI_ACCESS
// IF DEFINED: enable and extended CLI with a longer input buffer for example.
#define ENABLE_EXTENDED_CLI
// IF DEFINED: physical UI use wakes CLI (not needed when CLI can auto-wake from serial).
#undef ENABLE_UI_WAKES_CLI
// IF DEFINED: allow setting of ID from CLI to replace devices in situ; not recommended by default to avoid confusion.
#undef ENABLE_ID_SET_FROM_CLI
//////////////////////////////////////// DEVICE UI OPTIONS (and support for them)
// IF DEFINED: basic FROST/WARM temperatures are settable and stored in EEPROM.
#define ENABLE_SETTABLE_TARGET_TEMPERATURES
// IF DEFINED: fast temp pot/dial sampling to partly compensate for less good mechanics (at some energy cost).
#define ENABLE_FAST_TEMP_POT_SAMPLING
// IF DEFINED: support one on and one off time per day (possibly in conjunction with 'learn' button).
#define ENABLE_SINGLETON_SCHEDULE
// IF DEFINED: use active-low LEARN button(s).  Needs ENABLE_SINGLETON_SCHEDULE.
#define ENABLE_LEARN_BUTTON // OPTIONAL ON V0.09 PCB1
// IF DEFINED: enable use of second UI LED if available.
#undef ENABLE_UI_LED_2_IF_AVAILABLE
// IF DEFINED: simplified mode button behaviour: tapping button invokes BAKE, not mode cycling.
#undef ENABLE_SIMPLIFIED_MODE_BAKE
// IF DEFINED: enabled frequent stats TX, eg every minute, for diagnostics.
#undef ENABLE_FREQUENT_STATS_TX
// IF DEFINED: the (>>8) value of this flag is the maximum JSON frame size allowed (bytes).
#undef ENABLE_JSON_STATS_LEN_CAP
// IF DEFINED: unconditionally suppress the "@" ID field (carrier supplies it or equiv) to save bandwidth.
#undef ENABLE_JSON_SUPPRESSED_ID
// IF DEFINED: unconditionally suppress the "+" ID field and aim for minimum JSON frame size, for poor/noisy comms channels.
// NOTE: minimising the JSON frame will overall *reduce* bandwidth efficiency and ability to diagnose TX problems.
#undef ENABLE_JSON_FRAME_MINIMISED
// IF DEFINED: enable a CLI-settable setback lockout (hours/days) to establish a baseline before engaging energy saving setbacks.
#undef ENABLE_SETBACK_LOCKOUT_COUNTDOWN
//////////////////////////////////////// SENSOR OPTIONS (and support for them)
// IF DEFINED: allow use of ambient light sensor.
#define ENABLE_AMBLIGHT_SENSOR
// IF DEFINED: allow for less light on sideways-pointing ambient light sensor, eg on cut4 2014/03/17 REV2 boards (TODO-209).
#undef ENABLE_AMBLIGHT_EXTRA_SENSITIVE
// IF DEFINED: use RoHS-compliant phototransistor in place of default LDR.
#undef ENABLE_AMBIENT_LIGHT_SENSOR_PHOTOTRANS_TEPT4400
// IF DEFINED: use the temperature-setting potentiometer/dial if present.
#define ENABLE_TEMP_POT_IF_PRESENT
// Enable use of OneWire devices.
#undef ENABLE_MINIMAL_ONEWIRE_SUPPORT
// IF DEFINED: enable use of on-board SHT21 primary temperature and RH% sensor (in lieu of default TMP112).
#define ENABLE_PRIMARY_TEMP_SENSOR_SHT21
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
// IF DEFINED: had RFM23B as the primary radio module: default from REV1 to REV11.
#define ENABLE_RADIO_RFM23B
// IF DEFINED: make RFM23B the primary radio.
#define ENABLE_RADIO_PRIMARY_RFM23B
// IF DEFINED: enable a secondary (typically WAN-relay) radio module.
#undef ENABLE_RADIO_SECONDARY_MODULE
// IF DEFINED: enable a WAN-relay radio module, primarily to relay stats outbound.
#undef ENABLE_RADIO_SECONDARY_MODULE_AS_RELAY
// IF DEFINED: enable a 'null' radio module; can be used to simplify code for a radio-less unit.
#undef ENABLE_RADIO_NULL
// IF DEFINED: enable periodic secure beacon broadcast.
#undef ENABLE_SECURE_RADIO_BEACON
// IF DEFINED: enable support for fast (>50kbps) packet-handling carrier (leading length byte).
#define ENABLE_FAST_FRAMED_CARRIER_SUPPORT
// IF DEFINED: enable support for FS20 carrier for RX or TX.
#undef ENABLE_FS20_CARRIER_SUPPORT
// IF DEFINED: enable raw preamble injection/framing eg for FS20 over RFM23B.
#undef ENABLE_RFM23B_FS20_RAW_PREAMBLE
// IF DEFINED: use FHT8V wireless radio module/valve, eg to control FHT8V local valve.
#undef ENABLE_FHT8VSIMPLE
// IF DEFINED: enable support for FS20 carrier for RX of raw FS20 and piggybacked binary (non-JSON) stats.
#define ENABLE_FS20_NATIVE_AND_BINARY_STATS_RX
// IF DEFINED: enable support for FS20 encoding/decoding, eg to send to FHT8V.
#undef ENABLE_FS20_ENCODING_SUPPORT
// IF DEFINED: enable OpenTRV secure frame encoding/decoding (as of 2015/12).
#define ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
// IF DEFINED: allow non-secure OpenTRV secure frame RX (as of 2015/12): DISABLED BY DEFAULT.
#undef ENABLE_OTSECUREFRAME_INSECURE_RX_PERMITTED
// IF DEFINED: allow RX of stats frames.
#define ENABLE_STATS_RX
// IF DEFINED: allow TX of local stats frames (should not affect relaying).
#define ENABLE_STATS_TX
// IF DEFINED: always allow some kind of stats TX, whatever the privacy settings.
// HAS HUGE PRIVACY IMPLICATIONS: DO NOT ENABLE UNNECESSARILY!
#define ENABLE_ALWAYS_TX_ALL_STATS
// IF DEFINED: allow minimal binary format in addition to more generic one: ~400 bytes code cost.
#undef ENABLE_MINIMAL_STATS_TXRX
// IF DEFINED: allow binary stats to be TXed.
#undef ENABLE_BINARY_STATS_TX
// IF DEFINED: allow radio listen/RX.
#define ENABLE_RADIO_RX
// IF DEFINED: forced always-on radio listen/RX, eg not requiring setup to explicitly enable.
#define ENABLE_DEFAULT_ALWAYS_RX
// IF DEFINED: XXX
#define ENABLE_HUB_LISTEN
#define ENABLE_CONTINUOUS_RX // was #define CONFIG_IMPLIES_MAY_NEED_CONTINUOUS_RX true

#endif

