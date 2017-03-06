#!/bin/sh -e
#
# List primary configurations of V0p2_Main to be tested during CI.
# This testing may only be for compilation success.
#
# Available CONFIGs are listed one per line to stdout.

# CONFIGs are extracted from the V0p2_Main/V0p2_Generic_Config.h header.
#
# Any lines starting exactly with //#define CONFIG_ are primary configurations
# (plus any single 'live' config line starting #define CONFIG_)
# and all may be dynamically uncommented and tested in CI scripts for viability
# while all other #define CONFIG_ lines will be commented out
# including any current 'live' config.
# Leave space after the CONFIG_XXX token before any trailing comment.
# (None of these dynamic changes should ever be checked back in.)

# Example primary configurations:
# //#define CONFIG_Trial2013Winter_Round2_STATSHUB // REV2 cut4 as stats hub.
# //#define CONFIG_Trial2013Winter_Round2_NOHUB // REV2 cut4 as TX-only leaf node.
# #define CONFIG_DORM1 // REV7 / DORM1 / TRV1.x all-in-one valve unit, secure TX.
# //#define CONFIG_DORM1_BOILER // REV8 / DORM1 boiler-control unit.

HEADER=Arduino/V0p2_Main/V0p2_Generic_Config.h

# Extract config name from lines starting //#define CONFIG_ or #define CONFIG_
exec awk < $HEADER '/^(\/){0,2}#define CONFIG_/ { print $2; }'
