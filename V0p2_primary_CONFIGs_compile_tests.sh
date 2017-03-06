#!/bin/sh -e
#
# Test all primary configurations of V0p2_Main to be tested during CI.
#
# Copies V0p2_Main to a temporary working area
# and adjusts its generic config header to test all primary configs in turn.

# Name of generic config header which should #define one CONFIG_...
GENERICCONFIGHEADER=V0p2_Generic_Config.h

# Name of sketch.
SKETCHNAME=V0p2_Main

# Path to V0p2_Main sketch.
MAIN=$PWD/Arduino/V0p2_Main

echo Hello world
exit 0