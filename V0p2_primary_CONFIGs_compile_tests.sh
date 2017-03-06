#!/bin/sh -e
#
# Test all primary configurations of V0p2_Main to be tested during CI.
#
# Copies V0p2_Main to a temporary working area
# and adjusts its generic config header to test all primary configs in turn.

# Name of generic config header which should #define one CONFIG_...
GENERICCONFIGHEADER=V0p2_Generic_Config.h

# Name of main sketch.
SKETCHNAME=V0p2_Main

# Relative path to V0p2_Main sketch.
MAIN=Arduino/$SKETCHNAME

# Target copy of main sketch to update.
# MUST NEVER BE EMPTY!
WORKINGDIR=$PWD/tmp/build-area

if [ -e $WORKINGDIR ]; then
    echo Temporary working copy directory $WORKINGDIR exists, aborting.
    exit 3
fi

# Create the temporary directory.
mkdir -p $WORKINGDIR

# Copy the main sketch to the working area.
cp -rp $PWD/$MAIN $WORKINGDIR

echo @@@@@@ Testing primary target as-is.
arduino --verify --board $BUILD_TARGET $WORKINGDIR/$SKETCHNAME/$SKETCHNAME.ino

# Tidy up: delete the working directory.
rm -rf $WORKINGDIR


exit 0