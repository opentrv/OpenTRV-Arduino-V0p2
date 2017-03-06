#!/bin/sh -e
#
# Test all primary configurations of V0p2_Main to be tested during CI.
#
# Copies V0p2_Main to a temporary working area
# and adjusts its generic config header to test all primary configs in turn.
#
# Shows all failures before exiting (no -e flag)

echo Test compilation of primary configs of main Arduino projects.

# Target Arduino board to build for.
BUILD_TARGET=opentrv:avr:opentrv_v0p2

# Name of generic config header which should #define one CONFIG_...
GENERICCONFIGHEADER=V0p2_Generic_Config.h

# Name of main sketch.
SKETCHNAME=V0p2_Main

# Relative path to V0p2_Main sketch.
MAIN=Arduino/$SKETCHNAME

# Target copy of main sketch to update.
# MUST NEVER BE EMPTY!
WORKINGDIR=$PWD/tmp-build-area

if [ -e $WORKINGDIR ]; then
    echo Temporary working copy directory $WORKINGDIR exists, aborting.
    exit 99
fi

# Create the temporary directory.
mkdir -p $WORKINGDIR

# Copy the main sketch to the working area.
cp -rp $PWD/$MAIN $WORKINGDIR

ls $WORKINGDIR/$SKETCHNAME/
TARGETINO=$WORKINGDIR/$SKETCHNAME/$SKETCHNAME.ino
if [ ! -f $TARGETINO ]; then
    echo Missing $TARGETINO
    exit 99
fi

# Set status non-zero if a test/compilation fails.
STATUS=0

echo @@@@@@ Testing default target: $WORKINGDIR/$SKETCHNAME/$SKETCHNAME.ino
if arduino --verify --board $BUILD_TARGET $WORKINGDIR/$SKETCHNAME/$SKETCHNAME.ino; then
    echo OK
else
    echo FAILED
    STATUS=1
fi

# Fetch all primary configs.
CONFIGS="`./V0p2_list_primary_CONFIGs.sh`"
if [ "X" = "X$CONFIGS" ]; then
    echo No primary configs found, aborting.
    exit 99
fi

# Test each of the primary configs.
for config in $CONFIGS;
do
    echo @@@@@@ Testing config $config
    # Overwrite generic config header with single #define for this config.
    echo "#define $config" > $WORKINGDIR/$SKETCHNAME/$GENERICCONFIGHEADER
    # Compile...
    if arduino --verify --board $BUILD_TARGET $WORKINGDIR/$SKETCHNAME/$SKETCHNAME.ino; then
        echo OK
	else
	    echo FAILED $config
	    STATUS=2
	fi
done

# Tidy up: delete the working directory.
rm -rf $WORKINGDIR

echo Final status: $STATUS
exit $STATUS