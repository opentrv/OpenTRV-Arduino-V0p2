#!/bin/sh -e
#
# Set up OpenTRV Arduino IDE environment and verify compilation.
# Exits 0 on success.
# Refer to https://github.com/arduino/Arduino/blob/master/build/shared/manpage.adoc for arduino CLI docs.
#
# Script assumes that required libraries (OTRadioLink and OTAESGCM) are already installed in $HOME/Arduino/libs.
# Dependencies:
#   firmware:
#       OpenTRV-Arduino-V0p2 : https://github.com/opentrv/OpenTRV-Arduino-V0p2
#   board config:
#       OpenTRV V0p2 board: https://github.com/opentrv/OpenTRV-Config/tree/master/Arduino
#   libs:
#       OTRadioLink: https://github.com/opentrv/OTRadioLink
#       OTAESGCM: https://github.com/opentrv/OTAESGCM
#
#
# NOTE!!! Arduino IDE requires an X server to run, even in CLI mode.
# See https://github.com/arduino/Arduino/blob/master/build/shared/manpage.adoc#bugs for instructions on how to set up a dummy X server in Linux.

BUILD_TARGET=opentrv:avr:opentrv_v0p2  # Target board to build for.
SKETCH_PATH=$PWD/Arduino/hardware  # Path to hardware directory

# List of minimal hardware configurations to be tested.
# Partial paths to the individual .ino files under $SKETCH_PATH.
PARTPATHS="
    V0p2_Main_PCB_REV7_DORM1_and_REV8/REV7HardwareTest/REV7HardwareTest.ino
    V0p2_Main_PCB_REV7_DORM1_and_REV8/REV8HardwareTest/REV8HardwareTest.ino
    REV10/REV10HardwareTest/REV10HardwareTest.ino
    REV11/REV11HardwareTest/REV11HardwareTest.ino
    "

# Loop through and test each configuration.
# Note which one is being tested to make clear which one has failed, if any.
for pp in $PARTPATHS;
do
    echo Testing $SKETCH_PATH/$pp
    arduino --verify --board $BUILD_TARGET $SKETCH_PATH/$pp
done

# Tests
# Script uses sh -e which will cause silent fail if test is not enclosed within the brackets below.
#(  # Put tests inside these brackets!
# Verify REV7 minimal test.
#arduino --verify --board $BUILD_TARGET $SKETCH_PATH/V0p2_Main_PCB_REV7_DORM1_and_REV8/REV7HardwareTest/REV7HardwareTest.ino
# Verify REV8 minimal test.
#arduino --verify --board $BUILD_TARGET $SKETCH_PATH/V0p2_Main_PCB_REV7_DORM1_and_REV8/REV8HardwareTest/REV8HardwareTest.ino
# Verify REV10 minimal test.
#arduino --verify --board $BUILD_TARGET $SKETCH_PATH/REV10/REV10HardwareTest/REV10HardwareTest.ino
# Verify REV11 minimal test.
#arduino --verify --board $BUILD_TARGET $SKETCH_PATH/REV11/REV11HardwareTest/REV11HardwareTest.ino
#)
