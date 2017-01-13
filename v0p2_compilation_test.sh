#!/usr/bin/env bash
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

ARDUINO_PATH=$HOME/arduino-1.6.12  # Path to Arduino bin
BOARD_URL=https://raw.githubusercontent.com/opentrv/OpenTRV-Config/master/Arduino/package_opentrv_index.json
INSTALL_TARGET="opentrv:avr"  # Target board to install
BUILD_TARGET=opentrv:avr:opentrv_v0p2  # Target board to build for.
SKETCH_PATH=$HOME/git/OpenTRV-Arduino-V0p2/Arduino/hardware  # Path to hardware directory

# Setup preferences to point at OpenTRV board config.
$ARDUINO_PATH/arduino --pref boardsmanager.additional.urls=$BOARD_URL

# Install V0p2 board.
$ARDUINO_PATH/arduino --install-boards $INSTALL_TARGET

# Make script terminate when something returns false.
set -e

# Verify sketch compiles.
#$ARDUINO_PATH/arduino --verify --board $BUILD_TARGET $SKETCH_PATH

# Verify REV7 minimal test.
$ARDUINO_PATH/arduino --verify --board $BUILD_TARGET $SKETCH_PATH/V0p2_Main_PCB_REV7_DORM1_and_REV8/REV7HardwareTest/REV7HardwareTest.ino
# Verify REV8 minimal test.
$ARDUINO_PATH/arduino --verify --board $BUILD_TARGET $SKETCH_PATH/V0p2_Main_PCB_REV7_DORM1_and_REV8/REV8HardwareTest/REV8HardwareTest.ino
# Verify REV11 minimal test.
$ARDUINO_PATH/arduino --verify --board $BUILD_TARGET $SKETCH_PATH/REV11/REV11HardwareTest/REV11HardwareTest.ino
