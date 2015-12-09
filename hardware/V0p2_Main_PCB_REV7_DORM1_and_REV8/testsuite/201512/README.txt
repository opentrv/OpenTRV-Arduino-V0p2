Software test suit for REV7 as of 2015/12.

REV7test1:

 1) Flashes primary and secondary UI LEDs and prints out which buttons are pressed while any is pressed.

 2) Mode button toggles between interactive and soak-testing modes; starts in interactive mode.

2a) In interactive mode the pin will advance to one end or the other as one of the learn buttons
    is held pressed, stopping when stall current is detected.

2b) In soak-test mode the pin will be driven back and forth between end stops automatically.

 3) The light, temperature, relative humidity and dial sensors/inputs are read and printed to serial
    on a 30s cycle.

Derived from at subset of the the V0p2_Main mainline code.

Built with Arduino 1.6.5 against the OTRadioLink library checked in in this directory.

The output .hex file are checked in.

Edit the AltMain.cpp file to make simple changes.

