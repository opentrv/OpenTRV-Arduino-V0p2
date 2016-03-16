// Uncomment exactly one of the following CONFIG_ lines to select which board is being built for.
//#define CONFIG_Trial2013Winter_Round2_CC1HUB // REV2 cut4 as CC1 hub.
#define CONFIG_REV9 // REV9 as CC1 relay, cut 2 of the board.

//#ifndef BAUD
//#define BAUD 4800 // Ensure that OpenTRV 'standard' UART speed is set unless explicitly overridden.
//#endif

#include <Arduino.h>
#include <Wire.h>
#include <OTV0p2Base.h>
#include <OTRadioLink.h>
#include <OTRadValve.h>
#include <OTV0p2_CONFIG_REV2.h>
#include <OTV0p2_CONFIG_REV9.h>
#include <OTV0p2_Board_IO_Config.h> // I/O pin allocation and setup: include ahead of I/O module headers.




void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
