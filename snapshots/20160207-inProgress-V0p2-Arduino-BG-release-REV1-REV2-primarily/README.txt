Primarily for BG to install on his home REV1 and REV2 nodes.
DHD20160207

Take the OTRadioLink.ZIP from the snapshot, or download the latest from the master (as of 20160207 19:20 UTC), or download that in the 20160207-BG-alpha tag.

Configurations of interest are:

#define CONFIG_Trial2013Winter_Round1_NOHUB // REV1 as TX-only leaf node.

and:

#define CONFIG_Trial2013Winter_Round2_NOHUB // REV2 cut4 as TX-only leaf node.

Exactly one CONFIG_XXX should be uncommented at a time.




Q: How do I upgrade the OTRadioLink library?

Shut down the Arduino IDE.

Remove the existing installed one.

On OS X it is under:

~/Documents/Arduino/libraries/OTRadioLink

and on Linux:

~/Arduino/libraries/OTRadioLink

So I run:

rm -rf ~/Documents/Arduino/libraries/OTRadioLink

To install the new one reopen the IDE and:

Sketch -> Include Library -> Add .ZIP Library

and browse to your chosen OTRadioLink.zip to install it.