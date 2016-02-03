SNAPSHOT of COHEAT functions moved to mainline 2016/01/23 15:00 UTC.
DHD20160123

I have tested the current COHEAT functionality on the main line:

  * FHT8V control by REV9
  * alert/poll/response radio traffic
  * UI button & LEDS on REV9

and they seem to be fine.

For your reference (and ours) I have tagged OTProtocolCC, OTRadioLink and OpenTRV with:

20160123-COHEAT-REV2-REV9-functional

so in principle you could take this and get the benefit of improved RX behaviour (deeper queues), better serial behaviour, and elimination of IDLE mode as per COH-72. Just needs config switching for REV2/REV9 builds as before.

I don’t regard these as fully tested yet, but if you felt like giving them a spin RealSoonNow I’d be grateful for any early feedback of errors/oversights as there has been an enormous change in the underlying mainline since the last release to you.
