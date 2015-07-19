Add to IDE's boards.txt, eg /Applications/Arduino.app/Contents/Resources/Java/hardware/arduino/boards.txt.
(Or create as a boards.txt to put under hardware/2AAbreadboard in Arduino sketches directory, but will break compilation.)

8MHz internal RC clock / 8, so 1MHz effective (low fuse 0x62, default is not /8),
and with BOD set for 1.8V to allow operation with 2xAA rechargeables (extended fuse 0x06, default is 2.7V).
