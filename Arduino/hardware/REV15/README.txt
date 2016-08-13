REV15+

A set of modular processor and sensor boards that can be stacked on top of each other as needed. These boards would have the I2CEXT connector, located in a common position.

The boards should include:
- A processor board.
- A power supply board.
- A radio board.
- Various sensor boards.


Specifications:

Physical layout:
- 5 cm x 5 cm form factor
- Standard positioning of 4 screw holes (as on REV2).
- Standoffs to work with connectors, long lead connectors.
- The I2CEXT connector as defined in https://github.com/DamonHD/OpenTRV/blob/master/standards/I2CEXT/tech-specs/201602-AVR-I2CEXT-connector.txt

Individual boards:
- Processor board (REV15):
    - Atmega328p
    - Watch xtal/resonator
    - Photodiode
    - Status LED
- Power supply board (REV16*):
    - Micro USB connector.
    - 2xAAA Battery connector (as on REV11)
    - 3.3 V regulator
- Radio board (REV17*):
    - Pads and connections for a single RFM23B or pin compatible RFM69.
    - Not backwards compatible with RFM22.
    - Decide on antenna.
- Sensor boards:
    - Temp/humidity (REV18*)
- More TBD. Suggestions include:
    - CO2 sensor board (sensor TBD).
    - PIR
    - CO
    - PMx
    - VO

* To be renamed.

Dir structure:
./
    - rev15.sch
    - rev15.brd
    - README.txt
    - CHECKLIST.txt
    + DRU+CAM/
    + gerbers/ *THIS CONTAINS GERBERS FOR ALL BOARDS*
    + REV16/
    + REV17/
    + REV18/


