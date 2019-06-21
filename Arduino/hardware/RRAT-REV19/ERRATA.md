# Pending
## 20190620
- No throughhole for whip antenna.
- Board size needs to be reduced by ~0.5 mm on antenna side.
- Modify trace antenna for easier length tuning.
- J3 should be on the back side of the board.
- U2 footprint is wrong in position file.

### Further notes from MH 20190621
- J3 - location wrong, board side wrong
- U2 - pinout 90 deg rotated from correct - pin 1 is towards centre of board
- bottom silkscreen - move license statement to top side
- antenna - do tuning work; build in length tuning solder pads and extra track (see EnOcean application note for process)
 - impedance - do a better job of matching impedance
 - board - consider a smaller (not pi hat compatible) board size and an f antenna
 - standoffs - structural support for edge of board opposite side to 40 pin connector
 -consider a helical antenna