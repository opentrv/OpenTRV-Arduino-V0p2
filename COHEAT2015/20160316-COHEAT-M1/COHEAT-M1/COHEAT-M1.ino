// Uncomment exactly one of the following CONFIG_... lines to select which board is being built for.
//#define CONFIG_Trial2013Winter_Round2_CC1HUB // REV2 cut4 as CC1 hub.
#define CONFIG_REV9 // REV9 as CC1 relay, cut 2 of the board.

#include <Arduino.h>
#include <Wire.h>
#include <OTV0p2Base.h>
#include <OTRadioLink.h>
#include <OTRFM23BLink.h>
#include <OTRadValve.h>
#include <OTProtocolCC.h>
#include <OTV0p2_CONFIG_REV2.h>
#include <OTV0p2_CONFIG_REV9.h>
#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
#include <OTAESGCM.h>
#endif
#include <OTV0p2_Board_IO_Config.h> // I/O pin allocation and setup: include ahead of I/O module headers.

#define ENABLE_HUB_LISTEN
// Force-enable RX if not already so.
#define ENABLE_RADIO_RX
#define ENABLE_CONTINUOUS_RX // was #define CONFIG_IMPLIES_MAY_NEED_CONTINUOUS_RX true
// By default (up to 2015), use the RFM22/RFM23 module to talk to an FHT8V wireless radiator valve.
#ifdef ENABLE_FHT8VSIMPLE
#define ENABLE_RADIO_RFM23B
#define ENABLE_FS20_CARRIER_SUPPORT
#define ENABLE_FS20_ENCODING_SUPPORT
#define ENABLE_FHT8VSIMPLE_RX
#define LISTEN_FOR_FTp2_FS20_native
#endif // ENABLE_FHT8VSIMPLE

// Indicate that the system is broken in an obvious way (distress flashing of the main UI LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep the distress beacon running for a while.
void panic();
// Panic with fixed message.
void panic(const __FlashStringHelper *s);

// Version (code/board) information printed as one line to serial (with line-end, and flushed); machine- and human- parseable.
// Format: "board VXXXX REVY; code YYYY/Mmm/DD HH:MM:SS".
void serialPrintlnBuildVersion();

// Call this to do an I/O poll if needed; returns true if something useful happened.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// Should also do nothing that interacts with Serial.
// Limits actual poll rate to something like once every 8ms, unless force is true.
//   * force if true then force full poll on every call (ie do not internally rate-limit)
// Not thread-safe, eg not to be called from within an ISR.
// NOTE: implementation may not be in power-management module.
bool pollIO(bool force = false);

// Call this to productively burn tens to hundreds of CPU cycles, and poll I/O, eg in a busy-wait loop.
// This may churn PRNGs or gather entropy for example.
// This call should typically take << 1ms at 1MHz CPU.
// Does not change CPU clock speeds, mess with interrupts (other than possible brief blocking), or sleep.
// May capture some entropy in secure and non-secure PRNGs.
inline void burnHundredsOfCyclesProductivelyAndPoll()
  {
  if(pollIO()) { OTV0P2BASE::seedRNG8(OTV0P2BASE::getCPUCycleCount(), 37 /* _watchdogFired */, OTV0P2BASE::_getSubCycleTime()); }
  else { OTV0P2BASE::captureEntropy1(); }
  }

extern OTRadioLink::OTRadioLink &PrimaryRadio;

//---------------------

// Controller's view of Least Significant Digits of the current (local) time, in this case whole seconds.
#define TIME_CYCLE_S 60 // TIME_LSD ranges from 0 to TIME_CYCLE_S-1, also major cycle length.
static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.

// 'Elapsed minutes' count of minute/major cycles; cheaper than accessing RTC and not tied to real time.
// Starts at or just above zero (within the first 4-minute cycle) to help avoid collisions between units after mass power-up.
// Wraps at its maximum (0xff) value.
static uint8_t minuteCount;


#ifndef DEBUG
#define DEBUG_SERIAL_PRINT(s) // Do nothing.
#define DEBUG_SERIAL_PRINTFMT(s, format) // Do nothing.
#define DEBUG_SERIAL_PRINT_FLASHSTRING(fs) // Do nothing.
#define DEBUG_SERIAL_PRINTLN_FLASHSTRING(fs) // Do nothing.
#define DEBUG_SERIAL_PRINTLN() // Do nothing.
#define DEBUG_SERIAL_TIMESTAMP() // Do nothing.
#else
// Send simple string or numeric to serial port and wait for it to have been sent.
// Make sure that Serial.begin() has been invoked, etc.
#define DEBUG_SERIAL_PRINT(s) { OTV0P2BASE::serialPrintAndFlush(s); }
#define DEBUG_SERIAL_PRINTFMT(s, fmt) { OTV0P2BASE::serialPrintAndFlush((s), (fmt)); }
#define DEBUG_SERIAL_PRINT_FLASHSTRING(fs) { OTV0P2BASE::serialPrintAndFlush(F(fs)); }
#define DEBUG_SERIAL_PRINTLN_FLASHSTRING(fs) { OTV0P2BASE::serialPrintlnAndFlush(F(fs)); }
#define DEBUG_SERIAL_PRINTLN() { OTV0P2BASE::serialPrintlnAndFlush(); }
// Print timestamp with no newline in format: MinutesSinceMidnight:Seconds:SubCycleTime
extern void _debug_serial_timestamp();
#define DEBUG_SERIAL_TIMESTAMP() _debug_serial_timestamp()
#endif // DEBUG

// Use WDT-based timer for xxxPause() routines.
// Very tiny low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define VERYTINY_PAUSE_MS 5
static void inline veryTinyPause() { OTV0P2BASE::sleepLowPowerMs(VERYTINY_PAUSE_MS); }
// Tiny low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
#define TINY_PAUSE_MS 15
static void inline tinyPause() { OTV0P2BASE::nap(WDTO_15MS); } // 15ms vs 18ms nominal for PICAXE V0.09 impl.
// Small low-power sleep.
#define SMALL_PAUSE_MS 30
static void inline smallPause() { OTV0P2BASE::nap(WDTO_30MS); }
// Medium low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
// Premature wakeups MAY be allowed to avoid blocking I/O polling for too long.
#define MEDIUM_PAUSE_MS 60
static void inline mediumPause() { OTV0P2BASE::nap(WDTO_60MS); } // 60ms vs 144ms nominal for PICAXE V0.09 impl.
// Big low-power sleep to approximately match the PICAXE V0.09 routine of the same name.
// Premature wakeups MAY be allowed to avoid blocking I/O polling for too long.
#define BIG_PAUSE_MS 120
static void inline bigPause() { OTV0P2BASE::nap(WDTO_120MS); } // 120ms vs 288ms nominal for PICAXE V0.09 impl.
// Pause between flashes to allow them to be distinguished (>100ms); was mediumPause() for PICAXE V0.09 impl.
static void inline offPause()
  {
  bigPause(); // 120ms, was V0.09 144ms mediumPause() for PICAXE V0.09 impl.
  pollIO(); // Slip in an I/O poll.
  }

#ifdef ALLOW_CC1_SUPPORT_RELAY_IO // REV9 CC1 relay...
// Call this on even numbered seconds (with current time in seconds) to allow the CO UI to operate.
// Should never be skipped, so as to allow the UI to remain responsive.
bool tickUICO(uint_fast8_t sec);
// Directly adjust LEDs.
//   * light-colour         [0,3] bit flags 1==red 2==green (lc) 0 => stop everything
//   * light-on-time        [1,15] (0 not allowed) 30-450s in units of 30s (lt) ???
//   * light-flash          [1,3] (0 not allowed) 1==single 2==double 3==on (lf)
// If fromPollAndCmd is true then this is being called from an incoming Poll/Cms message receipt.
// Not ISR- safe.
void setLEDsCO(uint8_t lc, uint8_t lt, uint8_t lf, bool fromPollAndCmd);

// Get the switch toggle state.
// The hub should monitor this changing,
// taking the change as indication of a boost request.
// This is allowed to toggle only much slower than the hub should poll,
// thus ensuring that the hub doesn't miss a boost request.
// Safe to call from an ISR (though this would be unexpected).
bool getSwitchToggleStateCO();
#endif


// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic()
  {
#ifdef ENABLE_RADIO_PRIMARY_MODULE
  // Reset radio and go into low-power mode.
  PrimaryRadio.panicShutdown();
#endif
#ifdef ENABLE_RADIO_SECONDARY_MODULE
  // Reset radio and go into low-power mode.
  SecondaryRadio.panicShutdown();
#endif
  // Power down almost everything else...
  OTV0P2BASE::minimisePowerWithoutSleep();
#ifdef LED_HEATCALL
  pinMode(LED_HEATCALL, OUTPUT);
#else
  pinMode(LED_HEATCALL_L, OUTPUT);
#endif
  for( ; ; )
    {
    LED_HEATCALL_ON();
    tinyPause();
    LED_HEATCALL_OFF();
    bigPause();
    }
  }

// Panic with fixed message.
void panic(const __FlashStringHelper *s)
  {
  OTV0P2BASE::serialPrintlnAndFlush(); // Start new line to highlight error.  // May fail.
  OTV0P2BASE::serialPrintAndFlush('!'); // Indicate error with leading '!' // May fail.
  OTV0P2BASE::serialPrintlnAndFlush(s); // Print supplied detail text. // May fail.
  panic();
  }

// Rearrange date into sensible most-significant-first order, and make it fully numeric.
// FIXME: would be better to have this in PROGMEM (Flash) rather than RAM, eg as F() constant.
static const char _YYYYMmmDD[] =
  {
  __DATE__[7], __DATE__[8], __DATE__[9], __DATE__[10],
  '/',
  __DATE__[0], __DATE__[1], __DATE__[2],
  '/',
  ((' ' == __DATE__[4]) ? '0' : __DATE__[4]), __DATE__[5],
  '\0'
  };
// Version (code/board) information printed as one line to serial (with line-end, and flushed); machine- and human- parseable.
// Format: "board VX.X REVY YYYY/Mmm/DD HH:MM:SS".
void serialPrintlnBuildVersion()
  {
  OTV0P2BASE::serialPrintAndFlush(F("board V0.2 REV"));
  OTV0P2BASE::serialPrintAndFlush(V0p2_REV);
  OTV0P2BASE::serialPrintAndFlush(' ');
  OTV0P2BASE::serialPrintAndFlush(_YYYYMmmDD);
  OTV0P2BASE::serialPrintlnAndFlush(F(" " __TIME__));
  }

// Singleton FHT8V valve instance (to control remote FHT8V valve by radio).
static const uint8_t _FHT8V_MAX_EXTRA_TRAILER_BYTES = (1 + max(OTV0P2BASE::MESSAGING_TRAILING_MINIMAL_STATS_PAYLOAD_BYTES, OTV0P2BASE::FullStatsMessageCore_MAX_BYTES_ON_WIRE));
OTRadValve::FHT8VRadValve<_FHT8V_MAX_EXTRA_TRAILER_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTES, OTRadValve::FHT8VRadValveBase::RFM23_PREAMBLE_BYTE> FHT8V(NULL);
// This unit may control a local TRV.
// Returns TRV if valve/radiator is to be controlled by this unit.
// Usually the case, but may not be for (a) a hub or (b) a not-yet-configured unit.
// Returns false if house code parts are set to invalid or uninitialised values (>99).
#if defined(ENABLE_LOCAL_TRV) || defined(ENABLE_SLAVE_TRV)
inline bool localFHT8VTRVEnabled() { return(!FHT8V.isUnavailable()); }
#else
#define localFHT8VTRVEnabled() (false) // Local FHT8V TRV disabled.
#endif

#if defined(ENABLE_SLAVE_TRV)
#define ENABLE_NOMINAL_RAD_VALVE
// Simply alias directly to FHT8V for REV9 slave.
#define NominalRadValve FHT8V
#endif


#ifdef ALLOW_CC1_SUPPORT_RELAY
// Send a CC1 Alert message with this unit's house code via the RFM23B.
bool sendCC1AlertByRFM23B()
  {
  OTProtocolCC::CC1Alert a = OTProtocolCC::CC1Alert::make(FHT8V.nvGetHC1(), FHT8V.nvGetHC2());
  if(a.isValid()) // Might be invalid if house codes are, eg if house codes not set.
    {
    uint8_t txbuf[OTProtocolCC::CC1Alert::primary_frame_bytes+1]; // More than large enough for preamble + sync + alert message.
    const uint8_t bodylen = a.encodeSimple(txbuf, sizeof(txbuf), true);
#if 0 && defined(DEBUG)
OTRadioLink::printRXMsg(p, txbuf, bodylen);
#endif
    // Send loud since the hub may be relatively far away,
    // there is no 'ACK', and these messages should not be sent very often.
    // Should be consistent with automatically-generated alerts to help with diagnosis.
    return(PrimaryRadio.sendRaw(txbuf, bodylen, 0, OTRadioLink::OTRadioLink::TXmax));
    }
  return(false); // Failed.
  }
#endif

#ifdef ALLOW_CC1_SUPPORT_RELAY_IO // REV9 CC1 relay...
// Do basic static LED setting.
static void setLEDs(const uint8_t lc)
    {
    // Assume primary UI LED is the red one (at least fot REV9 boards)...
    if(lc & 1) { LED_HEATCALL_ON(); } else { LED_HEATCALL_OFF(); }
    // Assume secondary UI LED is the green one (at least fot REV9 boards)...
    if(lc & 2) { LED_UI2_ON(); } else { LED_UI2_OFF(); }
    }

// Logical last-requested light colour (lc).
static uint8_t lcCO;
// Count down in 2s ticks until LEDs go out (derived from lt).
static uint8_t countDownLEDSforCO;
// Requested flash type (lf).
static uint8_t lfCO;

// Handle boost button-press semantics.
// Timeout in minutes before a new boot request will be fully actioned.
// This is kept long enough to ensure that the hub cannot have failed to see the status flip
// unless all contact has in fact been lost.
static const uint8_t MIN_BOOST_INTERVAL_M = 30;
// Count down from last flip of switch-toggle state, minutes.  Cannot toggle unless this is zero.
static uint8_t toggle_blocked_countdown_m;
// True if the button was active on the previous tick.
static bool oldButtonPressed;
// Switch state toggled when user activates boost function.
// Marked volatile to allow safe lock-free read access from an ISR if necessary.
static volatile bool switch_toggle_state;
// True while waiting for poll after a boost request.
// Cleared after a poll which is presumed to notice the request.
static bool waitingForPollAfterBoostRequest;

// Get the switch toggle state.
// The hub should monitor this changing,
// taking the change as indication of a boost request.
// This is allowed to toggle only much slower than the hub should poll,
// thus ensuring that the hub doesn't miss a boost request.
// Safe to call from an ISR (though this would be unexpected).
bool getSwitchToggleStateCO() { return(switch_toggle_state); }

// Call this on even numbered seconds (with current time in seconds) to allow the CO UI to operate.
// Should never be skipped, so as to allow the UI to remain responsive.
//
// The boost button for the CO relay is BUTTON_MODE_L.
// This routine/UI cares about off-to-on active edges of the button, ie the moment of being pressed,
// at which it will:
//    * turn the user-visible LED solid red (for a while)
//    * flip the status flag providing it has been more than 30 minutes since the last one
//      (this 30 minutes being the time at which contact with the hub would be deemed lost if no comms)
//    * send an alert message immediately (with the usual 'likely-to-get-heard' loudness settings)
//      and possibly periodically until a new poll request comes in (as indicated by a call to setLEDsCO())
bool tickUICO(const uint_fast8_t sec)
  {
  // Deal with the countdown timers.
  if((0 == sec) && (toggle_blocked_countdown_m > 0)) { --toggle_blocked_countdown_m; }
  if(countDownLEDSforCO > 0) { --countDownLEDSforCO; }

  // Note whether the button is pressed on this tick.
  const bool buttonPressed = (LOW == fastDigitalRead(BUTTON_MODE_L));
  // Note whether the button has just been pressed.
  const bool buttonJustPressed = (buttonPressed && !oldButtonPressed);
  oldButtonPressed = buttonPressed;
  if(buttonJustPressed)
    {
    // Set the LED to solid red until up to the comms timeout.
    // When the hub poll the LEDs will be set to whatever the poll specifies.
    setLEDsCO(1, MIN_BOOST_INTERVAL_M*2, 3, false);
    // If not still counting down since the last switch-state toggle,
    // toggle it now,
    // and restart the count-down.
    if(0 == toggle_blocked_countdown_m)
        {
        switch_toggle_state = !switch_toggle_state;
        toggle_blocked_countdown_m = MIN_BOOST_INTERVAL_M;
        }
    // Set up to set alerts periodically until polled.
    // Has the effect of allow the hub to know when boost is being requested
    // even if it's not yet time to flip the toggle.
    waitingForPollAfterBoostRequest = true;
    // Send an alert message immediately,
    // AFTER adjusting all relevant state so as to avoid a race,
    // inviting the hub to poll this node ASAP and eg notice the toggle state.
    sendCC1AlertByRFM23B();
    // Do no further UI processing this tick.
    // Note the user interaction to the caller.
    return(true);
    }

  // All LEDs off when their count-down timer is/hits zero.
  if(0 == countDownLEDSforCO) { lcCO = 0; setLEDs(0); }
  // Else force 'correct' requested light colour and deal with any 'flash' state.
  else
    {
    setLEDs(lcCO);

    // Deal with flashing (non-solid) output here.
    // Do some friendly I/O polling while waiting!
    if(lfCO != 3)
      {
      // Make this the first flash.
      mediumPause();
      setLEDs(0); // End of first flash.
      pollIO(); // Poll while LEDs are off.
      if(2 == lfCO)
        {
        offPause();
        pollIO(); // Poll while LEDs are off.
        // Start the second flash.
        setLEDs(lcCO);
        mediumPause();
        setLEDs(0); // End of second flash.
        pollIO(); // Poll while LEDs are off.
        }
      }
    }

  // If still waiting for a poll after a boost request,
  // arrange to send extra alerts about once every two minutes,
  // randomly so as to minimise collisions with other regular traffic.
  if(waitingForPollAfterBoostRequest && (sec == (OTV0P2BASE::randRNG8() & 0x3e)))
    { sendCC1AlertByRFM23B(); }

  return(false); // No human interaction this tick...
  }

// Directly adjust LEDs.
// May be called from a message handler, so minimise blocking.
//   * light-colour         [0,3] bit flags 1==red 2==green (lc) 0 => stop everything
//   * light-on-time        [1,15] (0 not allowed) 30-450s in units of 30s (lt) ???
//   * light-flash          [1,3] (0 not allowed) 1==single 2==double 3==on (lf)
// If fromPollAndCmd is true then this was called from an incoming Poll/Cms message receipt.
// Not ISR- safe.
void setLEDsCO(const uint8_t lc, const uint8_t lt, const uint8_t lf, const bool fromPollAndCmd)
    {
    lcCO = lc;
    countDownLEDSforCO = (lt >= 17) ? 255 : lt * 15; // Units are 30s, ticks are 2s; overflow is avoided.
    lfCO = lf;
    setLEDs(lc); // Set correct colour immediately.
    if(3 != lf)
      {
      // Only a flash of some sort is requested,
      // so just flicker the LED(s),
      // then turn off again until proper flash handler.
      tinyPause();
      setLEDs(0);
      }
    // Assume that the hub will shortly know about any pending request.
    if(fromPollAndCmd) { waitingForPollAfterBoostRequest = false; }
    }
#endif

bool pollIO(const bool force)
  {
  static volatile uint8_t _pO_lastPoll;
  // Poll RX at most about every ~8ms.
  const uint8_t sct = OTV0P2BASE::getSubCycleTime();
  if(force || (sct != _pO_lastPoll))
    {
    _pO_lastPoll = sct;
    // Poll for inbound frames.
    // If RX is not interrupt-driven then
    // there will usually be little time to do this
    // before getting an RX overrun or dropped frame.
    PrimaryRadio.poll();
    }
  return(false);
  }

// Decode and handle inbound raw message (msg[-1] contains the count of bytes received).
// A message may contain trailing garbage at the end; the decoder/router should cope.
// The buffer may be reused when this returns,
// so a copy should be taken of anything that needs to be retained.
// If secure is true then this message arrived over an inherently secure channel.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This routine is NOT allowed to alter in any way the content of the buffer passed.
static void decodeAndHandleRawRXedMessage(Print *p, const bool secure, const uint8_t * const msg)
  {
  const uint8_t msglen = msg[-1];

  // TODO: consider extracting hash of all message data (good/bad) and injecting into entropy pool.
#if 0 && defined(DEBUG)
  OTRadioLink::printRXMsg(p, msg-1, msglen+1); // Print len+frame.
#endif

  if(msglen < 2) { return; } // Too short to be useful, so ignore.

//   // Length-first OpenTRV secureable-frame format...
//#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT) // && defined(ENABLE_FAST_FRAMED_CARRIER_SUPPORT)
//  if(decodeAndHandleOTSecureableFrame(p, secure, msg)) { return; }
//#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT

  const uint8_t firstByte = msg[0];

#ifdef ENABLE_FS20_ENCODING_SUPPORT
  switch(firstByte)
    {
    default: // Reject unrecognised leading type byte.
    case OTRadioLink::FTp2_NONE: // Reject zero-length with leading length byte.
      break;

#ifdef ALLOW_CC1_SUPPORT_HUB
    // Handle alert message (at hub).
    // Dump onto serial to be seen by the attached host.
    case OTRadioLink::FTp2_CC1Alert:
      {
      OTProtocolCC::CC1Alert a;
      a.OTProtocolCC::CC1Alert::decodeSimple(msg, msglen);
      // After decode instance should be valid and with correct (source) house code.
      if(a.isValid())
        {
        // Pass message to host to deal with as "! hc1 hc2" after prefix indicating relayed (CC1 alert) message.
        p->print(F("+CC1 ! ")); p->print(a.getHC1()); p->print(' '); p->println(a.getHC2());
        }
      return;
      }
#endif

#ifdef ALLOW_CC1_SUPPORT_HUB
    // Handle poll-response message (at hub).
    // Dump onto serial to be seen by the attached host.
    case OTRadioLink::FTp2_CC1PollResponse:
      {
      OTProtocolCC::CC1PollResponse a;
      a.OTProtocolCC::CC1PollResponse::decodeSimple(msg, msglen);
      // After decode instance should be valid and with correct (source) house code.
      if(a.isValid())
        {
        // Pass message to host to deal with as:
        //     * hc1 hc2 rh tp tr al s w sy
        // after prefix indicating relayed (CC1) message.
        // (Parameters in same order as make() factory method, see below.)
//   * House code (hc1, hc2) of valve controller that the poll/command is being sent to.
//   * relative-humidity    [0,50] 0-100 in 2% steps (rh)
//   * temperature-ds18b20  [0,199] 0.000-99.999C in 1/2 C steps, pipe temp (tp)
//   * temperature-opentrv  [0,199] 0.000-49.999C in 1/4 C steps, room temp (tr)
//   * ambient-light        [1,62] no units, dark to light (al)
//   * switch               [false,true] activation toggle, helps async poll detect intermittent use (s)
//   * window               [false,true] false=closed,true=open (w)
//   * syncing              [false,true] if true, (re)syncing to FHT8V (sy)
// Returns instance; check isValid().
//            static CC1PollResponse make(uint8_t hc1, uint8_t hc2,
//                                        uint8_t rh,
//                                        uint8_t tp, uint8_t tr,
//                                        uint8_t al,
//                                        bool s, bool w, bool sy);
        p->print(F("+CC1 * "));
            p->print(a.getHC1()); p->print(' '); p->print(a.getHC2()); p->print(' ');
            p->print(a.getRH()); p->print(' ');
            p->print(a.getTP()); p->print(' '); p->print(a.getTR()); p->print(' ');
            p->print(a.getAL()); p->print(' ');
            p->print(a.getS()); p->print(' '); p->print(a.getW()); p->print(' ');
               p->println(a.getSY());
        }
      return;
      }
#endif

#ifdef ALLOW_CC1_SUPPORT_RELAY
    // Handle poll/cmd message (at relay).
    // IFF this message is addressed to this (target) unit's house code
    // then action the commands and respond (quickly) with a poll response.
    case OTRadioLink::FTp2_CC1PollAndCmd:
      {
      OTProtocolCC::CC1PollAndCommand c;
      c.OTProtocolCC::CC1PollAndCommand::decodeSimple(msg, msglen);
      // After decode instance should be valid and with correct house code.
      if(c.isValid())
        {
//        p->print(F("+CC1 * ")); p->print(a.getHC1()); p->print(' '); p->println(a.getHC2());
        // Process the message only if it is targetted at this node.
        const uint8_t hc1 = FHT8VGetHC1();
        const uint8_t hc2 = FHT8VGetHC2();
        if((c.getHC1() == hc1) && (c.getHC2() == hc2))
          {
          // Act on the incoming command.
          // Set LEDs.
          setLEDsCO(c.getLC(), c.getLT(), c.getLF(), true);
          // Set radiator valve position.
          NominalRadValve.set(c.getRP());

          // Respond to the hub with sensor data.
          // Can use read() for very freshest values at risk of some delay/cost.
#ifdef HUMIDITY_SENSOR_SUPPORT
          const uint8_t rh = RelHumidity.read() >> 1; // Scale from [0,100] to [0,50] for TX.
#else
          const uint8_t rh = 0; // RH% not available.
#endif
          const uint8_t tp = (uint8_t) constrain(extDS18B20_0.read() >> 3, 0, 199); // Scale to to 1/2C [0,100[ for TX.
          const uint8_t tr = (uint8_t) constrain(TemperatureC16.read() >> 2, 0, 199); // Scale from 1/16C to 1/4C [0,50[ for TX.
          const uint8_t al = AmbLight.read() >> 2; // Scale from [0,255] to [1,62] for TX (allow value coercion at extremes).
          const bool s = getSwitchToggleStateCO();
          const bool w = (fastDigitalRead(BUTTON_LEARN2_L) != LOW); // BUTTON_LEARN2_L high means open circuit means door/window open.
          const bool sy = !NominalRadValve.isInNormalRunState(); // Assume only non-normal FHT8V state is 'syncing'.
          OTProtocolCC::CC1PollResponse r =
              OTProtocolCC::CC1PollResponse::make(hc1, hc2, rh, tp, tr, al, s, w, sy);
          // Send message back to hub.
          // Hub can poll again if it does not see the response.
          // TODO: may need to insert a delay to allow hub to be ready if use of read() above is not enough.
          uint8_t txbuf[OTProtocolCC::CC1PollResponse::primary_frame_bytes+1]; // More than large enough for preamble + sync + alert message.
          const uint8_t bodylen = r.encodeSimple(txbuf, sizeof(txbuf), true);
#if 0 && defined(DEBUG)
OTRadioLink::printRXMsg(p, txbuf, bodylen);
#endif
          if(PrimaryRadio.sendRaw(txbuf, bodylen)) // Send at default volume...  One going missing won't hurt that much.
            {
#if 1 && defined(DEBUG)
            p->println(F("polled")); // Done it!
#endif
            }
          }
        }
      return;
      }
#endif // ALLOW_CC1_SUPPORT_RELAY
    }
#endif // ENABLE_FS20_ENCODING_SUPPORT

  // Unparseable frame: drop it; possibly log it as an error.
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
  p->print(F("!RX bad msg, len+prefix: ")); OTRadioLink::printRXMsg(p, msg-1, min(msglen+1, 8));
#endif
  return;
  }

// Incrementally process I/O and queued messages, including from the radio link.
// This may mean printing them to Serial (which the passed Print object usually is),
// or adjusting system parameters,
// or relaying them elsewhere, for example.
// This will write any output to the supplied Print object,
// typically the Serial output (which must be running if so).
// This will attempt to process messages in such a way
// as to avoid internal overflows or other resource exhaustion,
// which may mean deferring work at certain times
// such as the end of minor cycle.
// The Print object pointer must not be NULL.
bool handleQueuedMessages(Print *p, bool wakeSerialIfNeeded, OTRadioLink::OTRadioLink *rl)
  {
  // Avoid starting any potentially-slow processing very late in the minor cycle.
  // This is to reduce the risk of loop overruns
  // at the risk of delaying some processing
  // or even dropping some incoming messages if queues fill up.
  // Decoding (and printing to serial) a secure 'O' frame takes ~60 ticks (~0.47s).
  // Allow for up to 0.5s of such processing worst-case,
  // ie don't start processing anything later that 0.5s before the minor cycle end.
  const uint8_t sctStart = OTV0P2BASE::getSubCycleTime();
  if(sctStart >= ((OTV0P2BASE::GSCT_MAX/4)*3)) { return(false); }

  // Deal with any I/O that is queued.
  bool workDone = pollIO(true);

  // Check for activity on the radio link.
  rl->poll();

  bool neededWaking = false; // Set true once this routine wakes Serial.
  const volatile uint8_t *pb;
  if(NULL != (pb = rl->peekRXMsg()))
    {
    if(!neededWaking && wakeSerialIfNeeded && OTV0P2BASE::powerUpSerialIfDisabled<V0P2_UART_BAUD>()) { neededWaking = true; } // FIXME
    // Don't currently regard anything arriving over the air as 'secure'.
    // FIXME: shouldn't have to cast away volatile to process the message content.
    decodeAndHandleRawRXedMessage(p, false, (const uint8_t *)pb);
    rl->removeRXMsg();
    // Note that some work has been done.
    workDone = true;
    }

  // Turn off serial at end, if this routine woke it.
  if(neededWaking) { OTV0P2BASE::flushSerialProductive(); OTV0P2BASE::powerDownSerial(); }

  return(workDone);
  }

// Returns true if continuous background RX has been set up.
static bool setUpContinuousRX()
  {
  // Possible paranoia...
  // Periodically (every few hours) force radio off or at least to be not listening.
  if((30 == TIME_LSD) && (128 == minuteCount)) { PrimaryRadio.listen(false); }

  const bool needsToListen = true; // By default listen if always doing RX.

  // Act on eavesdropping need, setting up or clearing down hooks as required.
  PrimaryRadio.listen(needsToListen);

  if(needsToListen)
    {
#if 1 && defined(DEBUG) && defined(ENABLE_RADIO_RX) && !defined(ENABLE_TRIMMED_MEMORY)
    for(uint8_t lastErr; 0 != (lastErr = PrimaryRadio.getRXErr()); )
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING("!RX err ");
      DEBUG_SERIAL_PRINT(lastErr);
      DEBUG_SERIAL_PRINTLN();
      }
    const uint8_t dropped = PrimaryRadio.getRXMsgsDroppedRecent();
    static uint8_t oldDropped;
    if(dropped != oldDropped)
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING("!RX DROP ");
      DEBUG_SERIAL_PRINT(dropped);
      DEBUG_SERIAL_PRINTLN();
      oldDropped = dropped;
      }
#endif
#if 0 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
    // Filtered out messages are not an error.
    const uint8_t filtered = PrimaryRadio.getRXMsgsFilteredRecent();
    static uint8_t oldFiltered;
    if(filtered != oldFiltered)
      {
      DEBUG_SERIAL_PRINT_FLASHSTRING("RX filtered ");
      DEBUG_SERIAL_PRINT(filtered);
      DEBUG_SERIAL_PRINTLN();
      oldFiltered = filtered;
      }
#endif
    }
  return(needsToListen);
  }


// Mask for Port B input change interrupts.
#define MASK_PB_BASIC 0b00000000 // Nothing.
#if defined(PIN_RFM_NIRQ) && defined(ENABLE_RADIO_RX) // RFM23B IRQ only used for RX.
  #if (PIN_RFM_NIRQ < 8) || (PIN_RFM_NIRQ > 15)
    #error PIN_RFM_NIRQ expected to be on port B
  #endif
  #define RFM23B_INT_MASK (1 << (PIN_RFM_NIRQ&7))
  #define MASK_PB (MASK_PB_BASIC | RFM23B_INT_MASK)
#else
  #define MASK_PB MASK_PB_BASIC
#endif

// Mask for Port C input change interrupts.
#define MASK_PC_BASIC 0b00000000 // Nothing.

// Mask for Port D input change interrupts.
#define MASK_PD_BASIC 0b00000001 // Serial RX by default.
#if defined(ENABLE_VOICE_SENSOR)
  #if VOICE_NIRQ > 7
    #error VOICE_NIRQ expected to be on port D
  #endif
  #define VOICE_INT_MASK (1 << (VOICE_NIRQ&7))
  #define MASK_PD1 (MASK_PD_BASIC | VOICE_INT_MASK)
#else
  #define MASK_PD1 MASK_PD_BASIC // Just serial RX, no voice.
#endif
#if defined(ENABLE_SIMPLIFIED_MODE_BAKE)
#if BUTTON_MODE_L > 7
  #error BUTTON_MODE_L expected to be on port D
#endif
  #define MODE_INT_MASK (1 << (BUTTON_MODE_L&7))
  #define MASK_PD (MASK_PD1 | MODE_INT_MASK) // MODE button interrupt (et al).
#else
  #define MASK_PD MASK_PD1 // No MODE button interrupt.
#endif

#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
// Previous state of port B pins to help detect changes.
static volatile uint8_t prevStatePB;
// Interrupt service routine for PB I/O port transition changes.
ISR(PCINT0_vect)
  {
//  ++intCountPB;
  const uint8_t pins = PINB;
  const uint8_t changes = pins ^ prevStatePB;
  prevStatePB = pins;

#if defined(RFM23B_INT_MASK)
  // RFM23B nIRQ falling edge is of interest.
  // Handler routine not required/expected to 'clear' this interrupt.
  // TODO: try to ensure that OTRFM23BLink.handleInterruptSimple() is inlineable to minimise ISR prologue/epilogue time and space.
  if((changes & RFM23B_INT_MASK) && !(pins & RFM23B_INT_MASK))
    { PrimaryRadio.handleInterruptSimple(); }
#endif
  }
#endif

#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
// Previous state of port C pins to help detect changes.
static volatile uint8_t prevStatePC;
// Interrupt service routine for PC I/O port transition changes.
ISR(PCINT1_vect)
  {
//  const uint8_t pins = PINC;
//  const uint8_t changes = pins ^ prevStatePC;
//  prevStatePC = pins;
//
// ...
  }
#endif

#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
// Previous state of port D pins to help detect changes.
static volatile uint8_t prevStatePD;
// Interrupt service routine for PD I/O port transition changes (including RX).
ISR(PCINT2_vect)
  {
  const uint8_t pins = PIND;
  const uint8_t changes = pins ^ prevStatePD;
  prevStatePD = pins;

// FIXME FIXME FIXME
//  // If an interrupt arrived from no other masked source then wake the CLI.
//  // The will ensure that the CLI is active, eg from RX activity,
//  // eg it is possible to wake the CLI subsystem with an extra CR or LF.
//  // It is OK to trigger this from other things such as button presses.
//  // FIXME: ensure that resetCLIActiveTimer() is inlineable to minimise ISR prologue/epilogue time and space.
//  if(!(changes & MASK_PD & ~1)) { resetCLIActiveTimer(); }
  }
#endif

static const uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
static const int8_t RFM23B_IRQ_PIN = PIN_RFM_NIRQ;
static const bool RFM23B_allowRX = true;
OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> RFM23B;
// Assigns radio to PrimaryRadio alias.
OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B;

#if defined(ALLOW_CC1_SUPPORT_RELAY)
// For a CC1 relay, ignore everything except FTp2_CC1PollAndCmd messages.
// With care (not accessing EEPROM for example) this could also reject anything with wrong house code.
static bool FilterRXISR(const volatile uint8_t *buf, volatile uint8_t &buflen)
  {
  if((buflen < 8) || (OTRadioLink::FTp2_CC1PollAndCmd != buf[0])) { return(false); }
  buflen = 8; // Truncate message to correct size for efficiency.
  return(true); // Accept message.
  }
#elif defined(ALLOW_CC1_SUPPORT_HUB)
// For a CC1 hub, ignore everything except FTp2_CC1Alert and FTp2_CC1PollResponse messages.
static bool FilterRXISR(const volatile uint8_t *buf, volatile uint8_t &buflen)
  {
  if(buflen < 8) { return(false); }
  const uint8_t t = buf[0];
  if((OTRadioLink::FTp2_CC1Alert != t) && (OTRadioLink::FTp2_CC1PollResponse != t)) { return(false); }
  buflen = 8; // Truncate message to correct size for efficiency.
  return(true); // Accept message.
  }
#endif

#if defined(ALLOW_CC1_SUPPORT)
// COHEAT: REV2/REV9 talking on fast GFSK channel 0, REV9 TX to FHT8V on slow OOK.
#define RADIO_CONFIG_NAME "COHEAT DUAL CHANNEL"
static const uint8_t nPrimaryRadioChannels = 2;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] =
  {
  // GFSK channel 0 full config, RX/TX, not in itself secure.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true),
  // FS20/FHT8V compatible channel 1 full config, used for TX only, not secure, unframed.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsOOK5000, true, false, true, false, false, true),
  };
#elif defined(ENABLE_FAST_FRAMED_CARRIER_SUPPORT)
#define RADIO_CONFIG_NAME "GFSK"
// Nodes talking on fast GFSK channel 0.
static const uint8_t nPrimaryRadioChannels = 1;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] =
  {
  // GFSK channel 0 full config, RX/TX, not in itself secure.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true),
  };
#endif

// One-off setup.
void setup()
  {
  // Set appropriate low-power states, interrupts, etc, ASAP.
  OTV0P2BASE::powerSetup();
  // IO setup for safety, and to avoid pins floating.
  OTV0P2BASE::IOSetup();
  // Restore previous RTC state if available.
  OTV0P2BASE::restoreRTC();
#if defined(LED_UI2_EXISTS) && defined(ENABLE_UI_LED_2_IF_AVAILABLE)
  LED_UI2_ON();
#endif
  OTV0P2BASE::serialPrintAndFlush(F("\r\nOpenTRV: ")); // Leading CRLF to clear leading junk, eg from bootloader.
    serialPrintlnBuildVersion();
#if defined(LED_UI2_EXISTS) && defined(ENABLE_UI_LED_2_IF_AVAILABLE)
  OTV0P2BASE::nap(WDTO_120MS); // Sleep to let UI2 LED be seen.
  LED_UI2_OFF();
#endif

  // Count resets to detect unexpected crashes/restarts.
  const uint8_t oldResetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
  eeprom_write_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT, 1 + oldResetCount);

#if defined(DEBUG) && !defined(ENABLE_MIN_ENERGY_BOOT)
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("DEBUG");
#endif

  DEBUG_SERIAL_PRINT_FLASHSTRING("Resets: ");
  DEBUG_SERIAL_PRINT(oldResetCount);
  DEBUG_SERIAL_PRINTLN();
#if !defined(ALT_MAIN_LOOP) && !defined(UNIT_TESTS)
  const uint8_t overruns = (~eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_OVERRUN_COUNTER)) & 0xff;
  if(0 != overruns)
    {
    DEBUG_SERIAL_PRINT_FLASHSTRING("Overruns: ");
    DEBUG_SERIAL_PRINT(overruns);
    DEBUG_SERIAL_PRINTLN();
    }
#endif

  // Have 32678Hz clock at least running before going any further.
#if defined(ENABLE_WAKEUP_32768HZ_XTAL)
  if(!::OTV0P2BASE::HWTEST::check32768HzOsc()) { panic(F("xtal")); } // Async clock not running correctly.
#else
  DEBUG_SERIAL_PRINTLN_FLASHSTRING("(No xtal.)");
#endif

//  // Signal that xtal is running AND give it time to settle.
//  posPOST(0 /*, F("about to test radio module") */);

  // Initialise the radio, if configured, ASAP because it can suck a lot of power until properly initialised.
  PrimaryRadio.preinit(NULL);
  // Check that the radio is correctly connected; panic if not...
  if(!PrimaryRadio.configure(nPrimaryRadioChannels, RFM23BConfigs) || !PrimaryRadio.begin()) { panic(F("r1")); }
  // Apply filtering, if any, while we're having fun...
#ifndef NO_RX_FILTER
  PrimaryRadio.setFilterRXISR(FilterRXISR);
#endif // NO_RX_FILTER

//  posPOST(1, F("Radio OK, checking buttons/sensors and xtal"));

// FIXME FIXME FIXME
//  // Collect full set of environmental values before entering loop() in normal mode.
//  // This should also help ensure that sensors are properly initialised.
//  const int heat = TemperatureC16.read();
//#if defined(ENABLE_AMBLIGHT_SENSOR)
//  const int light = AmbLight.read();
//#endif
//#if defined(HUMIDITY_SENSOR_SUPPORT)
//  const uint8_t rh = RelHumidity.read();
//#endif

  // Seed RNGs, after having gathered some sensor values in RAM...
  OTV0P2BASE::seedPRNGs();

//#if defined(ENABLE_NOMINAL_RAD_VALVE)
//  // Update targets, output to TRV and boiler, etc, to be sensible before main loop starts.
//  NominalRadValve.read();
//#endif

  // Ensure that the unique node ID is set up (mainly on first use).
  // Have one attempt (don't want to stress an already failing EEPROM) to force-reset if not good, then panic.
  // Needs to have had entropy gathered, etc.
  if(!OTV0P2BASE::ensureIDCreated())
    {
    if(!OTV0P2BASE::ensureIDCreated(true)) // Force reset.
      { panic(F("ID")); }
    }

  // Initialised: turn main/heatcall UI LED off.
  LED_HEATCALL_OFF();

  // Help user get to CLI.
  OTV0P2BASE::serialPrintlnAndFlush(F("At CLI > prompt enter ? for help"));

// FIXME FIXME FIXME
//  // Report initial status.
//  serialStatusReport();

  // Radio not listening to start with.
  // Ignore any initial spurious RX interrupts for example.
  PrimaryRadio.listen(false);

  // Set up async edge interrupts.
  ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
    {
    //PCMSK0 = PB; PCINT  0--7    (Radio)
    //PCMSK1 = PC; PCINT  8--15
    //PCMSK2 = PD; PCINT 16--24   (Serial RX)

    PCICR =
#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
        1 | // 0x1 enables PB/PCMSK0.
#endif
#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
        2 | // 0x2 enables PC/PCMSK1.
#endif
#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
        4 | // 0x4 enables PD/PCMSK2.
#endif
        0;

#if defined(MASK_PB) && (MASK_PB != 0) // If PB interrupts required.
    PCMSK0 = MASK_PB;
#endif
#if defined(MASK_PC) && (MASK_PC != 0) // If PC interrupts required.
    PCMSK1 = MASK_PC;
#endif
#if defined(MASK_PD) && (MASK_PD != 0) // If PD interrupts required.
    PCMSK2 = MASK_PD;
#endif
    }

  // Start local counters in randomised positions to help avoid inter-unit collisions,
  // eg for mains-powered units starting up together after a power cut,
  // but without (eg) breaking any of the logic about what order things will be run first time through.
  // Uses some decent noise to try to start the units separated.
  const uint8_t b = OTV0P2BASE::getSecureRandomByte(); // randRNG8();
  // Start within bottom half of minute (or close to); sensor readings happen in second half.
  OTV0P2BASE::setSeconds(b >> 2);
  // Start anywhere in first 4 minute cycle.
  minuteCount = b & 3;

#ifdef ENABLE_FHT8VSIMPLE
  // Set up radio with FHT8V.
  FHT8V.setRadio(&PrimaryRadio);
  // Load EEPROM house codes into primary FHT8V instance at start.
  FHT8V.nvLoadHC();
#ifdef ALLOW_CC1_SUPPORT
  FHT8V.setChannelTX(1);
#endif // ALLOW_CC1_SUPPORT
#endif // ENABLE_FHT8VSIMPLE

  // Set appropriate loop() values just before entering it.
  TIME_LSD = OTV0P2BASE::getSecondsLT();
  }


// Main code here, loops every 2s.
void loop()
  {
  const bool needsToListen = setUpContinuousRX();

  OTV0P2BASE::powerDownSerial(); // Ensure that serial I/O is off.
  // Power down most stuff (except radio for hub RX).
  OTV0P2BASE::minimisePowerWithoutSleep();
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = OTV0P2BASE::getSecondsLT()))
    {
    // Poll I/O and process message incrementally (in this otherwise idle time)
    // before sleep and on wakeup in case some IO needs further processing now,
    // eg work was accrued during the previous major slow/outer loop
    // or the in a previous orbit of this loop sleep or nap was terminated by an I/O interrupt.
    // Come back and have another go if work was done, until the next tick at most.
    if(handleQueuedMessages(&Serial, true, &PrimaryRadio)) { continue; }

    // Normal long minimal-power sleep until wake-up interrupt.
    // Rely on interrupt to force fall through to I/O poll() below.
    OTV0P2BASE::sleepUntilInt();
    }
  TIME_LSD = newTLSD;

  // Use the zeroth second in each minute to force extra deep device sleeps/resets, etc.
  const bool second0 = (0 == TIME_LSD);
  
#ifdef ALLOW_CC1_SUPPORT_RELAY_IO // REV9 CC1 relay...
    // Run the CC1 relay UI.
    if(tickUICO(TIME_LSD))
      {
//      showStatus = true;
      }
#endif

  }

