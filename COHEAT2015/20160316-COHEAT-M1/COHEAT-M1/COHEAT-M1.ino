// Uncomment exactly one of the following CONFIG_ lines to select which board is being built for.
#define CONFIG_Trial2013Winter_Round2_CC1HUB // REV2 cut4 as CC1 hub.
//#define CONFIG_REV9 // REV9 as CC1 relay, cut 2 of the board.

//#ifndef BAUD
//#define BAUD 4800 // Ensure that OpenTRV 'standard' UART speed is set unless explicitly overridden.
//#endif

#include <Arduino.h>
#include <Wire.h>
#include <OTV0p2Base.h>
#include <OTRadioLink.h>
#include <OTRFM23BLink.h>
#include <OTRadValve.h>
#include <OTProtocolCC.h>
#include <OTV0p2_CONFIG_REV2.h>
#include <OTV0p2_CONFIG_REV9.h>
//#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT) || defined(ENABLE_SECURE_RADIO_BEACON)
//#include <OTAESGCM.h>
//#endif
#include <OTV0p2_Board_IO_Config.h> // I/O pin allocation and setup: include ahead of I/O module headers.

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






// Controller's view of Least Significant Digits of the current (local) time, in this case whole seconds.
// See PICAXE V0.1/V0.09/DHD201302L0 code.
#define TIME_LSD_IS_BINARY // TIME_LSD is in binary (cf BCD).
#define TIME_CYCLE_S 60 // TIME_LSD ranges from 0 to TIME_CYCLE_S-1, also major cycle length.
static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.

// 'Elapsed minutes' count of minute/major cycles; cheaper than accessing RTC and not tied to real time.
// Starts at or just above zero (within the first 4-minute cycle) to help avoid collisions between units after mass power-up.
// Wraps at its maximum (0xff) value.
static uint8_t minuteCount;

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
    //PCMSK0 = PB; PCINT  0--7    (LEARN1 and Radio)
    //PCMSK1 = PC; PCINT  8--15
    //PCMSK2 = PD; PCINT 16--24   (Serial RX and LEARN2 and MODE and Voice)

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

  // Set appropriate loop() values just before entering it.
  TIME_LSD = OTV0P2BASE::getSecondsLT();
  }

void loop() {
  // put your main code here, to run repeatedly:

}











