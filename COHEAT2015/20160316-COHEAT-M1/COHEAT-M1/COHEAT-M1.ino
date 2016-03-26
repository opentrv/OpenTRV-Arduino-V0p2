// Uncomment exactly one of the following CONFIG_... lines to select which board is being built for.
//#define CONFIG_Trial2013Winter_Round2_CC1HUB // REV2 cut4 as CC1 hub.
#define CONFIG_REV9 // REV9 as CC1 relay, cut 2 of the board.

// IF DEFINED: entire comms model switches to secure.
#define ENABLE_OTSECUREFRAME_ENCODING_SUPPORT

#define DEBUG // Uncomment for debug output.

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
// Force-enable always-on RX if not already so.
#define ENABLE_RADIO_RX
#define ENABLE_CONTINUOUS_RX

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
bool pollIO(bool force = false);

// Sends a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
void serialStatusReport();

// Reset CLI active timer to the full whack before it goes inactive again (ie makes CLI active for a while).
// Thread-safe.
void resetCLIActiveTimer();
// Returns true if the CLI is (or should currently be) active, at least intermittently.
// Thread-safe.
bool isCLIActive();
// Used to poll user side for CLI input until specified sub-cycle time.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// Times itself out after at least a minute or two of inactivity. 
// NOT RENTRANT (eg uses static state for speed and code space).
void pollCLI(uint8_t maxSCT, bool startOfMinute);

const uint8_t nearOverrunThreshold = OTV0P2BASE::GSCT_MAX - 8; // ~64ms/~32 serial TX chars of grace time...

// Primary radio module.
extern OTRadioLink::OTRadioLink &PrimaryRadio;

// Sense (usually non-linearly) over full likely internal ambient lighting range of a (UK) home,
// down to levels too dark to be active in (and at which heating could be set back for example).
#ifdef ENABLE_AMBLIGHT_SENSOR
// Sensor for ambient light level; 0 is dark, 255 is bright.
typedef OTV0P2BASE::SensorAmbientLight AmbientLight;
#else // !defined(ENABLE_AMBLIGHT_SENSOR)
typedef OTV0P2BASE::DummySensorAmbientLight AmbientLight; // Dummy stand-in.
#endif // ENABLE_AMBLIGHT_SENSOR
// Singleton implementation/instance.
extern AmbientLight AmbLight;

// Create very light-weight standard-speed OneWire(TM) support if a pin has been allocated to it.
// Meant to be similar to use to OneWire library V2.2.
// Supports search but not necessarily CRC.
// Designed to work with 1MHz/1MIPS CPU clock.
#if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
#define SUPPORTS_MINIMAL_ONEWIRE
extern OTV0P2BASE::MinimalOneWire<> MinOW_DEFAULT_OWDQ;
#endif

// Cannot have internal and external use of same DS18B20 at same time...
#if defined(ENABLE_EXTERNAL_TEMP_SENSOR_DS18B20) && !defined(ENABLE_PRIMARY_TEMP_SENSOR_DS18B20) && defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
#define SENSOR_EXTERNAL_DS18B20_ENABLE_0 // Enable sensor zero.
extern OTV0P2BASE::TemperatureC16_DS18B20 extDS18B20_0;
#endif

// Ambient/room temperature sensor, usually on main board.
#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
extern OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.
#elif defined(ENABLE_PRIMARY_TEMP_SENSOR_DS18B20)
  #if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
  // DSB18B20 temperature impl, with slightly reduced precision to improve speed.
  extern OTV0P2BASE::TemperatureC16_DS18B20 TemperatureC16;
  #endif // defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
#else // Don't use TMP112 if SHT21 or DS18B20 have been selected.
extern OTV0P2BASE::RoomTemperatureC16_TMP112 TemperatureC16;
#endif

// HUMIDITY_SENSOR_SUPPORT is defined if at least one humidity sensor has support compiled in.
// Simple implementations can assume that the sensor will be present if defined;
// more sophisticated implementations may wish to make run-time checks.
// If SHT21 support is enabled at compile-time then its humidity sensor may be used at run-time.
#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
#define HUMIDITY_SENSOR_SUPPORT // Humidity sensing available.
#endif

#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
// Singleton implementation/instance.
extern OTV0P2BASE::HumiditySensorSHT21 RelHumidity;
#else
// Dummy implementation to minimise coding changes.
extern OTV0P2BASE::DummyHumiditySensorSHT21 RelHumidity;
#endif

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
static void inline offPause() { bigPause(); pollIO(); }

//---------------------

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

//---------------------

// Returns true if there is time to andle at least one message inbound our outbound.
// Includes time required to encrypt/decrypt/print a message if need be (~0.5s at 1MHz CPU).
static bool timeToHandleMessage()
  {
  const uint8_t sct = OTV0P2BASE::getSubCycleTime();
  return(sct < min((OTV0P2BASE::GSCT_MAX/4)*3, nearOverrunThreshold - 1)); 
  }

// Controller's view of Least Significant Digits of the current (local) time, in this case whole seconds.
#define TIME_CYCLE_S 60 // TIME_LSD ranges from 0 to TIME_CYCLE_S-1, also major cycle length.
static uint_fast8_t TIME_LSD; // Controller's notion of seconds within major cycle.

// 'Elapsed minutes' count of minute/major cycles; cheaper than accessing RTC and not tied to real time.
// Starts at or just above zero (within the first 4-minute cycle) to help avoid collisions between units after mass power-up.
// Wraps at its maximum (0xff) value.
static uint8_t minuteCount;

// Indicate that the system is broken in an obvious way (distress flashing the main LED).
// DOES NOT RETURN.
// Tries to turn off most stuff safely that will benefit from doing so, but nothing too complex.
// Tries not to use lots of energy so as to keep distress beacon running for a while.
void panic()
  {
  // Reset radio and go into low-power mode.
  PrimaryRadio.panicShutdown();
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


#ifdef ENABLE_AMBLIGHT_SENSOR
// Normal 2 bit shift between raw and externally-presented values.
static const uint8_t shiftRawScaleTo8Bit = 2;
// This implementation expects a phototransitor TEPT4400 (50nA dark current, nominal 200uA@100lx@Vce=50V) from IO_POWER_UP to LDR_SENSOR_AIN and 220k to ground.
static const int LDR_THR_LOW = 270U;
static const int LDR_THR_HIGH = 400U;
// Singleton implementation/instance.
AmbientLight AmbLight(LDR_THR_HIGH >> shiftRawScaleTo8Bit);
#endif // ENABLE_AMBLIGHT_SENSOR

#if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
OTV0P2BASE::MinimalOneWire<> MinOW_DEFAULT;
#endif

#if defined(SENSOR_EXTERNAL_DS18B20_ENABLE_0) // Enable sensor zero.
OTV0P2BASE::TemperatureC16_DS18B20 extDS18B20_0(MinOW_DEFAULT, 0);
#endif

#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
// Singleton implementation/instance.
OTV0P2BASE::HumiditySensorSHT21 RelHumidity;
#else
OTV0P2BASE::DummyHumiditySensorSHT21 RelHumidity;
#endif

// Ambient/room temperature sensor, usually on main board.
#if defined(ENABLE_PRIMARY_TEMP_SENSOR_SHT21)
OTV0P2BASE::RoomTemperatureC16_SHT21 TemperatureC16; // SHT21 impl.
#elif defined(ENABLE_PRIMARY_TEMP_SENSOR_DS18B20)
#if defined(ENABLE_MINIMAL_ONEWIRE_SUPPORT)
// DSB18B20 temperature impl, with slightly reduced precision to improve speed.
OTV0P2BASE::TemperatureC16_DS18B20 TemperatureC16(MinOW_DEFAULT, 0, OTV0P2BASE::TemperatureC16_DS18B20::MAX_PRECISION - 1);
#endif
#else // Don't use TMP112 if SHT21 or DS18B20 are selected.
OTV0P2BASE::RoomTemperatureC16_TMP112 TemperatureC16;
#endif

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
 

#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
// TX 2 bytes of ID in each secure frame, corresponding to the FHT8V housecode.
static const uint8_t lenTXID = 2;
#if defined(ALLOW_CC1_SUPPORT_RELAY)
// Support for secure TX side using FHT8V ID plus 0x80 padding from REV9 relay.
// (High bits of trailing bytes should be high for traffic FROM node whose ID is in header.)
// ID is of length OTV0P2BASE::OpenTRV_Node_ID_Bytes.
static bool getTXID(uint8_t *const id)
  {
  id[0] = FHT8V.getHC1();
  id[1] = FHT8V.getHC2();
  memset(id+2, 0x80, OTV0P2BASE::OpenTRV_Node_ID_Bytes-2);
  return(true);
  }
OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2SuppliedID secureTXState(getTXID);
#else
// Support for secure TX side using FHT8V ID plus 0x80 padding from REV2 hub.
// (High bits of trailing bytes should be low for traffic TO node whose ID is in header.)
OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2SuppliedID secureTXState(NULL);
// Set TX ID for target; two bytes of housecode padded with zeros (for reverse traffic).
static void setTXID(const uint8_t hc1, const uint8_t hc2)
  {
  uint8_t id[OTV0P2BASE::OpenTRV_Node_ID_Bytes];
  id[0] = hc1;
  id[1] = hc2;
  memset(id + 2, 0, OTV0P2BASE::OpenTRV_Node_ID_Bytes - 2);
  secureTXState.setTXID(id);
  }
#endif // defined(ALLOW_CC1_SUPPORT_RELAY)
#endif // defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)


#ifdef ALLOW_CC1_SUPPORT_RELAY
// Send a CC1 Alert message with this unit's house code; returns false on failure.
bool sendCC1Alert()
  {
#if !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
  OTProtocolCC::CC1Alert a = OTProtocolCC::CC1Alert::make(FHT8V.nvGetHC1(), FHT8V.nvGetHC2());
  if(a.isValid()) // Might be invalid if house codes are, eg if house codes not set.
    {
    uint8_t txbuf[OTProtocolCC::CC1Alert::primary_frame_bytes+1]; // More than large enough for preamble + sync + alert message.
    const uint8_t bodylen = a.encodeSimple(txbuf, sizeof(txbuf), true);
    // Send loud (high power and/or double) since the hub may be relatively far away,
    // there is no 'ACK', and these messages should not be sent very often.
    // Should be consistent with automatically-generated alerts to help with diagnosis.
    return(PrimaryRadio.sendRaw(txbuf, bodylen, 0, OTRadioLink::OTRadioLink::TXmax));
    }
#else
  uint8_t key[16];
  if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key))
    { OTV0P2BASE::serialPrintlnAndFlush(F("!TX key")); return(false); } // FAIL
  const OTRadioLink::SimpleSecureFrame32or0BodyTXBase::fixed32BTextSize12BNonce16BTagSimpleEnc_ptr_t e = OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleEnc_DEFAULT_STATELESS;
  uint8_t sbuf[OTRadioLink::SimpleSecureFrame32or0BodyTXBase::generateSecureBeaconMaxBufSize];
  const uint8_t sbodylen = secureTXState.generateSecureBeaconRawForTX(sbuf, sizeof(sbuf), lenTXID, e, NULL, key); // 2 byte ID.
  // ASSUME FRAMED CHANNEL 0 (but could check with config isUnframed flag).
  // When sending on a channel with framing, do not explicitly send the frame length byte.
  // DO NOT attempt to send if construction of the secure frame failed;
  // doing so may reuse IVs and destroy the cipher security.
  const bool success = (0 != sbodylen) && PrimaryRadio.sendRaw(sbuf+1, sbodylen-1);
#if 1 && defined(DEBUG)
  if(!success) { OTV0P2BASE::serialPrintlnAndFlush(F("!TX")); }
  else { OTV0P2BASE::serialPrintlnAndFlush(F("TX alert")); }
#endif
  if(success) { return(true); } // Done!
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
  // FAILED if fallen through to here.
  return(false);
  }
#endif // ALLOW_CC1_SUPPORT_RELAY

#ifdef ALLOW_CC1_SUPPORT_RELAY
// True if a poll response is needed.
// Cleared upon successful send.
static bool pollResponseNeeded;
// Send a CC1 poll response message with this unit's house code; returns false on failure.
bool sendCC1PollResponse()
  {
  // Respond to the hub with sensor data.
  // Can use read() for very freshest values at risk of some delay/cost.
  const uint8_t hc1 = FHT8V.nvGetHC1();
  const uint8_t hc2 = FHT8V.nvGetHC2();
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
#if 1 && defined(DEBUG)
  OTV0P2BASE::serialPrintlnAndFlush(F("polled"));
#endif
#if !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
  // Non-secure: send raw frame as-is.
  // Send at default power...  One going missing won't hurt that much.
  if(!PrimaryRadio.sendRaw(txbuf, bodylen))
    { OTV0P2BASE::serialPrintlnAndFlush(F("!TX fail")); return(false); } // FAIL
#else
  // Secure: wrap frame in encrypted layer...
  uint8_t key[16];
  if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key))
    { OTV0P2BASE::serialPrintlnAndFlush(F("!TX key")); return(false); } // FAIL
  const OTRadioLink::SimpleSecureFrame32or0BodyTXBase::fixed32BTextSize12BNonce16BTagSimpleEnc_ptr_t e = OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleEnc_DEFAULT_STATELESS;
  uint8_t sbuf[OTRadioLink::SecurableFrameHeader::maxSmallFrameSize];
  const uint8_t sbodylen = secureTXState.generateSecureOStyleFrameForTX(sbuf, sizeof(sbuf), OTRadioLink::FTS_RESERVED_A, lenTXID, txbuf, bodylen, e, NULL, key);
  const bool success = (0 != sbodylen) && PrimaryRadio.sendRaw(sbuf+1, sbodylen-1);
#if 1 && defined(DEBUG)
  if(!success) { OTV0P2BASE::serialPrintlnAndFlush(F("!TX A")); }
  else { OTV0P2BASE::serialPrintlnAndFlush(F("TX A")); }
#endif
  if(success)
    {
    // Note successful dispatch of response.
    pollResponseNeeded = false;
    return(true);
    }
#endif // !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
  // FAILED if fallen through to here.
  return(false);
  }
#endif // ALLOW_CC1_SUPPORT_RELAY


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
    sendCC1Alert();
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
  // no closer together than every about every 8 seconds,
  // randomly so as to minimise collisions with other regular traffic.
  if(waitingForPollAfterBoostRequest && (sec == (OTV0P2BASE::randRNG8() & 0x38)))
    { sendCC1Alert(); }

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

#if 0 && defined(DEBUG)
  OTRadioLink::printRXMsg(p, msg-1, msglen+1); // Print len+frame.
#endif

#if !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
   // For non-secure, check that there enough bytes for expected (fixed) frame size.
   if(msglen < 8) { return; } // Too short to be useful, so ignore.
   const uint8_t *cleartextBody = msg;
   const uint8_t cleartextBodyLen = msglen;
#else 
  // For length-first OpenTRV secureable-frame format, validate structure of header/frame first.
  // This is quick and checks for insane/dangerous values throughout.
  OTRadioLink::SecurableFrameHeader sfh;
  const uint8_t l = sfh.checkAndDecodeSmallFrameHeader(msg-1, msglen+1);
  if((0 == l) || !sfh.isSecure()) // Invalid header, or not in secure format.
    {
#if 1 && defined(DEBUG)
DEBUG_SERIAL_PRINTLN_FLASHSTRING("!RX bad secure header");
#endif
    return; // FAIL
    }
  uint8_t key[16];
  // Get the 'building' key.
  if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key))
    {
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("!RX key");
    return; // FAIL
    }
  // Attempt to authenticate (and decrypt) the frame before inspecting content.
  // Buffer for receiving secure frame body.
  // (Non-secure frame bodies should be read directly from the frame buffer.)
  uint8_t secBodyBuf[OTRadioLink::ENC_BODY_SMALL_FIXED_PTEXT_MAX_SIZE];
  // Body length after any decryption, etc.
  uint8_t decryptedBodyOutSize = 0;
  // Check that there is an association/key for the inbound message.
  uint8_t senderNodeID[OTV0P2BASE::OpenTRV_Node_ID_Bytes];
  // Look up full ID in associations table,
  // validate RX message counter,
  // authenticate and decrypt,
  // update RX message counter.
  const uint8_t dfl = OTRadioLink::SimpleSecureFrame32or0BodyRXV0p2::getInstance().decodeSecureSmallFrameSafely(&sfh, msg-1, msglen+1,
                                          OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleDec_DEFAULT_STATELESS,
                                          NULL, key,
                                          secBodyBuf, sizeof(secBodyBuf), decryptedBodyOutSize,
                                          senderNodeID,
                                          true);
  if(0 == dfl)
    {
#if 1 // && defined(DEBUG)
    // Useful brief network diagnostics: a couple of bytes of the claimed ID of rejected frames.
    // Warnings rather than errors because there may legitimately be multiple disjoint networks.
    OTV0P2BASE::serialPrintAndFlush(F("?RX auth")); // Missing association or failed auth.
    if(sfh.getIl() > 0) { OTV0P2BASE::serialPrintAndFlush(' '); OTV0P2BASE::serialPrintAndFlush(sfh.id[0], HEX); }
    if(sfh.getIl() > 1) { OTV0P2BASE::serialPrintAndFlush(' '); OTV0P2BASE::serialPrintAndFlush(sfh.id[1], HEX); }
    OTV0P2BASE::serialPrintlnAndFlush();
#endif
    return; // FAIL
    }
   const uint8_t *cleartextBody = secBodyBuf;
   const uint8_t cleartextBodyLen = decryptedBodyOutSize;
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT

  const uint8_t firstByte = msg[0];
  switch(firstByte)
    {
    default: // Reject unrecognised leading type byte.
    case OTRadioLink::FTp2_NONE: // Reject zero-length with leading length byte.
      break;

#if defined(ALLOW_CC1_SUPPORT_HUB)
    // Handle alert message (at hub).
    // Dump onto serial to be seen by the attached host.
#if !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
    // Non-secure.
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
      return; // OK
      }
#else
    // Secure.
    case 0x80 | OTRadioLink::FTp2_CC1Alert:
      {
      // Already authenticated and no body to decode.
      // Pass message to host to deal with as "! hc1 hc2" after prefix indicating relayed (CC1 alert) message.
      p->print(F("+CC1 ! ")); p->print(senderNodeID[0]); p->print(' '); p->println(senderNodeID[1]);
      return; // OK
      }
#endif // defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
#endif // defined(ALLOW_CC1_SUPPORT_HUB)

#ifdef ALLOW_CC1_SUPPORT_HUB
    // Handle poll-response message (at hub).
    // Dump onto serial to be seen by the attached host.
#if !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
    case OTRadioLink::FTp2_CC1PollResponse: // Non-secure.
#else
    case 0x80 | OTRadioLink::FTp2_CC1PollResponse: // Secure.
#endif // !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      {
      OTProtocolCC::CC1PollResponse a;
      a.OTProtocolCC::CC1PollResponse::decodeSimple(cleartextBody, cleartextBodyLen);
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
#if !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
    case OTRadioLink::FTp2_CC1PollAndCmd: // Non-secure.
#else
    case 0x80 | OTRadioLink::FTp2_CC1PollAndCmd: // Secure.
#endif // !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      {
      OTProtocolCC::CC1PollAndCommand c;
      c.OTProtocolCC::CC1PollAndCommand::decodeSimple(cleartextBody, cleartextBodyLen);
      // After decode instance should be valid and with correct house code.
      if(c.isValid())
        {
        // Process the message only if it is targetted at this node.
        const uint8_t hc1 = FHT8V.nvGetHC1();
        const uint8_t hc2 = FHT8V.nvGetHC2();
        if((c.getHC1() == hc1) && (c.getHC2() == hc2))
          {
          // Act on the incoming command.
          // Note that a poll response will be needed.
          pollResponseNeeded = true;
          // Set LEDs immediately.
          setLEDsCO(c.getLC(), c.getLT(), c.getLF(), true);
          // Set radiator valve position immediately.
          NominalRadValve.set(c.getRP());
          // If relatively early in the cycle then send the response immediately.
          if(timeToHandleMessage()) { sendCC1PollResponse(); }
          }
        }
      return;
      }
#endif // ALLOW_CC1_SUPPORT_RELAY
    }

  // Unparseable frame: drop it; possibly log it as an error.
#if 1 && defined(DEBUG) && !defined(ENABLE_TRIMMED_MEMORY)
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
  if(!timeToHandleMessage()) { return(false); }

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
#if 1 && defined(DEBUG) // && defined(ENABLE_RADIO_RX) && !defined(ENABLE_TRIMMED_MEMORY)
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

  // If an interrupt arrived from no other masked source then wake the CLI.
  // The will ensure that the CLI is active, eg from RX activity,
  // eg it is possible to wake the CLI subsystem with an extra CR or LF.
  // It is OK to trigger this from other things such as button presses.
  // FIXME: ensure that resetCLIActiveTimer() is inlineable to minimise ISR prologue/epilogue time and space.
  if(!(changes & MASK_PD & ~1)) { resetCLIActiveTimer(); }
  }
#endif

static const uint8_t RFM23B_RX_QUEUE_SIZE = OTRFM23BLink::DEFAULT_RFM23B_RX_QUEUE_CAPACITY;
static const int8_t RFM23B_IRQ_PIN = PIN_RFM_NIRQ;
static const bool RFM23B_allowRX = true;
OTRFM23BLink::OTRFM23BLink<PIN_SPI_nSS, RFM23B_IRQ_PIN, RFM23B_RX_QUEUE_SIZE, RFM23B_allowRX> RFM23B;
// Assigns radio to PrimaryRadio alias.
OTRadioLink::OTRadioLink &PrimaryRadio = RFM23B;

// Quickly screen/filter RX traffic to preserve queue space for stuff likely to be of interest.
// If in doubt, accept a frame, ie should not reject incorrectly.
#if defined(ALLOW_CC1_SUPPORT_RELAY)
// For a CC1 relay, ignore everything except FTp2_CC1PollAndCmd messages.
// With care (not accessing EEPROM for example) this can also reject anything with wrong house code.
static bool FilterRXISR(const volatile uint8_t *buf, volatile uint8_t &buflen)
  {
#ifndef ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
  if((buflen < 8) || (OTRadioLink::FTp2_CC1PollAndCmd != buf[0])) { return(false); }
  // Filter for only this unit address/housecode as FHT8V.getHC{1,2}() are thread-safe.
  if((FHT8V.getHC1() != buf[1]) || (FHT8V.getHC2() != buf[2])) { return(false); }
#else
  // Expect secure frame with 2-byte ID and 32-byte encrypted body.
  if((buflen < 60) || ((0x80|OTRadioLink::FTp2_CC1PollAndCmd) != buf[0])) { return(false); }
  // Filter for only this unit address/housecode as FHT8V.getHC{1,2}() are thread-safe.
  if((buf[1] & 0xf) < 2) { return(false); }
  if((FHT8V.getHC1() != buf[2]) || (FHT8V.getHC2() != buf[3])) { return(false); }
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
  return(true); // Accept message.
  }
#elif defined(ALLOW_CC1_SUPPORT_HUB)
// For a CC1 hub, ignore everything except FTp2_CC1Alert and FTp2_CC1PollResponse messages.
static bool FilterRXISR(const volatile uint8_t *buf, volatile uint8_t &buflen)
  {
#ifndef ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
  if(buflen < 8) { return(false); }
  const uint8_t t = buf[0];
  if((OTRadioLink::FTp2_CC1Alert != t) && (OTRadioLink::FTp2_CC1PollResponse != t)) { return(false); }
  // TODO: filter for only associated relay address/housecodes.
#else
  // Expect secure frame with 2-byte ID and 0- or 32-byte encrypted body.
  if(buflen < 28) { return(false); }
  const uint8_t t = buf[0];
  if(((0x80|OTRadioLink::FTp2_CC1Alert) != t) && ((0x80|OTRadioLink::FTp2_CC1PollResponse) != t)) { return(false); }
  // TODO: filter for only associated relay address/housecodes.
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT
  return(true); // Accept message.
  }
#endif

// COHEAT: REV2/REV9 talking on fast GFSK channel 0, REV9 TX to FHT8V on slow OOK.
static const uint8_t nPrimaryRadioChannels = 2;
static const OTRadioLink::OTRadioChannelConfig RFM23BConfigs[nPrimaryRadioChannels] =
  {
  // GFSK channel 0 full config, RX/TX, not in itself secure.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsGFSK57600, true),
  // FS20/FHT8V compatible channel 1 full config, used for TX only, not secure, unframed.
  OTRadioLink::OTRadioChannelConfig(OTRFM23BLink::StandardRegSettingsOOK5000, true, false, true, false, false, true),
  };

// Sends a short 1-line CRLF-terminated status report on the serial connection (at 'standard' baud).
void serialStatusReport()
  {
#if !defined(ENABLE_FHT8VSIMPLE)
  OTV0P2BASE::serialPrintlnAndFlush(F("="));
#else
  // Force sensor read as not polled in main loop for COHEAT... (And flush any serial before messing with clocks, etc.)
  OTV0P2BASE::flushSerialSCTSensitive();
  TemperatureC16.read();
  const bool neededWaking = OTV0P2BASE::powerUpSerialIfDisabled<V0P2_UART_BAUD>();
  // Stats line starts with distinguished marker character '='.
  Serial.print((char) OTV0P2BASE::SERLINE_START_CHAR_STATS);
  Serial.print(NominalRadValve.get()); Serial.print('%'); // Target valve position.
  const int temp = TemperatureC16.get();
  Serial.print('@'); Serial.print(temp >> 4); Serial.print('C'); // Unrounded whole degrees C.
      Serial.print(temp & 0xf, HEX); // Show 16ths in hex.
  // Print optional house code section if codes set.
  const uint8_t hc1 = FHT8V.nvGetHC1();
  if(hc1 != 255)
    {
    Serial.print(F(";HC"));
    Serial.print(hc1);
    Serial.print(' ');
    Serial.print(FHT8V.nvGetHC2());
    if(!FHT8V.isInNormalRunState())
      {
      Serial.print(' ');
      Serial.print('s'); // Indicate syncing with trailing lower-case 's' in field...
      }
    // Now show TX ID for secure association.
    Serial.print(F(" TX ID"));
    uint8_t idbuf[OTV0P2BASE::OpenTRV_Node_ID_Bytes];
    getTXID(idbuf);
    for(uint8_t i = 0; i < V0P2BASE_EE_LEN_ID; ++i)
      {
      Serial.print(' ');
      Serial.print(idbuf[i], HEX);
      }
    Serial.println();
    }
  // Terminate line.
  Serial.println();
  // Ensure that all text is sent before this routine returns, in case any sleep/powerdown follows that kills the UART.
  OTV0P2BASE::flushSerialSCTSensitive();
  if(neededWaking) { OTV0P2BASE::powerDownSerial(); }
#endif 
  }

// Handle CLI extension commands.
// Commands of form:
//   +EXT .....
// where EXT is the name of the extension, usually 3 letters.
//
// It is acceptable for extCLIHandler() to alter the buffer passed,
// eg with strtok_t().
static bool extCLIHandler(Print *const p, char *const buf, const uint8_t n)
  {

#ifdef ALLOW_CC1_SUPPORT_RELAY
  // If CC1 relay then allow +CC1 ! command to send an alert to the hub.
  // Full command is:
  //    +CC1 !
  // This unit's housecode is used in the frame sent.
  const uint8_t CC1_A_PREFIX_LEN = 6;
  // Falling through rather than return(true) indicates failure.
  if((n >= CC1_A_PREFIX_LEN) && (0 == strncmp("+CC1 !", buf, CC1_A_PREFIX_LEN)))
    {
    // Send the alert!
    return(sendCC1Alert());
    }
#endif

#ifdef ALLOW_CC1_SUPPORT_HUB
  // If CC1 hub then allow +CC1 ? command to poll a remote relay.
  // Full command is:
  //    +CC1 ? hc1 hc2 rp lc lt lf
  // ie six numeric arguments, see below, with out-of-range values coerced (other than housecodes):
//            // Factory method to create instance.
//            // Invalid parameters (except house codes) will be coerced into range.
//            //   * House code (hc1, hc2) of valve controller that the poll/command is being sent to.
//            //   * rad-open-percent     [0,100] 0-100 in 1% steps, percent open approx to set rad valve (rp)
//            //   * light-colour         [0,3] bit flags 1==red 2==green (lc) 0 => stop everything
//            //   * light-on-time        [1,15] (0 not allowed) 30-450s in units of 30s (lt) ???
//            //   * light-flash          [1,3] (0 not allowed) 1==single 2==double 3==on (lf)
//            // Returns instance; check isValid().
//            static CC1PollAndCommand make(uint8_t hc1, uint8_t hc2,
//                                          uint8_t rp,
//                                          uint8_t lc, uint8_t lt, uint8_t lf);
  const uint8_t CC1_Q_PREFIX_LEN = 7;
  const uint8_t CC1_Q_PARAMS = 6;
  // Falling through rather than return(true) indicates failure.
  if((n >= CC1_Q_PREFIX_LEN) && (0 == strncmp("+CC1 ? ", buf, CC1_Q_PREFIX_LEN)))
    {
    char *last; // Used by strtok_r().
    char *tok1;
    // Attempt to parse the parameters.
    if((n-CC1_Q_PREFIX_LEN >= CC1_Q_PARAMS*2-1) && (NULL != (tok1 = strtok_r(buf+CC1_Q_PREFIX_LEN, " ", &last))))
      {
      char *tok2 = strtok_r(NULL, " ", &last);
      char *tok3 = (NULL == tok2) ? NULL : strtok_r(NULL, " ", &last);
      char *tok4 = (NULL == tok3) ? NULL : strtok_r(NULL, " ", &last);
      char *tok5 = (NULL == tok4) ? NULL : strtok_r(NULL, " ", &last);
      char *tok6 = (NULL == tok5) ? NULL : strtok_r(NULL, " ", &last);
      if(NULL != tok6)
        {
        OTProtocolCC::CC1PollAndCommand q = OTProtocolCC::CC1PollAndCommand::make(
            atoi(tok1),
            atoi(tok2),
            atoi(tok3),
            atoi(tok4),
            atoi(tok5),
            atoi(tok6));
        if(q.isValid())
          {
          uint8_t txbuf[OTProtocolCC::CC1PollAndCommand::primary_frame_bytes+1]; // More than large enough for preamble + sync + alert message.
          const uint8_t bodylen = q.encodeSimple(txbuf, sizeof(txbuf), true);
#if !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
          // Non-secure: send raw frame as-is.
          // TX at normal power since ACKed and can be repeated if necessary.
          if(PrimaryRadio.sendRaw(txbuf, bodylen))
            { return(true); } // Success!
#else
          // Secure: wrap frame in encrypted layer...
          uint8_t key[16];
          if(!OTV0P2BASE::getPrimaryBuilding16ByteSecretKey(key))
            { OTV0P2BASE::serialPrintlnAndFlush(F("!TX key")); return(false); } // FAIL
          const OTRadioLink::SimpleSecureFrame32or0BodyTXBase::fixed32BTextSize12BNonce16BTagSimpleEnc_ptr_t e = OTAESGCM::fixed32BTextSize12BNonce16BTagSimpleEnc_DEFAULT_STATELESS;
          // Hub must indicate which relay it is sending (reverse traffic) to.
          setTXID(q.getHC1(), q.getHC2());
          uint8_t sbuf[OTRadioLink::SecurableFrameHeader::maxSmallFrameSize];
          const uint8_t sbodylen = secureTXState.generateSecureOStyleFrameForTX(sbuf, sizeof(sbuf), OTRadioLink::FTS_RESERVED_Q, lenTXID, txbuf, bodylen, e, NULL, key);
          const bool success = (0 != sbodylen) && PrimaryRadio.sendRaw(sbuf+1, sbodylen-1);
#if 1 && defined(DEBUG)
          if(!success) { OTV0P2BASE::serialPrintlnAndFlush(F("!TX Q")); }
          else { OTV0P2BASE::serialPrintlnAndFlush(F("TX Q")); }
#endif
          if(success) { return(true); }
#endif // !defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT) 
          // Fall-through is failure...
          OTV0P2BASE::serialPrintlnAndFlush(F("!TX fail"));
          }
        }
      }
    return(false); // FAILED if fallen through from above.
    }
#endif

  return(false); // FAILED if not otherwise handled.
  }

// Remaining minutes to keep CLI active; zero implies inactive.
// Starts up with full value to allow easy setting of time, etc, without specially activating CLI.
// Marked volatile for thread-safe lock-free non-read-modify-write access to byte-wide value.
// Compound operations on this value must block interrupts.
#define CLI_DEFAULT_TIMEOUT_M 2
static volatile uint8_t CLITimeoutM = CLI_DEFAULT_TIMEOUT_M;
// Reset CLI active timer to the full whack before it goes inactive again (ie makes CLI active for a while).
// Thread-safe.
void resetCLIActiveTimer() { CLITimeoutM = CLI_DEFAULT_TIMEOUT_M; }
// Returns true if the CLI is active, at least intermittently.
// Thread-safe.
bool isCLIActive() { return(0 != CLITimeoutM); }
#if defined(ENABLE_EXTENDED_CLI) || defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
static const uint8_t MAXIMUM_CLI_RESPONSE_CHARS = 1 + OTV0P2BASE::CLI::MAX_TYPICAL_CLI_BUFFER;
#else
static const uint8_t MAXIMUM_CLI_RESPONSE_CHARS = 1 + OTV0P2BASE::CLI::MIN_TYPICAL_CLI_BUFFER;
#endif
// Used to poll user side for CLI input until specified sub-cycle time.
// Commands should be sent terminated by CR *or* LF; both may prevent 'E' (exit) from working properly.
// A period of less than (say) 500ms will be difficult for direct human response on a raw terminal.
// A period of less than (say) 100ms is not recommended to avoid possibility of overrun on long interactions.
// Times itself out after at least a minute or two of inactivity. 
// NOT RENTRANT (eg uses static state for speed and code space).
void pollCLI(const uint8_t maxSCT, const bool startOfMinute)
  {
  // Perform any once-per-minute operations.
  if(startOfMinute)
    {
    ATOMIC_BLOCK (ATOMIC_RESTORESTATE)
      {
      // Run down CLI timer if need be.
      if(CLITimeoutM > 0) { --CLITimeoutM; }
      }
    }

  const bool neededWaking = OTV0P2BASE::powerUpSerialIfDisabled<V0P2_UART_BAUD>();

  // Wait for input command line from the user (received characters may already have been queued)...
  // Read a line up to a terminating CR, either on its own or as part of CRLF.
  // (Note that command content and timing may be useful to fold into PRNG entropy pool.)
  static char buf[MAXIMUM_CLI_RESPONSE_CHARS+1]; // Note: static state, efficient for small command lines.  Space for terminating '\0'.
  const uint8_t n = OTV0P2BASE::CLI::promptAndReadCommandLine(maxSCT, buf, sizeof(buf), NULL);

  if(n > 0)
    {
    // Got plausible input so keep the CLI awake a little longer.
    resetCLIActiveTimer();

    // Process the input received, with action based on the first char...
    bool showStatus = true; // Default to showing status.
    switch(buf[0])
      {
      // Explicit request for help, or unrecognised first character.
      // Avoid showing status as may already be rather a lot of output.
      default: case '?': { /* dumpCLIUsage(maxSCT); */ showStatus = false; break; }

#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      // Set new node association (nodes to accept frames from).
      // Only needed if able to RX and/or some sort of hub.
      case 'A': { showStatus = OTV0P2BASE::CLI::SetNodeAssoc().doCommand(buf, n); break; }
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT

      // Exit/deactivate CLI immediately.
      // This should be followed by JUST CR ('\r') OR LF ('\n')
      // else the second will wake the CLI up again.
      case 'E': { CLITimeoutM = 0; break; }

#if defined(ENABLE_FHT8VSIMPLE) && (defined(ENABLE_LOCAL_TRV) || defined(ENABLE_SLAVE_TRV))
      // H [nn nn]
      // Set (non-volatile) HC1 and HC2 for single/primary FHT8V wireless valve under control.
      // Missing values will clear the code entirely (and disable use of the valve).
      case 'H': { showStatus = OTRadValve::FHT8VRadValveBase::SetHouseCode(&FHT8V).doCommand(buf, n); break; }
#endif
 
#if defined(ENABLE_OTSECUREFRAME_ENCODING_SUPPORT)
      // Set secret key.
      case 'K': { showStatus = OTV0P2BASE::CLI::SetSecretKey(OTRadioLink::SimpleSecureFrame32or0BodyTXV0p2::resetRaw3BytePersistentTXRestartCounterCond).doCommand(buf, n); break; }
#endif // ENABLE_OTSECUREFRAME_ENCODING_SUPPORT

      // Status line and optional smart/scheduled warming prediction request.
      case 'S':
        {
        Serial.print(F("Resets/overruns: "));
        const uint8_t resetCount = eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_RESET_COUNT);
        Serial.print(resetCount);
        Serial.print(' ');
        const uint8_t overrunCount = (~eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_OVERRUN_COUNTER)) & 0xff;
        Serial.print(overrunCount);
        Serial.println();
        break; // Note that status is by default printed after processing input line.
        }

#ifdef ENABLE_EXTENDED_CLI
      // Handle CLI extension commands.
      // Command of form:
      //   +EXT .....
      // where EXT is the name of the extension, usually 3 letters.
      //
      // It is acceptable for extCLIHandler() to alter the buffer passed,
      // eg with strtok_t().
      case '+':
        {
        const bool success = extCLIHandler(&Serial, buf, n);
        Serial.println(success ? F("OK") : F("FAILED"));
        break;
        }
#endif 
      }

    // Almost always show status line afterwards as feedback of command received and new state.
    if(showStatus) { serialStatusReport(); }
    // Else show ack of command received.
    else { Serial.println(F("OK")); }
    }
  else { Serial.println(); } // Terminate empty/partial CLI input line after timeout.

  // Force any pending output before return / possible UART power-down.
  OTV0P2BASE::flushSerialSCTSensitive();

  if(neededWaking) { OTV0P2BASE::powerDownSerial(); }
  }


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
  if(!::OTV0P2BASE::HWTEST::check32768HzOsc()) { panic(F("xtal")); } // Async clock not running correctly.

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

  // Seed RNGs, after having gathered some sensor values in RAM...
  OTV0P2BASE::seedPRNGs();

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

  // Report initial status.
  serialStatusReport();

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

  // Start listening.
  PrimaryRadio.listen(true);

  // Set appropriate loop() values just before entering it.
  TIME_LSD = OTV0P2BASE::getSecondsLT();
  }


// Main code here, loops every 2s.
void loop()
  {
  setUpContinuousRX();
  OTV0P2BASE::powerDownSerial(); // Ensure that serial I/O is off.
  // Power down most stuff (except radio for hub RX).
  OTV0P2BASE::minimisePowerWithoutSleep();
  uint_fast8_t newTLSD;
  while(TIME_LSD == (newTLSD = OTV0P2BASE::getSecondsLT()))
    {
    // Poll I/O and process message incrementally (in this otherwise idle time).
    // Come back and have another go immediately until no work remaining.
    if(handleQueuedMessages(&Serial, true, &PrimaryRadio)) { continue; }
#if defined(ALLOW_CC1_SUPPORT_RELAY)
    // Handle any pending poll response needed.
    if(pollResponseNeeded && timeToHandleMessage()) { sendCC1PollResponse(); continue; }
#endif

    // Normal long minimal-power sleep until wake-up interrupt.
    // Rely on interrupt to force quick loop round to I/O poll.
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

#if defined(ENABLE_FHT8VSIMPLE)
  // Try for double TX for more robust conversation with valve?
  const bool doubleTXForFTH8V = false;
  // FHT8V is highest priority and runs first.
  // ---------- HALF SECOND #0 -----------
  bool useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8V.FHT8VPollSyncAndTX_First(doubleTXForFTH8V); // Time for extra TX before UI.
  if(useExtraFHT8VTXSlots)
    {
    handleQueuedMessages(&Serial, true, &PrimaryRadio);
    // ---------- HALF SECOND #1 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8V.FHT8VPollSyncAndTX_Next(doubleTXForFTH8V); 
    }
  if(useExtraFHT8VTXSlots)
    {
    handleQueuedMessages(&Serial, true, &PrimaryRadio);
    // ---------- HALF SECOND #2 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8V.FHT8VPollSyncAndTX_Next(doubleTXForFTH8V); 
    }
  if(useExtraFHT8VTXSlots)
    {
    handleQueuedMessages(&Serial, true, &PrimaryRadio);
    // ---------- HALF SECOND #3 -----------
    useExtraFHT8VTXSlots = localFHT8VTRVEnabled() && FHT8V.FHT8VPollSyncAndTX_Next(doubleTXForFTH8V); 
    }
#endif

  // If time to do some trailing processing, CLI, etc...
  while(timeToHandleMessage())
    {
    if(handleQueuedMessages(&Serial, true, &PrimaryRadio)) { continue; }
#if defined(ALLOW_CC1_SUPPORT_RELAY)
    // Handle any pending poll response needed.
    if(pollResponseNeeded && timeToHandleMessage()) { sendCC1PollResponse(); continue; }
#endif
    // Done queued work...
    // Command-Line Interface (CLI) polling, if still active.
    if(isCLIActive())
      {
      // Don't wait too late to start listening for a command
      // to give the user a decent chance to enter a command string
      // and/or that may involve encryption.
      const uint8_t stopBy = min((OTV0P2BASE::GSCT_MAX/4)*3, nearOverrunThreshold - 1);
      if(timeToHandleMessage()) { pollCLI(stopBy, 0 == TIME_LSD); }
      }
    break;
    }

  // Detect and handle (actual or near) overrun, if it happens, though it should not.
  if(TIME_LSD != OTV0P2BASE::getSecondsLT())
    {
    // Increment the overrun counter (stored inverted, so 0xff initialised => 0 overruns).
    const uint8_t orc = 1 + ~eeprom_read_byte((uint8_t *)V0P2BASE_EE_START_OVERRUN_COUNTER);
    OTV0P2BASE::eeprom_smart_update_byte((uint8_t *)V0P2BASE_EE_START_OVERRUN_COUNTER, ~orc);
#if 1 && defined(DEBUG)
    DEBUG_SERIAL_PRINTLN_FLASHSTRING("!loop overrun");
#endif
#if defined(ENABLE_FHT8VSIMPLE)
    FHT8V.resyncWithValve(); // Assume that sync with valve may have been lost, so re-sync.
#endif
    TIME_LSD = OTV0P2BASE::getSecondsLT(); // Prepare to sleep until start of next full minor cycle.
    }
  }

