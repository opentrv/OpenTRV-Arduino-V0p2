#ifndef SECONDARY_RADIO_CONFIG_H_
#define SECONDARY_RADIO_CONFIG_H_

//For EEPROM: TODO make a spec for how config should be stored in EEPROM to make changing them easy
//- Set the first field of SIM900LinkConfig to true.
//- The configs are stored as \0 terminated strings starting at 0x300.
//- You can program the eeprom using ./OTRadioLink/dev/utils/sim900eepromWrite.ino
//  static const void *SIM900_PIN      = (void *)0x0300;
//  static const void *SIM900_APN      = (void *)0x0305;
//  static const void *SIM900_UDP_ADDR = (void *)0x031B;
//  static const void *SIM900_UDP_PORT = (void *)0x0329;
//  const OTSIM900Link::OTSIM900LinkConfig_t SIM900Config(
//                                                  true,
//                                                  SIM900_PIN,
//                                                  SIM900_APN,
//                                                  SIM900_UDP_ADDR,
//                                                  SIM900_UDP_PORT);
//For Flash:
//- Set the first field of SIM900LinkConfig to false.
//- The configs are stored as \0 terminated strings.
//- Where multiple options are available, uncomment whichever you want
  static const char SIM900_PIN[5] PROGMEM       = "1111";

// APN Configs - Uncomment based on what SIM you are using
//  static const char SIM900_APN[] PROGMEM      = "\"everywhere\",\"eesecure\",\"secure\""; // EE
static const char SIM900_APN[] PROGMEM      = "\"mobiledata\""; // GeoSIM

// UDP Configs - Edit SIM900_UDP_ADDR for relevant server. NOTE: The server IP address should never be committed to GitHub.
// IP adress in seperate header to avoid accidentally committing.
static const char SIM900_UDP_ADDR[16] PROGMEM = ""; // Of form "1.2.3.4".
static const char SIM900_UDP_PORT[5] PROGMEM = "";             // Standard port for OpenTRV servers
const OTSIM900Link::OTSIM900LinkConfig_t SIM900Config(
                                                false,
                                                SIM900_PIN,
                                                SIM900_APN,
                                                SIM900_UDP_ADDR,
                                                SIM900_UDP_PORT);


#endif // SECONDARY_RADIO_CONFIG_H_
