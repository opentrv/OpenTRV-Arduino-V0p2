/*
The OpenTRV project licenses this file to you
under the Apache Licence, Version 2.0 (the "Licence");
you may not use this file except in compliance
with the Licence. You may obtain a copy of the Licence at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing,
software distributed under the Licence is distributed on an
"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
KIND, either express or implied. See the Licence for the
specific language governing permissions and limitations
under the Licence.

Author(s) / Copyright (s): Deniz Erbilgin 2016
*/

/**
 * Set of tests for REV14 hardware.
 * Tests:
 *     - Supply voltage : Print measured voltage to serial
 *     - Xtal           : Check resonator frequency against internal RC
 *     - Phototransistor: Perform a reading
 *     - TMP112         : Test for ACK
 *     - SHT21          : Test for ACK
 *     - QM-1 module    : Test for ACK
 *     - RN2483         : Test for ACK
 *     - Mobdet         : TBD
 *     - DS18B20        : TBD
 *     
 * @todo   Work out how to setup serial connection to RN2483
 *         Move stuff out into libraries
 */

#include <Wire.h>
#include <OTV0p2Base.h>

/************************* Pin Defs **********************************/
// Power control pins
static const uint8_t IO_Powerup_pin  =  7; // IO power up pin.
static const uint8_t BoostRegEn_pin  = A3; // Boost regulator enable pin.
// Peripheral connect pins
static const uint8_t lightSensor_pin = A0; // analogue pin phototransistor is connected to.
static const uint8_t QM1_IRQ_pin     =  6; // interrupt pin connected to QM-1 module.
static const uint8_t RN2483_Rx_pin   =  8; // Software serial Rx pin connected to RN2483.
static const uint8_t RN2483_Tx_pin   =  5; // Software serial Tx pin connected to RN2483.
static const uint8_t RN2483_Rst_pin  = A2; // Pin connected to RN2483 reset line.

/************************ Other Constants ****************************/
// Xtal
static const uint8_t xtalExpectedValue = 122;  // Expected value from xtal
static const uint8_t xtalMaxDeviation  =   2;  // Max acceptable deviation from xtal
// TMP112
static const uint8_t tmp112_i2cAddr = 0x48; // TMP112 I2C bus address
// SHT21
static const uint8_t sht21_i2cAddr  = 0x40; // SHT21 I2C bus address
// QM-1
static const uint8_t qm1_i2cAddr    = 0x09; // QM1 I2C bus address
// RN2483
static const uint16_t RN2483_baud    = 23040;     // Baud rate to talk to RN2483 at. Need to autobaud with break+'U' after reset.
//static const char    RN2483_END[3]  = "\r\n";   // RN2483 message end

/************************ Objects and variables **********************/
OTV0P2BASE::SupplyVoltageCentiVolts Supply_cV;
OTV0P2BASE::OTSoftSerial rn2483(RN2483_Rx_pin, RN2483_Tx_pin);

void setup() {
    /**
     * 1) Put all pins in safe state (low power not as important) - AVR should automatically init correctly
     * 2) Setup Xtal first to give time to settle
     * 3) Setup ADC0
     * 4) Setup software serial
     * 5) Setup I2C
     * 6) Wait a couple of secs
     */
     Serial.begin(4800); // start serial connection for debug output
     Serial.println(F("REV14 Hardware Tests"));
     
     Serial.print(F("Setting up Xtal..."));
     delay(500);
     setupXtal();
     Serial.println(F("done"));
     Serial.print(F("Setting up SoftSerial..."));
     setupSoftSerial();
     Serial.println(F("done"));
     Serial.print(F("Setting up I2C..."));
     setupI2C();
     Serial.println(F("done"));
     Serial.print(F("Setup pins..."));
//     setupPins();
     Serial.println(F("done"));
     Serial.println(F("Waiting for Xtal to settle"));
     delay(2000);
}

void loop() {
    /**
     * 1) Test Vcc
     * 2) Test Xtal
     * 3) Test light sensor
     * 4) Test RN2483
     * 5) Scan I2C bus
     * 6) Test available I2C devices sequentially
     */
     Serial.println(F("\n\nStarting Tests..."));

     testVcc();
     testXtal();
     delay(500);
     testLightSensor();
     testRN2483();
     scanI2C();
     delay(5000);
}

/**
 * @brief   Sets up TOSC registers for asyncronous capture to Timer X
 * @note    powerSetup() disables everything and then enables@
 *          - Timer 0 (arduino functions)
 *          - Timer 2 (xtal capture timer)
 */
void setupXtal()
{
    OTV0P2BASE::powerSetup(); // Standard V0p2 startup method.
    OTV0P2BASE::powerUpSerialIfDisabled<4800>();
    OTV0P2BASE::powerUpADCIfDisabled();
}

/**
 * @brief   Sets up ADC0 for one shot capture
 */
// void setupADC0()
// {
//  
// }

/**
 * @brief   Sets up software serial on relevant pins
 */
 void setupSoftSerial()
 {
    rn2483.begin(RN2483_baud);
 }

/**
 * @brief   Sets up I2C bus
 */
 void setupI2C()
 {
    OTV0P2BASE::powerUpTWIIfDisabled();
 }

/**
 * @brief   Sets up pins that are not defined by other peripheral setup
 *          - Set RN2483 ~reset to high (non reset state)
 *          - Power up photodiode/any "intermittent peripherals"
 *          - Enable boost reg to power up QM-1
 */
void setupPins()
{
    pinMode(RN2483_Rst_pin, OUTPUT);
    pinMode(IO_Powerup_pin, OUTPUT);
    pinMode(BoostRegEn_pin, OUTPUT);
    digitalWrite(RN2483_Rst_pin, HIGH);
    digitalWrite(IO_Powerup_pin, HIGH);
    digitalWrite(BoostRegEn_pin, HIGH);
}

/**
 * @brief   Tests supply voltage against internal reference
 * @todo    Implement test
 */
 void testVcc()
 {
    uint16_t value = 0;
    Serial.print(F("Testing Supply..."));
    value = Supply_cV.read();
    Serial.print(value); // print supply voltage
    Serial.println(F("V"));
 }

/**
 * @brief   Tests xtal against internal high speed RC oscillator
 */
 void testXtal()
 {
    Serial.print(F("Testing Xtal..."));
    delay(500);
    if(::OTV0P2BASE::HWTEST::check32768HzOscExtended()) Serial.println(F("passed"));
    else Serial.println(F("failed"));
 }

/**
 * @brief   Tests light sensor. Prints failed if value is 0 or 255, else prints value.
 */
void testLightSensor()
{
    uint8_t value = 0;
    Serial.print(F("Testing Light Sensor..."));
//    OTV0P2BASE::power_intermittent_peripherals_enable();
//    delay(100);
    value = analogRead(lightSensor_pin);
    if( (value == 0) || (value == 255) ) Serial.println(F("failed"));
    else Serial.println(value); // print light value;
}
/**
 * @brief   Tests RN2483 is present
 * @todo    get this working
 */
void testRN2483()
{
    char buf[9];
    memset(buf, 0, sizeof(buf));
    buf[8] = ' ';
    Serial.println(F("Testing RN2483..."));
    // reset RN2483
    digitalWrite(RN2483_Rst_pin, LOW);
    delay(100);
    digitalWrite(RN2483_Rst_pin, HIGH);
    delay(1000);
    // autobaud (break+'U')
    digitalWrite(RN2483_Tx_pin, LOW);
    delay(10);
    digitalWrite(RN2483_Tx_pin, HIGH);
    rn2483.print('U');
    delay(100);
    // send test message and check reply
    rn2483.print("sys get ver\r\n");
    rn2483.read( (uint8_t *)buf, (sizeof(buf)-1) );
    Serial.println(buf);
}
/**
 * @brief   Scans I2C bus for possible devices on the REV10 and prints to serial.
 */
void scanI2C()
{
    uint8_t value = 0;
    Serial.println(F("Scanning I2C bus:"));
    testTMP112();
    testSHT21();
    testQM1();
    
//    Serial.println(value); // print supply voltage;
}
/**
 * @brief   Tests TMP112
 */
void testTMP112()
{
    Serial.print(F("Checking for TMP112..."));
    if(!testI2CDev(tmp112_i2cAddr)) Serial.println(F("present."));
    else Serial.println(F("not present."));
}
/**
 * @brief   Tests SHT21
 */
void testSHT21()
{
    Serial.print(F("Checking for SHT21..."));
    if(!testI2CDev(sht21_i2cAddr)) Serial.println(F("present."));
    else Serial.println(F("not present."));
}
/**
 * @brief   Tests QM-1
 * @note    This requires boostRegEn_pin to be set high (+delay for QM-1 to start properly?)
 */
void testQM1()
{
    Serial.print(F("Checking for QM-1..."));
    if(!testI2CDev(qm1_i2cAddr)) Serial.println(F("present."));
    else Serial.println(F("not present."));
}

/**
 * @brief   Tests I2C device at address for ACK
 * @retval  0 success
 *          1 buffer error (should never happen)
 *          2 NACK received
 *          3 NACK received
 *          4 other error
 */
uint8_t testI2CDev(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission();
}

