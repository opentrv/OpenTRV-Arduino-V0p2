uint8_t ch = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  Serial.println("start");
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t inBuf[64];
  uint8_t outBuf[128];

  for(uint8_t i = 0; i < sizeof(inBuf); i++) {
    inBuf[i] = i + (ch * sizeof(inBuf));
  }

  Serial.print("ReturnVal: ");
  Serial.println(getHex(inBuf, outBuf, sizeof(outBuf)));
  for(uint8_t i = 0; i < sizeof(inBuf); i++) {
    Serial.print(inBuf[i], HEX);
  }
  Serial.println();
  Serial.write(outBuf, sizeof(outBuf));
  Serial.println();
  Serial.println();
  ch++;
  while (ch == 4) {}
  delay(500);
}

bool getHex(const uint8_t *string, uint8_t *output, uint8_t outputLen)
{
  if((string || output) == NULL) return false; // check for null pointer
    uint8_t counter = outputLen;
    // convert to hex
  while(counter) {
    uint8_t highValue = *string >> 4;
    uint8_t lowValue  = *string & 0x0f;
    uint8_t temp;
    
    temp = highValue;
    if(temp <= 9) temp += '0';
    else temp += ('A' - 10);
    *output++ = temp;

    temp = lowValue;
    if(temp <= 9) temp += '0';
    else temp += ('A'-10);
    *output++ = temp;

    string++;
    counter -= 2;
  }
  return true;
}
