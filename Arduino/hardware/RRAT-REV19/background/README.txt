Notes on RRAT.








Mapping to Pi HAT pins:

https://learn.adafruit.com/program-an-avr-or-arduino-using-raspberry-pi-gpio-pins/configuration

with the ‘example’ connections given, none of which are RPi SPI bus pins, 

Arduino ICSP VCC to Raspberry Pi 5 volt pin. 
Arduino ICSP GND to Raspberry Pi ground pin. 
Arduino ICSP RESET to Raspberry Pi GPIO #12.
Arduino ICSP SCK to Raspberry Pi GPIO #24. 
Arduino ICSP MOSI to Raspberry Pi GPIO #23. 
Arduino ICSP MISO to Raspberry Pi GPIO #18
it looks like plain GPIO is needed and avrdude will bit-bang.  As is only going to be used once in a blue moon, if ever, it doesn’t need to be fast,  Keeping it away from a real used SPi bus seems better, so that the RPi pins can be tri-stated.
