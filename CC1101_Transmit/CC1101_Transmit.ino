/*
   RadioLib CC1101 Transmit Example

   This example transmits packets using CC1101 FSK radio module.
   Each packet contains up to 64 bytes of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#cc1101

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/

// include the library
#include <RadioLib.h>

#define CC_CS   5
#define CC_GD0 16
#define CC_GD2 4
#define CC_MOSI 23
#define CC_MISO 19
#define CC_SCK 18
CC1101 radio = new Module(CC_CS, CC_GD0, RADIOLIB_NC, CC_GD2);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//CC1101 radio = RadioShield.ModuleA;

void setup() {
  Serial.begin(115200);
  SPI.begin(CC_SCK, CC_MISO, CC_MOSI, CC_CS);
  // initialize CC1101 with default settings
  Serial.print(F("[CC1101] Initializing ... "));
  //int state = radio.begin();
  int state = radio.begin(868.35, 9, 60, 270.0, 10, 32);//(freq, br, freqDev, rxBW, power, preambleLength)
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
}

void loop() {
  Serial.print(F("[CC1101] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to 63 characters long
  int state = radio.transmit("Hello World!");

  // you can also transmit byte array up to 63 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);
  */

  if (state == RADIOLIB_ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));

  } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 64 bytes
    Serial.println(F("too long!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }

  // wait for a second before transmitting again
  delay(1000);
}
