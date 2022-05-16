/*
This Demo outputs the captured Weather station Values of a Bresser 5in1 Station that I found cheap on ebay
Currently supported:
- Temp
- Humidity
- Rain
- Winddirection
- Windspeed

Link to Weatherstation-Product:
https://www.ebay.de/itm/294905798218

Link to 868 MHz reciever:
https://www.ebay.de/itm/283548978565

Link to a good ESP32 Board with eink Display
https://www.ebay.de/itm/165350071073


   Inspirations can be found here:
// https://github.com/vekexasia/esp32-tesla-opener-and-stuff/blob/master/src/weatherstation/weatherstation.cpp
// https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_6in1.c
//https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_5in1.c#L155-L164 //Old sensor


BITBENCH
DIGEST:8h8h ID?8h8h8h8h FLAGS:4h BATT:1b CH:3d WINDGUST 8h WINDSPEED 8h WINDRICHTUNG 8h TEMP:8h 8h MOIST:8h TRAILER:8h8h8h8h4h
Wenn Station ID 44513318 ist dann kommt Temperatur richtig

Root cause of issues were
https://github.com/merbanan/rtl_433/blob/ba09426e808786e215346007ab6f10a0815bcdc6/src/devices/bresser_5in1.c
https://github.com/merbanan/rtl_433/issues/719
There are different versions of Bresser sensors


   
*/

// include the library
#include <RadioLib.h>
// https://github.com/jgromes/RadioLib


// CC1101 has the following connections:
//ESP32
#define CC_CS  5
#define CC_GD0 16
#define CC_GD2 4
#define CC_MOSI 23
#define CC_MISO 19
#define CC_SCK 18


CC1101 radio = new Module(CC_CS, CC_GD0, RADIOLIB_NC, CC_GD2);
byte fullMessage[40];
bool receivedFlag = false;
bool enableInterrupt = true;

static void IRAM_ATTR setFlag(void) {
    // check if the interrupt is enabled
    if (!enableInterrupt) {
        return;
    }

    // we got a packet, set the flag
    receivedFlag = true;
}

void setup() {
  Serial.begin(115200);
  SPI.begin(CC_SCK, CC_MISO, CC_MOSI, CC_CS);
  Serial.print(F("[CC1101] Initializing ... "));
  //8.22
  int state = radio.begin(868.30, 8.22, 60 , 270.0, 10, 32);//(freq, br, freqDev, rxBW, power, preambleLength)
  state = radio.disableAddressFiltering();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
    state = radio.setCrcFiltering(false);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.println(F("FAILED SET CRC FILTERING"));
            while (true)
                ;
        }
        delay(1000);
        state = radio.fixedPacketLengthMode(27);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.println(F("FAILED SET PACKET LENGTH MODE"));
            while (true)
                ;
        }
        delay(1000);
        state = radio.setSyncWord(0xAA, 0X2d, 0, false);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.println(F("FAILED SET PACKET LENGTH MODE"));
            while (true)
                ;
        }
    
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  //radio.setCrcFiltering(false);
  /*state = radio.fixedPacketLengthMode(27);
  state = radio.setSyncWord(0xAA, 0x2d, 0, false);*/
  state = radio.enableSyncWordFiltering();
  
  state = radio.startReceive();
  Serial.println("Start Reciever ");

  // Set interrupt function.
  radio.setGdo0Action(setFlag);
  
}

void loop() {
  if (!receivedFlag) {
        return;
    }
    enableInterrupt = false;
    receivedFlag = false;
    tryReadAndDecode();
    radio.startReceive();
    enableInterrupt = true;
}


void tryReadAndDecode() {
    int state = radio.readData(fullMessage, 27);
    byte *msg = &fullMessage[1];
    //Serial.print("LQI:");  Serial.print(radio.getLQI());Serial.print(" ");
    uint8_t lqi_quality = radio.getLQI();
    if (state == RADIOLIB_ERR_NONE && fullMessage[0] == 0xd4 && lqi_quality < 50) {
        Serial.print(" Link Quality:");Serial.print(lqi_quality);Serial.print(" ");
        for (int i = 0; i < 40; i++){
          //Serial.print(fullMessage[i], HEX); //DebugPrint for checking Message
          //Serial.print(" ");//DebugPrint for checking Message
        }
        
        //Serial.println(F(" "));
        
        uint32_t id  = ((uint32_t)msg[2] << 24) | (msg[3] << 16) | (msg[4] << 8) | (msg[5]);
        int s_type   = (msg[6] >> 4); // 1: weather station, 2: indoor?, 4: soil probe
        int batt     = (msg[6] >> 3) & 1;
        int chan     = (msg[6] & 0x7);

        Serial.print("BAT: ");Serial.print(batt);
        Serial.print(" Channel: ");Serial.println(chan);

        // temperature, humidity, shared with rain counter, only if valid BCD digits
        int temp_ok  = msg[12] <= 0x99 && (msg[13] & 0xf0) <= 0x90;
        int temp_raw = (msg[12] >> 4) * 100 + (msg[12] & 0x0f) * 10 + (msg[13] >> 4);
        float temp_c = temp_raw * 0.1f;
        if (temp_raw > 600)
            temp_c = (temp_raw - 1000) * 0.1f;
        
        int humidity    = (msg[14] >> 4) * 10 + (msg[14] & 0x0f);
        if (temp_ok){
          Serial.print("Temp OK: ");Serial.print(temp_ok);
          Serial.print(" Temp: ");Serial.print(temp_c);
          Serial.print(" Feuchte: ");Serial.println(humidity);
        }
        // apparently ff01 or 0000 if not available, ???0 if valid inverted BCD
        int uv_ok  = (msg[16] & 0x0f) == 0 && (~msg[15] & 0xff) <= 0x99 && (~msg[16] & 0xf0) <= 0x90;
        int uv_raw = ((~msg[15] & 0xf0) >> 4) * 100 + (~msg[15] & 0x0f) * 10 + ((~msg[16] & 0xf0) >> 4);
        float uv   = uv_raw * 0.1f;
        int flags  = (msg[16] & 0x0f); // looks like some flags, not sure
        /*if(uv_ok){
          Serial.print(" UV_OK: ");Serial.print(uv_ok);
          Serial.print(" UV_RAW: ");Serial.print(uv_raw);
          Serial.print(" UV: ");Serial.print(uv);
        }*/
        
        //int unk_ok  = (msg[16] & 0xf0) == 0xf0;
        //int unk_raw = ((msg[15] & 0xf0) >> 4) * 10 + (msg[15] & 0x0f);
        
        // rain counter, inverted 3 bytes BCD, shared with temp/hum, only if valid digits
        msg[12] ^= 0xff;
        msg[13] ^= 0xff;
        msg[14] ^= 0xff;
        int rain_ok   = msg[12] <= 0x99 && msg[13] <= 0x99 && msg[14] <= 0x99;
        int rain_raw  = (msg[12] >> 4) * 100000 + (msg[12] & 0x0f) * 10000
                + (msg[13] >> 4) * 1000 + (msg[13] & 0x0f) * 100
                + (msg[14] >> 4) * 10 + (msg[14] & 0x0f);
        float rain_mm = rain_raw * 0.1f;
        if (rain_ok){
          Serial.print(" Regen_OK: ");Serial.print(rain_ok);
          Serial.print(" Regen_mm: ");Serial.print(rain_mm);
          Serial.print(" Regen_Roh: ");Serial.print(rain_raw);
        }
          
        // invert 3 bytes wind speeds
        msg[7] ^= 0xff;
        msg[8] ^= 0xff;
        msg[9] ^= 0xff;
        int wind_ok = (msg[7] <= 0x99) && (msg[8] <= 0x99) && (msg[9] <= 0x99);
    
        int gust_raw    = (msg[7] >> 4) * 100 + (msg[7] & 0x0f) * 10 + (msg[8] >> 4);
        float wind_gust = gust_raw * 0.1f;
        int wavg_raw    = (msg[9] >> 4) * 100 + (msg[9] & 0x0f) * 10 + (msg[8] & 0x0f);
        float wind_avg  = wavg_raw * 0.1f;
        int wind_dir    = ((msg[10] & 0xf0) >> 4) * 100 + (msg[10] & 0x0f) * 10 + ((msg[11] & 0xf0) >> 4);
        if(wind_ok){
          Serial.print(" Wind_OK: ");Serial.print(wind_ok);
          Serial.print(" Windrichtung: ");Serial.print(wind_dir);
          Serial.print(" Wind_Gust: ");Serial.println(wind_gust);
        }
        
    }
}
