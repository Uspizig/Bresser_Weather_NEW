//Features:
// - V3 Stack TTN Kompatibel, Ardunio ESP32 Core V2.0 Tested
// - Macht OTAA Join einmal im Leben, dann nur noch ABP
// - Decodiert 
// Geraet 9 = Self Made Hardware
// Geraet10 = TTGO LORA32 T3 V1.6

//To make this sketch work:

//Download these libs
/*
Libs: 
https://www.partsnotincluded.com/fastled-rgbw-neopixels-sk6812/
https://github.com/FastLED/FastLED
https://github.com/matthias-bs/BresserWeatherSensorReceiver
https://github.com/mcci-catena/arduino-lmic

*/

//MANDATORY CHANGES to MAKE IT WORK
//Hal.cpp changes:
//in hal.cpp replaced spi.begin(); to spi.begin(14, 2, 15, 26); to match YOUR pinning (lmic_pins.sck, lmic_pins.miso, lmic_pins.mosi, lmic_pins.nss)
  //SPI.begin();
/*
static void hal_spi_init () {
    //SPI.begin();//Original Änderung in hal.cpp
  //SPI.begin(lmic_pins.sck, lmic_pins.miso, lmic_pins.mosi, lmic_pins.nss);
  SPI.begin(14, 2, 15, 26);//this should match your pinning.. i copied a demo hal.cpp in the github folder
}

//Weathersensor.cpp
//Replace in the lib SRC folder https://github.com/matthias-bs/BresserWeatherSensorReceiver the weathersensor.cpp file with mine
// add in your folder the WaetherSensor.cfg file.
*/




//for Debugging
//#define SingleChannelMode 1 //to check on own gateway Join Behaviour
#define LMIC_DEBUG_LEVEL 1
//#define geraet9 //ESP32 Micro Pico D4 Board mit CC1101 Bresser &SK6812 Diode -> TTN Stack V3
#define geraet10 //TTGO LORA32 V1.6 T3

//const unsigned long UpdateInterval = (60L * 20L - 03) * 1000000L; // Update delay in microseconds
//#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */


//Battery Pin
#define BATTERY_PIN 35
#include "OTA.h"
#include "credentials.h" 

#include "lorapindef_and_features.h"  
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>

#include <FastLED.h> //https://github.com/FastLED/FastLED
#include <LoraMessage.h> //https://github.com/thesolarnomad/lora-serialization/

// include the library
#ifdef CC1101_CONNECTED
  #include <RadioLib.h>
  // https://github.com/jgromes/RadioLib
  SPIClass spiCC1101(VSPI);//https://github.com/espressif/arduino-esp32/issues/1219
  CC1101 radio = new Module(CC_CS, CC_GD0, RADIOLIB_NC, CC_GD2, spiCC1101, SPISettings()); //https://github.com/jgromes/RadioLib/issues/168
  //CC1101 radio = new Module(CC_CS, CC_GD0, RADIOLIB_NC, CC_GD2);
  byte fullMessage[40];
  bool receivedFlag = false;
  bool enableInterrupt = true;
  
#endif

#ifdef RFM95_WEATHER_DECODER
    #include "WeatherSensorCfg.h"
    #include "WeatherSensor.h" //https://github.com/matthias-bs/BresserWeatherSensorReceiver
    WeatherSensor weatherSensor;
#endif


#ifdef SSD1306_DISPLAY
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  #define OLED_SDA 21
  #define OLED_SCL 22 
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
#endif

//Led Einstellungen
  
  FASTLED_USING_NAMESPACE
  #define MAX_POWER_MILLIAMPS 500
  #ifdef WS2812_LEDS
    CRGB leds[ANZAHL_LEDS];
    CRGB leds2[OBEN_LEDS];
  #endif
  #ifdef SK6812_LEDS
    #include "FastLED_RGBW.h" //https://www.partsnotincluded.com/fastled-rgbw-neopixels-sk6812/
    CRGBW leds[ANZAHL_LEDS];
    CRGB *ledsRGB = (CRGB *) &leds[0];

    //Beleuchtung des Drehreglers
    CRGBW leds2[OBEN_LEDS];
    CRGB *ledsRGB2 = (CRGB *) &leds2[0];
  #endif  
  static uint8_t gHue = 0;

uint8_t hue, hue2 = 0;
boolean zielwert_erreicht =0;
uint8_t Aktuelle_LED=0;
uint8_t schleifenzaehler = 0;
int vorher_message =0;
int counter = 0;
int lastcounter=0;
int counter_last=0;
int setup_lora_sucesful_flag =0;



//Temperatur:
#include <Wire.h>
#ifdef BMP280_CONNECTED
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp; // I2C
#endif

#ifdef BME280_CONNECTED
  #define SEALEVELPRESSURE_HPA (1013.25)
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme;
#endif

#ifdef CCS811_CONNECTED 
  #include <SparkFunCCS811.h> //Click here to get the library: http://librarymanager/All#SparkFun_CCS811
  CCS811 myCCS811(CCS811_ADRESS);
#endif



#ifdef geraet3
  #define sda 21 ///* I2C Pin Definition */
  #define scl 22 ///* I2C Pin Definition */
#else
  #define sda 21 ///* I2C Pin Definition */
  #define scl 22 ///* I2C Pin Definition */
#endif


#ifdef WIFISCAN_CONNECTED
  long lastMsg = 0;
  #define updaterateWIFI 3000
#endif


#ifdef CC1101_CONNECTED
  float temp_cc1101 =0;
  float humidity_cc1101 =0;
  int pressure_cc1101 =0;
  int rain_cc1101 =0;
  int wind_speed_cc1101 =0;
  int wind_speed_max_cc1101 =0;
  int wind_dir_cc1101 =0;
  int count_decoded_wind_messages =0;
#endif

#ifdef RFM95_WEATHER_DECODER
  float temp_cc1101 =0;
  float humidity_cc1101 =0;
  int pressure_cc1101 =0;
  int rain_cc1101 =0;
  int wind_speed_cc1101 =0;
  int wind_speed_max_cc1101 =0;
  int wind_dir_cc1101 =0;
  int count_decoded_wind_messages =0;
#endif

float temp = 23;
float pressure = 980;
float humidity = 50;
float ccs811_CO2 = 450;
float ccs811_TVOC = 2;
int battery = 50;
int port = 1;

typedef struct {
   int Temp;
   int Feuchte;
   int Temp_max;
   int Temp_min;
   int Condition_Code;
   int Muell_Code;
} payload_record_type;
  
payload_record_type vorhersage[1];





#include "esp_timer.h"
#include "Arduino.h"
#include "fb_gfx.h"
//#include "fd_forward.h"
//#include "fr_forward.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"
#include "WiFi.h"
#include <WiFiClientSecure.h>
#include <ArduinoJson.h> 
#include "credentials.h"  
//char ssid[] = mySSID;     // your network SSID (name)
//char password[] = myPASSWORD; // your network key


WiFiClientSecure client;

/*Wifi Sort Test*/
#define WIFI_DELAY        500
#define MAX_SSID_LEN      32 /* Max SSID octets. */
char ssid2[MAX_SSID_LEN] = "";/* SSID that to be stored to connect. */
#define WIFI_DELAY        500/* Delay paramter for connection. */

typedef struct { /*Struct für einfache Speicherung der Daten*/
  
   byte mac0;
   byte mac1;
   byte mac2;
   byte mac3;
   byte mac4;
   byte mac5;
   String mac;
   int EmpfangsPegel;
   int Encrypted;
} Wifi_record_type;
Wifi_record_type Found_Wifi[5];




uint32_t start_time;
uint32_t next_time;
uint32_t previous_time;
uint32_t previous_full_update;

uint32_t total_seconds = 0;
uint32_t startup_seconds = 0;
uint32_t seconds, minutes, hours, days;

uint32_t join_total_seconds = 0;
uint32_t join_seconds, join_minutes, join_hours, join_days;

//Partial Update:
const uint32_t partial_update_period_s = 30;
//const uint32_t full_update_period_s = 60 * 60 * 60;//alle 2.5 Tage
const uint32_t full_update_period_s = 60 * 60;//(jede Stunde)
//const uint32_t full_update_period_s = 60;(jede Minute)
/*EINK Teile ENDE*/

//Variables for ABP after OTAA Join
u1_t NWKSKEY[16] ;                               // LoRaWAN NwkSKey, network session key.
u1_t APPSKEY[16] ;                               // LoRaWAN AppSKey, application session key.
u4_t DEVADDR ;



//RealTimeVariables which don't get erased during Sleep
  struct Daten_struct {//https://esp32.com/viewtopic.php?t=11984
    uint16_t Stack[15];
    uint8_t Counter_History;
  };
  RTC_DATA_ATTR uint32_t SAVED_dataValid ;                           // DATAVALID if valid data (joined)
  RTC_DATA_ATTR uint8_t  SAVED_devaddr[4] ;                          // Device address after join
  RTC_DATA_ATTR uint8_t  SAVED_nwkKey[16] ;                          // Network session key after join
  RTC_DATA_ATTR uint8_t  SAVED_artKey[16] ;                          // Aplication session key after join
  RTC_DATA_ATTR uint32_t SAVED_seqnoUp ;                             // Sequence number       
  bool OTAA = true ;   //startup with OTAA if No EEPROM was Saved before
  //History of Data
  RTC_DATA_ATTR  struct Daten_struct Druck_Werte;                 
  RTC_DATA_ATTR  struct Daten_struct Temp_Werte;
  RTC_DATA_ATTR  struct Daten_struct CO2_Werte;
  RTC_DATA_ATTR  struct Daten_struct TVOC_Werte;                                                   
  RTC_DATA_ATTR  struct Daten_struct Feuchte_Werte;


// Boot Counter
RTC_DATA_ATTR uint32_t SAVED_boot_counter =0;

//variables for OTAA JOIN
  static const u1_t PROGMEM APPEUI[8]={ APP_EUI_LORA };//lsb
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  static const u1_t PROGMEM DEVEUI[8]={ DEV_EUI_LORA };//lsb
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

  //msb
  static const u1_t PROGMEM APPKEY[16] = { APP_KEY_LORA };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}  



int verbunden_indicator = 0;
int Packet_Transmission_ongoing = 0; // Display wird nur aktualisiert wenn ein Paket gesendet wird
int packet_counter =0;
int packet_counter_rx =0;
static osjob_t sendjob;
devaddr_t DeviceName = 0;

//CC1101 Interruped Routine
#ifdef CC1101_CONNECTED
  static void IRAM_ATTR setFlag(void) {
      // check if the interrupt is enabled
      if (!enableInterrupt) {
          return;
      }
  
      // we got a packet, set the flag
      receivedFlag = true;
  }
#endif


// Pin mapping
#ifdef geraet10
  const lmic_pinmap lmic_pins = {
      .nss = LORA_PIN_SPI_NSS,
      .rxtx = LMIC_UNUSED_PIN,
      .rst = LORA_PIN_SPI_RST,
      .dio = {LORA_PIN_SPI_DIO0, LORA_PIN_SPI_DIO1, LMIC_UNUSED_PIN}, 
      .rxtx_rx_active = 0,//kopiert von TTGO Board
      .rssi_cal = 10,//kopiert von TTGO Board
      //.spi_freq = 8000000, /* 8MHz */ //kopiert von TTGO Board
      .spi_freq = 4000000, //4 MHZ from GxEPD2 https://github.com/ZinggJM/GxEPD2/tree/master/extras/sw_spi
  };
  
#elif geraet9
  const lmic_pinmap lmic_pins = {
      .nss = LORA_PIN_SPI_NSS,
      .rxtx = LMIC_UNUSED_PIN,
      .rst = LORA_PIN_SPI_RST,
      .dio = {LORA_PIN_SPI_DIO0, LORA_PIN_SPI_DIO1, LMIC_UNUSED_PIN}, 
      .rxtx_rx_active = 0,//kopiert von TTGO Board
      .rssi_cal = 10,//kopiert von TTGO Board
      //.spi_freq = 8000000, /* 8MHz */ //kopiert von TTGO Board
      .spi_freq = 4000000, //4 MHZ from GxEPD2 https://github.com/ZinggJM/GxEPD2/tree/master/extras/sw_spi
  };

#else
const lmic_pinmap lmic_pins = {
    .nss = LORA_PIN_SPI_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    #ifdef geraet1
      .dio = {LORA_PIN_SPI_DIO0, LORA_PIN_SPI_DIO1, LMIC_UNUSED_PIN}, 
    #else
      .dio = {LORA_PIN_SPI_DIO1, LORA_PIN_SPI_DIO1, LMIC_UNUSED_PIN}, 
    #endif
    .rxtx_rx_active = 0,//kopiert von TTGO Board
    .rssi_cal = 10,//kopiert von TTGO Board
    //.spi_freq = 8000000, /* 8MHz */ //kopiert von TTGO Board
    .spi_freq = 4000000, //4 MHZ from GxEPD2 https://github.com/ZinggJM/GxEPD2/tree/master/extras/sw_spi
};
#endif


  


//Hilfsfunktion um Hex Vafriablen auszugeben
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            verbunden_indicator = LORA_TX_JOINING;
            break;
        case EV_JOINED:
            verbunden_indicator = LORA_TX_JOINED;
            total_seconds =0;
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: "); Serial.println(netid, DEC);
              Serial.print("devaddr: "); Serial.println(devaddr, HEX);
              DeviceName = devaddr;
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
              saveToRTC(); // Speichere Werte
              //SAVED_dataValid = 0;// Setze zu Testzwecken Datavalid = 0 damit Daten aus dem EEPROM Gelesen werden
              //retrieveKeys(); /Rücklesen der Werte aus EEPROM
            }
            /* Disable link check validation (automatically enabled during join, but because slow data rates change max TX size, we don't use it in this example.*/
            LMIC_setLinkCheckMode(0);
            break;
        
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            verbunden_indicator = LORA_TX_COMPLETED;
            packet_counter++;
            //Serial.println(F("EV_TXCOMPLETE (includes RX time)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(LMIC.dataLen); Serial.println(F("Bytes of Payload-Data Received "));
              for (int i = 0; i < LMIC.dataLen; i++) {
                /*if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {Serial.print(F("0"));}
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);*/
                //recieved_payload[i] = LMIC.frame[LMIC.dataBeg + i]; 
                packet_counter_rx++;
              }
              //decode_rec_payload();
            }
            Packet_Transmission_ongoing = 0;
            saveToRTC();// Tryout to save every time to RTC Memory once a packet has been transmitted correct.
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send); // Schedule next transmission
            //kurzschlaf();
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            verbunden_indicator = LORA_TX_STARTED;
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: No JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}



// combines 2 Bytes high and low Bytes and combines it to one integer
int BitShiftCombine( unsigned char x_low, unsigned char x_high)
{
  int combined;
  combined = x_high;              //send x_high to rightmost 8 bits
  combined = combined<<8;         //shift x_high over to leftmost 8 bits
  combined |= x_low;                 //logical OR keeps x_high intact in combined and fills in                                                             //rightmost 8 bits
  return combined;
}


// Transmits Data to Lora
void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {        
        LoraMessage message;
        #ifdef CC1101_CONNECTED
            if (port == 1){
              message.addTemperature(temp_cc1101);//1
              message.addHumidity(humidity_cc1101);//2
              message.addUint16(int(rain_cc1101));//3
              message.addUint16(int(wind_speed_cc1101));//4
              message.addUint16(int(wind_speed_max_cc1101));//5
              message.addUint16(int(wind_dir_cc1101));//6*/
              LMIC_setTxData2(port, message.getBytes(), message.getLength(), 0);
            }
        #endif   
        #ifdef RFM95_WEATHER_DECODER
            if (port == 1){
              message.addTemperature(temp_cc1101);//1
              message.addHumidity(humidity_cc1101);//2
              message.addUint16(int(rain_cc1101));//3
              message.addUint16(int(wind_speed_cc1101));//4
              message.addUint16(int(wind_speed_max_cc1101));//5
              message.addUint16(int(wind_dir_cc1101));//6*/
              LMIC_setTxData2(port, message.getBytes(), message.getLength(), 0);
            }
        #endif   
        
       
        #ifdef WIFISCAN_CONNECTED
          if (port == 2){
                port=2;
                uint8_t buffer[7];
                uint8_t buffer2[7];
                uint8_t buffer3[7];
                uint8_t payload[21];
                int wifi_signalstaerke = Found_Wifi[port].EmpfangsPegel + 140;
                memcpy(buffer, &Found_Wifi[port].mac0, sizeof(Found_Wifi[port].mac0));
                memcpy(buffer+sizeof(Found_Wifi[port].mac0), &Found_Wifi[port].mac1, sizeof(Found_Wifi[port].mac1));
                memcpy(buffer+sizeof(Found_Wifi[port].mac0)+sizeof(Found_Wifi[port].mac1), &Found_Wifi[port].mac2, sizeof(Found_Wifi[port].mac2));
                memcpy(buffer+sizeof(Found_Wifi[port].mac0)+sizeof(Found_Wifi[port].mac1)+sizeof(Found_Wifi[port].mac2), &Found_Wifi[port].mac3, sizeof(Found_Wifi[port].mac3));
                memcpy(buffer+sizeof(Found_Wifi[port].mac0)+sizeof(Found_Wifi[port].mac1)+sizeof(Found_Wifi[port].mac2)+sizeof(Found_Wifi[port].mac3), &Found_Wifi[port].mac4, sizeof(Found_Wifi[port].mac4));
                memcpy(buffer+sizeof(Found_Wifi[port].mac0)+sizeof(Found_Wifi[port].mac1)+sizeof(Found_Wifi[port].mac2)+sizeof(Found_Wifi[port].mac3)+sizeof(Found_Wifi[port].mac4), &Found_Wifi[port].mac5, sizeof(Found_Wifi[port].mac5));
                memcpy(buffer+sizeof(Found_Wifi[port].mac0)+sizeof(Found_Wifi[port].mac1)+sizeof(Found_Wifi[port].mac2)+sizeof(Found_Wifi[port].mac3)+sizeof(Found_Wifi[port].mac4)+sizeof(Found_Wifi[port].mac5), &wifi_signalstaerke, sizeof(wifi_signalstaerke)); 

                wifi_signalstaerke = Found_Wifi[port+1].EmpfangsPegel + 140;
                memcpy(buffer2, &Found_Wifi[port+1].mac0, sizeof(Found_Wifi[port+1].mac0));
                memcpy(buffer2+sizeof(Found_Wifi[port+1].mac0), &Found_Wifi[port+1].mac1, sizeof(Found_Wifi[port+1].mac1));
                memcpy(buffer2+sizeof(Found_Wifi[port+1].mac0)+sizeof(Found_Wifi[port+1].mac1), &Found_Wifi[port+1].mac2, sizeof(Found_Wifi[port+1].mac2));
                memcpy(buffer2+sizeof(Found_Wifi[port+1].mac0)+sizeof(Found_Wifi[port+1].mac1)+sizeof(Found_Wifi[port+1].mac2), &Found_Wifi[port+1].mac3, sizeof(Found_Wifi[port+1].mac3));
                memcpy(buffer2+sizeof(Found_Wifi[port+1].mac0)+sizeof(Found_Wifi[port+1].mac1)+sizeof(Found_Wifi[port+1].mac2)+sizeof(Found_Wifi[port+1].mac3), &Found_Wifi[port+1].mac4, sizeof(Found_Wifi[port+1].mac4));
                memcpy(buffer2+sizeof(Found_Wifi[port+1].mac0)+sizeof(Found_Wifi[port+1].mac1)+sizeof(Found_Wifi[port+1].mac2)+sizeof(Found_Wifi[port+1].mac3)+sizeof(Found_Wifi[port+1].mac4), &Found_Wifi[port+1].mac5, sizeof(Found_Wifi[port+1].mac5));
                memcpy(buffer2+sizeof(Found_Wifi[port+1].mac0)+sizeof(Found_Wifi[port+1].mac1)+sizeof(Found_Wifi[port+1].mac2)+sizeof(Found_Wifi[port+1].mac3)+sizeof(Found_Wifi[port+1].mac4)+sizeof(Found_Wifi[port+1].mac5), &wifi_signalstaerke, sizeof(wifi_signalstaerke)); 
                
                wifi_signalstaerke = Found_Wifi[port+2].EmpfangsPegel + 140;
                memcpy(buffer3, &Found_Wifi[port+2].mac0, sizeof(Found_Wifi[port+2].mac0));
                memcpy(buffer3+sizeof(Found_Wifi[port+2].mac0), &Found_Wifi[port+2].mac1, sizeof(Found_Wifi[port+2].mac1));
                memcpy(buffer3+sizeof(Found_Wifi[port+2].mac0)+sizeof(Found_Wifi[port+2].mac1), &Found_Wifi[port+2].mac2, sizeof(Found_Wifi[port+2].mac2));
                memcpy(buffer3+sizeof(Found_Wifi[port+2].mac0)+sizeof(Found_Wifi[port+2].mac1)+sizeof(Found_Wifi[port+2].mac2), &Found_Wifi[port+2].mac3, sizeof(Found_Wifi[port+2].mac3));
                memcpy(buffer3+sizeof(Found_Wifi[port+2].mac0)+sizeof(Found_Wifi[port+2].mac1)+sizeof(Found_Wifi[port+2].mac2)+sizeof(Found_Wifi[port+2].mac3), &Found_Wifi[port+2].mac4, sizeof(Found_Wifi[port+2].mac4));
                memcpy(buffer3+sizeof(Found_Wifi[port+2].mac0)+sizeof(Found_Wifi[port+2].mac1)+sizeof(Found_Wifi[port+2].mac2)+sizeof(Found_Wifi[port+2].mac3)+sizeof(Found_Wifi[port+2].mac4), &Found_Wifi[port+2].mac5, sizeof(Found_Wifi[port+2].mac5));
                memcpy(buffer3+sizeof(Found_Wifi[port+2].mac0)+sizeof(Found_Wifi[port+2].mac1)+sizeof(Found_Wifi[port+2].mac2)+sizeof(Found_Wifi[port+2].mac3)+sizeof(Found_Wifi[port+2].mac4)+sizeof(Found_Wifi[port+2].mac5), &wifi_signalstaerke, sizeof(wifi_signalstaerke)); 

                memcpy(payload, &buffer, sizeof(buffer));
                memcpy(payload+sizeof(buffer), &buffer2, sizeof(buffer2));
                memcpy(payload+sizeof(buffer2)+sizeof(buffer), &buffer3, sizeof(buffer3));
                
                /*Serial.print("0:");  Serial.println(Found_Wifi[port].mac0);
                Serial.print("1:");  Serial.println(Found_Wifi[port].mac1);
                Serial.print("2:");  Serial.println(Found_Wifi[port].mac2);
                Serial.print("3:");  Serial.println(Found_Wifi[port].mac3);
                Serial.print("4:");  Serial.println(Found_Wifi[port].mac4);
                Serial.print("5:");  Serial.println(Found_Wifi[port].mac5);*/
                // Serial.print("Port:");  Serial.println(port);
                // Serial.print("Pegel:"); Serial.println(Found_Wifi[port].EmpfangsPegel);
                LMIC_setTxData2(port, payload, sizeof(payload), 0);
          }
        #endif 
        
        Packet_Transmission_ongoing = 1;
        #ifdef WIFISCAN_CONNECTED
          if (port == 1){
            port = 2;
          }
          else{
            port = 1;
          }
        #else
          port = 1;
        #endif  
        //Serial.println(F("Packet ongoing"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup_air_sensor(void){
  #ifdef BMP280_CONNECTED
      bool status2 = BMP_Start();//alles ok?
      BMP_Test();
      delay(2000);
    #endif
    
    #ifdef BME280_CONNECTED
      bool status2 = BME_Start();//alles ok?
      BME_Test();
      delay(2000);
    #endif

    #ifdef CCS811_CONNECTED
      CCS811Core::CCS811_Status_e returnCode = myCCS811.beginWithStatus();
      Serial.print("CCS811 begin exited with: ");
      Serial.println(myCCS811.statusString(returnCode));
      CCS811_Test();
    #endif
}



//Init Lora Stack, sets ADR Mode, 
void setup_lora(void){
  
  pinMode(LORA_PIN_SPI_DIO1, INPUT_PULLDOWN);//to enable PullDown but update your ESP32 Lib to avoid https://esp32.com/viewtopic.php?t=439
  pinMode(LORA_PIN_SPI_DIO0, INPUT_PULLDOWN);

  retrieveKeys(); //Stellt Lora Keys wieder her aus RTC oder EEPROM
  // LMIC init
    os_init();
    
    LMIC_reset(); // Reset the MAC state. Session and pending data transfers will be discarded.
    
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    
    //Debugging Purpose for Single Channel
    #ifdef SingleChannelMode
      #define CHANNEL  1
      for (uint8_t i = 0; i < 9; i++) {
        if (i != CHANNEL) {
          LMIC_disableChannel(i);
        }
      }
    #endif
    
    //Adaptive Data Rate Mode https://www.thethingsnetwork.org/docs/lorawan/adaptive-data-rate.html 
    LMIC_setAdrMode(1); //Adaptiert Datenrate nach 64 Paketen // war ausgeschaltet
    LMIC_setLinkCheckMode(1);//war 0

    
    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;
    
    // Set data rate and transmit power for uplink moved depending if OTAA or ABP
    //LMIC_setDrTxpow(DR_SF12,14);//Langsamster Modus: elendslange AirTime ~820ms aber sichere Übertragung
    //LMIC_setDrTxpow(DR_SF8,14); //Kurz schnell unzuverlässig 
    
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);//https://www.thethingsnetwork.org/forum/t/need-help-with-mcci-lmic-and-ttn-join-wait-issue/30846

    if ( OTAA )
    {
      Serial.println ( "Start joining" ) ;
      //LMIC_setDrTxpow(DR_SF12,14);//Langsamster Modus: elendslange AirTime ~820ms aber sichere Übertragung
      LMIC_setDrTxpow(DR_SF9,14); //with SF9 initial Join is faster... yes I know for this Reset Cycle it will stay on SF9 and consume more airtime
      do_send (&sendjob) ;
    }
    else
    {
      Serial.printf ( "starte mit SF8: gespeichertes SAVED_seqnoUp: %d\n", SAVED_seqnoUp ) ;
      LMIC_setDrTxpow(DR_SF8,14); //otherwise stupid ABP will start with SF12!!! -> Test
      /* //Only for Debugging
      memdmp ( "No OTAA needed - Set Session, DEVADDR:", (uint8_t*)&DEVADDR, 4 ) ;
      memdmp ( "NWKSKEY:", NWKSKEY, 16 ) ;
      memdmp ( "APPSKEY:", APPSKEY, 16 ) ;*/
      LMIC_setSession ( 0x13, DEVADDR, NWKSKEY, APPSKEY ) ;
      //Serial.printf ( "Seqnr set to %d\n", SAVED_seqnoUp ) ;
      LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
      LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
      LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
      LMIC_setLinkCheckMode(1); //test Disable Link Check Mode
      LMIC.dn2Dr = DR_SF9;//test
      LMIC.seqnoUp = SAVED_seqnoUp ;  
      do_send (&sendjob) ;
    }
    
    setup_lora_sucesful_flag = 1;
    //Eingebaut in /src/lmic.c geht nicht
    //LMIC_dn2dr = EU868_DR_SF9;//https://github.com/mcci-catena/arduino-lmic/issues/455
    //LMIC_selectSubBand(0); //https://github.com/mcci-catena/arduino-lorawan/issues/74
    // Start job (sending automatically starts OTAA too)
}
//***************************************************************************************************
//                                      INIT DATA 2 RTC                                                 *
//***************************************************************************************************
// Sets basic Value for RTC Data fields
// Set all Values of Temp Pressure on first startup to a typical value
// If this is a Deep Sleep wakeup and valid Data has been saved before Data is kept and not erased
//***************************************************************************************************
void INIT_DATA_2_RTC(int value, struct Daten_struct *daten){
  if (daten->Counter_History != 0){ // Da war schonmal ein Wert im RTC Memory. kein erster Startup
    Serial.println("RTC Stack war schon befüllt mit Werten... nur Sleep");
  }
  else{//in dem RTC Memory war noch nie was drin
    Serial.print("\n RTC Stack wird initalisiert mit Werten:");
    for (int i = 0; i < 15; i++){
      daten->Stack[i] = value+i;
      Serial.print(i);Serial.print(":");Serial.print(daten->Stack[i]);Serial.print("/");
    }
    daten->Counter_History = 14;
    Serial.print(" DatenCounter:");Serial.println(daten->Counter_History);
  }
}
//***************************************************************************************************
//                                      SAVE DATA 2 RTC                                                 *
//***************************************************************************************************
// Saves Data of one parameter to RTC Memory.
// This is useful as a tempory storage.. If you want to sent multiple values in one package
// Consider Scrambling??
//***************************************************************************************************
void SAVE_DATA_2_RTC(int value, struct Daten_struct *daten){
   //Serial.println("Gespeicherte RTC-Daten:");
   for (int i = 14; i > 0; i--){
    daten->Stack[i] = daten->Stack[i-1];
    //Serial.print(i);Serial.print(":");Serial.print(daten->Stack[i]);Serial.print("/");
   }
   daten->Stack[0] = value;
   //Serial.print("0:");Serial.println(daten->Stack[0]);
}

//***************************************************************************************************
//                                      M E M D M P                                                 *
//***************************************************************************************************
// Dump memory for debug
//***************************************************************************************************
void memdmp ( const char* header, uint8_t* p, uint16_t len )
{
  uint16_t i ;                                                        // Loop control

  Serial.print ( header ) ;                                           // Show header
  for ( i = 0 ; i < len ; i++ )
  {
    if ( ( i & 0x0F ) == 0 )                                          // Continue opn next line?
    {
      if ( i > 0 )                                                    // Yes, continuation line?
      {
        Serial.printf ( "\n" ) ;                                      // Yes, print it
      }
      Serial.printf ( "%04X: ", i ) ;                                 // Print index
    }
    Serial.printf ( "%02X ", *p++ ) ;                                 // Print one data byte
  }
  Serial.println() ;
}


//***************************************************************************************************
//                                    S A V E T O R T C                                             *
//***************************************************************************************************
// Save data in RTC memory.  Every 50th call the data will also be saved in EEPROM memory.         *
// The EEPROM is also updates if OTAA was used.                                                     *
// The space in RTC memory is limited to 512 bytes.                                                 *
//***************************************************************************************************
void saveToRTC()
{
  uint16_t        eaddr ;                                  // Address in EEPROM
  uint8_t*        p ;                                      // Points into savdata

  //Serial.printf ( "\n Save data to RTC memory:\n" ) ;
  memcpy ( SAVED_devaddr, &LMIC.devaddr, 4 ) ;           // Fill struct to save
  memcpy ( SAVED_nwkKey,  LMIC.nwkKey, 16 ) ;
  memcpy ( SAVED_artKey,  LMIC.artKey, 16 ) ;
  SAVED_seqnoUp = LMIC.seqnoUp ;
  SAVED_dataValid = DATAVALID ;
  /*memdmp ( "devaddr:", SAVED_devaddr, 4 ) ;  
  memdmp ( "artKey:",  SAVED_artKey, 16 ) ;
  memdmp ( "nwkKey:",  SAVED_nwkKey, 16 ) ;*/
  //Serial.printf ( "Saved SeqnoUp is: %d Current SeqnoUp is: %d\n", SAVED_seqnoUp, LMIC.seqnoUp ) ;
  Serial.printf ( " SeqnoUp: %d ", SAVED_seqnoUp);
  //Serial.printf ( "SeqnoDown is %d\n", LMIC.seqnoDn ) ;
  if ( ( ( LMIC.seqnoUp % 20 ) == 0 ) || OTAA )           // Need to save data in EEPROM?
  {
    int EEPROM_Data_Counter =0;
    
    Serial.println ( "Saving to EEPROM" ) ;
    p = (uint8_t*)&SAVED_dataValid ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < sizeof(SAVED_dataValid) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
      
    }
    Serial.printf ( "\n Saved %d Bytes of datavalid to EEPROM", EEPROM_Data_Counter);
    p = (uint8_t*)&SAVED_devaddr ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Saved %d Bytes of devadr to EEPROM", EEPROM_Data_Counter);   
    p = (uint8_t*)& SAVED_nwkKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
      Serial.printf ( "\n Saved %d Bytes of nwkey to EEPROM", EEPROM_Data_Counter);   
    p = (uint8_t*)& SAVED_artKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Saved %d Bytes of artkey to EEPROM", EEPROM_Data_Counter);   
    
    p = (uint8_t*)& SAVED_seqnoUp ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)+sizeof(SAVED_seqnoUp)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Saved %d Bytes of seqnr to EEPROM", EEPROM_Data_Counter);   
    
    EEPROM.commit() ;                                      // Commit data to EEPROM
    Serial.printf ( "\n EEPROM operation finished Number of bytes Written: %d", EEPROM_Data_Counter);   
  }
}

//***************************************************************************************************
//                                R E T R I E V E K E Y S                                           *
//***************************************************************************************************
// Try to retrieve the keys en seqnr from non-volitile memory.                                      *
//***************************************************************************************************
void retrieveKeys()
{
  uint16_t eaddr ;                                          // Address in EEPROM
  uint8_t* p ;                                              // Pointer into savdata
  
  // return ;                                               // Return if OTAA is required
  
  //Hier Entscheidung ob RTC Gültig ist oder Defaultwerte drin und aus EEPROM Gelesen werden muss
  //z.B bei Reboot, battery getauscht usw.
  if ( SAVED_dataValid == DATAVALID )                     // DATA in RTC memory valid?
  {
    //Serial.println ( "Keys retrieved from RTC memory\n" ) ; // Show retrieve result 
  }
  else
  {
    Serial.println ( "\n Reading Keys from EEPROM :\n" ) ; 
    // No data vailable in RTC memory.  Use EEPROM data. Hole alles aus EEPROM
    int EEPROM_Data_Counter =0;
    p = (uint8_t*)&SAVED_dataValid ;
    for ( eaddr = EEPROM_Data_Counter ; eaddr < sizeof(SAVED_dataValid) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of datavalid from EEPROM", EEPROM_Data_Counter);   
    p = (uint8_t*)&SAVED_devaddr ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of devadr from EEPROM", EEPROM_Data_Counter);   
     p = (uint8_t*)& SAVED_nwkKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
      Serial.printf ( "\n Recovered %d Bytes of nwkKey from EEPROM", EEPROM_Data_Counter);   
     p = (uint8_t*)& SAVED_artKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of artKey from EEPROM", EEPROM_Data_Counter);   
     p = (uint8_t*)& SAVED_seqnoUp ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)+sizeof(SAVED_seqnoUp)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of SeqnoUp from EEPROM", EEPROM_Data_Counter);    
    SAVED_seqnoUp += 50 ;                                // Counter may be not up-to-date
    Serial.println ( "Recovered Keys from EEPROM are:\n" ) ; 
    /*//Only for Debugging not needed
    memdmp ( "devaddr is:",
             SAVED_devaddr, 4 ) ;
    memdmp ( "appsKey is:",
             SAVED_artKey, 16 ) ;
    memdmp ( "nwksKey is:",
             SAVED_nwkKey, 16 ) ;
    */
  }

  //check ob jetzt nach EEPROM Lesen die Daten gültig
  if ( SAVED_dataValid == DATAVALID )                     // DATA in RTC or EEPROM memory valid?
  {
    Serial.printf ( "Valid data in NVS\n" ) ;               // Yes, show
    /*memdmp ( "devaddr is:",
             SAVED_devaddr, 4 ) ;
    memdmp ( "nwksKey is:",
             SAVED_nwkKey, 16 ) ;
    memdmp ( "appsKey is:",
             SAVED_artKey, 16 ) ;*/
    Serial.printf ( "seqnr is %d\n", SAVED_seqnoUp ) ;
    memcpy ( (uint8_t*)&DEVADDR,
             SAVED_devaddr, sizeof(DEVADDR) ) ;          // LoraWAN DEVADDR, end node device address
    memcpy ( NWKSKEY,
             SAVED_nwkKey,  sizeof(NWKSKEY) ) ;          // LoRaWAN NwkSKey, network session key.
    memcpy ( APPSKEY,
             SAVED_artKey,  sizeof(APPSKEY) ) ;          // LoRaWAN AppSKey, application session key.
    OTAA = false ;                                         // Do not use OTAA
  }
  else
  {
    Serial.printf ( "No saved data, using OTAA\n" ) ;
  }
}

void enable_low_power_mode(void){
    WiFi.disconnect(true);  // Disconnect from the network
    WiFi.mode(WIFI_OFF);    // Switch WiFi off
    btStop(); //Switch Bluetooth OFF
    setCpuFrequencyMhz(80); //Set Prozessor Takt auf 80MHz
    
}

void disable_low_power_mode(void){
    setCpuFrequencyMhz(240); //Set Prozessor Takt auf 240MHz
    WiFi.mode(WIFI_MODE_STA); // calls esp_wifi_set_mode(WIFI_MODE_STA); and esp_wifi_start();
    WiFi.enableSTA(true);
    btStart(); //Switch Bluetooth ON
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    enable_low_power_mode();
    
    
    Serial.begin(115200);
    #ifdef SSD1306_DISPLAY
      Wire.begin(OLED_SDA, OLED_SCL);
      if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false)) { // Address 0x3C for 128x32
        Serial.println(F("SSD1306 Display not found"));
      }
      else{
        display.clearDisplay();
        display.setTextColor(WHITE);
        display.setTextSize(1);
        display.setCursor(0,0);
        display.print("LORA WEATHER SENDER ");
        display.display();
      }
    #endif
    //IM EEPROM sind LoRa-Schlüssel gespeichert. 
    EEPROM.begin ( 514 ) ; //alt 512 +1 für Bildnummer  -> komisch mit 513 zickt EEPROM Funktion. Bild Nummer ist immer 0; deshalb 1 erhöht
    Serial.println(F("Starting, Wifi OFF, 80MHz ... NO WIFI/NO BLE POSSIBLE due to LOW POWER MODE"));
    #ifdef OTAWIFI_ENABLED
      setupOTA("WILDLORAWETTER");
    #endif
    Serial.print("Anzahl Resets: ");Serial.println(SAVED_boot_counter);
    Serial.println(__FILE__); //https://forum.arduino.cc/t/which-program-am-i-running/410688/18
   #ifdef WIFISCAN_CONNECTED
      disable_low_power_mode();
      scanAndSort(); //Wifi Scan Location - Test
   #endif
          
          
          #ifdef WS2812_LEDS
            LEDS.addLeds<WS2812,RGB_LED_DATA_PIN,GRB>(leds, ANZAHL_LEDS);
            LEDS.setBrightness(154);
            LEDS_rot();
            delay(3000);
            //LEDS_gruen_oben();
            LEDS_blau();
            delay(3000);
            LEDS_aus();
            //Serial.println(F("LEDS_AUS"));
            pinMode(RGB_LED_DATA_PIN, OUTPUT); digitalWrite(RGB_LED_DATA_PIN, LOW);
          #endif
          
          #ifdef SK6812_LEDS
            FastLED.addLeds<WS2812B, RGB_LED_DATA_PIN, RGB>(ledsRGB, getRGBWsize(ANZAHL_LEDS));
            FastLED.addLeds<WS2812B, OBEN_LED_DATA_PIN, RGB>(ledsRGB2, getRGBWsize(OBEN_LEDS));
            FastLED.setBrightness(154);
            LEDS_rot();
            delay(3000);
            rainbow();
            //LEDS_gruen_oben();
            //LEDS_blau();
            delay(3000);
            LEDS_aus();
            Serial.println(F("LEDS_AUS"));
          #endif
          
          #ifdef CC1101_CONNECTED
            setup_cc1101();
            //Serial.println(F("Setting Up CC1101 completed"));
          #endif  

          #ifdef RFM95_WEATHER_DECODER
            weatherSensor.begin();
          #endif        
}

void init_wifi(void){
  //Init Wifi:
  // Set WiFi to station mode and disconnect from an AP if it was Previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
   WiFi.begin(ssid, password);
    int wifi_retries = 0;
    while (WiFi.status() != WL_CONNECTED) {
        wifi_retries++;
        delay(200);
        Serial.print(".");
        if (wifi_retries > 35) {
          WiFi.mode(WIFI_STA); // WLAN auf Client setzen 
          WiFi.begin(ssid, password);
          delay(1500);
          if(WiFi.status() != WL_CONNECTED){
              Serial.println("WLAN geht gar nicht :-( ");
          }
          else{
            Serial.println("2. Anlauf WLAN geht :-) ");
          }
       }
        
    }
    Serial.println("");
    Serial.println("WiFi connected");
}

void loop() {
    #ifdef OTAWIFI_ENABLED
      ArduinoOTA.handle();
    #endif  
    
    #ifdef CC1101_CONNECTED
       if(verbunden_indicator == LORA_TX_INIT && count_decoded_wind_messages < 3){  
        if (!receivedFlag) {
          return; /// Mach nix wenn nix rempfangen wurde
        }
        else{
          enableInterrupt = false;
          receivedFlag = false;
          tryReadAndDecode();
          radio.startReceive();
          enableInterrupt = true;
         }
       }
       else{
          if (count_decoded_wind_messages >= 3 && setup_lora_sucesful_flag == 0){
            enableInterrupt = false;
            digitalWrite(CC_CS, LOW); //CC1101 Abschalten
            Serial.println("Genug Wetter Nachrichten empfangen - Starte LoRa Sender");
            setup_lora(); 
          }
          else{//Fall verbunden indicator ist höher als 0??
            os_runloop_once();  
            Serial.println("BIN hier drin");   
          }
       }
    #endif
    #ifdef RFM95_WEATHER_DECODER
      if(verbunden_indicator == LORA_TX_INIT && count_decoded_wind_messages < 3){  
        bool decode_ok = weatherSensor.getMessage();
        if (!decode_ok) {
          return; /// Mach nix wenn nix rempfangen wurde
        }
        else{
          tryReadAndDecodeRFM95();
         }
       }
       else{
          if (count_decoded_wind_messages >= 3 && setup_lora_sucesful_flag == 0){
            #ifdef SSD1306_DISPLAY
              displayWeatherData();
            #endif  
            Serial.println("Genug Wetter Nachrichten empfangen - Starte LoRa Sender");
            setup_lora(); 
          }
          else{//Fall verbunden indicator ist höher als 0??
            os_runloop_once();     
          }
       }
    #endif
    
    if (verbunden_indicator == LORA_TX_COMPLETED){
     kurzschlaf();
    }
   
}

//Temperaturmessungen

#ifdef BME280_CONNECTED
    int BME_Start(void){
      bool status;
        Wire.begin(sda, scl);
        status = bme.begin(BME_ADRESS);   
        Serial.print("BME check: "); Serial.print(status);
        if (!status) {
            Serial.println("No BME280 I2C sensor, check wiring!");
            return 0;
        }
        else{
          Serial.println("BME gefunden");
          return 1;
        }
    }
    
    void BME_Test(void){
         
          Serial.print("BME280 I2C: ");
          Serial.print("Temp = ");
          Serial.print(bme.readTemperature());
          Serial.print(" *C ");
      
          Serial.print("Druck = ");
          Serial.print(bme.readPressure() / 100.0F);
          Serial.print(" hPa ");
    
          Serial.print("Feuchte = ");
          Serial.print(bme.readHumidity());
          Serial.print(" % \n");
          
          temp = bme.readTemperature();
          pressure = bme.readPressure()/ 100.0F;
          humidity = bme.readHumidity();
          SAVE_DATA_2_RTC((int)temp, &Temp_Werte);
          SAVE_DATA_2_RTC((int)humidity, &Feuchte_Werte);
          //SAVE_DATA_2_RTC((int)(pressure-870.0), &Druck_Werte);//Alte Version mit Uint8_t 
          SAVE_DATA_2_RTC((int)(pressure), &Druck_Werte);
    }
#endif

#ifdef BMP280_CONNECTED
    int BMP_Start(void){
      bool status;
        Wire.begin(sda, scl);
        
          status = bmp.begin(BMP_ADRESS,0x58); //alternative BMP Adress
          //status = bmp.begin();  
        
        
        Serial.print("BMP check: "); Serial.print(status);
        if (!status) {
            Serial.println("No BMP280 I2C sensor, check wiring!");
            return 0;
        }
        else{
          Serial.println(" BMP280 gefunden");
          /* Default settings from datasheet. */
          bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
          return 1;
        }
    }
    
    void BMP_Test(void){
                           
          temp = bmp.readTemperature();
          pressure = bmp.readPressure()/ 100.0F;
          //SAVE_DATA_2_RTC((int)temp, Temp_Werte);Serial.print("Temp:");Serial.println((int)temp);
          //SAVE_DATA_2_RTC((int)pressure, Druck_Werte);Serial.print("Druck:");Serial.println((int)pressure);
          SAVE_DATA_2_RTC((int)temp, &Temp_Werte);
          //SAVE_DATA_2_RTC((int)(pressure-870.0), &Druck_Werte); //870 = minimal jemals gemessener Druck-Alte Version
          SAVE_DATA_2_RTC((int)(pressure), &Druck_Werte);
    }
#endif 

#ifdef CCS811_CONNECTED
  void CCS811_Test(void){
      if (myCCS811.dataAvailable()){
          //Calling this function updates the global tVOC and eCO2 variables
          myCCS811.readAlgorithmResults();
          ccs811_CO2 = float(myCCS811.getCO2());
          ccs811_TVOC = float(myCCS811.getTVOC());
          
          Serial.print("CCS811 I2C: CO2 = "); Serial.print(ccs811_CO2); Serial.print(" ppm");
          Serial.print(" TVOC: "); Serial.print(ccs811_TVOC); Serial.println(" ppb");
          //Serial.print(myCCS811.getCO2());
          //Serial.print(myCCS811.getTVOC());
          
          #ifdef BME280_CONNECTED //for Calibration of Temp and humidity
            float BMEtempC = bme.readTemperature();
            float BMEhumid = bme.readHumidity();
            //Recalibration of CCS822 with BME280 Data
            myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);
            Serial.print("Set CCS811 new values (deg C, %): ");
            Serial.print(BMEtempC); Serial.print(",");
            Serial.println(BMEhumid); Serial.println();
          #endif    
      }
  }
#endif

void kurzschlaf(void){
  //Serial.println("Schlafe in 5 sec...ztztztzt");
  SAVED_boot_counter++;
  delay(250);
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 1);
  #define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */ 
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  Serial.println("\n Going to sleep now");
  delay(500);
  esp_deep_sleep_start();
  delay(1000);
}




/* Scan available networks and sort them in order to their signal strength. */
void scanAndSort() {
  WiFi.softAPdisconnect();
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  delay(WIFI_DELAY);
  memset(ssid2, 0, MAX_SSID_LEN);
  int n = WiFi.scanNetworks();
  Serial.println("Scan done!");
  if (n == 0) {
    Serial.println("No networks found!");
  } else {
    Serial.print(n);
    Serial.println(" networks found.");
    int indices[n];
    for (int i = 0; i < n; i++) {
      indices[i] = i;
    }
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (WiFi.RSSI(indices[j]) > WiFi.RSSI(indices[i])) {
          std::swap(indices[i], indices[j]);
        }
      }
    }
    
    for (int i = 0; i < n; ++i) {
      Serial.print(WiFi.SSID(indices[i]));
      Serial.print(" ");
      Serial.print(WiFi.RSSI(indices[i]));
      Serial.print(" ");
      Serial.print(WiFi.encryptionType(indices[i]));
      Serial.print(" ");
      Serial.println(WiFi.BSSIDstr(indices[i]));
      //Serial.print();
      
      if (i < 4 && n > 2){ //nur die stäksten 3 Wifis , nur sortieren ablegen wenn min 3 Wifis vorhanden
       
       Found_Wifi[i].mac = WiFi.BSSIDstr(indices[i]); 
       Found_Wifi[i].EmpfangsPegel = WiFi.encryptionType(indices[i]);
       Found_Wifi[i].Encrypted = 0;
       Found_Wifi[i].mac.replace(":", "");
       Serial.print(" Länge: "); Serial.print(Found_Wifi[i].mac.length());Serial.print(" MAC: "); Serial.println(Found_Wifi[i].mac);
       //Serial.print("0:");  Serial.println(WiFi.BSSID(indices[i])[0]);
       //Serial.print("1:");  Serial.println(WiFi.BSSID(indices[i])[1]);
       //Serial.print("2:");  Serial.println(WiFi.BSSID(indices[i])[2]);
       //Serial.print("3:");  Serial.println(WiFi.BSSID(indices[i])[3]);
       //Serial.print("4:");  Serial.println(WiFi.BSSID(indices[i])[4]);
       //Serial.print("5:");  Serial.println(WiFi.BSSID(indices[i])[5]);
       
       Found_Wifi[i].mac0 = WiFi.BSSID(indices[i])[0];
       Found_Wifi[i].mac1 = WiFi.BSSID(indices[i])[1];
       Found_Wifi[i].mac2 = WiFi.BSSID(indices[i])[2];
       Found_Wifi[i].mac3 = WiFi.BSSID(indices[i])[3];
       Found_Wifi[i].mac4 = WiFi.BSSID(indices[i])[4];
       Found_Wifi[i].mac5 = WiFi.BSSID(indices[i])[5];
       Found_Wifi[i].EmpfangsPegel = WiFi.RSSI(indices[i]);
      }
      //if(WiFi.encryptionType(indices[i]) == WIFI_AUTH_OPEN) {
      //  Serial.println("Found non-encrypted network. Store it and exit to connect.");
      //  memset(ssid2, 0, MAX_SSID_LEN);
      //  strncpy(ssid2, WiFi.SSID(indices[i]).c_str(), MAX_SSID_LEN);
      //  break;
      //}
    }
  }
}
#ifdef WS2812_LEDS
  void fadeall() { for(int i = 0; i < ANZAHL_LEDS; i++) { leds[i].nscale8(250); } }
#endif

#ifdef WS2812_LEDS
  void rainbow()
  {
    // FastLED's built-in rainbow generator
    fill_rainbow( leds, ANZAHL_LEDS, gHue, 7);
    FastLED.show();  
  }
  void einzel(void){
      for(int i = 0; i < ANZAHL_LEDS; i++) {
        // Set the i'th led to red 
        if (counter < 20)      leds[i] = CRGB::Green;
        else if (counter < 30) leds[i] = CRGB::Yellow;
        else                      leds[i] = CRGB::Red;
        // Show the leds
        FastLED.show(); 
      }
  }
#endif
#ifdef SK6812_LEDS
  void rainbow(){
    static uint8_t hue;
   
    for(int i = 0; i < ANZAHL_LEDS; i++){
      leds[i] = CHSV((i * 256 / ANZAHL_LEDS) + hue, 255, 255);
    }
    FastLED.show();
    hue++;
  }
  
  void rainbowLoop(){
    long millisIn = millis();
    long loopTime = 5000; // 5 seconds
   
    while(millis() < millisIn + loopTime){
      rainbow();
      delay(5);
    }
  }
  //Schaltet Alle Leds nacheinander an /mit delay
  void einzel(void){
    for(int i = 0; i < ANZAHL_LEDS; i++){
      leds[i] = CRGBW(0, 0, 0, 100);
      FastLED.show();
      delay(250);
    }
  }
  //Schaltet Einzelne Leds gezielt an
  void einzel_led(uint8_t led_nummer, uint8_t red, uint8_t  green, uint8_t blue, uint8_t white){
    leds[led_nummer] = CRGBW(red, green, blue, white);
    FastLED.show();
  }

    //Fährt gleichzeitig alle LEDs an und bringt die Helligkeit auf den Zielwert
  void fade_white(uint8_t zielwert){
      Serial.print("Millis: ");           Serial.print(millis());
      Serial.print(" Aktueller Wert: ");   Serial.print(hue);
      Serial.print(" Zielwert:");         Serial.println(zielwert);
      if(hue < zielwert){
        hue++;  
      }
      else if (hue == zielwert){
        zielwert_erreicht = 1;
        Serial.println("Zielwert erreicht");
      }
      else{
        hue--;
      }
      fill_solid(leds, ANZAHL_LEDS, CRGBW(0,0,0,hue));
      FastLED.show();
  }
  
  //fährt Nacheinander alle Leds in einen bestimmten Wert
  //Dazu muss der Zielwert erreicht sein
  void fade_white_single(uint8_t startwert, uint8_t zielwert, int x){
      int dimmen_um_wert=10;
      Serial.print("Millis: ");          Serial.print(millis());
      Serial.print(" Diode:");          Serial.print(x);
      Serial.print(" Aktueller Wert: "); Serial.print(hue2);
      Serial.print(" Zielwert:");       Serial.println(zielwert);
      
      if(hue2 < zielwert){
        hue2+=dimmen_um_wert;  
        leds[x] = CRGBW(0, 0, 0, hue2);
      }
      else if (hue2 == zielwert){
        zielwert_erreicht = 1;
        hue2 = startwert; //rücksetzen des aktuellen Licht Werts für nächste LED
        Serial.println("Zielwert erreicht");
        Aktuelle_LED++;
      }
      else{
        hue2-=dimmen_um_wert;
        leds[x] = CRGBW(0, 0, 0, hue2);
      }    
      FastLED.show();
  }

  
#endif
#ifdef WS2812_LEDS
  void dimmen(void){
    fill_gradient(leds, 0, CHSV(lastcounter, 255,255), ANZAHL_LEDS, CHSV(counter,255,255), SHORTEST_HUES);    // up to 4 CHSV values
    FastLED.show();
  }
  
  void dimmen(int farbe_jetzt, int farbe_vorher){
    fill_gradient(leds, 0, CHSV(farbe_jetzt, 255,255), ANZAHL_LEDS, CHSV(farbe_vorher,255,255), SHORTEST_HUES);    // up to 4 CHSV values
    FastLED.show();
  }
#endif
void LEDS_weiss(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(55, 55, 55);  
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,0,0, 25); 
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();  
  #endif
  
}



void LEDS_aus(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(0, 0, 0);  
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,0,0,0);
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();
   #endif
  
}

void LEDS_rot(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(20, 0, 0);  
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(25,0,0, 0);   
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();
  #endif
  
}

void LEDS_blau(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(0, 0, 20);  
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,0,25, 0);   
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();
  #endif
  
}

void LEDS_gruen(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(0, 20, 0);  
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,25,0, 0);   
    fill_solid(leds, ANZAHL_LEDS, color);
    FastLED.show();
  #endif
  
}

void LEDS_gruen_oben(void){
  #ifdef WS2812_LEDS
    CRGB color = CRGB(0, 20, 0);  
    fill_solid(leds2, OBEN_LEDS, color);
    FastLED.show();
  #endif
  #ifdef SK6812_LEDS
    CRGBW color = CRGBW(0,25,0, 0);   
    fill_solid(leds2, OBEN_LEDS, color);
    FastLED.show();
  #endif
 
}

#ifdef RFM95_WEATHER_DECODER
    
    void tryReadAndDecodeRFM95() {

              
              const float METERS_SEC_TO_MPH = 2.237;
              Serial.printf("Id: [%8X] Battery: [%s] ",
                  weatherSensor.sensor_id,
                  weatherSensor.battery_ok ? "OK " : "Low");
              #ifdef BRESSER_6_IN_1
                  Serial.printf("Ch: [%d] ", weatherSensor.chan);
              #endif
              if (weatherSensor.temp_ok) {
                  Serial.printf("Temp: [%5.1fC] Hum: [%3d%%] ",weatherSensor.temp_c, weatherSensor.humidity);
                  temp_cc1101 = weatherSensor.temp_c;
                  humidity_cc1101 = weatherSensor.humidity;
              } else {
                  Serial.printf("Temp: [---.-C] Hum: [---%%] ");
              }
              if (weatherSensor.wind_ok) {
                  Serial.printf("Wind max: [%4.1fm/s] Wind avg: [%4.1fm/s] Wind dir: [%5.1fdeg] ", weatherSensor.wind_gust_meter_sec, weatherSensor.wind_avg_meter_sec, weatherSensor.wind_direction_deg);
                  count_decoded_wind_messages++;
                  wind_speed_cc1101 = (int)(weatherSensor.wind_avg_meter_sec *10);
                  wind_speed_max_cc1101 = (int)(weatherSensor.wind_gust_meter_sec *10);
                  wind_dir_cc1101 = weatherSensor.wind_direction_deg;
              } else {
                  Serial.printf("Wind max: [--.-m/s] Wind avg: [--.-m/s] ");
              }
              if (weatherSensor.rain_ok) {
                  Serial.printf("Rain: [%7.1fmm] ", weatherSensor.rain_mm);
                  rain_cc1101 = weatherSensor.rain_mm;
              } else {
                  Serial.printf("Rain: [-----.-mm] "); 
              }
              if (weatherSensor.moisture_ok) {
                  Serial.printf("Moisture: [%2d%%] ", weatherSensor.moisture);
                  //humidity_cc1101 = weatherSensor.moisture;
              }
              Serial.printf("RSSI: [%4.1fdBm]\n", weatherSensor.rssi);
                  }
            
      
#endif

#ifdef CC1101_CONNECTED
    //CC1101
    void tryReadAndDecode() {
        int state = radio.readData(fullMessage, 27);
        byte *msg = &fullMessage[1];
        //Serial.print("LQI:");  Serial.print(radio.getLQI());Serial.print(" ");
        uint8_t lqi_quality = radio.getLQI();
        if (state == RADIOLIB_ERR_NONE && fullMessage[0] == 0xd4 && lqi_quality < 90) {
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

            Serial.print(" Id: ");Serial.print(id);
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
              temp_cc1101 = temp_c;
              humidity_cc1101 = humidity;
              
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
              rain_cc1101 = rain_mm;
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
              //Serial.print(" Wind_OK: ");Serial.print(wind_ok);
              Serial.print(" Windrichtung: ");Serial.print(wind_dir);
              Serial.print(" Wind_Gust: ");Serial.println(wind_gust);
              wind_speed_cc1101 = (int)(wind_avg *10);
              wind_speed_max_cc1101 = (int)(wind_gust *10);
              wind_dir_cc1101 = wind_dir;
              count_decoded_wind_messages++;
              
            }
            
        }
    }
    
    void setup_cc1101(void){
        //Legt Pins auf sauberen Zustand um Dreckeffekte beim Start durch PWM zu vermeiden
        #ifdef TTGO105_CONNECTED
         //Pin Nur input kein Output
       #else
        pinMode(CC_MOSI, OUTPUT); digitalWrite(CC_MOSI, LOW);// Serial.println(F("MOSI"));      
        pinMode(CC_MISO, OUTPUT); digitalWrite(CC_MISO, LOW);// Serial.println(F("MISO"));
       #endif        
       pinMode(CC_SCK, OUTPUT); digitalWrite(CC_SCK, LOW); //Serial.println(F("SCK"));
       pinMode(CC_GD2, OUTPUT); digitalWrite(CC_GD2, LOW);//Serial.println(F("GD2"));
       pinMode(CC_GD0, OUTPUT); digitalWrite(CC_GD0, LOW);//Serial.println(F("GD0"));
       pinMode(CC_CS, OUTPUT); digitalWrite(CC_CS, LOW);//Serial.println(F("CS"));
      spiCC1101.begin(CC_SCK, CC_MISO, CC_MOSI, CC_CS);
      Serial.print(F("[CC1101] Initializing ... "));   
      int state = radio.begin(868.30, 8.22, 60 , 270.0, 10, 32);//(freq, br, freqDev, rxBW, power, preambleLength)
      state = radio.disableAddressFiltering();
      if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
        state = radio.setCrcFiltering(false);
            if (state != RADIOLIB_ERR_NONE) {
                Serial.println(F("FAILED SET CRC FILTERING"));
                delay(3000); ESP.restart();
            }
            delay(1000);
            state = radio.fixedPacketLengthMode(27);
            if (state != RADIOLIB_ERR_NONE) {
                Serial.println(F("FAILED SET PACKET LENGTH MODE"));
                delay(3000); ESP.restart();
            }
            delay(1000);
            state = radio.setSyncWord(0xAA, 0X2d, 0, false);
            if (state != RADIOLIB_ERR_NONE) {
                Serial.println(F("FAILED SET SYNC WORD MODE"));
                delay(3000); ESP.restart();    
            }
        
      } else {
        Serial.print(F("failed, code ")); Serial.println(state);
        delay(3000); ESP.restart();
      }
      state = radio.enableSyncWordFiltering();
      if (state != RADIOLIB_ERR_NONE) {
            Serial.println(F("FAILED ENABLE SYNC WORD FILTERING"));
            delay(3000);ESP.restart();
      }
  
      
      state = radio.startReceive();
      Serial.println("Start Reciever ");
    
      // Set interrupt function.
      radio.setGdo0Action(setFlag);
        
    }
#endif

 #ifdef SSD1306_DISPLAY
      /*float temp_cc1101 =0;
  float humidity_cc1101 =0;
  int pressure_cc1101 =0;
  int rain_cc1101 =0;
  int wind_speed_cc1101 =0;
  int wind_speed_max_cc1101 =0;
  int wind_dir_cc1101 =0;
  int count_decoded_wind_messages =0;*/
      void displayWeatherData(void){
                display.setTextSize(1);
                display.setCursor(0,10); display.print("Regen:");
                display.setCursor(50,10);display.print(rain_cc1101); 
                display.setCursor(0,20); display.print("Wind:");
                display.setCursor(50,20);display.print(wind_speed_cc1101); 
                display.setCursor(0,30); display.print("WindRichtung:");
                display.setCursor(80,30); display.print(wind_dir_cc1101);      
                display.setCursor(0,40); display.print("Temp:");
                display.setCursor(50,40); display.print(temp_cc1101);      
                display.setCursor(0,50); display.print("Feuchte:");
                display.setCursor(50,50); display.print(humidity_cc1101);      
                display.display();
              }
#endif              