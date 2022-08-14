
#define LORA_TX_INIT 0
#define LORA_TX_JOINING 1
#define LORA_TX_JOINED 2
#define LORA_TX_STARTED 3
#define LORA_TX_COMPLETED 4



// Pinning


#ifdef geraet9 //ESP32 Base Board with Bresser Weatherstation CC1101
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4
  #define E_INK_PIN_SPI_RST  16
  #define E_INK_PIN_SPI_DC   17
  #define E_INK_PIN_SPI_CS   13
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for ESP32 Base Board with Bresser Weather Station
  #define LORA_PIN_SPI_MOSI 23
  #define LORA_PIN_SPI_MISO 19//n/A
  #define LORA_PIN_SPI_SCK  18
  #define LORA_PIN_SPI_NSS  5  
  #define LORA_PIN_SPI_RST  14//27   
  #define LORA_PIN_SPI_DIO1 25
  #define LORA_PIN_SPI_DIO0 27
 
//CC1101 Connections --- NOT Really NEEDED
  #define CC_MOSI 21
  #define CC_SCK 22
  #define CC_MISO 10
  #define CC_GD2 9
  #define CC_GD0 4
  #define CC_CS  2

  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //(use LSB ORDER)
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //use LSB Order
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // use MSB ORDER
#endif

#ifdef geraet10 //TTGO LORA 32 V1.6(Paxcounter Board)
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4
  #define E_INK_PIN_SPI_RST  16
  #define E_INK_PIN_SPI_DC   17
  #define E_INK_PIN_SPI_CS   13
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for ESP32 Base Board with Bresser Weather Station
  #define LORA_PIN_SPI_MOSI 27
  #define LORA_PIN_SPI_MISO 19
  #define LORA_PIN_SPI_SCK  5
  #define LORA_PIN_SPI_NSS  18  
  #define LORA_PIN_SPI_RST  23//27   
  #define LORA_PIN_SPI_DIO1 12
  #define LORA_PIN_SPI_DIO0 26

//CC1101 Connections --- NOT Really NEEDED
  #define CC_MOSI 21
  #define CC_SCK 22
  #define CC_MISO 10
  #define CC_GD2 9
  #define CC_GD0 4
  #define CC_CS  2

  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //use LSB order
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //use LSB order
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //use MSB order
#endif

//EEPROM HASH für Joins OTAA gegen die geprüft wird


#ifdef geraet9
  #define DATAVALID 0xACF2AFCB                        // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet10
  #define DATAVALID 0xACF2AFCB                     // Pattern for data valid in EEPROM/RTC memory
#endif
// Features



#ifdef geraet9 //ESP32 Base Board with Bresser CC1101
  #define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 40; //180
  //#define CC1101_CONNECTED
  #define RFM95_WEATHER_DECODER
  //#define WIFISCAN_CONNECTED
  //#define OTAWIFI_ENABLED
  /*#define BME280_CONNECTED
  #define BME_ADRESS 0x77
  #define CCS811_CONNECTED
  #define CCS811_ADRESS 0x5A*/
  #define ANZAHL_LEDS 1//36
  #define RGB_LED_DATA_PIN 0
  
  #define OBEN_LED_DATA_PIN 26
  #define OBEN_LEDS 1
  //#define SK6812_LEDS
  //#define WS2812_LEDS
  
#endif

#ifdef geraet10 //ESP32 TTGO LORA 32 V1.6
  #define TIME_TO_SLEEP  100        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 40; //180 TX Interval in Seconds
  #define SSD1306_DISPLAY
  //#define CC1101_CONNECTED
  #define RFM95_WEATHER_DECODER
  #define WIFISCAN_CONNECTED
  //#define OTAWIFI_ENABLED
  //#define BME280_CONNECTED
  //#define BME_ADRESS 0x77
  //#define CCS811_CONNECTED
  //#define CCS811_ADRESS 0x5A
  #define ANZAHL_LEDS 1//36
  #define RGB_LED_DATA_PIN 0
  
  #define OBEN_LED_DATA_PIN 27
  #define OBEN_LEDS 1
  //#define SK6812_LEDS
  //#define WS2812_LEDS
  
#endif
