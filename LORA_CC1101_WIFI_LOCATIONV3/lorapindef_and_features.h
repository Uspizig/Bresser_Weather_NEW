
#define LORA_TX_INIT 0
#define LORA_TX_JOINING 1
#define LORA_TX_JOINED 2
#define LORA_TX_STARTED 3
#define LORA_TX_COMPLETED 4



// Pinning

#ifdef geraet1
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4//19
  #define E_INK_PIN_SPI_RST  16//21
  #define E_INK_PIN_SPI_DC   17//22
  #define E_INK_PIN_SPI_CS   5
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for TTGO T5 Addon Board
  #define LORA_PIN_SPI_MOSI 15//19
  #define LORA_PIN_SPI_MISO 2//n/A
  #define LORA_PIN_SPI_SCK  14
  #define LORA_PIN_SPI_NSS  26  
  #define LORA_PIN_SPI_RST  33  
  #define LORA_PIN_SPI_DIO1 25
  #define LORA_PIN_SPI_DIO0 34

  
  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //msb0x12
#endif
#ifdef geraet2
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4//19
  #define E_INK_PIN_SPI_RST  16//21
  #define E_INK_PIN_SPI_DC   17//22
  #define E_INK_PIN_SPI_CS   5
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for TTGO T5 Addon Board
  #define LORA_PIN_SPI_MOSI 15//19
  #define LORA_PIN_SPI_MISO 2//n/A
  #define LORA_PIN_SPI_SCK  14
  #define LORA_PIN_SPI_NSS  26  
  #define LORA_PIN_SPI_RST  33  
  #define LORA_PIN_SPI_DIO1 25
  #define LORA_PIN_SPI_DIO0 34

  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //msb0x12
  
#endif
#ifdef geraet3 //ESP32 TTGO MINI V2
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4
  #define E_INK_PIN_SPI_RST  16
  #define E_INK_PIN_SPI_DC   17
  #define E_INK_PIN_SPI_CS   13
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for TTGO ESP32 Mini V2 mit EINK+Lora Addon Board
  #define LORA_PIN_SPI_MOSI 5
  #define LORA_PIN_SPI_MISO 19
  #define LORA_PIN_SPI_SCK  18//14 = 1 Jumperstellung //18 = 3 Jumperstellung
  #define LORA_PIN_SPI_NSS  26  
  #define LORA_PIN_SPI_RST  33  
  #define LORA_PIN_SPI_DIO1 27
  #define LORA_PIN_SPI_DIO0 27

  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //msb0x12
#endif

#ifdef geraet4 //ESP32 CAM
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4
  #define E_INK_PIN_SPI_RST  16
  #define E_INK_PIN_SPI_DC   17
  #define E_INK_PIN_SPI_CS   13
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for TTGO CAM
  #define LORA_PIN_SPI_MOSI 13
  #define LORA_PIN_SPI_MISO 12//n/A
  #define LORA_PIN_SPI_SCK  14
  #define LORA_PIN_SPI_NSS  00  
  #define LORA_PIN_SPI_RST  03//27   
  #define LORA_PIN_SPI_DIO1 16
  #define LORA_PIN_SPI_DIO0 16

  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //msb0x12
#endif


#ifdef geraet5 //ESP32 TTGO MINI V2
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4
  #define E_INK_PIN_SPI_RST  16
  #define E_INK_PIN_SPI_DC   17
  #define E_INK_PIN_SPI_CS   13
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for TTGO ESP32 Mini V2 mit EINK+Lora Addon Board
  #define LORA_PIN_SPI_MOSI 23
  #define LORA_PIN_SPI_MISO 19//n/A
  #define LORA_PIN_SPI_SCK  18
  #define LORA_PIN_SPI_NSS  26  
  #define LORA_PIN_SPI_RST  03//27   
  #define LORA_PIN_SPI_DIO1 05
  #define LORA_PIN_SPI_DIO0 05
  
  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //msb0x12
#endif

#ifdef geraet6 //ESP32 TTGO MINI V2
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4
  #define E_INK_PIN_SPI_RST  16
  #define E_INK_PIN_SPI_DC   17
  #define E_INK_PIN_SPI_CS   13
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for TTGO ESP32 Mini V2 mit EINK+Lora Addon Board
  #define LORA_PIN_SPI_MOSI 23
  #define LORA_PIN_SPI_MISO 19//n/A
  #define LORA_PIN_SPI_SCK  18
  #define LORA_PIN_SPI_NSS  26  
  #define LORA_PIN_SPI_RST  03//27   
  #define LORA_PIN_SPI_DIO1 05
  #define LORA_PIN_SPI_DIO0 05

  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //msb0x12
#endif

#ifdef geraet7 //ESP32 Schreibtischlampe V1
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4
  #define E_INK_PIN_SPI_RST  16
  #define E_INK_PIN_SPI_DC   17
  #define E_INK_PIN_SPI_CS   13
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for ESP32 Schreibtischlampe V1
  #define LORA_PIN_SPI_MOSI 19
  #define LORA_PIN_SPI_MISO 23//n/A
  #define LORA_PIN_SPI_SCK  05
  #define LORA_PIN_SPI_NSS  16  
  #define LORA_PIN_SPI_RST  03//27   
  #define LORA_PIN_SPI_DIO1 27
  #define LORA_PIN_SPI_DIO0 27

  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //msb0x12
#endif

#ifdef geraet8 //ESP32 Base Board with Bresser Weatherstation CC1101
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
  #define LORA_PIN_SPI_MOSI 17
  #define LORA_PIN_SPI_MISO 32//n/A
  #define LORA_PIN_SPI_SCK  18
  #define LORA_PIN_SPI_NSS  19  
  #define LORA_PIN_SPI_RST  03//27   
  #define LORA_PIN_SPI_DIO1 27
  #define LORA_PIN_SPI_DIO0 27

//CC1101 Connections
  #define CC_MOSI 22
  #define CC_SCK 21
  #define CC_MISO 5
  #define CC_GD2 23
  #define CC_GD0 15
  #define CC_CS  13

  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //msb0x12
#endif

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
  #define LORA_PIN_SPI_DIO1 27
  #define LORA_PIN_SPI_DIO0 27

//CC1101 Connections
  #define CC_MOSI 21
  #define CC_SCK 22
  #define CC_MISO 10
  #define CC_GD2 9
  #define CC_GD0 4
  #define CC_CS  2

  #define APP_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define DEV_EUI_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //lsb
  #define APP_KEY_LORA 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 //msb0x12
#endif

//EEPROM HASH für Joins OTAA gegen die geprüft wird

#ifdef geraet1
  #define DATAVALID 0xACF2AFC2//1                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet2
  #define DATAVALID 0xACF2AFC2//3                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet3
  #define DATAVALID 0xACF2AFCA                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet4
  #define DATAVALID 0xACF2AFCA                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet5
  #define DATAVALID 0xACF2AFCA                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet6
  #define DATAVALID 0xACF2AFCB                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet7
  #define DATAVALID 0xACF2AFCC                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet8
  #define DATAVALID 0xACF2AFCA                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet9
  #define DATAVALID 0xACF2AFCB                     // Pattern for data valid in EEPROM/RTC memory
#endif
// Features

#ifdef geraet1
  #define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 20; 
  #define BMP_ADRESS 0x76
  #define BMP280_CONNECTED //geraet1
#endif
#ifdef geraet2 //limit uplinks on testboard
  #define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 180; 
  #define BMP_ADRESS 0x76
  #define BMP280_CONNECTED //geraet1
#endif
#ifdef geraet3 //limit uplinks on testboard
  #define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 45; 
  #define BME280_CONNECTED
  #define BME_ADRESS 0x77
  #define CCS811_CONNECTED
  #define CCS811_ADRESS 0x5A
#endif
#ifdef geraet4 //limit uplinks on testboard
  #define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 45; 
  /*#define BME280_CONNECTED
  #define BME_ADRESS 0x77
  #define CCS811_CONNECTED
  #define CCS811_ADRESS 0x5A*/
#endif
#ifdef geraet5 //limit uplinks on testboard
  #define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 45; 
  /*#define BME280_CONNECTED
  #define BME_ADRESS 0x77
  #define CCS811_CONNECTED
  #define CCS811_ADRESS 0x5A*/
#endif
#ifdef geraet6 //limit uplinks on testboard
  #define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 40; //180
  /*#define BME280_CONNECTED
  #define BME_ADRESS 0x77
  #define CCS811_CONNECTED
  #define CCS811_ADRESS 0x5A*/
  #define ANZAHL_LEDS 6//36
  #define RGB_LED_DATA_PIN 25
  
  #define OBEN_LED_DATA_PIN 26
  #define OBEN_LEDS 1
  #define SK6812_LEDS
#endif
#ifdef geraet7 //limit uplinks on testboard
  #define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 40; //180
  #define CC1101_CONNECTED
  #define WIFISCAN_CONNECTED
  #define OTAWIFI_ENABLED
  /*#define BME280_CONNECTED
  #define BME_ADRESS 0x77
  #define CCS811_CONNECTED
  #define CCS811_ADRESS 0x5A*/
  #define ANZAHL_LEDS 6//36
  #define RGB_LED_DATA_PIN 25
  
  #define OBEN_LED_DATA_PIN 26
  #define OBEN_LEDS 1
  #define SK6812_LEDS
  
#endif
#ifdef geraet8 //ESP32 Base Board with Bresser CC1101
  #define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 40; //180
  #define CC1101_CONNECTED
  //#define WIFISCAN_CONNECTED
  //#define OTAWIFI_ENABLED
  /*#define BME280_CONNECTED
  #define BME_ADRESS 0x77
  #define CCS811_CONNECTED
  #define CCS811_ADRESS 0x5A*/
  #define ANZAHL_LEDS 1//36
  #define RGB_LED_DATA_PIN 25
  
  #define OBEN_LED_DATA_PIN 26
  #define OBEN_LEDS 1
  //#define SK6812_LEDS
  #define WS2812_LEDS
  
#endif

#ifdef geraet9 //ESP32 Base Board with Bresser CC1101
  #define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */
  const unsigned TX_INTERVAL = 40; //180
  #define CC1101_CONNECTED
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
