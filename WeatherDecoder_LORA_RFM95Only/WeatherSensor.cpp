///////////////////////////////////////////////////////////////////////////////////////////////////
// WeatherSensor.cpp
//
// Bresser 5-in-1/6-in-1 868 MHz Weather Sensor Radio Receiver 
// based on CC1101 or SX1276/RFM95W and ESP32/ESP8266
//
// https://github.com/matthias-bs/WeatherSensor
//
// Based on:
// ---------
// Bresser5in1-CC1101 by Sean Siford (https://github.com/seaniefs/Bresser5in1-CC1101)
// RadioLib by Jan Gromeš (https://github.com/jgromes/RadioLib)
// rtl433 by Benjamin Larsson (https://github.com/merbanan/rtl_433) 
//     - https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_5in1.c
//     - https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_6in1.c
//
// created: 05/2022
//
//
// MIT License
//
// Copyright (c) 2022 Matthias Prinke
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// History:
//
// 20220523 Created from https://github.com/matthias-bs/Bresser5in1-CC1101
// 20220524 Moved code to class WeatherSensor
// 20220526 Implemented getData(), changed debug output to macros,
//          changed radio transceiver instance to member variable of WeatherSensor
//          (with initialization of 'Module' from WeatherSensor's constructor parameters)
//
//
// ToDo: 
// -
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#include "WeatherSensor.h"

#if defined(USE_CC1101)
    static CC1101 radio = new Module(PIN_RECEIVER_CS, PIN_RECEIVER_IRQ, RADIOLIB_NC, PIN_RECEIVER_GPIO);
#endif
#if defined(USE_SX1276)
//    static SX1276 radio = new Module(PIN_RECEIVER_CS, PIN_RECEIVER_IRQ, PIN_RECEIVER_RST, PIN_RECEIVER_GPIO);
SPIClass spiCC1101(HSPI);//https://github.com/espressif/arduino-esp32/issues/1219      
static SX1276 radio = new Module(PIN_RECEIVER_CS, PIN_RECEIVER_IRQ, PIN_RECEIVER_RST, PIN_RECEIVER_GPIO,  spiCC1101, SPISettings());
#endif

int16_t WeatherSensor::begin(void) {
    // https://github.com/RFD-FHEM/RFFHEM/issues/607#issuecomment-830818445
    // Freq: 868.300 MHz, Bandwidth: 203 KHz, rAmpl: 33 dB, sens: 8 dB, DataRate: 8207.32 Baud
    DEBUG_PRINT(RECEIVER_CHIP);
    DEBUG_PRINTLN(" Initializing ... ");
    // carrier frequency:                   868.3 MHz
    // bit rate:                            8.22 kbps
    // frequency deviation:                 57.136417 kHz
    // Rx bandwidth:                        270.0 kHz (CC1101) / 250 kHz (SX1276)
    // output power:                        10 dBm
    // preamble length:                     40 bits
    #ifdef USE_CC1101
        int state = radio.begin(868.3, 8.21, 57.136417, 270, 10, 32);
    #else
        spiCC1101.begin(CC_SCK, CC_MISO, CC_MOSI, PIN_RECEIVER_CS);
        int state = radio.beginFSK(868.3, 8.21, 57.136417, 250, 10, 32);
    #endif
    if (state == RADIOLIB_ERR_NONE) {
        DEBUG_PRINTLN("success!");
        state = radio.setCrcFiltering(false);
        if (state != RADIOLIB_ERR_NONE) {
            DEBUG_PRINT(RECEIVER_CHIP);
            DEBUG_PRINT(" Error disabling crc filtering: [");
            DEBUG_PRINT(state);
            DEBUG_PRINTLN("]");
            while (true)
                ;
        }
        state = radio.fixedPacketLengthMode(27);
        if (state != RADIOLIB_ERR_NONE) {
            DEBUG_PRINT(RECEIVER_CHIP);
            DEBUG_PRINT(" Error setting fixed packet length: [");
            DEBUG_PRINT(state);
            DEBUG_PRINTLN("]");
            while (true)
                ;
        }
        // Preamble: AA AA AA AA AA
        // Sync is: 2D D4 
        // Preamble 40 bits but the CC1101 doesn't allow us to set that
        // so we use a preamble of 32 bits and then use the sync as AA 2D
        // which then uses the last byte of the preamble - we recieve the last sync byte
        // as the 1st byte of the payload.
        #ifdef USE_CC1101
            state = radio.setSyncWord(0xAA, 0x2D, 0, false);
        #else
            uint8_t sync_word[] = {0xAA, 0x2D};
            state = radio.setSyncWord(sync_word, 2);
        #endif
        if (state != RADIOLIB_ERR_NONE) {
            DEBUG_PRINT(RECEIVER_CHIP);
            DEBUG_PRINT(" Error setting sync words: [");
            DEBUG_PRINT(state);
            DEBUG_PRINTLN("]");
            while (true)
                ;
        }
    } else {
        DEBUG_PRINT(RECEIVER_CHIP);
        DEBUG_PRINT(" Error initialising: [");
        DEBUG_PRINT(state);
        DEBUG_PRINTLN("]");
        while (true)
            ;
    }
    DEBUG_PRINT(RECEIVER_CHIP);
    DEBUG_PRINTLN(" Setup complete - awaiting incoming messages...");
    rssi = radio.getRSSI();
    
    return state;
}


bool WeatherSensor::getData(uint32_t timeout, bool complete, void (*func)())
{
    const uint32_t timestamp = millis();
    bool saved_temp_ok = false;
    bool saved_rain_ok = false;
    bool decode_status = false;
    
    // Reset status flag 
    data_ok = false;
    
    while ((millis() - timestamp) < timeout) { 
        decode_status = getMessage();
    
        // Callback function (see https://www.geeksforgeeks.org/callbacks-in-c/)
        if (func) {
            (*func)();
        }
        
        // BRESSER_6_IN_1 message contains either temperature OR rain data
        // Save what has already been sent
        if (decode_status) {
            if (!complete) {
                // Incomplete data data is sufficient
                return data_ok = true;
            }
            if (temp_ok) {
                saved_temp_ok = true;
            }
            if (rain_ok) {
                saved_rain_ok = true;
            }
            if (saved_temp_ok && saved_rain_ok) {
                // Data complete
                return data_ok = true;
            }
        } // if (decode_status)
    } //  while ((millis() - timestamp) < timeout)
    
    // Timeout
    return false;
}


bool WeatherSensor::getMessage(void)
{
    uint8_t recvData[27];
    
    // reset status flag
    message_ok = false;
    
    // Receive data
    //     1. flush RX buffer
    //     2. switch to RX mode
    //     3. wait for expected RX packet or timeout [~500us in this configuration]
    //     4. flush RX buffer
    //     5. switch to standby
    int state = radio.receive(recvData, 27);
    rssi = radio.getRSSI();
    
    if (state == RADIOLIB_ERR_NONE) {
        // Verify last syncword is 1st byte of payload (see setSyncWord() above)
        if (recvData[0] == 0xD4) {
            #ifdef _DEBUG_MODE_
                // print the data of the packet
                DEBUG_PRINT(RECEIVER_CHIP);
                DEBUG_PRINT(" Data:\t\t");
                for(int i = 0 ; i < sizeof(recvData) ; i++) {
                    DEBUG_PRINT((recvData[0] < 16) ? " 0" : " ");
                    DEBUG_PRINT(recvData[i], HEX);
                }
                DEBUG_PRINTLN();

                DEBUG_PRINT(RECEIVER_CHIP);
                DEBUG_PRINT(" R [");
                DEBUG_PRINT((recvData[0] < 16) ? "0" : "");
                DEBUG_PRINT(recvData[0], HEX);
                DEBUG_PRINT("] RSSI: ");
                DEBUG_PRINTLN(radio.getRSSI(), 1);
            #endif

            #ifdef _DEBUG_MODE_
                printRawdata(&recvData[1], sizeof(recvData));
            #endif

            #ifdef BRESSER_6_IN_1
                message_ok = (decodeBresser6In1Payload(&recvData[1], sizeof(recvData) - 1) == DECODE_OK);
            #else
                message_ok = (decodeBresser5In1Payload(&recvData[1], sizeof(recvData) - 1) == DECODE_OK);
          
                // Fixed set of data for 5-in-1 sensor
                temp_ok     = true;
                uv_ok       = false;
                wind_ok     = true;
                rain_ok     = true;
                moisture_ok = false;
            #endif
        } // if (recvData[0] == 0xD4)
        else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            #ifdef _DEBUG_MODE_
                DEBUG_PRINT("T");
            #endif
        } // if (state == RADIOLIB_ERR_RX_TIMEOUT)
        else {
            // some other error occurred
            DEBUG_PRINT(RECEIVER_CHIP);
            DEBUG_PRINT(" Receive failed - code: ");
            DEBUG_PRINTLN(state);
        }
    } // if (state == RADIOLIB_ERR_NONE)
    
    return message_ok;
}

//
// Generate sample data for testing
//
bool WeatherSensor::genMessage(void)
{
    sensor_id               = 0xff;
    temp_c                  = 22.2f;
    humidity                = 55;
    wind_direction_deg      = 111.1;
    wind_direction_deg_fp1  = 1111;
    wind_gust_meter_sec     = 4.4f;
    wind_gust_meter_sec_fp1 = 44;
    wind_avg_meter_sec      = 3.3f;
    wind_avg_meter_sec_fp1  = 33;
    rain_mm                 = 9.9f;
    battery_ok              = true;
    rssi                    = 88.8;

    message_ok = true;
    data_ok    = true;
    return true;
}

//
// From from rtl_433 project - https://github.com/merbanan/rtl_433/blob/master/src/util.c
//
uint16_t WeatherSensor::lfsr_digest16(uint8_t const message[], unsigned bytes, uint16_t gen, uint16_t key)
{
    uint16_t sum = 0;
    for (unsigned k = 0; k < bytes; ++k) {
        uint8_t data = message[k];
        for (int i = 7; i >= 0; --i) {
            // fprintf(stderr, "key at bit %d : %04x\n", i, key);
            // if data bit is set then xor with key
            if ((data >> i) & 1)
                sum ^= key;

            // roll the key right (actually the lsb is dropped here)
            // and apply the gen (needs to include the dropped lsb as msb)
            if (key & 1)
                key = (key >> 1) ^ gen;
            else
                key = (key >> 1);
        }
    }
    return sum;
}

//
// From from rtl_433 project - https://github.com/merbanan/rtl_433/blob/master/src/util.c
//
int WeatherSensor::add_bytes(uint8_t const message[], unsigned num_bytes)
{
    int result = 0;
    for (unsigned i = 0; i < num_bytes; ++i) {
        result += message[i];
    }
    return result;
}


// Cribbed from rtl_433 project - but added extra checksum to verify uu
//
// Example input data:
//   EA EC 7F EB 5F EE EF FA FE 76 BB FA FF 15 13 80 14 A0 11 10 05 01 89 44 05 00
//   CC CC CC CC CC CC CC CC CC CC CC CC CC uu II SS GG DG WW  W TT  T HH RR  R Bt
// - C = Check, inverted data of 13 byte further
// - uu = checksum (number/count of set bits within bytes 14-25)
// - I = station ID (maybe)
// - G = wind gust in 1/10 m/s, normal binary coded, GGxG = 0x76D1 => 0x0176 = 256 + 118 = 374 => 37.4 m/s.  MSB is out of sequence.
// - D = wind direction 0..F = N..NNE..E..S..W..NNW
// - W = wind speed in 1/10 m/s, BCD coded, WWxW = 0x7512 => 0x0275 = 275 => 27.5 m/s. MSB is out of sequence.
// - T = temperature in 1/10 °C, BCD coded, TTxT = 1203 => 31.2 °C
// - t = temperature sign, minus if unequal 0
// - H = humidity in percent, BCD coded, HH = 23 => 23 %
// - R = rain in mm, BCD coded, RRxR = 1203 => 31.2 mm
// - B = Battery. 0=Ok, 8=Low.
// - S = sensor type, only low nibble used, 0x9 for Bresser Professional Rain Gauge
//
// Parameters:
//
// msg     - Pointer to message
// msgSize - Size of message
// pOut    - Pointer to WeatherData
//
// Returns:
//
// DECODE_OK      - OK - WeatherData will contain the updated information
// DECODE_PAR_ERR - Parity Error
// DECODE_CHK_ERR - Checksum Error
//
#ifdef BRESSER_5_IN_1
DecodeStatus WeatherSensor::decodeBresser5In1Payload(uint8_t *msg, uint8_t msgSize) { 
    // First 13 bytes need to match inverse of last 13 bytes
    for (unsigned col = 0; col < msgSize / 2; ++col) {
        if ((msg[col] ^ msg[col + 13]) != 0xff) {
            DEBUG_PRINT("Parity wrong at column ");
            DEBUG_PRINTLN(col);
            return DECODE_PAR_ERR;
        }
    }

    // Verify checksum (number number bits set in bytes 14-25)
    uint8_t bitsSet = 0;
    uint8_t expectedBitsSet = msg[13];

    for(uint8_t p = 14 ; p < msgSize ; p++) {
      uint8_t currentByte = msg[p];
      while(currentByte) {
        bitsSet += (currentByte & 1);
        currentByte >>= 1;
      }
    }

    if (bitsSet != expectedBitsSet) {
        DEBUG_PRINT("Checksum wrong - actual [");
        DEBUG_PRINT((bitsSet < 16) ? "0" : "");
        DEBUG_PRINT(bitsSet, HEX);
        DEBUG_PRINT("] != [");
        //DEBUG_PRINT((expectedBits < 16) ? "0" : "");
        DEBUG_PRINT(expectedBitsSet, HEX);
        DEBUG_PRINTLN("]");
        return DECODE_CHK_ERR;
    }

    sensor_id = msg[14];

    int temp_raw = (msg[20] & 0x0f) + ((msg[20] & 0xf0) >> 4) * 10 + (msg[21] &0x0f) * 100;
    if (msg[25] & 0x0f) {
        temp_raw = -temp_raw;
    }
    temp_c = temp_raw * 0.1f;

    humidity = (msg[22] & 0x0f) + ((msg[22] & 0xf0) >> 4) * 10;

    wind_direction_deg     = ((msg[17] & 0xf0) >> 4) * 22.5f;
    wind_direction_deg_fp1 = wind_direction_deg * 10; 

    int gust_raw = ((msg[17] & 0x0f) << 8) + msg[16];
    wind_gust_meter_sec_fp1 = gust_raw;
    wind_gust_meter_sec     = gust_raw * 0.1f;

    int wind_raw = (msg[18] & 0x0f) + ((msg[18] & 0xf0) >> 4) * 10 + (msg[19] & 0x0f) * 100;
    wind_avg_meter_sec_fp1 = wind_raw;
    wind_avg_meter_sec     = wind_raw * 0.1f;

    int rain_raw = (msg[23] & 0x0f) + ((msg[23] & 0xf0) >> 4) * 10 + (msg[24] & 0x0f) * 100;
    rain_mm = rain_raw * 0.1f;

    battery_ok = (msg[25] & 0x80) ? false : true;

    return DECODE_OK;
}
#endif


//
// From from rtl_433 project - https://github.com/merbanan/rtl_433/blob/master/src/devices/bresser_6in1.c
//
// Decoder for Bresser Weather Center 6-in-1.
// - also Bresser Weather Center 7-in-1 indoor sensor.
// - also Bresser new 5-in-1 sensors.
// - also Froggit WH6000 sensors.
// - also rebranded as Ventus C8488A (W835)
// - also Bresser 3-in-1 Professional Wind Gauge / Anemometer PN 7002531
// There are at least two different message types:
// - 24 seconds interval for temperature, hum, uv and rain (alternating messages)
// - 12 seconds interval for wind data (every message)
// Also Bresser Explore Scientific SM60020 Soil moisture Sensor.
// https://www.bresser.de/en/Weather-Time/Accessories/EXPLORE-SCIENTIFIC-Soil-Moisture-and-Soil-Temperature-Sensor.html
// Moisture:
//     f16e 187000e34 7 ffffff0000 252 2 16 fff 004 000 [25,2, 99%, CH 7]
//     DIGEST:8h8h ID?8h8h8h8h FLAGS:4h BATT:1b CH:3d 8h 8h8h 8h8h TEMP:12h 4h MOIST:8h TRAILER:8h8h8h8h4h
// Moisture is transmitted in the humidity field as index 1-16: 0, 7, 13, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 99.
// {206}55555555545ba83e803100058631ff11fe6611ffffffff01cc00 [Hum 96% Temp 3.8 C Wind 0.7 m/s]
// {205}55555555545ba999263100058631fffffe66d006092bffe0cff8 [Hum 95% Temp 3.0 C Wind 0.0 m/s]
// {199}55555555545ba840523100058631ff77fe668000495fff0bbe [Hum 95% Temp 3.0 C Wind 0.4 m/s]
// {205}55555555545ba94d063100058631fffffe665006092bffe14ff8
// {206}55555555545ba860703100058631fffffe6651ffffffff0135fc [Hum 95% Temp 3.0 C Wind 0.0 m/s]
// {205}55555555545ba924d23100058631ff99fe68b004e92dffe073f8 [Hum 96% Temp 2.7 C Wind 0.4 m/s]
// {202}55555555545ba813403100058631ff77fe6810050929ffe1180 [Hum 94% Temp 2.8 C Wind 0.4 m/s]
// {205}55555555545ba98be83100058631fffffe6130050929ffe17800 [Hum 95% Temp 2.8 C Wind 0.8 m/s]
//                                           TEMP  HUM
// 2dd4  1f 40 18 80 02 c3 18 ff 88 ff 33 08 ff ff ff ff 80 e6 00 [Hum 96% Temp 3.8 C Wind 0.7 m/s]
// 2dd4  cc 93 18 80 02 c3 18 ff ff ff 33 68 03 04 95 ff f0 67 3f [Hum 95% Temp 3.0 C Wind 0.0 m/s]
// 2dd4  20 29 18 80 02 c3 18 ff bb ff 33 40 00 24 af ff 85 df    [Hum 95% Temp 3.0 C Wind 0.4 m/s]
// 2dd4  a6 83 18 80 02 c3 18 ff ff ff 33 28 03 04 95 ff f0 a7 3f
// 2dd4  30 38 18 80 02 c3 18 ff ff ff 33 28 ff ff ff ff 80 9a 7f [Hum 95% Temp 3.0 C Wind 0.0 m/s]
// 2dd4  92 69 18 80 02 c3 18 ff cc ff 34 58 02 74 96 ff f0 39 3f [Hum 96% Temp 2.7 C Wind 0.4 m/s]
// 2dd4  09 a0 18 80 02 c3 18 ff bb ff 34 08 02 84 94 ff f0 8c 0  [Hum 94% Temp 2.8 C Wind 0.4 m/s]
// 2dd4  c5 f4 18 80 02 c3 18 ff ff ff 30 98 02 84 94 ff f0 bc 00 [Hum 95% Temp 2.8 C Wind 0.8 m/s]
// {147} 5e aa 18 80 02 c3 18 fa 8f fb 27 68 11 84 81 ff f0 72 00 [Temp 11.8 C  Hum 81%]
// {149} ae d1 18 80 02 c3 18 fa 8d fb 26 78 ff ff ff fe 02 db f0
// {150} f8 2e 18 80 02 c3 18 fc c6 fd 26 38 11 84 81 ff f0 68 00 [Temp 11.8 C  Hum 81%]
// {149} c4 7d 18 80 02 c3 18 fc 78 fd 29 28 ff ff ff fe 03 97 f0
// {149} 28 1e 18 80 02 c3 18 fb b7 fc 26 58 ff ff ff fe 02 c3 f0
// {150} 21 e8 18 80 02 c3 18 fb 9c fc 33 08 11 84 81 ff f0 b7 f8 [Temp 11.8 C  Hum 81%]
// {149} 83 ae 18 80 02 c3 18 fc 78 fc 29 28 ff ff ff fe 03 98 00
// {150} 5c e4 18 80 02 c3 18 fb ba fc 26 98 11 84 81 ff f0 16 00 [Temp 11.8 C  Hum 81%]
// {148} d0 bd 18 80 02 c3 18 f9 ad fa 26 48 ff ff ff fe 02 ff f0
// Wind and Temperature/Humidity or Rain:
//     DIGEST:8h8h ID:8h8h8h8h FLAGS:4h BATT:1b CH:3d WSPEED:~8h~4h ~4h~8h WDIR:12h ?4h TEMP:8h.4h ?4h HUM:8h UV?~12h ?4h CHKSUM:8h
//     DIGEST:8h8h ID:8h8h8h8h FLAGS:4h BATT:1b CH:3d WSPEED:~8h~4h ~4h~8h WDIR:12h ?4h RAINFLAG:8h RAIN:8h8h UV:8h8h CHKSUM:8h
// Digest is LFSR-16 gen 0x8810 key 0x5412, excluding the add-checksum and trailer.
// Checksum is 8-bit add (with carry) to 0xff.
// Notes on different sensors:
// - 1910 084d 18 : RebeckaJohansson, VENTUS W835
// - 2030 088d 10 : mvdgrift, Wi-Fi Colour Weather Station with 5in1 Sensor, Art.No.: 7002580, ff 01 in the UV field is (obviously) invalid.
// - 1970 0d57 18 : danrhjones, bresser 5-in-1 model 7002580, no UV
// - 18b0 0301 18 : konserninjohtaja 6-in-1 outdoor sensor
// - 18c0 0f10 18 : rege245 BRESSER-PC-Weather-station-with-6-in-1-outdoor-sensor
// - 1880 02c3 18 : f4gqk 6-in-1
// - 18b0 0887 18 : npkap
// 
// Parameters:
// 
//  msg     - Pointer to message
//  msgSize - Size of message
//  pOut    - Pointer to WeatherData
// 
//  Returns:
// 
//  DECODE_OK      - OK - WeatherData will contain the updated information
//  DECODE_DIG_ERR - Digest Check Error
//  DECODE_CHK_ERR - Checksum Error
#ifdef BRESSER_6_IN_1
DecodeStatus WeatherSensor::decodeBresser6In1Payload(uint8_t *msg, uint8_t msgSize) {
    int const moisture_map[] = {0, 7, 13, 20, 27, 33, 40, 47, 53, 60, 67, 73, 80, 87, 93, 99}; // scale is 20/3
    
    // LFSR-16 digest, generator 0x8810 init 0x5412
    int chkdgst = (msg[0] << 8) | msg[1];
    int digest  = lfsr_digest16(&msg[2], 15, 0x8810, 0x5412);
    if (chkdgst != digest) {
        //decoder_logf(decoder, 2, __func__, "Digest check failed %04x vs %04x", chkdgst, digest);
        DEBUG_PRINT("Digest check failed - ");
        DEBUG_PRINT(chkdgst, HEX);
        DEBUG_PRINT(" vs ");
        DEBUG_PRINTLN(digest, HEX);
        return DECODE_DIG_ERR;
    }
    // Checksum, add with carry
    int chksum = msg[17];
    int sum    = add_bytes(&msg[2], 16); // msg[2] to msg[17]
    if ((sum & 0xff) != 0xff) {
        //decoder_logf(decoder, 2, __func__, "Checksum failed %04x vs %04x", chksum, sum);
        DEBUG_PRINT("Checksum failed - ");
        DEBUG_PRINT(chksum, HEX);
        DEBUG_PRINT(" vs ");
        DEBUG_PRINTLN(sum, HEX);
        return DECODE_CHK_ERR;
    }

    sensor_id  = ((uint32_t)msg[2] << 24) | (msg[3] << 16) | (msg[4] << 8) | (msg[5]);
    s_type     = (msg[6] >> 4); // 1: weather station, 2: indoor?, 4: soil probe
    battery_ok = (msg[6] >> 3) & 1;
    chan       = (msg[6] & 0x7);
    
    // temperature, humidity, shared with rain counter, only if valid BCD digits
    temp_ok  = msg[12] <= 0x99 && (msg[13] & 0xf0) <= 0x90;
    if (temp_ok) {
        int temp_raw   = (msg[12] >> 4) * 100 + (msg[12] & 0x0f) * 10 + (msg[13] >> 4);
        float temp   = temp_raw * 0.1f;
        if (temp_raw > 600)
            temp = (temp_raw - 1000) * 0.1f;
    
        temp_c   = temp;
        humidity = (msg[14] >> 4) * 10 + (msg[14] & 0x0f);
    }
    // apparently ff0(1) if not available
    uv_ok  = msg[15] <= 0x99 && (msg[16] & 0xf0) <= 0x90;
    if (uv_ok) {
        int uv_raw = ((msg[15] & 0xf0) >> 4) * 100 + (msg[15] & 0x0f) * 10 + ((msg[16] & 0xf0) >> 4);
        uv   = uv_raw * 0.1f;
    }
    int flags  = (msg[16] & 0x0f); // looks like some flags, not sure

    //int unk_ok  = (msg[16] & 0xf0) == 0xf0;
    //int unk_raw = ((msg[15] & 0xf0) >> 4) * 10 + (msg[15] & 0x0f);

    // invert 3 bytes wind speeds
    msg[7] ^= 0xff;
    msg[8] ^= 0xff;
    msg[9] ^= 0xff;
    wind_ok = (msg[7] <= 0x99) && (msg[8] <= 0x99) && (msg[9] <= 0x99);
    if (wind_ok) {
        int gust_raw                  = (msg[7] >> 4) * 100 + (msg[7] & 0x0f) * 10 + (msg[8] >> 4);
        wind_gust_meter_sec_fp1 = gust_raw;
        wind_gust_meter_sec     = gust_raw * 0.1f;
        int wavg_raw                  = (msg[9] >> 4) * 100 + (msg[9] & 0x0f) * 10 + (msg[8] & 0x0f);
        wind_avg_meter_sec_fp1  = wavg_raw;
        wind_avg_meter_sec      = wavg_raw * 0.1f;
        int wind_dir_raw        = ((msg[10] & 0xf0) >> 4) * 100 + (msg[10] & 0x0f) * 10 + ((msg[11] & 0xf0) >> 4);
        wind_direction_deg_fp1  = wind_dir_raw * 10; 
        wind_direction_deg      = wind_dir_raw * 1.0f;
    }

    // rain counter, inverted 3 bytes BCD, shared with temp/hum, only if valid digits
    msg[12] ^= 0xff;
    msg[13] ^= 0xff;
    msg[14] ^= 0xff;
    rain_ok   = msg[12] <= 0x99 && msg[13] <= 0x99 && msg[14] <= 0x99;
    if (rain_ok) {
        int rain_raw    = (msg[12] >> 4) * 100000 + (msg[12] & 0x0f) * 10000
                + (msg[13] >> 4) * 1000 + (msg[13] & 0x0f) * 100
                + (msg[14] >> 4) * 10 + (msg[14] & 0x0f);
        rain_mm   = rain_raw * 0.1f;
    }
    moisture_ok = false;
    if (s_type == 4 && temp_ok && humidity >= 1 && humidity <= 16) {
        moisture_ok = true;
        if (moisture_ok) {
            moisture = moisture_map[humidity - 1];
        }
    }
    return DECODE_OK;

}
#endif

//
// Convert wind speed from meters per second to Beaufort
// [https://en.wikipedia.org/wiki/Beaufort_scale]
//
uint8_t WeatherSensor::windspeed_ms_to_bft(float ms)
{
  if (ms < 5.5) {
    // 0..3 Bft
    if (ms < 0.9) {
      return 0;
    } else if (ms < 1.6) {
      return 1;
    } else if (ms < 3.4) {
      return 2;
    } else {
      return 3;
    }
  } else if (ms < 17.2) { 
    // 4..7 Bft
    if (ms < 8) {
      return 4;
    } else if (ms < 10.8) {
      return 5;
    } else if (ms < 13.9) {
      return 6;
    } else {
      return 7;
    }
  } else {
    // 8..12 Bft
    if (ms < 20.8) {
      return 8;
    } else if (ms < 24.5) {
      return 9;
    } else if (ms < 28.5) {
      return 10;
    } else if (ms < 32.7) {
      return 11;
    } else {
      return 12;
    }
  }
}

