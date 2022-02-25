#include <../.pio/libdeps/esp32-wroom-32/RadioLib/src/RadioLib.h>
#include <../.pio/libdeps/esp32-wroom-32/ESPNtpClient/src/ESPNtpClient.h>
#include "Ticker.h"
#include "WiFi.h"
#include "WifiConfig.h"
#include <../.pio/libdeps/esp32-wroom-32/ESP32Ping/ESP32Ping.h>
#include <../.pio/libdeps/esp32-wroom-32/Sgp4/src/Sgp4.h>

const int timezone = -5;
const bool daylightsavings = false;
const char unix_timezone[] = TZ_America_New_York;
const char NTP_server[] = "time";
const char WiFi_SSID[] = SSID;
const char WiFi_PASSWD[] = PASSWD;
const char ping_host[] = "time";
const float gs_latitude = 26.69039365;
const float gs_longitude = -81.94721265;
const float gs_elevation = 7.964;
char satellite_name[] = "SATLLA-2B";
//https://celestrak.com/NORAD/elements/gp.php?NAME=SATLLA-2B&FORMAT=2LE
char satellite_tle1[] = "1 51014U 22002AG  22056.13652677  .00006137  00000+0  34487-3 0  9994";
char satellite_tle2[] = "2 51014  97.5108 125.2068 0014696  93.5229 266.7684 15.13570843  6432";

const double c = 299792458.0; // speed of light m/s

void setFlag();
void Predict(int passes);
void Track();

Sgp4 sat;
Ticker tkSecond;

// Connected as follows:
// ESP32    SX1280
//  5       SCK
// 19       MISO
// 27       MOSI
// 18       NSS
// 23       RST
// 35       DIO0/BUSY (LORA IRQ / Busy) (ESP32 pin 35 is Input only was using pin 26)
// 33       DIO1      (LORA IRQ / RF switch)
// 12       RX_EN     (Some Modules - LNA control pin, active high)
// 14       TX_EN     (Some Modules - PA control pin, active high)
SX1280 radio = new Module(18, 33, 23, 35);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

float frequency = 2401.000;
float doppler = 0.0;

void setup() {
    SPI.begin(5, 19, 27);
    Serial.begin(115200);
    unsigned long timer = millis();
    while (!Serial) {
        if ((millis() - timer) < 3000) {
            delay(100);
        } else {
            break;
        }
    }

    WiFi.begin(WiFi_SSID, WiFi_PASSWD);
    while (!WiFi.isConnected()) {
        delay(100);
    }
    NTP.setNtpServerName(NTP_server);
    NTP.setTimeZone(unix_timezone);
    NTP.setInterval(120);
    NTP.begin();
    Serial.println();
    Serial.println();
    Serial.print("WiFi connected with ip ");
    Serial.println(WiFi.localIP());

    Serial.println();
    Serial.print("Pinging host ");
    Serial.println(ping_host);

    if (Ping.ping(ping_host)) {
        Serial.println("Success!!");
    } else {
        Serial.println("Error :(");
    }
    Serial.println();

    while (NTP.syncStatus() != syncd) {
        delay(100);
    }
    Serial.println(NTP.getTimeDateStringUs());
    Serial.println();

    // initialize SX1280 with default settings
    Serial.print(F("[SX1280] Initializing ... "));
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.print(F("success!   "));
    } else {
        Serial.print(F("failed, code "));
        Serial.print(state);
        Serial.print(F("   "));
        while (true);
    }
    Serial.println(NTP.getTimeDateStringUs());

    // set the function that will be called when new packet is received
    radio.setDio1Action(setFlag);

    Serial.print(F("[SX1280] Starting to listen ...   "));
    radio.setRfSwitchPins(12, 14);
    radio.setFrequency(frequency + doppler);
    radio.setBandwidth(812.5);
    radio.setSpreadingFactor(10);
    radio.setCodingRate(5);
    radio.setSyncWord(42, 0x44);
    radio.setOutputPower(-18); //Sets output power. Allowed values are in range from -18 to 13 dBm.
    radio.setPreambleLength(12);
    radio.setCRC(0); // CRC length in bytes, Allowed values are 1, 2 or 3, set to 0 to disable CRC.
    radio.setHighSensitivityMode(true);
    radio.setGainControl(0);
    // start listening for LoRa packets
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.print(F("success!   "));
    } else {
        Serial.print(F("failed, code "));
        Serial.print(state);
        Serial.print(F("   "));
        while (true);
    }
    Serial.println(NTP.getTimeDateStringUs());
    Serial.println();

    // if needed, 'listen' mode can be disabled by calling any of the following methods:
    // radio_SX12xx.standby()
    // radio_SX12xx.sleep()
    // radio_SX12xx.transmit();
    // radio_SX12xx.receive();
    // radio_SX12xx.readData();
    // radio_SX12xx.scanChannel();

    sat.site(gs_latitude, gs_longitude, gs_elevation);
    sat.init(satellite_name, satellite_tle1, satellite_tle2);

    //Display TLE epoch time
    double jdC = sat.satrec.jdsatepoch;
    int year, mon, day, hr, min;
    double sec;
    invjday(jdC, timezone, daylightsavings, year, mon, day, hr, min, sec);
    Serial.printf("Epoch:  %02i/%02i/%04i %02i:%02i:%02.3f\n\n", day, mon, year, hr, min, sec);

    Predict(5);
    Serial.println();
    Serial.println();

    Track();
    tkSecond.attach(60, Track);
}

void loop() {
    if (receivedFlag) {
        // disable the interrupt service routine while processing the data
        enableInterrupt = false;

        // reset flag
        receivedFlag = false;

        unsigned int packetLength = radio.getPacketLength();
        byte byteArr[packetLength] = {};
        int state = radio.readData(byteArr, 0);

        if (state == RADIOLIB_ERR_NONE) {
            // packet was successfully received
            Serial.print(F("[SX1280] Received packet!  "));

            // print RSSI (Received Signal Strength Indicator)
            Serial.print(F("RSSI: "));
            Serial.print(radio.getRSSI());
            Serial.print(F(" dBm  "));

            // print SNR (Signal-to-Noise Ratio)
            Serial.print(F("SNR: "));
            Serial.print(radio.getSNR());
            Serial.print(F(" dB  "));

            // print Frequency Error
            Serial.print(F("Frequency Error: "));
            Serial.print(radio.getFrequencyError());
            Serial.print(F(" Hz    "));

            // print data of the packet
            Serial.print(NTP.getTimeDateStringUs());
            Serial.print(F("   "));
            Serial.println(F("Data: "));
            for (unsigned int i = 0; i < packetLength; i++) {
                Serial.printf("%c", byteArr[i]);
            }
            Serial.println();
            for (unsigned int i = 0; i < packetLength; i++) {
                Serial.printf("%02X", byteArr[i]);
            }
            Serial.println();

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            Serial.print(F("CRC error!   "));
            Serial.println(NTP.getTimeDateStringUs());

        } else {
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.print(state);
            Serial.print(F("   "));
            Serial.println(NTP.getTimeDateStringUs());
        }

        // put module back to listen mode
        radio.startReceive();

        // we're ready to receive more packets, enable interrupt service routine
        enableInterrupt = true;
    }

}

// this function is called when a complete packet is received by the module
// IMPORTANT: this function MUST be 'void' type and MUST NOT have any arguments!
void setFlag() {
    // check if the interrupt is enabled
    if (!enableInterrupt) {
        return;
    }
    // we got a packet, set the flag
    receivedFlag = true;
}

void Track() {
    time_t now = time(nullptr);
    sat.findsat((unsigned long) now - 1); // get satellite information last second
    double last_second_range = sat.satDist;
    sat.findsat((unsigned long) now); // get satellite information current second
    doppler = (float) (-(sat.satDist - last_second_range) * 1000.0 / c *
                       (frequency * 1000000)); // doppler for SX1280
    radio.setFrequency(frequency + (doppler / 1000000.0));

    Serial.println();
    Serial.println(NTP.getTimeDateStringUs());
    Serial.printf("Azimuth : %.2f  Elevation : %.2f  Slant range : %.0f km  range velocity : %.3f km/s\n",
                  sat.satAz, sat.satEl, sat.satDist, (sat.satDist - last_second_range));
    Serial.printf("Latitude : %.2f  Longitude : %.2f  Altitude : %.0f km\n", sat.satLat, sat.satLon, sat.satAlt);
    Serial.printf("Doppler at %.6fGHz : %.0f Hz  ", frequency / 1000.0, doppler);
    Serial.printf("Receive Frequency : %.3fMHz\n", frequency + (doppler / 1000000.0));
    switch (sat.satVis) {
        case -2:
            Serial.println("Visible : Under horizon");
            break;
        case -1:
            Serial.println("Visible : Daylight");
            break;
        default:
            Serial.println("Visible : " + String(sat.satVis));   //0:eclipsed - 1000:visible
            break;
    }
    Serial.println();
}

void Predict(int passes) {
    passinfo overpass{};
    int year, mon, day, hr, minute;
    double sec;
    time_t now = time(nullptr);
    sat.initpredpoint((unsigned long) now, 6.0); //finds the start time

    bool error;
    for (int i = 0; i < passes; i++) {
        error = sat.nextpass(&overpass, 20);
        if (error) {
            invjday(overpass.jdstart, timezone, daylightsavings, year, mon, day, hr, minute, sec);
            Serial.printf("Pass:  %02i/%02i/%04i  AOS:  az=%.1f %02i:%02i:%02.1f  ", day, mon, year, overpass.azstart, hr, minute, sec);
            invjday(overpass.jdmax, timezone, daylightsavings, year, mon, day, hr, minute, sec);
            Serial.printf("Max el=%.1f %02i:%02i:%02.1f  ", overpass.maxelevation, hr, minute, sec);
            invjday(overpass.jdstop, timezone, daylightsavings, year, mon, day, hr, minute, sec);
            Serial.printf("LOS az=%.1f %02i:%02i:%02.1f\n", overpass.azstop, hr, minute, sec);
        } else {
            Serial.println("Prediction error");
        }
        delay(0);
    }
}
