 /*
    Name:       measure.ino
    Created:	2019-04-06 12:16:00
    Author:     Dirk O. Kaar <dok@dok-net.net>
*/

#include "espsoftwareserial/src/SoftwareSerial.h"
#include "esp_sds011/src/esp_sds011.h"

#define SDS_PIN_RX 27 // green, corner pin
#define SDS_PIN_TX 26 // yellow

#ifdef ESP32
HardwareSerial& serialSDS(Serial2);
Sds011Async< HardwareSerial > sds011(serialSDS);
#else
EspSoftwareSerial::UART serialSDS;
Sds011Async< EspSoftwareSerial::UART > sds011(serialSDS);
#endif

// The example stops the sensor for 210s, then runs it for 30s, then repeats.
// At tablesizes 20 and below, the tables get filled during duty cycle
// and then measurement completes.
// At tablesizes above 20, the tables do not get completely filled
// during the 30s total runtime, and the rampup / 4 timeout trips,
// thus completing the measurement at whatever number of data points
// were recorded in the tables.
constexpr int pm_tablesize = 20;
int pm25_table[pm_tablesize];
int pm10_table[pm_tablesize];

bool is_SDS_running = true;

void start_SDS() {
    Serial.println(F("Start wakeup SDS011"));

    if (sds011.set_sleep(false)) { is_SDS_running = true; }

    Serial.println(F("End wakeup SDS011"));
}

void stop_SDS() {
    Serial.println(F("Start sleep SDS011"));

    if (sds011.set_sleep(true)) { is_SDS_running = false; }

    Serial.println(F("End sleep SDS011"));
}

// The setup() function runs once each time the micro-controller starts
void setup()
{
    Serial.begin(115200);

#ifdef ESP32
    serialSDS.begin(9600, SERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX);
    delay(100);
#else
    serialSDS.begin(9600, SWSERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX, false, 192);
#endif

    Serial.println(F("SDS011 start/stop and reporting sample"));

    Sds011::Report_mode report_mode;
    constexpr int GETDATAREPORTINMODE_MAXRETRIES = 2;
    for (auto retry = 1; retry <= GETDATAREPORTINMODE_MAXRETRIES; ++retry) {
        if (!sds011.get_data_reporting_mode(report_mode)) {
            if (retry == GETDATAREPORTINMODE_MAXRETRIES) {
                Serial.println(F("Sds011::get_data_reporting_mode() failed"));
            }
        }
        else {
            break;
        }
    }
    if (Sds011::REPORT_ACTIVE != report_mode) {
        Serial.println(F("Turning on Sds011::REPORT_ACTIVE reporting mode"));
        if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
            Serial.println(F("Sds011::set_data_reporting_mode(Sds011::REPORT_ACTIVE) failed"));
        }
    }
}

// Add the main program code into the continuous loop() function
void loop()
{
    // Per manufacturer specification, place the sensor in standby to prolong service life.
    // At an user-determined interval (here 210s down plus 30s duty = 4m), run the sensor for 30s.
    // Quick response time is given as 10s by the manufacturer, thus the library drops the
    // measurements obtained during the first 10s of each run.

    constexpr uint32_t down_s = 210;

    stop_SDS();
    Serial.print(F("stopped SDS011 (is running = "));
    Serial.print(is_SDS_running);
    Serial.println(')');

    uint32_t deadline = millis() + down_s * 1000;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
        delay(1000);
        Serial.println(static_cast<int32_t>(deadline - millis()) / 1000);
        sds011.perform_work();
    }

    constexpr uint32_t duty_s = 30;

    start_SDS();
    Serial.print(F("started SDS011 (is running = "));
    Serial.print(is_SDS_running);
    Serial.println(')');

    sds011.on_query_data_auto_completed([](int n) {
        Serial.println(F("Begin Handling SDS011 query data"));
        int pm25;
        int pm10;
        Serial.print(F("n = ")); Serial.println(n);
        if (sds011.filter_data(n, pm25_table, pm10_table, pm25, pm10) &&
            !isnan(pm10) && !isnan(pm25)) {
            Serial.print(F("PM10: "));
            Serial.println(float(pm10) / 10);
            Serial.print(F("PM2.5: "));
            Serial.println(float(pm25) / 10);
        }
        Serial.println(F("End Handling SDS011 query data"));
        });

    if (!sds011.query_data_auto_async(pm_tablesize, pm25_table, pm10_table)) {
        Serial.println(F("measurement capture start failed"));
    }

    deadline = millis() + duty_s * 1000;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
        delay(1000);
        Serial.println(static_cast<int32_t>(deadline - millis()) / 1000);
        sds011.perform_work();
    }
}
