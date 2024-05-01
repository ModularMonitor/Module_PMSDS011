#pragma once

#include "espsoftwareserial/src/SoftwareSerial.h"
#include "esp_sds011/src/esp_sds011.h"

#include "SelfThreadable/self_threadable.h"

// mPMSD wraps PMSDS011 in a async class. This way, less things to worry about.
class mPMSD : protected SelfThreadable {
    float m_pm25 = 0.0f;
    float m_pm10 = 0.0f;
    
    bool m_fail_flag = false;
    bool m_has_new_data = false;

    void async() {
        // Per manufacturer specification, place the sensor in standby to prolong service life.
        // At an user-determined interval (here 210s down plus 30s duty = 4m), run the sensor for 30s.
        // Quick response time is given as 10s by the manufacturer, thus the library drops the
        // measurements obtained during the first 10s of each run.
    
        constexpr uint32_t down_s = 210; // downtime
        constexpr uint32_t duty_s = 30; // time running
        constexpr int get_data_reporting_max_tries = 2;
        
        constexpr int pm_tablesize = 20;
        int pm25_table[pm_tablesize]{0};
        int pm10_table[pm_tablesize]{0};
        
        constexpr int SDS_PIN_RX = 27;
        constexpr int SDS_PIN_TX = 26;
        
#ifdef ESP32
        HardwareSerial& serialSDS(Serial2);
        Sds011Async<HardwareSerial> sds011(serialSDS);
        
        serialSDS.begin(9600, SERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX);
        delay(100);
#else
        EspSoftwareSerial::UART serialSDS;
        Sds011Async<EspSoftwareSerial::UART> sds011(serialSDS);
        
        serialSDS.begin(9600, SWSERIAL_8N1, SDS_PIN_RX, SDS_PIN_TX, false, 192);
#endif

        bool failed_flag;
        do {
            failed_flag = false;
            
            Sds011::Report_mode report_mode{};
        
            for (int retry = 0; retry < get_data_reporting_max_tries; ++retry) {
                if (!sds011.get_data_reporting_mode(report_mode)) {
                    if (retry == get_data_reporting_max_tries) {
                        failed_flag = true;
                    }
                }
                else break;
            }
            
            if (!failed_flag && Sds011::REPORT_ACTIVE != report_mode) {
                if (!sds011.set_data_reporting_mode(Sds011::REPORT_ACTIVE)) {
                    failed_flag = true;
                }
            }
            
            if (failed_flag) delay(1000);
            
        } while((m_fail_flag = failed_flag));
        

        const auto on_query_data_completed_ref = [&](int n){
            int pm25 = 0;
            int pm10 = 0;
            if (
                sds011.filter_data(n, pm25_table, pm10_table, pm25, pm10) &&
                !isnan(pm10) && !isnan(pm25)
            ) {
                m_pm10 = float(pm10) / 10.0f;
                m_pm25 = float(pm25) / 10.0f;
                m_has_new_data = true;
            }            
        };

        const auto do_wait_performing = [&](const uint32_t _time_s) {            
            uint32_t time_sleep = millis() + _time_s * 1000;
            while (static_cast<int32_t>(time_sleep - millis()) > 0) sds011.perform_work();
        };

        while(1) {
            sds011.set_sleep(true);
            
            do_wait_performing(down_s);
            
            sds011.set_sleep(false);
            
            sds011.on_query_data_auto_completed(on_query_data_completed_ref);
            sds011.query_data_auto_async(pm_tablesize, pm25_table, pm10_table);  
            
            do_wait_performing(duty_s);            
        }

        vTaskDelete(NULL);
    }
public:
    mPMSD() : SelfThreadable("ASYNC") { async_start(); }
    
    
    float get_pm10() const { return m_pm10; }
    float get_pm25() const { return m_pm25; }
    
    bool has_issues() const { return m_fail_flag; }
    bool has_new_data_autoreset() { bool had = m_has_new_data; m_has_new_data = false; return had; }
};

