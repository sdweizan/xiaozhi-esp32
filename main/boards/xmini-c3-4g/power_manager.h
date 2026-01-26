#pragma once
#include <vector>
#include <functional>
#include <algorithm>
#include <cmath>

#include <esp_timer.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>

class PowerManager {
private:
    esp_timer_handle_t timer_handle_ = nullptr;
    std::function<void(bool)> on_charging_status_changed_;
    std::function<void(uint8_t)> on_low_battery_alert_;

    gpio_num_t charging_pin_ = GPIO_NUM_NC;
    bool is_charging_ = false;
    
    uint8_t battery_level_ = 100;
    float battery_voltage_ = 0.0f;
    bool low_battery_notified_ = false; 
    const int kLowBatteryAlertThreshold = 15;  // 低电量提醒阈值
    const int kCheckIntervalUs = 2000000;      // 2秒检查一次
    
    // ADC 相关
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    const int kSampleCount = 20;
    const adc_channel_t kAdcChannel = ADC_CHANNEL_4;
    
    struct Calibration {
        float slope = 4.2f / 2781.0f;
        float offset = 0.0f; 
        bool is_calibrated = true;
    } calibration_;
    
    struct Hardware {
        const float divider_ratio = 2.0f;
        const float adc_max = 4095.0f;
    } hardware_;

    // 精确读取ADC
    uint32_t ReadAdc() {
        std::vector<uint16_t> samples;
        samples.reserve(kSampleCount);
        
        for (int i = 0; i < kSampleCount; i++) {
            int adc_value;
            if (adc_oneshot_read(adc_handle_, kAdcChannel, &adc_value) == ESP_OK) {
                samples.push_back(static_cast<uint16_t>(adc_value));
            }
        }
        if (samples.empty()) return 0;
        std::sort(samples.begin(), samples.end());
        int trim = samples.size() / 4;
        uint32_t sum = 0;
        int count = 0;
        for (size_t i = trim; i < samples.size() - trim; i++) {
            sum += samples[i];
            count++;
        }
        return count > 0 ? sum / count : 0;
    }
    
    float AdcToVoltageDirect(uint32_t adc_value) {
        if (adc_value == 0) return 0.0f;
        float current_voltage = calibration_.slope * adc_value + calibration_.offset;
        current_voltage = std::max(0.0f, std::min(5.0f, current_voltage));
        
        // 平滑滤波
        if (battery_voltage_ < 2.5f) {
            battery_voltage_ = current_voltage;
        } else {
            battery_voltage_ = current_voltage * 0.3f + battery_voltage_ * 0.7f;
        }
        return battery_voltage_;
    }

    uint8_t VoltageToLevel(float voltage) {
        struct Point { float v; uint8_t level; };
        static const std::vector<Point> table = {
            {3.50f, 0},   {3.60f, 10},  {3.70f, 20},
            {3.75f, 30},  {3.80f, 40},  {3.85f, 50},
            {3.90f, 60},  {3.95f, 70},  {4.00f, 80},
            {4.05f, 90},  {4.10f, 95},  {4.18f, 100}
        };

        if (voltage <= table.front().v) return table.front().level;
        if (voltage >= table.back().v) return table.back().level;

        for (size_t i = 0; i < table.size() - 1; i++) {
            if (voltage < table[i+1].v) {
                float v0 = table[i].v, v1 = table[i+1].v;
                uint8_t l0 = table[i].level, l1 = table[i+1].level;
                return static_cast<uint8_t>(l0 + (voltage - v0) / (v1 - v0) * (l1 - l0));
            }
        }
        return 0;
    }
    
    void CheckBatteryLevel() {
        if (is_charging_) {
            low_battery_notified_ = false;
            return;
        }
        
        if (battery_level_ <= kLowBatteryAlertThreshold) {
            if (!low_battery_notified_) {
                low_battery_notified_ = true;
                if (on_low_battery_alert_) {
                    on_low_battery_alert_(battery_level_);
                }
                ESP_LOGW("PowerManager", "Low battery threshold reached: %d%%. Alerting user once.", battery_level_);
            }
        }
    }
    
    void CheckBatteryStatus() {
        if (charging_pin_ != GPIO_NUM_NC) {
            bool new_charging = (gpio_get_level(charging_pin_) == 1);
            if (new_charging != is_charging_) {
                is_charging_ = new_charging;
                if (on_charging_status_changed_) on_charging_status_changed_(is_charging_);
                ESP_LOGI("PowerManager", "Charging status: %s", is_charging_ ? "Charging" : "Discharging");
            }
        }
        
        uint32_t adc_val = ReadAdc();
        float voltage = AdcToVoltageDirect(adc_val);
        battery_level_ = VoltageToLevel(voltage);
        
        CheckBatteryLevel();
    }
    
    static void TimerCallback(void* arg) {
        static_cast<PowerManager*>(arg)->CheckBatteryStatus();
    }
    
    bool InitADC() {
        adc_oneshot_unit_init_cfg_t init_config = {};
        init_config.unit_id = ADC_UNIT_1;
        
        if (adc_oneshot_new_unit(&init_config, &adc_handle_) != ESP_OK) return false;
        
        adc_oneshot_chan_cfg_t chan_config = {};
        chan_config.atten = ADC_ATTEN_DB_12;
        chan_config.bitwidth = ADC_BITWIDTH_12;
        
        return adc_oneshot_config_channel(adc_handle_, kAdcChannel, &chan_config) == ESP_OK;
    }

public:
    PowerManager(gpio_num_t charging_pin = GPIO_NUM_NC) : charging_pin_(charging_pin) {
        if (charging_pin_ != GPIO_NUM_NC) {
            gpio_config_t io_conf = {};
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pin_bit_mask = (1ULL << charging_pin_);
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            gpio_config(&io_conf);
        }
        InitADC();
    }

    void Start() {
        if (timer_handle_ == nullptr) {
            esp_timer_create_args_t timer_args = {};
            timer_args.callback = TimerCallback;
            timer_args.arg = this;
            timer_args.name = "power_mgr_timer";
            
            if (esp_timer_create(&timer_args, &timer_handle_) == ESP_OK) {
                esp_timer_start_periodic(timer_handle_, kCheckIntervalUs);
            }
        }
        CheckBatteryStatus();
    }
    
    ~PowerManager() {
        if (timer_handle_) {
            esp_timer_stop(timer_handle_);
            esp_timer_delete(timer_handle_);
        }
        if (adc_handle_) adc_oneshot_del_unit(adc_handle_);
    }
    
    bool IsCharging() const { return is_charging_; }
    bool IsDischarging() const { return !is_charging_; }
    uint8_t GetBatteryLevel() const { return battery_level_; }
    float GetBatteryVoltage() const { return battery_voltage_; }
    void OnChargingStatusChanged(std::function<void(bool)> cb) { on_charging_status_changed_ = cb; }
    void OnLowBatteryAlert(std::function<void(uint8_t)> cb) { on_low_battery_alert_ = cb; }
    void ForceUpdate() { CheckBatteryStatus(); }
};