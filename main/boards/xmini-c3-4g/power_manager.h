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
    esp_timer_handle_t timer_handle_;
    esp_timer_handle_t alert_timer_handle_;
    std::function<void(bool)> on_charging_status_changed_;
    std::function<void(uint8_t)> on_low_battery_alert_;

    gpio_num_t charging_pin_ = GPIO_NUM_NC;
    bool is_charging_ = false;
    
    uint8_t last_battery_level_ = 100;
    bool low_battery_triggered_ = false;
    bool is_alerting_ = false;
    
    const int kLowBatteryAlertThreshold = 15;  // 低电量提醒阈值
    const int kCheckIntervalUs = 2000000;     // 2秒检查一次
    const int kAlertIntervalUs = 300000000;     // 300秒提醒一次
    
    // ADC 相关
    adc_oneshot_unit_handle_t adc_handle_;
    float battery_voltage_ = 0.0f;
    uint8_t battery_level_ = 0;
    
    // 采样参数
    const int kSampleCount = 20;
    const adc_channel_t kAdcChannel = ADC_CHANNEL_4;
    
    struct Calibration {
        float slope = 0.001511f;
        // 直接线性转换：V = slope * ADC + offset
        // 根据日志测量结果：2781 ADC -> 4.2V
        // 得出：slope = 4.2 / 2781 = 0.001511
        float offset = 0.0f; 
        bool is_calibrated = true;
    } calibration_;
    
    // 硬件参数
    struct Hardware {
        const float r1 = 100000.0f;   // 100K
        const float r2 = 100000.0f;   // 100K
        const float divider_ratio = 2.0f;  // (R1+R2)/R2
        const float adc_max = 4095.0f;
        float vref_correction = 1.0f;
    } hardware_;

    // 精确读取ADC
    uint32_t ReadAdc() {
        std::vector<uint16_t> samples;
        samples.reserve(kSampleCount);
        
        for (int i = 0; i < kSampleCount; i++) {
            int adc_value;
            esp_err_t ret = adc_oneshot_read(adc_handle_, kAdcChannel, &adc_value);
            if (ret == ESP_OK && adc_value >= 0) {
                samples.push_back(static_cast<uint16_t>(adc_value));
            }
            for (int j = 0; j < 100; j++) {
                asm volatile ("nop");
            }
        }
        
        if (samples.empty()) return 0;
        
        // 去掉最高和最低的25%
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
    
    // ADC值转电压
    float AdcToVoltageDirect(uint32_t adc_value) {
        if (!calibration_.is_calibrated || adc_value == 0) return 0.0f;
        
        float current_voltage = calibration_.slope * adc_value + calibration_.offset;
        current_voltage = std::max(0.0f, std::min(5.0f, current_voltage));
        
        if (battery_voltage_ < 0.1f) {
            battery_voltage_ = current_voltage;
        } else {
            battery_voltage_ = current_voltage * 0.3f + battery_voltage_ * 0.7f;
        }
        return battery_voltage_;
    }
    // 电压转电量百分比
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
                float v0 = table[i].v;
                float v1 = table[i+1].v;
                uint8_t l0 = table[i].level;
                uint8_t l1 = table[i+1].level;
                float ratio = (voltage - v0) / (v1 - v0);
                return static_cast<uint8_t>(l0 + ratio * (l1 - l0));
            }
        }
        return 0;
    }
    
    static void AlertTimerCallback(void* arg) {
        PowerManager* self = static_cast<PowerManager*>(arg);
        if (self && self->on_low_battery_alert_) {
            self->on_low_battery_alert_(self->kLowBatteryAlertThreshold);
        }
    }
    
    void StartAlerting() {
        if (!is_alerting_) {
            is_alerting_ = true;
            if (on_low_battery_alert_) {
                on_low_battery_alert_(kLowBatteryAlertThreshold);
            }
            esp_timer_start_periodic(alert_timer_handle_, kAlertIntervalUs);
        }
    }
    
    void StopAlerting() {
        if (is_alerting_) {
            is_alerting_ = false;
            if (alert_timer_handle_) {
                esp_timer_stop(alert_timer_handle_);
            }
        }
    }
    
    void CheckBatteryLevel() {
        if (IsCharging()) {
            if (low_battery_triggered_ || is_alerting_) {
                StopAlerting();
                low_battery_triggered_ = false;
            }
            last_battery_level_ = battery_level_;
            return;
        }
        
        if (battery_level_ <= kLowBatteryAlertThreshold && last_battery_level_ > kLowBatteryAlertThreshold && !low_battery_triggered_) {
            low_battery_triggered_ = true;
            StartAlerting();
        }
        
        if (battery_level_ > kLowBatteryAlertThreshold && low_battery_triggered_) {
            low_battery_triggered_ = false;
            StopAlerting();
        }
        
        last_battery_level_ = battery_level_;
    }
    
    // 检查电池状态
    void CheckBatteryStatus() {
        if (charging_pin_ != GPIO_NUM_NC) {
            bool new_charging = (gpio_get_level(charging_pin_) == 1);
            if (new_charging != is_charging_) {
                is_charging_ = new_charging;
                if (on_charging_status_changed_) {
                    on_charging_status_changed_(is_charging_);
                }
                ESP_LOGI("PowerManager", "Charging status changed: %s", 
                        is_charging_ ? "Charging" : "Not Charging");
            }
        }
        
        uint32_t adc_value = ReadAdc();
        
        float voltage = AdcToVoltageDirect(adc_value);
        
        battery_level_ = VoltageToLevel(voltage);
        
        // 调试日志
        // static int log_counter = 0;
        // if (++log_counter >= 5) {
        //     log_counter = 0;
        //     float raw_adc_v = (adc_value / hardware_.adc_max) * 1100.0f / 1000.0f;
        //     float theoretical_v = raw_adc_v * hardware_.divider_ratio;
        //     ESP_LOGI("PowerManager", 
        //             "ADC: %lu -> Direct: %.3fV -> Level: %d%% (Theoretical: %.3fV)",
        //             adc_value, voltage, battery_level_, theoretical_v);
        // }
        
        CheckBatteryLevel();
    }
    
    static void TimerCallback(void* arg) {
        PowerManager* self = static_cast<PowerManager*>(arg);
        if (self) self->CheckBatteryStatus();
    }
    
    bool InitADC() {
        adc_oneshot_unit_init_cfg_t init_config = {};
        init_config.unit_id = ADC_UNIT_1;
        init_config.ulp_mode = ADC_ULP_MODE_DISABLE;
        
        esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle_);
        if (ret != ESP_OK) {
            ESP_LOGE("PowerManager", "ADC init failed: %s", esp_err_to_name(ret));
            return false;
        }
        
        adc_oneshot_chan_cfg_t chan_config = {};
        chan_config.atten = ADC_ATTEN_DB_12;
        chan_config.bitwidth = ADC_BITWIDTH_12;
        
        ret = adc_oneshot_config_channel(adc_handle_, kAdcChannel, &chan_config);
        if (ret != ESP_OK) {
            ESP_LOGE("PowerManager", "ADC channel config failed: %s", esp_err_to_name(ret));
            adc_oneshot_del_unit(adc_handle_);
            return false;
        }
        
        return true;
    }

public:
    PowerManager(gpio_num_t charging_pin = GPIO_NUM_NC) : charging_pin_(charging_pin) {
        ESP_LOGI("PowerManager", "Initializing...");
        calibration_.slope = 4.2f / 2781.0f;
        calibration_.offset = 0.0f;
        calibration_.is_calibrated = true;
        ESP_LOGI("PowerManager", "Calibration: V = %.6f * ADC + %.3f", 
                calibration_.slope, calibration_.offset);
        
        if (charging_pin_ != GPIO_NUM_NC) {
            gpio_config_t io_conf = {};
            io_conf.intr_type = GPIO_INTR_DISABLE;
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pin_bit_mask = (1ULL << charging_pin_);
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            gpio_config(&io_conf);
            ESP_LOGI("PowerManager", "GPIO %d configured for charging detection", charging_pin_);
        }
        
        // 初始化ADC
        if (!InitADC()) {
            ESP_LOGE("PowerManager", "ADC initialization failed");
            return;
        }
        
        // 创建主检查定时器
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = TimerCallback;
        timer_args.arg = this;
        timer_args.dispatch_method = ESP_TIMER_TASK;
        timer_args.name = "power_mgr_timer";
        timer_args.skip_unhandled_events = true;
        
        esp_err_t ret = esp_timer_create(&timer_args, &timer_handle_);
        if (ret != ESP_OK) {
            ESP_LOGE("PowerManager", "Failed to create timer: %s", esp_err_to_name(ret));
            return;
        }
        
        // 创建提醒定时器
        esp_timer_create_args_t alert_timer_args = {};
        alert_timer_args.callback = AlertTimerCallback;
        alert_timer_args.arg = this;
        alert_timer_args.dispatch_method = ESP_TIMER_TASK;
        alert_timer_args.name = "low_battery_alert_timer";
        alert_timer_args.skip_unhandled_events = true;
        
        ret = esp_timer_create(&alert_timer_args, &alert_timer_handle_);
        if (ret != ESP_OK) {
            ESP_LOGE("PowerManager", "Failed to create alert timer: %s", esp_err_to_name(ret));
            esp_timer_delete(timer_handle_);
            timer_handle_ = nullptr;
            return;
        }
        
        // 每2秒检查一次
        ret = esp_timer_start_periodic(timer_handle_, kCheckIntervalUs);
        if (ret != ESP_OK) {
            ESP_LOGE("PowerManager", "Failed to start timer: %s", esp_err_to_name(ret));
            esp_timer_delete(timer_handle_);
            esp_timer_delete(alert_timer_handle_);
            timer_handle_ = nullptr;
            alert_timer_handle_ = nullptr;
            return;
        }
        
        CheckBatteryStatus();
    }
    
    ~PowerManager() {
        StopAlerting();
        if (timer_handle_) {
            esp_timer_stop(timer_handle_);
            esp_timer_delete(timer_handle_);
        }
        if (alert_timer_handle_) {
            esp_timer_stop(alert_timer_handle_);
            esp_timer_delete(alert_timer_handle_);
        }
        if (adc_handle_) {
            adc_oneshot_del_unit(adc_handle_);
        }
    }
    
    bool IsCharging() const { return is_charging_; }
    bool IsDischarging() const { return !is_charging_; }
    uint8_t GetBatteryLevel() const { return battery_level_; }
    float GetBatteryVoltage() const { return battery_voltage_; }
    
    bool IsLowBatteryAlerting() const { return is_alerting_; }
    bool IsLowBatteryTriggered() const { return low_battery_triggered_; }
    
    uint32_t GetRawAdcValue() {
        return ReadAdc();
    }
    
    void OnChargingStatusChanged(std::function<void(bool)> callback) {
        on_charging_status_changed_ = callback;
    }
    
    void OnLowBatteryAlert(std::function<void(uint8_t)> callback) {
        on_low_battery_alert_ = callback;
    }
    
    void StartLowBatteryAlert() {
        if (battery_level_ <= kLowBatteryAlertThreshold) {
            StartAlerting();
        }
    }
    
    void StopLowBatteryAlert() {
        StopAlerting();
        low_battery_triggered_ = false;
    }
    
    void ForceUpdate() {
        CheckBatteryStatus();
    }

};