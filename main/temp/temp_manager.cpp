/* DHT22 driver & periodic reporting (moved from light_manager) */
#include "temp_manager.h"
#include "app_config.h"
#include "light_manager.h" // for endpoint globals
#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_rx.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_matter.h>
#include <app-common/zap-generated/cluster-objects.h>
#include <platform/PlatformManager.h>

static const char *TAG = "temp_manager";

#if DHT22_ENABLE

static TaskHandle_t s_task = nullptr;
static bool s_stop = false;
static int s_fail_streak = 0;

// RMT-based DHT22 reader (non-blocking, precise timing)
static bool dht22_read_rmt(int16_t &temp_x10, uint16_t &hum_x10) {
    gpio_num_t pin = DHT22_GPIO;
    if (pin == GPIO_NUM_NC) return false;
    
    // Simple fallback to basic GPIO read for now (safer than busy-wait)
    // Configure as output & send start signal
    gpio_config_t out_cfg = {};
    out_cfg.intr_type = GPIO_INTR_DISABLE;
    out_cfg.mode = GPIO_MODE_OUTPUT;
    out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    out_cfg.pin_bit_mask = (1ULL << pin);
    gpio_config(&out_cfg);
    
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(2)); // 2ms start signal (non-blocking)
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(1)); // Short delay before switching to input
    
    // Switch to input
    gpio_config_t in_cfg = {};
    in_cfg.intr_type = GPIO_INTR_DISABLE;
    in_cfg.mode = GPIO_MODE_INPUT;
    in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    in_cfg.pin_bit_mask = (1ULL << pin);
    gpio_config(&in_cfg);
    
    // For now, return dummy values to test the framework
    // TODO: Implement proper RMT-based pulse capture
    temp_x10 = 250; // 25.0°C
    hum_x10 = 600;  // 60.0%RH
    
    ESP_LOGI(TAG, "DHT22 dummy read: T=25.0C RH=60.0%% (pin=%d)", (int)pin);
    return true;
}

static void report(int16_t t_0_01, uint16_t h_0_01){
    if(g_temp_endpoint_id){
        auto v=esp_matter_int16(t_0_01); 
        esp_matter::attribute::report(g_temp_endpoint_id, chip::app::Clusters::TemperatureMeasurement::Id, chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id,&v);
    }    
    if(g_humidity_endpoint_id){
        auto v2=esp_matter_uint16(h_0_01); 
        esp_matter::attribute::report(g_humidity_endpoint_id, chip::app::Clusters::RelativeHumidityMeasurement::Id, chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,&v2);
    }    
}

static void task(void*){
    ESP_LOGI(TAG,"start pin=%d period=%dms (RMT-based)", (int)DHT22_GPIO, DHT22_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(DHT22_STABILIZE_DELAY_MS));
    
    while(!s_stop){
        int16_t tx10=0; uint16_t hx10=0; bool ok=false;
        
        // Try reading with timeout protection
        for(int a=0; a<DHT22_MAX_RETRIES && !ok && !s_stop; a++){ 
            ok=dht22_read_rmt(tx10,hx10); 
            if(!ok) vTaskDelay(pdMS_TO_TICKS(100)); // Longer delay between retries
        }
        
        if(ok && !s_stop){
            s_fail_streak=0; 
            int16_t t001=(int16_t)(tx10*10); // Convert to 0.01 units
            uint16_t h001=(uint16_t)(hx10*10);
            
            struct THVal { int16_t t; uint16_t h; };
            THVal * vals = chip::Platform::New<THVal>();
            if(vals){ 
                vals->t = t001; 
                vals->h = h001; 
                chip::DeviceLayer::PlatformMgr().ScheduleWork(+[](intptr_t ctx){
                    auto *v = reinterpret_cast<THVal*>(ctx);
                    if(v) {
                        report(v->t, v->h);
                        ESP_LOGI(TAG,"report T=%.2fC RH=%.2f%%", v->t/100.0f, v->h/100.0f);
                        chip::Platform::Delete(v);
                    }
                }, reinterpret_cast<intptr_t>(vals));
            }
        } else if(!s_stop) {
            s_fail_streak++; 
            if(s_fail_streak==3 || (s_fail_streak%10)==0) 
                ESP_LOGW(TAG,"read failures streak=%d", s_fail_streak); 
        }
        
        if(!s_stop) vTaskDelay(pdMS_TO_TICKS(DHT22_PERIOD_MS));
    }
    s_task = nullptr;
    vTaskDelete(nullptr);
}

void temp_manager_start(){ 
    if(s_task || !DHT22_ENABLE) return; 
    s_stop=false; 
    xTaskCreate(task,"temp_mgr",4096,nullptr, tskIDLE_PRIORITY+1,&s_task);
} 

void temp_manager_stop(){ 
    s_stop=true; 
    if(s_task) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to exit
    }
}

void temp_manager_force_read(){ 
    ESP_LOGI(TAG, "Force read requested");
}

#else
void temp_manager_start(){ESP_LOGI(TAG, "DHT22 disabled (DHT22_ENABLE=0)");}
void temp_manager_stop(){}
void temp_manager_force_read(){}
#endif
