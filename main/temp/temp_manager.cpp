/* DHT22 driver & periodic reporting (moved from light_manager) */
#include "temp_manager.h"
#include "app_config.h"
#include "light_manager.h" // for endpoint globals
#include <esp_log.h>
#include <driver/gpio.h>
// Using new RMT RX driver API.
// TODO: migrate to new rmt_tx/rx API to eliminate deprecation warning.
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
static bool s_have_valid = false;
static int16_t s_last_t_0_01 = 0; // stored in 0.01C
static uint16_t s_last_h_0_01 = 0; // stored in 0.01%RH
static int s_warmup_discarded = 0;

// RMT-based DHT22 reader using new RMT RX driver (captures pulse widths instead of busy-wait)
/*
 * DHT22 timing (typical):
 *  Host start: pull low >=1ms (we use macro DHT22_START_LOW_MS) then release high ~20-40us.
 *  Sensor reply: 80us low + 80us high (presence).
 *  Each bit: 50us low then high: ~26-28us => '0', ~70us => '1'.
 *  We sample via RMT at 1MHz resolution (1us units). We parse symbols where
 *  level0=0 (low), level1=1 (high). duration0 (~50us) indicates bit frame start; duration1 discriminates value.
 */
// Static RMT channel state (created once). If creation fails (e.g. no free channels), we
// suppress further attempts to avoid log spam and simply skip readings (reported as failure).
static rmt_channel_handle_t s_rx_channel = nullptr;
static bool s_rmt_init_attempted = false;

static bool dht22_rmt_ensure_channel(gpio_num_t pin) {
#if !DHT22_USE_RMT
    return false;
#else
    if (s_rx_channel) return true;
    if (s_rmt_init_attempted) return false; // Already tried & failed earlier
    s_rmt_init_attempted = true;
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = DHT22_RMT_RESOLUTION_HZ,
        // DHT22 frame: presence (2 symbols) + 40 bit symbols + margin; 64 is sufficient
        .mem_block_symbols = 64,
        .flags = { .invert_in = 0, .with_dma = 0 }
    };
    esp_err_t e = rmt_new_rx_channel(&rx_cfg, &s_rx_channel);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "RMT channel alloc failed err=%d (will not retry)", (int)e);
        return false;
    }
    struct RxState { volatile bool done; volatile size_t symbols; };
    static RxState s_rx_local = { false, 0 };
    auto rx_done_cb = [](rmt_channel_handle_t, const rmt_rx_done_event_data_t *edata, void *user)->bool{
        auto *st = reinterpret_cast<RxState*>(user);
        if(st) { st->symbols = edata->num_symbols; st->done = true; }
        return false; // no need to yield
    };
    rmt_rx_event_callbacks_t cbs = { .on_recv_done = rx_done_cb };
    rmt_rx_register_event_callbacks(s_rx_channel, &cbs, &s_rx_local);
    rmt_enable(s_rx_channel);
    ESP_LOGI(TAG, "RMT RX channel created for DHT22 (pin=%d)", (int)pin);
    return true;
#endif
}

static bool dht22_read_rmt(int16_t &temp_x10, uint16_t &hum_x10) {
#if !DHT22_USE_RMT
    return false; // RMT disabled
#else
    gpio_num_t pin = DHT22_GPIO;
    if (pin == GPIO_NUM_NC) return false;
    if(!dht22_rmt_ensure_channel(pin)) {
        return false; // channel unavailable
    }

    // Local static capture buffer/state (separate from channel init helper)
    struct RxState { volatile bool done; volatile size_t symbols; };
    // We reuse the callback's static (registered during ensure_channel). Retrieve its address by static storage in ensure function.
    // For parsing we need a buffer of symbols.
    static rmt_symbol_word_t s_symbols[64];
    extern bool s_rmt_init_attempted; // silence unused warning if not used elsewhere

    // We can't get internal RxState pointer easily; so we re-register a lightweight callback each read.
    static RxState s_rx = { false, 0 };
    auto rx_done_cb2 = [](rmt_channel_handle_t, const rmt_rx_done_event_data_t *edata, void *user)->bool{
        auto *st = reinterpret_cast<RxState*>(user);
        if(st) { st->symbols = edata->num_symbols; st->done = true; }
        return false;
    };
    rmt_rx_event_callbacks_t cbs2 = { .on_recv_done = rx_done_cb2 };
    rmt_rx_register_event_callbacks(s_rx_channel, &cbs2, &s_rx);

    // --- Issue start signal ---
    gpio_config_t out_cfg = {};
    out_cfg.intr_type = GPIO_INTR_DISABLE;
    out_cfg.mode = GPIO_MODE_OUTPUT;
    out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_cfg.pull_up_en = GPIO_PULLUP_DISABLE; // we'll enable pull-up after release
    out_cfg.pin_bit_mask = (1ULL << pin);
    gpio_config(&out_cfg);
    gpio_set_level(pin, 0);
    vTaskDelay(pdMS_TO_TICKS(DHT22_START_LOW_MS));
    gpio_set_level(pin, 1);
    // Short 30us delay before switching to input (spec requires 20-40us)
    esp_rom_delay_us(30);

    // Switch pin to input (avoid bus contention) with pull-up
    gpio_config_t in_cfg = {};
    in_cfg.intr_type = GPIO_INTR_DISABLE;
    in_cfg.mode = GPIO_MODE_INPUT;
    in_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in_cfg.pin_bit_mask = (1ULL << pin);
    gpio_config(&in_cfg);

    // Prepare RX receive
    rmt_receive_config_t recv_cfg = {
        .signal_range_min_ns = 300,        // ignore very brief glitches
        .signal_range_max_ns = 1500000     // 1.5ms max to keep capture tolerant
    };
    s_rx.done = false; s_rx.symbols = 0;
    if(rmt_receive(s_rx_channel, s_symbols, sizeof(s_symbols), &recv_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "rmt_receive start failed");
        return false;
    }

    // Wait (poll) for completion: DHT22 frame <5ms. Give 8ms budget.
    const int MAX_WAIT_MS = 8;
    for(int waited=0; !s_rx.done && waited<MAX_WAIT_MS; waited++) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    if(!s_rx.done) {
        ESP_LOGW(TAG, "RMT timeout (no complete frame)");
        return false;
    }
    size_t symbol_count = s_rx.symbols;
    if(symbol_count < 10) { // Definitely too short (need presence + bits)
        ESP_LOGW(TAG, "Too few symbols=%u", (unsigned)symbol_count);
        return false;
    }

    // Build flat pulse list (level,duration) from symbol pairs to tolerate starting polarity.
    static uint16_t pulse_dur[128];
    static uint8_t pulse_lvl[128];
    size_t pulse_count = 0;
    for(size_t i=0; i<symbol_count && pulse_count+1 < 128; i++) {
        const rmt_symbol_word_t &sym = s_symbols[i];
        pulse_lvl[pulse_count] = sym.level0; pulse_dur[pulse_count] = sym.duration0; pulse_count++;
        pulse_lvl[pulse_count] = sym.level1; pulse_dur[pulse_count] = sym.duration1; pulse_count++;
    }
    // Scan for low-high pairs
    uint8_t data[5] = {0};
    int bit_index = 0;
    int presence_skipped = 0;
    for(size_t i=0; i+1<pulse_count && bit_index < 40; i++) {
        if(pulse_lvl[i] != 0 || pulse_lvl[i+1] != 1) continue; // need low then high consecutive
        uint32_t low = pulse_dur[i];
        uint32_t high = pulse_dur[i+1];
        // Presence pulse pair (~80/80) - skip first two
        if(presence_skipped < 2 && low >= 60 && low <= 110 && high >= 60 && high <= 110) {
            presence_skipped++;
            continue;
        }
        if(low < 30 || low > 100) continue; // filter improbable low
        uint8_t bit = (high > DHT22_BIT_THRESHOLD_US) ? 1 : 0;
        data[bit_index/8] = (uint8_t)((data[bit_index/8] << 1) | bit);
#if DHT22_DEBUG
        ESP_LOGD(TAG, "bit%02d low=%u high=%u val=%u", bit_index, (unsigned)low, (unsigned)high, bit);
#endif
        bit_index++;
        i++; // advance past the high we used
    }
    if(bit_index != 40) {
        if(bit_index == 0) {
            // Dump first few pulses for diagnostics
            char buf[192];
            int off=0;
            off += snprintf(buf+off, sizeof(buf)-off, "pulse dump (lvl:durus) ");
            size_t dump_n = pulse_count < 20 ? pulse_count : 20;
            for(size_t j=0;j<dump_n && off < (int)sizeof(buf)-8; j++) {
                off += snprintf(buf+off, sizeof(buf)-off, "%u:%u ", (unsigned)pulse_lvl[j], (unsigned)pulse_dur[j]);
            }
            ESP_LOGW(TAG, "%s", buf);
        }
        ESP_LOGW(TAG, "bits parsed=%d (expected 40) symbols=%u preskip=%d pulses=%u", bit_index, (unsigned)symbol_count, presence_skipped, (unsigned)pulse_count);
        return false;
    }
    uint8_t sum = (uint8_t)((data[0]+data[1]+data[2]+data[3]) & 0xFF);
    if(sum != data[4]) {
        ESP_LOGW(TAG, "Checksum mismatch %02X!=%02X", sum, data[4]);
        return false;
    }
    if(data[0]==0 && data[1]==0 && data[2]==0 && data[3]==0) {
        ESP_LOGW(TAG, "All-zero frame (RMT)");
        return false;
    }
    uint16_t raw_h = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_t_mag = ((uint16_t)(data[2] & 0x7F) << 8) | data[3];
    bool neg = (data[2] & 0x80) != 0;
    hum_x10 = raw_h;
    temp_x10 = neg ? -(int16_t)raw_t_mag : (int16_t)raw_t_mag;
    if(temp_x10 < DHT22_TEMP_MIN_X10 || temp_x10 > DHT22_TEMP_MAX_X10 || hum_x10 < DHT22_HUM_MIN_X10 || hum_x10 > DHT22_HUM_MAX_X10) {
        ESP_LOGW(TAG, "Out-of-range t=%d h=%u", (int)temp_x10, (unsigned)hum_x10);
        return false;
    }
    ESP_LOGI(TAG, "DHT22 (RMT) T=%.1fC RH=%.1f%% bits_ok symbols=%u", temp_x10/10.0f, hum_x10/10.0f, (unsigned)symbol_count);
    return true;
#endif // DHT22_USE_RMT
}

// MeasuredValue accepted types resolved earlier via probing: Temperature -> nullable int16, Humidity -> nullable uint16
static void report(int16_t t_0_01, uint16_t h_0_01){
    // Cluster spec: MeasuredValue is nullable. Always send nullable to avoid type mismatch.
    if(t_0_01 < -27315) t_0_01 = -27315;
    if(t_0_01 > 32767) t_0_01 = 32767;
    if(h_0_01 > 10000) h_0_01 = 10000;

    if (g_temp_endpoint_id) {
        esp_matter_attr_val_t v{}; v.type = ESP_MATTER_VAL_TYPE_NULLABLE_INT16; v.val.i16 = t_0_01;
        esp_err_t err = esp_matter::attribute::report(
            g_temp_endpoint_id,
            chip::app::Clusters::TemperatureMeasurement::Id,
            chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id,
            &v);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Temp report nullable failed err=%d", err);
        } else {
            ESP_LOGD(TAG, "Temp (nullable)=%d", (int)v.val.i16);
        }
    }
    if (g_humidity_endpoint_id) {
        esp_matter_attr_val_t v2{}; v2.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16; v2.val.u16 = h_0_01;
        esp_err_t errh = esp_matter::attribute::report(
            g_humidity_endpoint_id,
            chip::app::Clusters::RelativeHumidityMeasurement::Id,
            chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
            &v2);
        if (errh != ESP_OK) {
            ESP_LOGE(TAG, "Humidity report nullable failed err=%d", errh);
        } else {
            ESP_LOGD(TAG, "Humidity (nullable)=%u", (unsigned)v2.val.u16);
        }
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

#if DHT22_DISCARD_ZERO_FRAME
            bool zero_frame = (t001 == 0 && h001 == 0);
            if(zero_frame) {
                ESP_LOGW(TAG, "Discarding all-zero DHT22 frame (suspect)");
                ok = false; // treat as failure for retry/backoff logic
            }
#endif

            if(ok && s_warmup_discarded < DHT22_WARMUP_READS) {
                s_warmup_discarded++;
                ESP_LOGI(TAG, "Warmup discard %d/%d", s_warmup_discarded, DHT22_WARMUP_READS);
                ok = false; // skip report
            }
            
            if(ok) {
                s_have_valid = true;
                s_last_t_0_01 = t001;
                s_last_h_0_01 = h001;
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
            } else {
                // On invalid frame, optionally re-report last known good after several failures
                if(s_have_valid && (s_fail_streak == 5 || s_fail_streak == 15)) {
                    struct THVal { int16_t t; uint16_t h; };
                    THVal * vals = chip::Platform::New<THVal>();
                    if(vals){
                        vals->t = s_last_t_0_01; vals->h = s_last_h_0_01;
                        chip::DeviceLayer::PlatformMgr().ScheduleWork(+[](intptr_t ctx){
                            auto *v = reinterpret_cast<THVal*>(ctx);
                            if(v){ report(v->t, v->h); chip::Platform::Delete(v);} }, reinterpret_cast<intptr_t>(vals));
                        ESP_LOGI(TAG, "Re-reporting last valid reading after failures");
                    }
                }
            }
        } else if(!s_stop) {
            s_fail_streak++;
            if(s_fail_streak==3 || (s_fail_streak%10)==0) {
                ESP_LOGW(TAG,"read failures streak=%d", s_fail_streak);
            }
        }
        
        if(!s_stop) vTaskDelay(pdMS_TO_TICKS(DHT22_PERIOD_MS));
    }
    s_task = nullptr;
    vTaskDelete(nullptr);
}

void temp_manager_start(){ 
    if(s_task || !DHT22_ENABLE) return; 
    s_stop=false; 
    // Prime attributes with NULL so esp-matter sets internal type expectations
    if (g_temp_endpoint_id) {
        esp_matter_attr_val_t v{}; v.type = ESP_MATTER_VAL_TYPE_NULLABLE_INT16; v.val.i16 = 0; // non-null 0
        esp_matter::attribute::report(g_temp_endpoint_id,
            chip::app::Clusters::TemperatureMeasurement::Id,
            chip::app::Clusters::TemperatureMeasurement::Attributes::MeasuredValue::Id,
            &v);
    }
    if (g_humidity_endpoint_id) {
        esp_matter_attr_val_t v2{}; v2.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16; v2.val.u16 = 0;
        esp_matter::attribute::report(g_humidity_endpoint_id,
            chip::app::Clusters::RelativeHumidityMeasurement::Id,
            chip::app::Clusters::RelativeHumidityMeasurement::Attributes::MeasuredValue::Id,
            &v2);
    }
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
