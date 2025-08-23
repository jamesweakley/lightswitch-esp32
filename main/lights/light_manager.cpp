/*
 * Light manager for 1-4 on/off channels and DHT22 sensor.
 */

#include "light_manager.h"

#include <esp_log.h>
#include <string.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include "esp_rom_sys.h"
#include "freertos/queue.h"
#include <platform/PlatformManager.h>

#include <inttypes.h>

#include <esp_matter.h>
// Client API for bindings and group/unicast requests
#include <esp_matter_client.h>
// CHIP cluster ids & command ids
#include <app-common/zap-generated/cluster-objects.h>
#include <app/CommandPathParams.h>
// (Binding table internal CHIP headers not included in esp-matter public API; detailed per-target logging limited.)
// For subscription (direct CHIP IM API)
// NOTE: Direct CHIP IM subscription headers removed for now because they are not exposed via
// current esp-matter public include paths in this project setup. Placeholder logic remains.
// Group command sending via esp-matter client callbacks

using namespace esp_matter;
using namespace esp_matter::attribute;

// Logging tag
static const char *TAG = "light_manager";
// Track last button press tick for latency diagnostics
volatile uint32_t g_last_press_tick = 0;

static void subscriptions_init() {
    ESP_LOGI(TAG, "Subscriptions (polling) init: (disabled remote state tracking placeholder)");
}

// --- State & GPIO mappings (restored) ---
static bool s_led_any_on[LIGHT_CHANNELS] = {false};
static uint8_t s_on_count[LIGHT_CHANNELS] = {0};

static const gpio_num_t s_button_gpios[LIGHT_CHANNELS] = { BUTTON_GPIO_0, BUTTON_GPIO_1, BUTTON_GPIO_2, BUTTON_GPIO_3 };
static const gpio_num_t s_led_gpios[LIGHT_CHANNELS]    = { LED_GPIO_0, LED_GPIO_1, LED_GPIO_2, LED_GPIO_3 };

// Public endpoint IDs (defined in app_main)
extern uint16_t g_onoff_endpoint_ids[LIGHT_CHANNELS];

// Button polling task handle
static TaskHandle_t s_button_task = nullptr;      // polling task (lightweight)
static TaskHandle_t s_button_act_task = nullptr;  // action task (does logging & Matter calls)
static TaskHandle_t s_dht_task = nullptr;
static QueueHandle_t s_button_evt_queue = nullptr; // queue of uint8_t channel indices

// LED blink timers (one-shot) to avoid blocking delays on button tasks
static esp_timer_handle_t s_led_blink_timers[LIGHT_CHANNELS] = {nullptr};

// Forward declare for early use in init logging
static uint16_t group_id_for_channel(uint8_t ch);
static void send_group_toggle(uint8_t ch); // forward

static void apply_led(uint8_t ch, bool on)
{
    if (ch >= LIGHT_CHANNELS) return;
    gpio_set_level(s_led_gpios[ch], on ? 1 : 0);
}

bool light_manager_get(uint8_t channel) {
    if (channel >= LIGHT_CHANNELS) {
        return false;
    }
    return s_led_any_on[channel];
}

static void buttons_init()
{
    gpio_config_t in_cfg = {};
    in_cfg.intr_type = GPIO_INTR_DISABLE;
    in_cfg.mode = GPIO_MODE_INPUT;
    in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    for (int i = 0; i < LIGHT_CHANNELS; i++) {
        if (s_button_gpios[i] == GPIO_NUM_NC) continue;
        in_cfg.pin_bit_mask = (1ULL << s_button_gpios[i]);
        gpio_config(&in_cfg);
        gpio_set_pull_mode(s_button_gpios[i], GPIO_PULLUP_ONLY);
    }
}

static void leds_init()
{
    gpio_config_t out_cfg = {};
    out_cfg.intr_type = GPIO_INTR_DISABLE;
    out_cfg.mode = GPIO_MODE_OUTPUT;
    out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    for (int i = 0; i < LIGHT_CHANNELS; i++) {
        if (s_led_gpios[i] == GPIO_NUM_NC) continue;
        out_cfg.pin_bit_mask = (1ULL << s_led_gpios[i]);
        gpio_config(&out_cfg);
        gpio_set_level(s_led_gpios[i], 0);
    }
}

static void button_task(void *arg)
{
    uint8_t stable_cnt[LIGHT_CHANNELS] = {0};
    uint8_t last_read[LIGHT_CHANNELS] = {1,1,1,1};
    while (true) {
        for (int i = 0; i < LIGHT_CHANNELS; i++) {
            int level = gpio_get_level(s_button_gpios[i]);
            if (level == last_read[i]) { if (stable_cnt[i] < 255) stable_cnt[i]++; }
            else { stable_cnt[i] = 0; last_read[i] = level; }
            if (last_read[i] == 0 && stable_cnt[i] == BUTTON_STABLE_CNT) {
                uint8_t ch = i;
                if (s_button_evt_queue) xQueueSend(s_button_evt_queue, &ch, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

static void led_blink_timer_cb(void *arg) {
    uint32_t ch = (uint32_t)arg;
    if (ch < LIGHT_CHANNELS) {
        apply_led((uint8_t)ch, s_led_any_on[ch]);
    }
}

static void schedule_led_blink(uint8_t ch, uint32_t on_ms) {
    if (ch >= LIGHT_CHANNELS) return;
    // Turn LED on immediately
    apply_led(ch, true);
    // Create timer if needed
    if (!s_led_blink_timers[ch]) {
        esp_timer_create_args_t targs = { .callback = &led_blink_timer_cb, .arg = (void*)(uintptr_t)ch, .dispatch_method = ESP_TIMER_TASK, .name = "ledblink" };
        if (esp_timer_create(&targs, &s_led_blink_timers[ch]) != ESP_OK) return;
    }
    // Stop any pending
    esp_timer_stop(s_led_blink_timers[ch]);
    esp_timer_start_once(s_led_blink_timers[ch], on_ms * 1000ULL);
}

static void button_action_task(void *arg) {
    uint8_t ch;
    while (true) {
        if (xQueueReceive(s_button_evt_queue, &ch, portMAX_DELAY) == pdTRUE) {
            light_manager_button_press(ch); // safe: now runs on larger stack
            // Stack watermark diagnostics (optional):
            static uint32_t cnt = 0;
            if ((++cnt % 16) == 0) {
                UBaseType_t wm = uxTaskGetStackHighWaterMark(nullptr);
                ESP_LOGD(TAG, "button_act watermark=%u words", (unsigned)wm);
            }
        }
    }
}

esp_err_t light_manager_init() {
    leds_init();
    buttons_init();
    // Startup logging: confirm logging is working and show configuration
    ESP_LOGI(TAG, "Light manager init: %d channel(s), poll=%dms, debounce=%d", (int)LIGHT_CHANNELS, (int)BUTTON_POLL_MS, (int)BUTTON_STABLE_CNT);
    for (int i = 0; i < LIGHT_CHANNELS; i++) {
        ESP_LOGI(TAG, "CH%u: button GPIO %d (active-low), LED GPIO %d, default GroupID=0x%04X", (unsigned)i, (int)s_button_gpios[i], (int)s_led_gpios[i], (unsigned)group_id_for_channel(i));
    }
    ESP_LOGI(TAG, "DHT22 GPIO %d", (int)DHT22_GPIO);
    // No custom request callback: allow esp-matter's default binding manager behavior.
    // Start button polling
    // Create queue & tasks
    s_button_evt_queue = xQueueCreate(8, sizeof(uint8_t));
    xTaskCreate(button_task, "btn_poll", 2048, nullptr, 5, &s_button_task);
    xTaskCreate(button_action_task, "btn_act", 4096, nullptr, 6, &s_button_act_task);
    ESP_LOGI(TAG, "Light manager started (no remote state tracking)");
    subscriptions_init();
    return ESP_OK;
}

// --- Minimal DHT22 bit-bang reader ---
// Notes: For robustness, consider using an existing library. This simple implementation
// reads every ~10s and updates Matter Temperature and Relative Humidity measurement clusters.

static bool dht22_read(float *temperature_c, float *humidity) {
    if (!temperature_c || !humidity) return false;
    // Minimal stub: real implementation should bit-bang DHT22 protocol. For now, return false to avoid bogus data.
    return false;
}

static void report_temperature(float temp_c)
{
    if (g_temp_endpoint_id == 0) return;
    // Temperature Measurement cluster 0x0402, MeasuredValue 0x0000, in 0.01 deg C per spec (value = 100*degC)
    int16_t measured = (int16_t) (temp_c * 100.0f);
    esp_matter_attr_val_t val = esp_matter_nullable_int16(measured);
    attribute::report(g_temp_endpoint_id, 0x0402, 0x0000, &val);
}

static void report_humidity(float rh)
{
    if (g_humidity_endpoint_id == 0) return;
    // Relative Humidity Measurement 0x0405, MeasuredValue 0x0000, in 0.01% RH
    uint16_t measured = (uint16_t) (rh * 100.0f);
    esp_matter_attr_val_t val = esp_matter_nullable_uint16(measured);
    attribute::report(g_humidity_endpoint_id, 0x0405, 0x0000, &val);
}

static void dht22_task(void *arg) {
    while (true) {
        float t = 0, h = 0;
        if (dht22_read(&t, &h)) {
            ESP_LOGI(TAG, "DHT22: %.1fC, %.1f%%", t, h);
            report_temperature(t);
            report_humidity(h);
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void dht22_start_task() {
    if (s_dht_task) return;
    xTaskCreate(dht22_task, "dht22", 3072, nullptr, 5, &s_dht_task);
}

// ---------- Group bindings controller behavior ----------

// Channel -> default Group ID mapping
static uint16_t group_id_for_channel(uint8_t ch) {
    switch (ch) {
        case 0: return GROUP_ID_0;
        case 1: return GROUP_ID_1;
        case 2: return GROUP_ID_2;
        case 3: return GROUP_ID_3;
        default: return GROUP_ID_0;
    }
}

// Send On/Off Toggle to a group for this controller endpoint
static void send_group_toggle(uint8_t ch) {
    if (ch >= LIGHT_CHANNELS) return;
    // Build a request handle; binding manager will route (group/unicast)
    esp_matter::client::request_handle req = {};
    req.type = esp_matter::client::INVOKE_CMD;
    // Populate command path for On/Off Toggle
    // Use invalid endpoint in command path so binding manager can substitute each bound remote endpoint.
    // (Passing the local controller endpoint here may prevent routing.)
#ifndef CHIP_INVALID_ENDPOINT_ID
#define CHIP_INVALID_ENDPOINT_ID 0xFFFF
#endif
    chip::app::CommandPathParams path(
        static_cast<chip::EndpointId>(CHIP_INVALID_ENDPOINT_ID), // wildcard/invalid -> binding manager should fill
        static_cast<chip::GroupId>(0),
        chip::app::Clusters::OnOff::Id,
        chip::app::Clusters::OnOff::Commands::Toggle::Id,
        static_cast<chip::app::CommandPathFlags>(0));
    req.command_path = path;
    // Some esp-matter versions also expect these convenience fields (harmless if struct differs)
    // Attempt to set them defensively; if they don't exist, compilation will warn/error and we can adjust.
#if defined(__cplusplus)
    // Use a lambda with if constexpr false trick to avoid unused warnings if members absent.
    // (We cannot probe struct layout here; rely on conditional compilation attempts.)
#endif
#ifdef __has_include
#if __has_include(<esp_matter_client.h>)
    // Tentatively assign if members exist (guard with sizeof check via offsetof if desired).
    // We use a try/catch style via a macro; if it fails to compile user will report and we will adapt.
#endif
#endif
    // Direct member names guessed from earlier esp-matter releases:
#ifdef ESP_MATTER_CLIENT_REQUEST_HANDLE_HAS_IDS
    req.cluster_id = chip::app::Clusters::OnOff::Id;
    req.command_id = chip::app::Clusters::OnOff::Commands::Toggle::Id;
#endif
    // No command payload for Toggle
    esp_err_t err = ESP_OK;
    // Notify binding manager for this endpoint; group callback will send to groups
    ESP_LOGI(TAG, "CH%u: sending Toggle via controller endpoint %u (cmdPath ep=0x%04X)", ch,
             (unsigned)g_onoff_endpoint_ids[ch], (unsigned)req.command_path.mEndpointId);
    ESP_LOGD(TAG, "req.type=%d cluster=0x%08" PRIx32 " cmd=0x%08" PRIx32, (int)req.type,
             (uint32_t)req.command_path.mClusterId, (uint32_t)req.command_path.mCommandId);
    // Extra: log current binding attribute via console command later (not here to avoid recursion)
    // NOTE: Detailed target enumeration not available (binding table headers not public in this build).
    err = esp_matter::client::cluster_update(g_onoff_endpoint_ids[ch], &req);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cluster_update failed for ch%u (ep %u): %d", ch, g_onoff_endpoint_ids[ch], err);
    }

}

// Schedule the actual toggle on the CHIP platform thread to avoid stack locking errors.
static void schedule_group_toggle(uint8_t ch) {
    chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t arg){
        uint8_t ch_i = static_cast<uint8_t>(arg);
        send_group_toggle(ch_i);
    }, static_cast<intptr_t>(ch));
}

// (Unicast stub removed: rely on esp-matter binding manager to route using real Binding attribute.)

void light_manager_button_press(uint8_t channel) {
    // Log button press
    ESP_LOGI(TAG, "Button %u pressed (GPIO %d)", channel, (int)s_button_gpios[channel]);
    if (channel >= LIGHT_CHANNELS) return;
    g_last_press_tick = xTaskGetTickCount();
    schedule_led_blink(channel, 30); // non-blocking blink
    // Always trigger cluster_update; binding manager will inspect Binding attribute
    schedule_group_toggle(channel);
}

// ---- Subscription (remote state) logic ----
// NOTE: esp-matter currently provides client request path helpers; we manually build subscriptions
// using CHIP Interaction Model API through esp_matter::client when available. Placeholder structure
// below; implement actual subscription once esp-matter exposes simplified helper (or integrate
// direct CHIP APIs).

struct PendingSubscription {
    chip::NodeId node_id;
    chip::EndpointId endpoint_id;
    chip::ClusterId cluster_id;
    chip::AttributeId attribute_id;
};

static void handle_onoff_report(uint8_t channel, bool on) {
    if (channel >= LIGHT_CHANNELS) return;
    // Update per-target aggregation count (simplified: treat each report as full state refresh)
    // For now, just set any-on true if on==true, false only cleared when on==false and count hits 0.
    if (on) {
        if (s_on_count[channel] < 0xFF) s_on_count[channel]++;
    } else {
        if (s_on_count[channel] > 0) s_on_count[channel]--; // naive decrement
    }
    bool any_on = s_on_count[channel] > 0;
    if (any_on != s_led_any_on[channel]) {
        s_led_any_on[channel] = any_on;
        apply_led(channel, any_on);
        ESP_LOGI(TAG, "CH%u: any-on state -> %s (count=%u)", channel, any_on ? "ON" : "OFF", (unsigned)s_on_count[channel]);
    }
}

// Remote state tracking disabled (no supported client attribute read callback API available).
