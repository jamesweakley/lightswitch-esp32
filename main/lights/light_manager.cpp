/*
 * Light manager for 1-4 on/off channels and DHT22 sensor.
 */

#include "light_manager.h"

#include <esp_log.h>
#include <string.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include "esp_rom_sys.h"

#include <esp_matter.h>
// Client API for bindings and group/unicast requests
#include <esp_matter_client.h>
// CHIP cluster ids & command ids
#include <app-common/zap-generated/cluster-objects.h>
#include <app/CommandPathParams.h>
// Group command sending via esp-matter client callbacks

using namespace esp_matter;
using namespace esp_matter::attribute;

static const char *TAG = "light_manager";

// State storage (LED mirrors any-on from subscribed targets; default off)
static bool s_led_any_on[LIGHT_CHANNELS] = {false};

// GPIO mapping helpers
static const gpio_num_t s_button_gpios[LIGHT_CHANNELS] = { BUTTON_GPIO_0, BUTTON_GPIO_1, BUTTON_GPIO_2, BUTTON_GPIO_3 };
static const gpio_num_t s_led_gpios[LIGHT_CHANNELS]    = { LED_GPIO_0, LED_GPIO_1, LED_GPIO_2, LED_GPIO_3 };

// Public endpoint IDs (set from app_main when created). These are controller endpoints.
uint16_t g_onoff_endpoint_ids[LIGHT_CHANNELS] = {0};
uint16_t g_temp_endpoint_id = 0;
uint16_t g_humidity_endpoint_id = 0;

// Button polling task
static TaskHandle_t s_button_task = nullptr;

// DHT22 task
static TaskHandle_t s_dht_task = nullptr;

static void apply_led(uint8_t ch, bool on)
{
    if (ch >= LIGHT_CHANNELS) return;
    gpio_set_level(s_led_gpios[ch], on ? 1 : 0);
}

bool light_manager_get(uint8_t channel)
{
    if (channel >= LIGHT_CHANNELS) return false;
    return s_led_any_on[channel];
}

static void buttons_init()
{
    // Configure button inputs with pull-ups (active low)
    gpio_config_t in_cfg = {};
    in_cfg.intr_type = GPIO_INTR_DISABLE;
    in_cfg.mode = GPIO_MODE_INPUT;
    in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in_cfg.pull_up_en = GPIO_PULLUP_ENABLE;

    for (int i = 0; i < LIGHT_CHANNELS; i++) {
        if (s_button_gpios[i] == GPIO_NUM_NC) continue;
        // ESP32-C6 valid digital IO range is 8..30; warn if outside
        if (s_button_gpios[i] < GPIO_NUM_8 || s_button_gpios[i] > GPIO_NUM_30) {
            ESP_LOGW(TAG, "CH%u: button GPIO %d may be invalid on ESP32-C6 (use 8..30)", i, (int)s_button_gpios[i]);
        }
        in_cfg.pin_bit_mask = (1ULL << s_button_gpios[i]);
        gpio_config(&in_cfg);
        gpio_set_pull_mode(s_button_gpios[i], GPIO_PULLUP_ONLY);
        int lvl = gpio_get_level(s_button_gpios[i]);
        ESP_LOGI(TAG, "CH%u: button GPIO %d configured with pull-up, initial level=%d", i, (int)s_button_gpios[i], lvl);
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
    uint8_t last_read[LIGHT_CHANNELS] = {1,1,1,1}; // idle high

    while (true) {
        for (int i = 0; i < LIGHT_CHANNELS; i++) {
            int level = gpio_get_level(s_button_gpios[i]);
            if (level == last_read[i]) {
                if (stable_cnt[i] < 255) stable_cnt[i]++;
            } else {
                stable_cnt[i] = 0;
                last_read[i] = level;
            }
            // On stable falling edge (press)
            if (last_read[i] == 0 && stable_cnt[i] == BUTTON_STABLE_CNT) {
                light_manager_button_press(i);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

esp_err_t light_manager_init()
{
    leds_init();
    buttons_init();
    // Startup logging: confirm logging is working and show configuration
    ESP_LOGI(TAG, "Light manager init: %d channel(s), poll=%dms, debounce=%d", (int)LIGHT_CHANNELS, (int)BUTTON_POLL_MS, (int)BUTTON_STABLE_CNT);
    for (int i = 0; i < LIGHT_CHANNELS; i++) {
        ESP_LOGI(TAG, "CH%u: button GPIO %d (active-low), LED GPIO %d", (unsigned)i, (int)s_button_gpios[i], (int)s_led_gpios[i]);
    }
    ESP_LOGI(TAG, "DHT22 GPIO %d", (int)DHT22_GPIO);
    // Register group request callback to actually send group commands when bindings trigger updates
    esp_matter::client::set_request_callback(nullptr,
        [](uint8_t fabric_index, esp_matter::client::request_handle *req, void *priv) {
            // For Toggle and other simple commands with no payload, just forward the request
            // Binding manager fills group id and fabric; this sends it over group session
            (void) esp_matter::client::group_request_send(fabric_index, req);
        },
        nullptr);
    // Start button polling
    xTaskCreate(button_task, "btn_poll", 2048, nullptr, 5, &s_button_task);
    ESP_LOGI(TAG, "Light manager started");
    return ESP_OK;
}

// --- Minimal DHT22 bit-bang reader ---
// Notes: For robustness, consider using an existing library. This simple implementation
// reads every ~10s and updates Matter Temperature and Relative Humidity measurement clusters.

static bool dht22_read(float *temperature_c, float *humidity)
{
    if (!temperature_c || !humidity) return false;

    gpio_set_direction(DHT22_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT22_GPIO, 0);
    esp_rom_delay_us(1000); // pull low for at least 1ms
    gpio_set_level(DHT22_GPIO, 1);
    esp_rom_delay_us(30);
    gpio_set_direction(DHT22_GPIO, GPIO_MODE_INPUT);

    // Measure pulses
    uint32_t timings[85];
    int idx = 0;
    uint32_t last = 0;
    uint32_t timeout = 10000; // 10ms safety

    // Wait for sensor response (low then high)
    while (gpio_get_level(DHT22_GPIO) == 1 && timeout--) esp_rom_delay_us(1);
    while (gpio_get_level(DHT22_GPIO) == 0 && idx < 85) { timings[idx++] = last; last = 0; while (gpio_get_level(DHT22_GPIO) == 0 && last < 1000) { last++; esp_rom_delay_us(1);} }
    last = 0;
    while (gpio_get_level(DHT22_GPIO) == 1 && idx < 85) { timings[idx++] = last; last = 0; while (gpio_get_level(DHT22_GPIO) == 1 && last < 1000) { last++; esp_rom_delay_us(1);} }

    // Read 40 bits: sequence of low-high pulses; length of high indicates 0/1
    uint8_t data[5] = {0};
    int bit = 0;
    // Skip initial preamble pairs; parse following 40 cycles
    for (int i = 2; i + 1 < idx && bit < 40; i += 2) {
        uint32_t high_len = timings[i + 1];
        // threshold ~40us vs ~70us; our unit ~1us loops
        data[bit / 8] <<= 1;
        if (high_len > 50) data[bit / 8] |= 1;
        bit++;
    }

    if (bit < 40) return false;
    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    if (sum != data[4]) return false;

    uint16_t raw_h = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_t = ((uint16_t)data[2] << 8) | data[3];

    *humidity = raw_h / 10.0f;
    if (raw_t & 0x8000) {
        raw_t &= 0x7FFF;
        *temperature_c = -(raw_t / 10.0f);
    } else {
        *temperature_c = raw_t / 10.0f;
    }
    return true;
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

static void dht22_task(void *arg)
{
    while (true) {
        float t = 0, h = 0;
        if (dht22_read(&t, &h)) {
            ESP_LOGI(TAG, "DHT22: %.1fC, %.1f%%", t, h);
            report_temperature(t);
            report_humidity(h);
        } else {
            ESP_LOGW(TAG, "DHT22 read failed");
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // ~10s
    }
}

void dht22_start_task()
{
    if (s_dht_task) return;
    xTaskCreate(dht22_task, "dht22", 3072, nullptr, 5, &s_dht_task);
}

// ---------- Group bindings controller behavior ----------

// Channel -> default Group ID mapping
static uint16_t group_id_for_channel(uint8_t ch)
{
    switch (ch) {
        case 0: return GROUP_ID_0;
        case 1: return GROUP_ID_1;
        case 2: return GROUP_ID_2;
        case 3: return GROUP_ID_3;
        default: return GROUP_ID_0;
    }
}

// Send On/Off Toggle to a group for this controller endpoint
static void send_group_toggle(uint8_t ch)
{
    if (ch >= LIGHT_CHANNELS) return;
    // Build a request handle; binding manager will route (group/unicast)
    esp_matter::client::request_handle req = {};
    req.type = esp_matter::client::INVOKE_CMD;
    // Populate command path for On/Off Toggle
    chip::app::CommandPathParams path(
        static_cast<chip::EndpointId>(g_onoff_endpoint_ids[ch]),
        static_cast<chip::GroupId>(0),
        chip::app::Clusters::OnOff::Id,
        chip::app::Clusters::OnOff::Commands::Toggle::Id,
        chip::app::CommandPathFlags::kEndpointIdValid);
    req.command_path = path;
    // No command payload for Toggle
    esp_err_t err = ESP_OK;
    // Notify binding manager for this endpoint; group callback will send to groups
    err = esp_matter::client::cluster_update(g_onoff_endpoint_ids[ch], &req);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cluster_update failed for ch%u (ep %u): %d", ch, g_onoff_endpoint_ids[ch], err);
    }
}

void light_manager_button_press(uint8_t channel)
{
    // Log button press
    ESP_LOGI(TAG, "Button %u pressed (GPIO %d)", channel, (int)s_button_gpios[channel]);
    // Brief LED blink to acknowledge press
    if (channel >= LIGHT_CHANNELS) return;
    apply_led(channel, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    apply_led(channel, s_led_any_on[channel]);
    // Send group Toggle
    send_group_toggle(channel);
}
