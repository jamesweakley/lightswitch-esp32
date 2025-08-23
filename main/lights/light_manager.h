/*
 * Light manager for 1-4 on/off channels and DHT22 sensor.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

#include <esp_err.h>
#include <esp_matter.h>

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Expose Matter endpoint IDs so we can update attributes from tasks.
extern uint16_t g_onoff_endpoint_ids[LIGHT_CHANNELS];
extern uint16_t g_temp_endpoint_id;
extern uint16_t g_humidity_endpoint_id;

// Forward declare shadow binding list accessor (defined in app_main.cpp)
struct ShadowBindingList; // opaque (we only read fields we know)
extern const struct ShadowBindingList * shadow_binding_get_list(int ch);

esp_err_t light_manager_init();
void light_manager_button_press(uint8_t channel);
bool light_manager_get(uint8_t channel);

// DHT22 task controls
void dht22_start_task();

#ifdef __cplusplus
}
#endif
