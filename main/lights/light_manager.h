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

// Shadow binding structures shared by app_main and light_manager.
// Central definition here to avoid duplicate typedef redefinition errors.
typedef struct {
	bool is_group;
	uint64_t node_id;
	uint16_t endpoint;
	uint32_t cluster_id;
	uint16_t group_id;
	uint8_t fabric_index; // fabric that the binding entry belongs to (for proper CASE lookup)
} ShadowBindingEntry;
#ifndef MAX_SHADOW_BINDINGS_PER_CH
#define MAX_SHADOW_BINDINGS_PER_CH 10
#endif
typedef struct {
	int count;
	ShadowBindingEntry entries[MAX_SHADOW_BINDINGS_PER_CH];
} ShadowBindingList;
// Accessor (implemented in app_main.cpp)
extern const ShadowBindingList * shadow_binding_get_list(int ch);

esp_err_t light_manager_init();
void light_manager_button_press(uint8_t channel);
bool light_manager_get(uint8_t channel);

// Boot-time sync: query bound targets' OnOff attribute and set initial LED state.
// Safe to call after Matter stack started and shadow bindings committed.
void light_manager_sync_initial_state();

// DHT22 task controls
void dht22_start_task();

#ifdef __cplusplus
}
#endif
