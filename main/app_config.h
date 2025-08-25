/*
 * Project configuration for the Smart Light Switch (up to 4 gangs) with DHT22 sensor.
 * Adjust GPIO pins to match your custom hardware.
 */

#pragma once

#include "driver/gpio.h"

// Number of light channels (1..4)
#ifndef LIGHT_CHANNELS
#define LIGHT_CHANNELS 4
#endif

// Button GPIOs (active-low, use internal pull-ups). Use valid ESP32-C6 pins.
#ifndef BUTTON_GPIO_0
#define BUTTON_GPIO_0 GPIO_NUM_16
#endif
#ifndef BUTTON_GPIO_1
#define BUTTON_GPIO_1 GPIO_NUM_17
#endif
#ifndef BUTTON_GPIO_2
#define BUTTON_GPIO_2 GPIO_NUM_18
#endif
#ifndef BUTTON_GPIO_3
#define BUTTON_GPIO_3 GPIO_NUM_19
#endif

// LED GPIOs for the button indicator LEDs (active-high outputs)
#ifndef LED_GPIO_0
#define LED_GPIO_0 GPIO_NUM_8
#endif
#ifndef LED_GPIO_1
#define LED_GPIO_1 GPIO_NUM_9
#endif
#ifndef LED_GPIO_2
#define LED_GPIO_2 GPIO_NUM_10
#endif
#ifndef LED_GPIO_3
#define LED_GPIO_3 GPIO_NUM_11
#endif

// DHT22 (AM2302) data pin
#ifndef DHT22_GPIO
#define DHT22_GPIO GPIO_NUM_12
#endif

// Default Group IDs (per channel) for group bindings
#ifndef GROUP_ID_0
#define GROUP_ID_0 0x0001
#endif
#ifndef GROUP_ID_1
#define GROUP_ID_1 0x0002
#endif
#ifndef GROUP_ID_2
#define GROUP_ID_2 0x0003
#endif
#ifndef GROUP_ID_3
#define GROUP_ID_3 0x0004
#endif

// Debounce parameters
#define BUTTON_POLL_MS    20
#define BUTTON_STABLE_CNT 3   // 3*20ms = ~60ms debounce

// Periodic LED state resync interval (ms). The initial implementation performed
// a single sync ~10s after boot; now we repeat every 10s until proper
// subscription-based tracking is implemented. Guarded for override.
#ifndef LED_PERIODIC_SYNC_MS
#define LED_PERIODIC_SYNC_MS 10000
#endif
