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

// we will pair buttons with LEDs to make the plugs simpler, and they can share a GND
// Each block will be:
//  - GND
//  - Button GPIO (active-low input with pull-up)
//  - LED GPIO (active-high output)


// Button GPIOs (active-low, use internal pull-ups). Use valid ESP32-C6 pins.
// and
// LED GPIOs for the button indicator LEDs (active-high outputs)
#ifndef BUTTON_GPIO_0
#define BUTTON_GPIO_0 GPIO_NUM_2
#endif
#ifndef LED_GPIO_0
#define LED_GPIO_0 GPIO_NUM_3
#endif

#ifndef BUTTON_GPIO_1
#define BUTTON_GPIO_1 GPIO_NUM_23
#endif
#ifndef LED_GPIO_1
#define LED_GPIO_1 GPIO_NUM_22
#endif


#ifndef BUTTON_GPIO_2
#define BUTTON_GPIO_2 GPIO_NUM_4
#endif
#ifndef LED_GPIO_2
#define LED_GPIO_2 GPIO_NUM_5
#endif


#ifndef BUTTON_GPIO_3
#define BUTTON_GPIO_3 GPIO_NUM_14
#endif
#ifndef LED_GPIO_3
#define LED_GPIO_3 GPIO_NUM_15
#endif

// DHT22 (AM2302) data pin
#ifndef DHT22_GPIO
#define DHT22_GPIO GPIO_NUM_21
#endif

// Enable (1) or disable (0) the DHT22 task/driver at build time (override via -DDHT22_ENABLE=0)
#ifndef DHT22_ENABLE
#define DHT22_ENABLE 1
#endif

// DHT22 robustness tuning (override as needed)
#ifndef DHT22_PERIOD_MS
#define DHT22_PERIOD_MS 10000
#endif
#ifndef DHT22_MAX_RETRIES
#define DHT22_MAX_RETRIES 3   // attempts per period until success
#endif
#ifndef DHT22_READ_TIMEOUT_US
#define DHT22_READ_TIMEOUT_US 6000 // abort a single frame read if >6ms elapsed
#endif
#ifndef DHT22_STABILIZE_DELAY_MS
#define DHT22_STABILIZE_DELAY_MS 3000 // initial delay before first read
#endif
#ifndef DHT22_TEMP_MIN_X10
#define DHT22_TEMP_MIN_X10 -400 // -40.0 C
#endif
#ifndef DHT22_TEMP_MAX_X10
#define DHT22_TEMP_MAX_X10 800  // 80.0 C
#endif
#ifndef DHT22_HUM_MIN_X10
#define DHT22_HUM_MIN_X10 0    // 0 %
#endif
#ifndef DHT22_HUM_MAX_X10
#define DHT22_HUM_MAX_X10 1000 // 100.0 %
#endif

// Threshold (microseconds) distinguishing bit '1' from '0' high pulse
#ifndef DHT22_BIT_THRESHOLD_US
#define DHT22_BIT_THRESHOLD_US 50
#endif
// Enable detailed debug logging of each DHT22 read phase (set to 1 to enable)
#ifndef DHT22_DEBUG
#define DHT22_DEBUG 0
#endif
// Optionally disable interrupts during the 40-bit capture to reduce jitter (0 = off)
#ifndef DHT22_DISABLE_INTERRUPTS
#define DHT22_DISABLE_INTERRUPTS 0
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
