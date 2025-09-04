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
#ifndef LED_GPIO_0
#define LED_GPIO_0 GPIO_NUM_0
#endif
#ifndef BUTTON_GPIO_0
#define BUTTON_GPIO_0 GPIO_NUM_1
#endif

#ifndef LED_GPIO_1
#define LED_GPIO_1 GPIO_NUM_2
#endif
#ifndef BUTTON_GPIO_1
#define BUTTON_GPIO_1 GPIO_NUM_21
#endif


#ifndef LED_GPIO_2
#define LED_GPIO_2 GPIO_NUM_18
#endif
#ifndef BUTTON_GPIO_2
#define BUTTON_GPIO_2 GPIO_NUM_20
#endif


#ifndef LED_GPIO_3
#define LED_GPIO_3 GPIO_NUM_19
#endif
#ifndef BUTTON_GPIO_3
#define BUTTON_GPIO_3 GPIO_NUM_17
#endif

// DHT22 (AM2302) data pin
#ifndef DHT22_GPIO
#define DHT22_GPIO GPIO_NUM_16
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

// Optional tolerance (in 0.01 units) reported via the *Measurement clusters.
// Typical DHT22 accuracy: +/-0.5°C and +/-2% RH.
#ifndef DHT22_TEMP_TOLERANCE_0_01
#define DHT22_TEMP_TOLERANCE_0_01 50   // 0.50°C
#endif
#ifndef DHT22_HUM_TOLERANCE_0_01
#define DHT22_HUM_TOLERANCE_0_01 200   // 2.00%RH
#endif

// Threshold (microseconds) distinguishing bit '1' from '0' high pulse
#ifndef DHT22_BIT_THRESHOLD_US
#define DHT22_BIT_THRESHOLD_US 40 // closer to midpoint between ~26us (0) and ~70us (1)
#endif
// Start signal timing (host pull low then release)
#ifndef DHT22_START_LOW_MS
#define DHT22_START_LOW_MS 2
#endif
#ifndef DHT22_START_RELEASE_US
#define DHT22_START_RELEASE_US 30
#endif
// Handshake/bit timing tolerances (override via build flags if needed)
#ifndef DHT22_RESP_LOW_TIMEOUT_US
#define DHT22_RESP_LOW_TIMEOUT_US 2000  // wait for initial sensor 80us low
#endif
#ifndef DHT22_RESP_HIGH_TIMEOUT_US
#define DHT22_RESP_HIGH_TIMEOUT_US 2000 // wait for following 80us high
#endif
#ifndef DHT22_FIRST_BIT_PREP_TIMEOUT_US
#define DHT22_FIRST_BIT_PREP_TIMEOUT_US 1500 // wait for first 50us low preceding bit stream
#endif
#ifndef DHT22_BIT_HIGH_TIMEOUT_US
#define DHT22_BIT_HIGH_TIMEOUT_US 200   // widened to allow jitter
#endif
#ifndef DHT22_BIT_LOW_TIMEOUT_US
#define DHT22_BIT_LOW_TIMEOUT_US 160    // widened
#endif
// Enable detailed debug logging of each DHT22 read phase (set to 1 to enable)
#ifndef DHT22_DEBUG
#define DHT22_DEBUG 0
#endif
// Optionally disable interrupts during the 40-bit capture to reduce jitter (0 = off)
#ifndef DHT22_DISABLE_INTERRUPTS
#define DHT22_DISABLE_INTERRUPTS 0
#endif

// Number of initial successful frames to discard (sensor warmup / stabilization)
#ifndef DHT22_WARMUP_READS
#define DHT22_WARMUP_READS 2
#endif
// Treat a frame with all data bytes zero (and valid checksum) as invalid/noise
#ifndef DHT22_DISCARD_ZERO_FRAME
#define DHT22_DISCARD_ZERO_FRAME 1
#endif

// RMT configuration (legacy driver) for DHT22 capture
#ifndef DHT22_USE_RMT
#define DHT22_USE_RMT 1
#endif
#if DHT22_USE_RMT
#include <driver/rmt_rx.h>
#ifndef DHT22_RMT_SYMBOL_CAPACITY
#define DHT22_RMT_SYMBOL_CAPACITY 128
#endif
#ifndef DHT22_RMT_RESOLUTION_HZ
#define DHT22_RMT_RESOLUTION_HZ 1000000 // 1us
#endif
#ifndef DHT22_RMT_IDLE_TIMEOUT_US
#define DHT22_RMT_IDLE_TIMEOUT_US 1500
#endif
#ifndef DHT22_RMT_GPIO_PULLUP
#define DHT22_RMT_GPIO_PULLUP 1
#endif
#endif // DHT22_USE_RMT

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
