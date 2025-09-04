# Architecture

This document gives GitHub Copilot (and humans) a concise but complete map of the firmware.

## High-Level Overview

The device is a **Matter Light Switch Controller** implementing 4 logical switch endpoints (On/Off *client* cluster) and a Binding *server* cluster on each, plus Temperature & Humidity sensors (DHT22). It does NOT host On/Off *server* clusters; instead it issues Toggle commands to bound targets (group or unicast) via the esp-matter binding manager.

```
+-------------+        +-----------------+        +------------------+
| Buttons/LED |  -->   | light_manager   |  -->   | esp-matter stack |
+-------------+        +-----------------+        +------------------+
       ^                        |                          |
       |                        v                          v
  Physical input        cluster_update()            Network (Matter)
```

Tasks (FreeRTOS):
* `btn_poll` – polls GPIO, debounces
* `btn_act` – consumes queue events, schedules cluster updates
* `dht22` – stub for periodic sensor reads (10s cadence)

Timers: per-channel one-shot LED blink timers; an init watchdog (30s) during Matter start.

## Endpoints & Clusters

| Endpoint | Purpose                | Device Type | Clusters (dir)                              |
|----------|------------------------|-------------|----------------------------------------------|
| 0        | Root / Node            | Root Node   | Standard mandatory                           |
| 1..4     | Switch channels 0..3   | 0x0103      | On/Off (client), Binding (server)            |
| 5        | Temperature Sensor     | 0x0302      | Temperature Measurement (server)             |
| 6        | Humidity Sensor        | 0x0307      | Relative Humidity Measurement (server)       |

IDs stored in globals declared in `light_manager.h` / defined in `app_main.cpp`.

## Data Flow (Button Press)
1. `btn_poll` detects stable low (active‑low press) -> queue channel index.
2. `btn_act` dequeues -> `light_manager_button_press(ch)`.
3. LED blink is scheduled (non-blocking) & `schedule_group_toggle()` called.
4. A work item enqueued on CHIP Platform thread -> `send_group_toggle()` builds a client request handle (Toggle command) and calls `esp_matter::client::cluster_update()`.
5. Binding manager inspects Binding attribute for source endpoint; routes as unicast(s) and/or group(s).

## Shadow Binding Mechanism
File: `app_main.cpp` holds an internal shadow list per channel (struct `ShadowBindingList`). Console commands (`bind-add`, etc.) allow appending unicast entries without fully parsing/modifying the Binding attribute TLV (current esp-matter public API limitations). Shadow entries persist in NVS (`namespace: bindcfg`). On boot they are reloaded and a placeholder commit logs intent (future hook: actually rewrite Binding attribute list when API is exposed).

## GPIO & Configuration
Defined in `app_config.h` with macro overrides for:
* Buttons: `BUTTON_GPIO_[0-3]`
* LEDs: `LED_GPIO_[0-3]`
* DHT22 data: `DHT22_GPIO`
* Default group IDs: `GROUP_ID_[0-3]`

Override via CMake cache defines: `idf.py build -DGROUP_ID_0=0x0100`.

## DHT22 Implementation Status
Implemented minimal bit‑banged driver (timing‑sensitive) with 10s cadence. Values are read in 0.1 units and scaled to 0.01 for Matter `MeasuredValue` attributes on Temperature (0x0402) and Relative Humidity (0x0405) clusters. Failures are logged (checksum / timeout) and transient; a streak counter emits warnings at 3 and every 10 thereafter. Replace with a hardware‑timer or RMT based implementation for higher robustness if needed.

## Power & Watchdog
* Optional PM lock prevents light sleep (JTAG stability).
* 30s init watchdog restarts device if Matter stack fails to start (see `init_watchdog_timer`).

## Planned Extension Points
* Replace shadow binding commit placeholder with real TLV list writer once available.
* Add remote state aggregation: subscribe to On/Off server attributes and reflect "any on" with steady LED.
* Implement robust DHT22 or alternative sensor driver.
* Persist dynamic group/unicast bindings through full attribute parse & update.

## Error Handling Notes
* Fails early (ABORT_APP_ON_FAILURE macro) when critical allocations/endpoint creation fail.
* On NVS truncated errors: erases and re-inits automatically.
* BLE bonding corruption mitigated by targeted namespace erasure on boot (temporary workaround).

## Security / OTA
Conditional encrypted OTA decryption key support with `CONFIG_ENABLE_ENCRYPTED_OTA`; key is compiled in via `target_add_binary_data` (example only; replace for production).

## Rationale for Design Choices
* Polling buttons avoids ISR stack complexity & simplifies debouncing.
* Separate action task prevents heavy cluster interaction on small polling stack.
* Shadow binding avoids partial list overwrite risk until full list parsing APIs are stable.

## Glossary
* Binding attribute: List (cluster 0xF000 attr 0x0000) describing routing targets.
* cluster_update(): esp-matter client API to send commands/reads via binding manager.
* Group multicast: Efficient way to address multiple lights with one message (default groups).

-- End of architecture --
