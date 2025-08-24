# Development Guide

## Prerequisites
* ESP-IDF (v5.4 or matching required by your ESP-Matter snapshot)
* ESP-Matter repo checked out & `ESP_MATTER_PATH` exported
* Toolchain for ESP32-C6

Example environment (zsh):
```bash
export IDF_PATH=$HOME/esp/v5.4/esp-idf
source $IDF_PATH/export.sh
export ESP_MATTER_PATH=$HOME/esp/esp-matter
```

## Build & Flash
```bash
idf.py set-target esp32c6
idf.py build flash monitor
```
Common extras:
```bash
idf.py fullclean
idf.py -DLED_GPIO_0=GPIO_NUM_5 build
idf.py size
```

## Directory Structure (Key)
* `main/app_main.cpp` – endpoint creation, Matter start, shadow bindings, console commands
* `main/lights/light_manager.*` – GPIO, tasks, button handling, Toggle command scheduling
* `main/app_config.h` – macro configuration
* `docs/` – documentation consumed by GitHub Copilot
* `patches/` – (if any) local overrides

## Adding Features
1. Define new hardware pins or feature macros in `app_config.h` (preserve defaults in `#ifndef`).
2. Add endpoint(s) in `app_main.cpp` before `esp_matter::start()`; store IDs in globals.
3. For new periodic tasks, follow pattern of `dht22_start_task()` with separate stack size.
4. If sending commands, prefer existing binding manager via `esp_matter::client::cluster_update()`.

## Console Commands
Provided when `CONFIG_ENABLE_CHIP_SHELL` is enabled:
* `bind-add <ch> <node-id-hex> <endpoint> [cluster]`
* `bind-list [ch]`
* `bind-remove <ch> <index>`
* `bind-clear <ch>`
* `bind-commit <ch>`

These operate on shadow binding lists (NVS persisted) and log placeholder commit actions.

## Commissioning
1. Flash firmware; obtain QR / setup payload from log.
2. Use `chip-tool pairing onnetwork` or a mobile commissioner.
3. After success, write Binding entries (group or unicast). Examples in root README.

Re-open commissioning window after fabric removal handled automatically (see event callback in `app_main.cpp`).

## Troubleshooting
* Stuck initialization -> watchdog restarts (check logs around `Matter initialization appears stuck`).
* Binding writes not taking effect: ensure both devices on same fabric and target endpoint hosts On/Off **server** 0x0006.
* BLE IRK restore errors: boot code clears problematic NVS namespaces; confirm logs show clearance.
* DHT22 values absent: stub function returns false; implement real driver.

## Logging Verbosity
Adjust via `esp_log_level_set()` early in `app_main()`. Common tags: `app_main`, `light_manager`, `BindingManager`, `IM`.

## Adding Remote State Tracking (TODO Outline)
1. Enumerate Binding list targets (needs esp-matter public API or CHIP direct access).
2. Subscribe or periodically read On/Off server attributes.
3. Maintain per-channel active target count; drive steady LED on any ON.

## Code Style / Conventions
* Keep new macros guarded with `#ifndef` to preserve override capability.
* Avoid blocking delays in button handling paths; use timers or scheduled work.
* Keep tasks small & single-purpose.

## Security Notes
* Replace example OTA decryption key with secure provisioning method in production.
* Audit NVS namespaces if storing credentials or secrets (currently only binding shadow + BLE cleanups).

## Updating esp-matter
When updating ESP-Matter / CHIP revisions, re-test:
* Endpoint creation sequence
* Binding manager request path assumptions (wildcard endpoint usage)
* Console command registration API

## License
Public Domain / CC0 (example code) – verify before commercial deployment if adding third‑party libs.

-- End of development guide --
