# 4‑Gang Matter Light Switch Controller (ESP32‑C6)

Production‑oriented firmware turning an ESP32‑C6 into a 4‑channel ("gang") **Matter Light Switch Controller** with optional temperature + humidity sensing. This README is intentionally concise; deeper details live in `docs/` for both humans and GitHub Copilot. See:

* `docs/architecture.md` – high‑level design, data flow, tasks, endpoints
* `docs/development.md` – build, flash, debug, customization, common tasks
* `docs/hardware.md` – wiring, GPIO mapping, overrides
* `.github/copilot-instructions.md` – guidance for AI assistants (naming, extension points)

## Features

* 4 momentary buttons (active‑low, internal pull‑ups)
* 4 indicator LEDs (active‑high)
* DHT22 (Temperature + Humidity) sensor endpoints
* Per channel On/Off Light Switch device type (0x0103) with On/Off **client** cluster (0x0006) & Binding **server** cluster (0xF000)
* Default group IDs (0x0001..0x0004) for immediate group multicast control
* Shadow binding helper console commands (`bind-add`, `bind-list`, etc.) with NVS persistence stub
* Watchdog for stuck Matter init + optional power‑management lock while debugging

LEDs blink on press (remote state aggregation currently disabled; planned improvement).

## Hardware Pin Mapping (Summary)

| Channel | Button GPIO | LED GPIO | Default Group ID | Notes |
|---------|-------------|----------|------------------|-------|
| 0       | 16          | 8        | 0x0001           | First physical switch / LED |
| 1       | 17          | 9        | 0x0002           | Second |
| 2       | 18          | 10       | 0x0003           | Third |
| 3       | 19          | 11       | 0x0004           | Fourth |
| DHT22   | 12          | —        | —                | Temperature & Humidity sensor data pin |

All button inputs are active‑low (pressed = 0). LEDs are driven high to turn on.

Constants live in `app_config.h` and can be overridden at build time, e.g.:
```
idf.py build -DLED_GPIO_0=GPIO_NUM_5
```
More in `docs/hardware.md`.

## Endpoint Layout (Runtime)

After commissioning, the node exposes endpoints in this order:

| Endpoint | Purpose | Device Type | Clusters (Direction) |
|----------|---------|-------------|-----------------------|
| 0        | Root / Bridged Root | Root Node (0x0016) | Basic Information (srv), Access Control (srv), Descriptor (srv), Commissioning clusters, etc. |
| 1        | Switch Channel 0 | On/Off Light Switch (0x0103) | On/Off (client), Binding (server) |
| 2        | Switch Channel 1 | On/Off Light Switch (0x0103) | On/Off (client), Binding (server) |
| 3        | Switch Channel 2 | On/Off Light Switch (0x0103) | On/Off (client), Binding (server) |
| 4        | Switch Channel 3 | On/Off Light Switch (0x0103) | On/Off (client), Binding (server) |
| 5        | Temperature Sensor | Temperature Sensor (0x0302) | Temperature Measurement (server) |
| 6        | Humidity Sensor | Humidity Sensor (0x0307) | Relative Humidity Measurement (server) |

(`g_onoff_endpoint_ids[]` holds endpoint IDs 1–4 at runtime; sensor endpoint IDs are stored in `g_temp_endpoint_id` & `g_humidity_endpoint_id`.)

## Quick Build & Flash

```bash
idf.py set-target esp32c6
idf.py build flash monitor
```

Ensure `IDF_PATH` and `ESP_MATTER_PATH` are exported. Extended workflow (multi‑target, size, release profile, cleaning) lives in `docs/development.md`.

## Commissioning

Use a Matter commissioner (`chip-tool` or mobile). Setup payload / QR appears on first boot or factory reset. Extra notes: `docs/development.md#commissioning`.

## Binding & Group Control

Each switch endpoint (1–4) sends an OnOff Toggle to its bound targets. You can bind either:

1. A **Group** (group multicast) – simplest for multiple lights.
2. Specific **unicast targets** (Node ID + Endpoint + Cluster).

The Binding cluster (ID 0xF000) server lives on each switch endpoint (1–4). Its attribute `Binding` (ID 0x0000) is a list of binding entries. An entry can contain EITHER `{ group }` OR `{ node, endpoint, cluster }`.

### Option A: Group Binding (Recommended)

1. On each target light device, add it to the appropriate group (example for Group 0x0001 on endpoint 1 of target node 0x123456789ABCDEF):
   ```bash
   chip-tool groups add-group 0x0001 0x0001 0x123456789ABCDEF 1
   ```
   (Syntax may vary by chip-tool version: `chip-tool groups add-group <group-id> <group-name-hex> <target-node-id> <target-endpoint>`.)
2. On the controller, write a binding list containing the group to the matching switch endpoint:
   * Channel 0 (endpoint 1):
     ```bash
     chip-tool binding write binding '[{"group":0x0001}]' <controller-node-id> 1
     ```
   * Channel 1 (endpoint 2):
     ```bash
     chip-tool binding write binding '[{"group":0x0002}]' <controller-node-id> 2
     ```
   * Channel 2 (endpoint 3):
     ```bash
     chip-tool binding write binding '[{"group":0x0003}]' <controller-node-id> 3
     ```
   * Channel 3 (endpoint 4):
     ```bash
     chip-tool binding write binding '[{"group":0x0004}]' <controller-node-id> 4
     ```

3. Press the physical button – the firmware will issue an OnOff Toggle to the group via the binding manager.

### Option B: Unicast Binding (Single Light)

For a single target light with Node ID `<light-node-id>`, endpoint `<light-ep>` (which must host On/Off server cluster 0x0006):

```bash
chip-tool binding write binding '[{"node":<light-node-id>,"endpoint":<light-ep>,"cluster":0x0006}]' <controller-node-id> <switch-endpoint>
```

Examples:

* Bind Channel 0 (endpoint 1) to a single bulb:
  ```bash
  chip-tool binding write binding '[{"node":0x123456789ABCDEF,"endpoint":1,"cluster":0x0006}]' <controller-node-id> 1
  ```
* Bind Channel 2 (endpoint 3) to two bulbs (provide a **list** with two entries):
  ```bash
  chip-tool binding write binding '[{"node":0x111111111111111,"endpoint":1,"cluster":0x0006},{"node":0x222222222222222,"endpoint":1,"cluster":0x0006}]' <controller-node-id> 3
  ```

### Mixed Bindings

You can mix group and unicast entries in the same list; each press iterates all entries. Example (Channel 1 / endpoint 2):

```bash
chip-tool binding write binding '[{"group":0x0002},{"node":0x123456789ABCDEF,"endpoint":1,"cluster":0x0006}]' <controller-node-id> 2
```

### Verify Bindings

Read back the Binding list:

```bash
chip-tool binding read binding <controller-node-id> <switch-endpoint>
```

You should see the JSON list matching what you wrote.

### Shadow Binding Helper (Local Incremental Adds)

This firmware also exposes a local "shadow" binding list per channel that is persisted in NVS and can be modified incrementally via the serial console. This shadow list is intended for workflows where you want to accumulate unicast targets one at a time without rewriting the full Binding attribute from a commissioner.

Important limitations (Option A design choice):
* Shadow list DOES NOT currently rewrite the official Binding attribute (it is only logged + persisted locally).
* Commissioners will not see shadow-added entries when reading the Binding list; they only see what was written through standard Matter writes.
* Future work could promote the shadow content into the real Binding attribute once a stable public API for list writes is available.

Console commands (enable CHIP shell in sdkconfig):

| Command | Description |
|---------|-------------|
| `bind-add <ch> <node-id-hex> <endpoint> [cluster-hex]` | Append a unicast target to channel shadow list (default cluster 0x0006). Saves to NVS. |
| `bind-list [ch]` | List shadow bindings for all channels or a specific channel. |
| `bind-remove <ch> <index>` | Remove entry at index from channel's shadow list (then persists). |
| `bind-clear <ch>` | Clear channel's shadow list (then persists). |
| `bind-commit <ch>` | Re-persist & log channel list (placeholder for future real Binding write). |

Example: add two bulbs (Node IDs 0x111... & 0x222...) to channel 0 incrementally:

```
bind-add 0 1111111111111111 1
bind-add 0 2222222222222222 1
bind-list 0
```

After reboot the shadow list is auto-loaded and logged; use `bind-list` to inspect.

To actually create operative bindings today, still perform standard Binding cluster writes (previous sections). The shadow facility is a staging / persistence aid only in this Option A build.

### Clear / Replace Bindings

Write an empty list (`[]`) to clear, or just write a new list to replace.

```bash
chip-tool binding write binding '[]' <controller-node-id> 1
```

## Button / LED Behavior

* Short press: sends Toggle to all bound entries (group or unicast).
* LED blinks briefly on press (steady state tracking of remote lights currently disabled / TODO).

## DHT22 Sensor Endpoints

Temperature (endpoint 5) and Humidity (endpoint 6) periodically report measured values every ~10s via a minimal bit‑banged driver. On checksum / timing failures the read is skipped; warnings appear after 3 consecutive failures. Replace with an RMT or dedicated driver for higher robustness if you experience noise. Cluster IDs:

* Temperature Measurement (0x0402) – attribute MeasuredValue (0x0000) in 0.01 °C units.
* Relative Humidity Measurement (0x0405) – attribute MeasuredValue (0x0000) in 0.01 %RH.

## Factory Reset & Recommissioning

Use a standard Matter factory reset mechanism (e.g. long button hold if implemented, or `chip-tool` command). After clearing fabrics, the device reopens commissioning.

## Planned Enhancements

* Remote On/Off state aggregation to illuminate LEDs if **any** bound targets are ON.
* Persistent user-configurable group IDs via NVS.
* OTA / delta update integration (infrastructure already present in esp-matter dependencies).
* Promotion of shadow binding list into official Binding attribute (replacing placeholder) when public API allows safe structured list writes.

## References

* esp-matter Docs: https://docs.espressif.com/projects/esp-matter/en/latest/esp32/
* Matter Specification (CSA) – Clusters: On/Off (0x0006), Binding (0xF000), Temperature Measurement (0x0402), Relative Humidity (0x0405)

---
Troubleshooting tips & verbose provisioning snippets moved to `docs/development.md#troubleshooting`.