# 4‑Gang Matter Light Switch Controller (ESP32‑C6)

This firmware turns an ESP32‑C6 into a 4‑channel ("gang") Matter light switch controller with:

* 4 momentary buttons (active‑low, internal pull‑ups enabled)
* 4 indicator LEDs (active‑high)
* DHT22 (Temperature + Humidity) sensor endpoints
* On/Off Light Switch device type (0x0103) per channel with an On/Off **client** cluster (0x0006) + Binding **server** cluster (0xF000)
* Default Group IDs for quick group control (0x0001..0x0004) – can be overridden by Binding writes

It issues Toggle commands (On/Off cluster, command 0x02) via the esp-matter client & binding manager. Indicator LEDs currently blink on press (remote state feedback can be added in future).

## Hardware Pin Mapping

| Channel | Button GPIO | LED GPIO | Default Group ID | Notes |
|---------|-------------|----------|------------------|-------|
| 0       | 16          | 8        | 0x0001           | First physical switch / LED |
| 1       | 17          | 9        | 0x0002           | Second |
| 2       | 18          | 10       | 0x0003           | Third |
| 3       | 19          | 11       | 0x0004           | Fourth |
| DHT22   | 12          | —        | —                | Temperature & Humidity sensor data pin |

All button inputs are active‑low (pressed = 0). LEDs are driven high to turn on.

Constants are defined in `app_config.h` and can be overridden at compile time (e.g. via `-DLED_GPIO_0=GPIO_NUM_X`).

## Endpoint Layout

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

## Building & Flashing

Standard ESP-IDF + esp-matter flow, eg:

```bash
idf.py set-target esp32c6
idf.py build flash monitor
```

Make sure you have Thread / Wi‑Fi credentials set per your transport requirements before commissioning.

## Commissioning

Use a Matter commissioner (e.g. `chip-tool` or a mobile app) to commission the controller. The setup payload / QR code output appears in the serial log on first boot or after factory reset.

## Binding & Group Control Instructions

Each switch endpoint (1–4) sends an OnOff Toggle to its bound targets. You can bind either:

1. A **Group** (group multicast) – simplest for multiple lights.
2. Specific **unicast targets** (Node ID + Endpoint + Cluster).

The Binding cluster (ID 0xF000) server lives on each switch endpoint (1–4). Its attribute `Binding` (ID 0x0000) is a list of binding entries. An entry can contain EITHER `{ group }` OR `{ node, endpoint, cluster }`.

### Option A: Group Binding (Recommended for many lights)

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

### Option B: Unicast Binding (Single specific light)

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

### Verifying Bindings

Read back the Binding list:

```bash
chip-tool binding read binding <controller-node-id> <switch-endpoint>
```

You should see the JSON list matching what you wrote.

### Clearing / Replacing Bindings

Write an empty list (`[]`) to clear, or just write a new list to replace.

```bash
chip-tool binding write binding '[]' <controller-node-id> 1
```

## Button / LED Behavior

* Short press: sends Toggle to all bound entries (group or unicast).
* LED blinks briefly on press (steady state tracking of remote lights currently disabled / TODO).

## DHT22 Sensor Endpoints

Temperature (endpoint 5) and Humidity (endpoint 6) periodically report measured values (implementation stub may need enhancement for actual sensor reads if disabled in code). Cluster IDs:

* Temperature Measurement (0x0402) – attribute MeasuredValue (0x0000) in 0.01 °C units.
* Relative Humidity Measurement (0x0405) – attribute MeasuredValue (0x0000) in 0.01 %RH.

## Factory Reset & Recommissioning

Use a standard Matter factory reset mechanism (e.g. long button hold if implemented, or `chip-tool` command). After clearing fabrics, the device reopens commissioning.

## Future Enhancements (Planned / Optional)

* Remote On/Off state aggregation to illuminate LEDs if **any** bound targets are ON.
* Persistent user-configurable group IDs via NVS.
* OTA / delta update integration (infrastructure already present in esp-matter dependencies).

## References

* esp-matter Docs: https://docs.espressif.com/projects/esp-matter/en/latest/esp32/
* Matter Specification (CSA) – Clusters: On/Off (0x0006), Binding (0xF000), Temperature Measurement (0x0402), Relative Humidity (0x0405)

---
If you encounter issues with binding writes, ensure both controller and target devices share the same fabric and that target endpoints have the On/Off **server** cluster enabled.
