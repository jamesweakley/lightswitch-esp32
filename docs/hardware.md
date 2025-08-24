# Hardware Guide

## Summary
The firmware targets ESP32-C6 with 4 momentary buttons + 4 indicator LEDs + optional DHT22 sensor.

## Default GPIO Mapping
| Channel | Button (active-low) | LED (active-high) | Default Group ID |
|---------|---------------------|-------------------|------------------|
| 0       | 16                  | 8                 | 0x0001           |
| 1       | 17                  | 9                 | 0x0002           |
| 2       | 18                  | 10                | 0x0003           |
| 3       | 19                  | 11                | 0x0004           |
| DHT22   | 12 (data)           | —                 | —                |

## Electrical Notes
* Buttons use internal pull-ups; ground the GPIO when pressed.
* LEDs assumed to sink current to ground (drive GPIO HIGH to illuminate). Use appropriate resistors.
* DHT22 requires a 4.7k–10k pull-up resistor on data line to 3V3.

## Overriding Pins
Override at build time using defines:
```bash
idf.py build -DBUTTON_GPIO_0=GPIO_NUM_4 -DLED_GPIO_0=GPIO_NUM_6
```
Multiple overrides can be combined. All macros in `app_config.h` are guarded with `#ifndef`.

## Mechanical Considerations
* Debounce handled in software (~60 ms). Avoid additional RC networks that could elongate press recognition.
* Keep DHT22 away from MCU heat sources; use a vented enclosure.

## Expansion Ideas
* Replace DHT22 with I2C sensor (e.g., SHTC3). Add I2C init + endpoint update logic.
* Add long‑press detection (extend button state machine in `button_task`).

-- End of hardware guide --
