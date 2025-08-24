# GitHub Copilot Instructions

These guidelines help AI assistants produce consistent, safe, and maintainable contributions.

## Project Identity
* Name: `light_switch` (Matter Light Switch Controller)
* Core purpose: Send On/Off Toggle commands to bound lights (group/unicast) on physical button press.
* Controller only: No On/Off server endpoints.

## Key Files
* `main/app_main.cpp` – endpoint & cluster setup, Matter start, shadow bindings, console commands
* `main/lights/light_manager.cpp` – GPIO polling, LED control, command dispatch
* `main/app_config.h` – configurable macros (override friendly)
* `docs/architecture.md` – design constraints & flow

## Coding Conventions
* C++17 (already enforced via CMake). Keep additions compatible with ESP-IDF toolchain.
* Use `ABORT_APP_ON_FAILURE` for unrecoverable init failures.
* Guard new config macros with `#ifndef`.
* Prefer non-blocking timers / scheduled work over delays inside tasks responding to hardware events.
* Keep new FreeRTOS tasks under control: state a purpose, stack size, priority rationale in a comment.

## Binding / Command Dispatch
* Use existing pattern: build `esp_matter::client::request_handle req` and call `esp_matter::client::cluster_update(local_endpoint, &req)`.
* Do not hardcode remote Node IDs in code; use Binding attribute (or shadow list helper until full TLV writer is implemented).

## Adding a Sensor Endpoint
1. Create endpoint (matching esp-matter device type helper or generic + cluster).
2. Store endpoint ID in a new global (declare in `light_manager.h`, define in `app_main.cpp`).
3. Provide a report helper similar to `report_temperature()`.
4. Update `docs/architecture.md` table.

## Shadow Binding Extension
When esp-matter exposes list write helpers:
* Implement real TLV assembly in `shadow_binding_commit()`.
* Preserve NVS serialization format for backward compatibility or bump version key.

## Remote State Tracking (Future)
* Subscribe to On/Off server `OnOff` attribute of all bound targets.
* Maintain a per-channel active count (`s_on_count[]`).
* Drive LED steady ON if any bound target is ON; continue transient blink on button press.

## Error Handling
* Return `ESP_OK` for ignored attribute callbacks to avoid failing upstream logic.
* Log at INFO for state transitions; DEBUG for verbose protocol or timing data.
* Watchdog modifications: keep timeout macros configurable; avoid disabling without alternate fail-safe.

## Performance Constraints
* Button latency target: <100 ms from physical press to Toggle dispatch (current path ~ <10 ms typical). Avoid heavy work in `button_action_task`.
* Keep heap allocations during steady-state minimal; prefer static storage.

## Testing Suggestions (Manual)
* Commission -> Bind -> Press each button; verify Toggle in logs of target device.
* Simulate invalid GPIO by setting to `GPIO_NUM_NC`; ensure init skips gracefully.

## Security / Safety
* Do not embed production keys (OTA / certificates) directly in commits.
* Avoid logging sensitive fabric credentials; redaction preferred if modifying logs.

## Documentation Updates
For each non-trivial feature:
* Update architecture & development docs.
* Add usage notes to README if user-facing.

## PR Checklist (AI Self-Check)
- [ ] Build succeeds (`idf.py build`)
- [ ] No increase in warning count (unless justified)
- [ ] Docs updated
- [ ] New macros guarded
- [ ] No blocking delays in critical paths
- [ ] User-visible behavior described in commit message

-- End of Copilot instructions --
