# Arduino IDE build verification

Verified 27 October 2026 with Arduino CLI and Espressif Arduino ESP32 core 3.3.5.

- Board: XIAO_ESP32C3. USB CDC On Boot: Enabled.
- FQBN: `esp32:esp32:XIAO_ESP32C3:CDCOnBoot=default`.
- Application tabs: `Robot_ArduinoIDE.ino` and `Balance.ino`.
- Compile/link: PASS. Flash: **1,084,714 / 1,310,720 bytes (82%)**.
- Global RAM: **38,476 / 327,680 bytes (11%)**.
- Bundled third-party libraries compile from `src/lib`.
- Nothing uploaded or hardware-tested.

## Distance reference calibration verification

Tests execute the actual calibration, sampling, movement-decision and cancellation functions with mocked Echo timing, motor output and Preferences storage. Verified: capture accepts only distance mode; motors stop; cached/no-Echo readings never count; five fresh samples use a median; raw 32 cm maps to corrected 20 cm; the saved offset reloads after a simulated restart; corrected readings feed the forward/left/reverse rules; corrected negative values become zero; invalid Echo retains its previous handling.

Also verified duplicate capture commands do not restart a pending measurement, timeout and storage failures retain the previous correction and keep avoidance paused, invalid stored offsets fall back to uncalibrated readings, and STOP/emergency stop/mode changes/control timeout cancel capture. Pause does not capture or save; Resume avoidance is explicit.

Rendered-page tests pass with default and customized header values. They cover pause/capture/resume, pending-button disabling, raw/corrected readouts and messages, and existing driving/settings/reconnect behaviour. Page placeholders all resolve from the configuration header.

Existing PWM output dead zone 150, angle dead zone, PID control, saved settings and wiring remain. Calibration is an additive single-point distance correction; physical accuracy and robot dynamics require hardware testing. The build uses the isolated workspace board configuration without changing the shared Arduino installation.
