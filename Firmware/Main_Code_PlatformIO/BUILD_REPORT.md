# PlatformIO build verification

Verified 27 October 2026 for XIAO ESP32-C3 using the pinned Espressif32 6.12.0 / Arduino ESP32 2.0.17 environment.

- Compile/link: PASS.
- Flash: **855,424 / 1,310,720 bytes (65.3%)**.
- Global RAM: **41,444 / 327,680 bytes (12.6%)**.
- Eight software regression checks: PASS (motion, controller, webpage, upright calibration, PID settings, installed PID calculation, command/telemetry protocol, ultrasonic reference calibration).
- All **50 application function definitions** match the Arduino version after include-path and internal-linkage adaptations. Four shared headers are byte-identical.
- All pre-existing configuration values are preserved, including PWM output dead zone 150.
- Normal PlatformIO `.cpp`/`include` structure, pinned dependencies and USB settings retained.
- Nothing uploaded or hardware-tested.

The new tests cover pause, fresh median reference capture, saved offset reload, corrected movement decisions, no-Echo behaviour, timeout/save failure/cancellation, and explicit resume. Existing mode/settings/connection tests still pass. Page tests render configured limits and timing rather than executing unexpanded placeholders. Motion fixtures use sample timings/trim values to exercise the algorithm; the firmware uses `robot_config.h`.
