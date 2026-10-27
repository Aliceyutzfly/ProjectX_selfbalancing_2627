# Arduino IDE build verification

Verified 27 October 2026 with Arduino CLI and Espressif Arduino ESP32 core 3.3.5.

- Board: XIAO_ESP32C3. USB CDC On Boot: Enabled.
- FQBN: `esp32:esp32:XIAO_ESP32C3:CDCOnBoot=default`.
- Application tabs: `Robot_ArduinoIDE.ino` and `Balance.ino`.
- Compile/link: PASS. Flash: **1,078,918 / 1,310,720 bytes (82%)**.
- Global RAM: **38,404 / 327,680 bytes (11%)**.
- Bundled libraries compile from `src/lib`; no separate library installs required.
- No application `.cpp` files. Nothing uploaded or hardware-tested.

## Configuration cleanup checks

All 45 original configuration values are unchanged. Editable defaults and connection/timing parameters are centralized in `robot_config.h`; repeated wheel clamps reuse `BALANCE_MAX_PWM`. The output dead zone remains 150 and the default angle dead zone remains +/-2 degrees. Math expressions and chip-format checks remain beside their algorithms.

43 existing function definitions match the previous control/validation logic after expanding the named constants and normalizing the three revised validation messages. Setup and diagnostic reporting were checked separately through native compilation and page/configuration checks. The control task, motor pipeline, timeout/fault checks, saved Preferences keys and runtime calibration remain.

The function-reference audit found no unused application functions, including callbacks. Redundant explicit prototype blocks were removed; Arduino's generated prototypes compile successfully. The application now has 47 referenced functions, including the new URL and page-rendering helpers.

The rendered-page regression tests pass with default and customized header values: direction controls, mode selection, PID/dead-zone Apply/Save/acknowledgements, validation, keyboard input, reconnect and offline assets. HTML limits and JavaScript parameters are generated from the same configuration header. This checks software behaviour, not physical robot dynamics.

Verification uses the isolated workspace board-core configuration; the shared Arduino installation was not modified. Arduino-specific configuration cleanup is now maintained independently of the PlatformIO source layout.
