# Arduino IDE build verification

Verified 26 October 2026 with Arduino CLI and Espressif Arduino ESP32 core 3.3.5.

- Board: XIAO_ESP32C3. USB CDC On Boot: Enabled.
- FQBN: `esp32:esp32:XIAO_ESP32C3:CDCOnBoot=default`.
- Application tabs: `Robot_ArduinoIDE.ino` and `Balance.ino`.
- Compile/link: PASS. Flash: **1,076,448 / 1,310,720 bytes (82%)**.
- Global RAM: **38,404 / 327,680 bytes (11%)**.
- All four bundled third-party libraries compile from `src/lib`.
- No application `.cpp` files; only third-party libraries use `.cpp`/`.c`.
- Nothing uploaded or hardware-tested.

## PlatformIO parity

All **45 application function definitions** match the current PlatformIO source after normalizing application mode identifiers. `robot_config.h`, `balance.h` and `web_page.h` are byte-identical. Arduino-specific differences are bundled-library include paths, explicit function prototypes, shared enums in `robot_types.h`, distinct application mode identifiers to avoid PID macros, and removal of the internal anonymous namespace for Arduino sketch preprocessing.

Includes continuous drive plus attitude correction, the PID settings page and confirmations, saved upright calibration, saved PID/dead-zone settings, the default +/-2 degree angle dead zone, PWM output dead zone 50, and current motion/configuration values. Code organization and formatting follow the current PlatformIO version.

Verification used an isolated workspace board-core configuration and build directory; the shared Arduino installation was not modified.
