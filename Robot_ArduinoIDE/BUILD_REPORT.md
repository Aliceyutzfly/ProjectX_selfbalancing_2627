# Arduino IDE .ino build verification

Verified 23 October 2026 with Arduino CLI and Espressif Arduino ESP32 core 3.3.5.

- Board: XIAO_ESP32C3. USB CDC On Boot: Enabled.
- FQBN: esp32:esp32:XIAO_ESP32C3:CDCOnBoot=default.
- Actual application tabs: Robot_ArduinoIDE.ino and Balance.ino.
- Application .cpp files: none. Only bundled third-party libraries use .cpp/.c.
- All four third-party libraries compile from src/lib; no separate installations are required.
- Flash: 1,065,002 / 1,310,720 bytes.
- Global RAM: 38,356 / 327,680 bytes.
- Compile/link: PASS. Nothing uploaded or hardware-tested.

Arduino concatenates the .ino tabs into one translation unit. Explicit declarations and robot_types.h preserve type visibility; application mode identifiers avoid the PID library's MANUAL macro. Settings, webpage and robot behavior remain the same. Verification used an isolated workspace copy of the board core, without modifying the shared Arduino installation.
