# Robot Arduino IDE project

Open Robot_ArduinoIDE.ino in Arduino IDE 2.x. This folder is an independent copy of the current robot firmware, beside main and PID_test. Keep the sketch filename and folder name identical. No PlatformIO or filesystem upload is needed. Do not copy the standalone PID_test sketch into this folder.

## Board and upload

1. Install/select Espressif's esp32 board package in Boards Manager. Verified with Espressif Arduino ESP32 core 3.3.5; see BUILD_REPORT.md for the result.
2. Select XIAO_ESP32C3 (Seeed Studio XIAO ESP32C3).
3. Enable USB CDC On Boot. Set USB Mode to Hardware CDC and JTAG if that option is offered. This keeps Serial on USB while D6 is the MPU interrupt.
4. Select the board's current COM port, Verify, then Upload. Close Serial Monitor before Upload. Serial Monitor speed is 115200 baud.
5. Join the Wi-Fi network from src/robot_config.h and open http://192.168.4.1/.

## Arduino IDE tabs and folders

- Robot_ArduinoIDE.ino: the actual robot application, including setup(), loop(), Wi-Fi, motor ramps, distance mode and calibration saving. This is not a wrapper.
- Balance.ino: the complete upright PID controller and MPU6050 code, visible as an IDE tab.
- robot_config.h: editable pins, Wi-Fi, default upright angle, PID gains and timing; visible as an IDE tab.
- web_page.h: embedded control webpage; visible as an IDE tab.
- balance.h and robot_types.h: shared declarations/types; visible as IDE tabs.
- src/lib/: third-party library sources, automatically compiled. Only these libraries use .cpp/.c files.
- lib/: original library ZIP archives and license metadata.
- THIRD_PARTY.json: bundled versions and packaging notes.

All application code is now in the two .ino tabs. Arduino combines these tabs during compilation, so helper declarations are explicit and application mode names avoid the PID library's MANUAL macro. Keep both .ino files inside the Robot_ArduinoIDE folder. No third-party library installation is required.

## Current behavior

Manual: hold arrows/WASD to drive. Distance: forward above 20 cm, reverse below 5 cm, turn left from 5 through 20 cm. No echo keeps the previous movement; before any movement it starts forward. Forward/backward from STOP starts at PWM 50 and S-ramps to each wheel's target over 2000 ms. Turns react immediately. Releasing a command/STOP immediately requests upright balancing. EMERGENCY STOP disables motor output and balance. Wi-Fi timeouts also disable output.

IMU calibration mode keeps motors off. Hold upright, capture or enter the measured angle, then Save. First use defaults to 178 degrees. A saved angle overrides the default at startup through Preferences namespace imuCalib/key upright. A normal upload without flash erasure retains stored calibration and wheel trims. Angle/Target/Tilt remain visible beside Wi-Fi. STOP is centred in the direction pad; emergency stop is below it.

## Hardware

Seeed XIAO ESP32-C3; DRV8833 IN1..IN4 on D0..D3; MPU6050 SDA D4, SCL D5, INT D6; HC-SR04 Trig D10, Echo D7. Echo needs suitable 5 V to 3.3 V level shifting and common ground. The firmware does not confirm physical wiring. Support the robot for the first test: stopped balancing may move the wheels; continuing with no ultrasonic echo does not establish a clear path.

## Sources

Arduino sketch layout: https://docs.arduino.cc/arduino-cli/sketch-specification/
Seeed board setup: https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/
