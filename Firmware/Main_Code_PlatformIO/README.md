# Main robot firmware - PlatformIO

Firmware for the XIAO ESP32-C3 robot with DRV8833/N20 motors, MPU6050 and HC-SR04. Features and shared configuration now match the Arduino IDE version in `../Robot_ArduinoIDE`.

## Folder guide

| File | Purpose |
| --- | --- |
| `src/main.cpp` | Movement/ramping, distance decisions, saved settings, networking, setup and loop, grouped by topic. |
| `src/balance.cpp` | Independent IMU task, PID correction, motor mixing and final output. |
| `include/robot_config.h` | Wiring, Wi-Fi, motion defaults, thresholds, calibration, timing and validation. |
| `include/web_page.h` | Offline page and renderer; browser limits/timing come from the configuration header. |
| `include/balance.h`, `include/robot_types.h` | Shared API and types. |
| `platformio.ini` | XIAO board, framework, USB flags and pinned library dependencies. |
| `test/` | Node.js regression checks with mocked hardware/time/storage. |

## Build and connect

Open this folder as the PlatformIO project. Build, then upload to the board's current COM port; close Serial Monitor before uploading. Serial speed is **115200** and USB CDC is enabled. The pinned environment is `seeed_xiao_esp32c3`, Espressif32 **6.12.0**, Arduino ESP32 **2.0.17**.

Join **XIAO-Robot** (default password **robotdrive**) and open **http://192.168.4.1/**. Refresh the page after updating firmware. No filesystem upload or internet connection is needed.

## Operation

- **Remote drive:** hold arrows/WASD. Forward/reverse starts at requested PWM 100 and S-ramps over 1 second to saved wheel trims; defaults are 254 per wheel. Turns are immediate. Release/STOP zeros driving while attitude correction continues.
- **Avoid obstacles:** corrected distance above 20 cm goes forward, below 5 cm reverses, and 5..20 cm turns left. No Echo preserves movement; if stopped it starts forward, except while calibration is paused.
- **PID settings:** stops all motors. Apply changes RAM; Save remembers settings after restart. Default P/I/D are 21/0/0.8. Select a driving mode afterward to resume.
- **IMU calibration:** keeps motors off while capturing/saving the upright target. Default target is 178 degrees. Calibration accepts 90..270 degrees; this is not an operating tilt cutoff.

Both driving modes mix base wheel commands with continuous attitude correction. The default angle dead zone is +/-2 degrees. Final output is limited to +/-255, then magnitudes below the PWM output dead zone of **150** become zero; exactly +/-150 passes. These are separate settings. Emergency stop, lost control and sensor/control faults disable output; held refreshes do not automatically restart a faulted controller.

## Ultrasonic 20 cm reference

Under **Avoid obstacles**, press **Pause to position robot**, place the sensor 20 cm from a flat wall/object, then press **Set current reading as 20 cm**. Keep it still until saved, then press **Resume avoidance**.

Five fresh valid readings are collected and their median sets an additive offset. A raw reference of 32 cm becomes corrected 20 cm with offset -12 cm. The offset is saved in Preferences (`distanceCalib/offset`) and survives restarting. Raw readings and correction are shown on the page. Missing Echo times out; failed saves keep the old correction. Calibration always stays paused until explicit resume, and STOP/mode changes/disconnect cancel an unfinished capture.

Edit only `include/robot_config.h` for firmware settings. Saved wheel trims, upright target, PID gains and angle dead zone override their startup defaults. The sensor reference has its own stored offset. Settings keys are compatible with the Arduino version; normal uploads without flash erasure retain calibration.

## Wiring and verification

DRV8833 inputs D0..D3; MPU6050 SDA D4, SCL D5, INT D6; HC-SR04 Trig D10, Echo D7 with 5 V to 3.3 V level shifting and common ground. Support the robot for initial tests: attitude correction can move its wheels while stopped.

See `BUILD_REPORT.md`. Software checks exercise actual functions translated into mocked hardware/time/storage harnesses; they do not verify physical balancing or sensor accuracy.
