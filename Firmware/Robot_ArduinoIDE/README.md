# Robot Arduino IDE project

Open `Robot_ArduinoIDE.ino` in Arduino IDE 2.x. Keep the complete folder and its name intact. The application is in two `.ino` tabs; third-party sources are bundled in `src/lib`. No PlatformIO, separate library installation or filesystem upload is needed. Do not add the standalone PID test sketch to this folder.

## Board and upload

1. Install Espressif's **esp32** package in Boards Manager. See `BUILD_REPORT.md` for the verified core version.
2. Select **XIAO_ESP32C3** (Seeed Studio XIAO ESP32C3).
3. Enable **USB CDC On Boot**. If USB Mode is offered, select **Hardware CDC and JTAG**. D6 remains the MPU interrupt.
4. Select the board's current COM port. Close Serial Monitor, then Verify and Upload. Serial speed is **115200**.
5. Join the Wi-Fi network configured in `robot_config.h` (default **XIAO-Robot**, password **robotdrive**) and open **http://192.168.4.1/**. Refresh the browser after uploading changed firmware.

## Code layout

- `Robot_ArduinoIDE.ino`: shared state, movement/ramping, obstacle sensing, settings, browser commands, setup and loop, grouped by topic.
- `Balance.ino`: independent IMU task, PID calculation, drive/correction mixing and final motor output.
- `robot_config.h`: the single configuration header for wiring, Wi-Fi address/ports, wheel-speed defaults, driving, avoidance, attitude, output, browser timing, USB diagnostics and IMU settings. Validation is at the bottom.
- `web_page.h`: offline webpage and renderer. Limits, thresholds, connection port and browser timing are filled from `robot_config.h`, so there is no separate JavaScript configuration to keep in sync. PID and angle dead zone share one Apply/Save/confirmation flow.
- `balance.h`, `robot_types.h`: shared declarations. Arduino generates the internal function prototypes. Shared types and distinct application mode identifiers avoid preprocessor/PID macro conflicts.
- `src/lib`: bundled WebSockets, PID, I2Cdev and MPU6050 library sources. `lib` retains the original archives; see `THIRD_PARTY.json` for versions and licenses.

## Current operation

Both driving modes continuously combine base wheel commands with PID attitude correction around the saved upright target. Remote control uses arrows/WASD. Forward/reverse starts at requested PWM **100** and S-ramps to the saved wheel trims over **1 second**; turns are immediate. Release/STOP zeros base drive while attitude correction remains active. Distance mode goes forward above **20 cm**, reverses below **5 cm**, and turns left between them, including endpoints. No echo retains the last movement or starts forward if stopped.

Default PID gains are **P=21, I=0, D=0.8**. The default angle dead zone is **+/-2 degrees**: attitude correction is zero within the band, while base driving still works. Final mixed wheel outputs are limited to **+/-255**, then magnitudes below the PWM dead zone of **150** become zero; exactly +/-150 passes. The current correction limit is **+/-255**. There is no operating tilt cutoff.

**PID settings** stops all motor output. Apply PID / Apply dead zone are temporary; Save retains each setting after restart. The page confirms the active values. Select Remote drive or Avoid obstacles to test afterward. **IMU calibration** also keeps motors off; capture/enter the upright angle and Save. First-use upright target is **178 degrees**; calibration accepts 90..270 degrees in the pitch+180 convention. These bounds are not a tilt cutoff. Angle, target, tilt and actual wheel outputs remain visible by the Wi-Fi display.

Emergency stop, disconnect, stale held commands and sensor/control faults disable all motor output. After a fault, held refreshes do not auto-restart; fix the cause and explicitly request STOP or a new direction. Startup requests stopped attitude correction once valid IMU data is ready. Saved wheel trims, upright angle, PID gains and angle dead zone survive ordinary uploads without flash erasure; Preferences keys match PlatformIO.

## Wiring

XIAO ESP32-C3; DRV8833 inputs D0..D3; MPU6050 SDA D4, SCL D5, INT D6; HC-SR04 Trig D10, Echo D7. Level-shift Echo to 3.3 V and share ground. Support the robot during first tests: even stopped attitude correction can move the wheels. Software build checks do not validate physical balance or wiring.

## Editing settings

Edit `robot_config.h`, then compile and upload. Do not change the `.ino` globals to tune the robot: they hold runtime state, not separate configuration. `MOTOR_PWM_DEAD_ZONE` is **150**; `DRIVE_START_PWM` is **100**; `DRIVE_RAMP_MS` is **1000**. `BALANCE_MAX_PWM` is reused for all wheel-command clamps. The default wheel targets are `DEFAULT_LEFT_PWM` and `DEFAULT_RIGHT_PWM` (**254** each).

The browser form ranges and timing follow the header automatically. Saved wheel trims, upright target, PID gains and angle dead zone still override their startup defaults; updating those defaults does not erase saved calibration. The PWM output dead zone is a compile-time setting and is separate from the saved angle dead zone.

The configuration organization is maintained directly in this Arduino folder. Preserve it when syncing later changes from PlatformIO; the headers and rendering setup now deliberately differ while using the same control behaviour.

## Set the ultrasonic 20 cm reference

1. Select **Avoid obstacles**, then press **Pause to position robot**. All motor output, including attitude correction, stops.
2. Place the ultrasonic sensor **20 cm from a flat wall/object**.
3. Press **Set current reading as 20 cm** and keep it still until the page confirms the reference was saved.
4. Press **Resume avoidance** to move again. Avoidance uses the corrected distance.

Calibration takes the median of five new valid Echo measurements and saves an additive correction in Preferences namespace `distanceCalib`, key `offset`. For example, a raw reference reading of 32 cm produces a -12 cm correction, so that point reads 20 cm. Corrected distances are clamped at zero; no Echo stays invalid and retains the existing no-Echo movement rule after resuming. Raw distance and the saved correction are shown in the calibration panel.

The reference survives restarts and ordinary uploads. Recalibrate at the same known physical distance to replace it. Missing Echo times out after 1200 ms; storage failures retain the previous correction. Both outcomes leave avoidance paused. STOP, emergency stop, mode changes or loss of control cancel an unfinished capture. Calibration never resumes motion by itself.

The reference distance, sample count and timeout are configurable in `robot_config.h` as `DISTANCE_REFERENCE_CM`, `DISTANCE_CALIBRATION_SAMPLES` and `DISTANCE_CALIBRATION_TIMEOUT_MS`. The physical reference distance is independent of the forward/reverse decision thresholds.
