# PlatformIO robot: continuous drive plus attitude correction

This PlatformIO project now differs from the Arduino .ino package, which was intentionally left unchanged.

## Start here

| To understand or change | Read |
| --- | --- |
| Pins, gains, PWM limits and timing | `include/robot_config.h` |
| Forward, reverse, turns and S-ramp | `requestMotion()` and `updateDriveMotion()` in `src/main.cpp` |
| Distance decisions | `handleDistanceMode()` in `src/main.cpp` |
| Browser command meanings | `handleCommand()` in `src/main.cpp` |
| PID and final wheel output | `updateAttitudeCorrection()` and `writeMixedMotorOutputs()` in `src/balance.cpp` |
| Page layout and browser input | `include/web_page.h` |

The control path is: browser/distance decision -> base wheel requests -> PID correction -> mix -> saturation -> PWM dead zone -> motors. The saved upright target stays fixed while driving. `armPending` means one enable attempt is waiting for fresh IMU data; it clears after that attempt. Settings pages and emergency/fault stops cannot automatically resume movement.

The browser uses one shared settings flow (`submitTuning`/`showTuning`) for PID and the angle dead zone. Each form supplies its field names and limits; send, acknowledgement, timeout and reconnect handling are shared. Firmware command routing is separate from WebSocket connection handling.

## Controller

One 10 ms task owns motor PWM. The web/ultrasonic loop publishes base left/right requests; it never writes nonzero motor PWM directly. Each fresh IMU sample runs attitude PID around the saved upright target, adds the same bounded correction to both wheel requests, and clips each final output to -255..255. There is no output normalisation or alternate manual-versus-balance ownership. The base pair already contains wheel trim and differential steering.

Defaults: P=21, I=0, D=0.8. ATTITUDE_MAX_PWM=255 bounds correction separately from the final motor limits. I defaults to zero because sustained attitude integral can fight intentional driving on a self-righting chassis. These are starting values, not validated tuning. Near base PWM 254, almost no positive correction headroom remains; the page reports Output limited when mixing clips. There is no angle cutoff. Gains/sign/limit need supported hardware tests; bounded correction does not guarantee stability.

Manual: hold arrows/WASD. Forward/backward from STOP starts at magnitude 100 and S-ramps to saved wheel trims (source defaults 254) over 1000 ms. Turns change the base pair immediately. Release/STOP immediately zeros base drive while attitude correction continues; STOP exits distance mode. Distance: forward above 20 cm, backward below 5 cm, left from 5 through 20 cm. No echo preserves the previous motion; with no movement it starts forward. Ultrasonic sampling continues while the independent controller runs.

EMERGENCY STOP, disconnect, stale manual commands, sensor errors/stale data/backlog, delayed control or main-loop drive-publication timeout disable the controller and both motors. Faults do not auto-rearm from repeated held commands; use STOP or a new direction after fixing the cause. Startup requests zero-base stabilization when fresh IMU data arrives. Calibration disables all motor output. No motor can run without a healthy IMU in this architecture.

## Wireless PID controls

Select PID settings in the mode selector to show active P/I/D and editing controls. Entering this mode stops all motors; settings are hidden in other modes. Apply PID changes RAM only; Save PID to robot also persists the complete triple in one Preferences blob (attitudePID/gains). Gain ranges are P 0..200, I 0..500, D 0..50. All values must be finite. Both actions cancel held input, stop all motor output, reset PID history and leave control disabled in PID settings until you select Remote drive or Avoid obstacles. Saving failure restores the previous runtime gains and reports failure. There is no automatic flash write on every loop. Typed fields are not overwritten by incoming telemetry. Typing alone does not apply gains: press Apply or Save. The page waits for a numbered robot acknowledgement and compares all three returned gains; rejection or no acknowledgement is shown explicitly. Active gains come directly from PID_v1 getters. Motors remain off in PID settings; select Remote drive to test. Correction saturation at the configured correction limit can mask gain changes. Keyboard driving is ignored while a form input is focused.

## Angle dead zone

PID settings includes a separate angle dead zone, default +/-2 degrees around the saved upright target. Within and exactly on its boundaries, attitude PWM is zero and the PID integral/history is reset before correction resumes outside the band. Base driving commands remain active. Apply dead zone is temporary; Save dead zone persists attitudePID/deadzone independently of the P/I/D triple. Accepted half-width is 0..90 degrees; 0 disables the finite-width band. Editing keeps motors off; select a driving mode to test. The page displays the active half-width and robot confirmation. Invalid stored values fall back to 2 degrees.

## Upright calibration and wiring

IMU calibration still keeps motors off. Capture/enter the upright angle and Save; first-use default 178 is overridden by saved imuCalib/upright after restart. Angle/Target/Tilt stay visible beside Wi-Fi. This sets the target, not gyro offsets. Motor inputs D0..D3; MPU SDA/SCL/INT D4/D5/D6; HC-SR04 Trig D10, Echo D7 through suitable 5 V to 3.3 V level shifting; common ground. Use an appropriate motor supply, not the XIAO 3V3 output.

## Build and connect

PlatformIO environment seeed_xiao_esp32c3, espressif32 6.12.0, Arduino ESP32 2.0.17; USB CDC enabled. Build/upload from this folder and select the board's current COM port. Serial is 115200 baud. Join XIAO-Robot (password robotdrive) and open http://192.168.4.1/. All webpage assets are embedded. Dependencies are pinned in platformio.ini. Support the robot when testing; do not upload until ready for controller behavior to change.

Automated tests use actual firmware functions mechanically translated into mocked hardware/time harnesses. They verify requests, saturation, lifecycle, parsing, persistence and webpage behavior, not physical dynamics or native RTOS timing.

Source layout: `src/main.cpp` handles motion requests, modes, networking and stored settings; `src/balance.cpp` owns IMU sampling, PID and motor mixing (separate correction and output stages, with a shared PID reset); `include/robot_config.h` contains editable hardware/control values; `include/web_page.h` contains the offline page. Settings modes share a motor-disable guard, and all motion resets use one helper.

Final motor PWM dead zone: `MOTOR_PWM_DEAD_ZONE=150`. After mixing drive and PID correction and limiting to +/-255, each wheel output with magnitude below 150 becomes zero; exactly +/-150 passes. It applies in both driving modes and stopped attitude control. Wheel output telemetry shows the filtered values. The adjustable angle dead zone is separate.
