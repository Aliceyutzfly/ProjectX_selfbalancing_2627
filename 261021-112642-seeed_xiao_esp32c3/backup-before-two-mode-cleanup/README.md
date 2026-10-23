# XIAO two-mode robot

Modes: remote drive and ultrasonic obstacle avoidance. There is no third mode button. A normal STOP enables upright balancing after slowdown; direct driving disables balancing. A pre-change copy is in `backup-before-balance-on-stop`.

## Upload and connect

Build this project with PlatformIO, then upload using the board's current COM port. Stop Serial Monitor before uploading. Join the AP configured in `include/robot_config.h` (currently XIAO-Robot, password robotdrive) and open http://192.168.4.1/. No separate webpage upload or Internet connection is needed. The page displays the configured Wi-Fi name. One controlling browser is allowed.

## Operation

- Remote drive: hold arrows or WASD to move. Starting uses an S-shaped PWM ramp; normal release/STOP uses a linear slowdown, then hands the motors to upright PID. STOP while already idle tries balancing immediately; STOP from obstacle mode returns to manual and tries balancing. An invalid echo in obstacle mode just stops motor output. Durations are set by MANUAL_RAMP_MS and MANUAL_STOP_RAMP_MS (currently 1000 ms each).
- Avoid obstacles: move forward above 20 cm, otherwise turn right until clear. Missing/invalid echoes stop motor output. This mode shares the same S-shaped starting/direction-change ramp as remote drive through drive() and updateDriveMotion(). MANUAL_RAMP_MS sets the duration for both modes. Repeated autonomous requests do not reset the ramp. Invalid echoes still stop immediately. The held-command timeout applies only to remote drive; the heartbeat timeout applies to both modes.
- MOTORS OFF stops drive immediately and returns to remote mode. Connection loss and stale manual commands also stop immediately. The heartbeat timeout is 800 ms. Reconnection never resumes motion. Startup, disconnect, malformed commands, stale movement/heartbeat, and MOTORS OFF leave balancing disabled. A tilt/sensor/task fault latches balancing off until an explicit STOP retries it.
- Distance telemetry continues in both modes except while upright balancing is active, when ultrasonic waits are suspended. Stopping sensor triggers would not switch off the sensor's power.

## Wiring and trim

DRV8833 inputs: D0, D1, D2, D3. Ultrasonic Trig: D10; Echo: D7. All grounds must be common. HC-SR04 Echo needs level shifting from its 5 V signal to a suitable 3.3 V input; a 3.3 V Trig does not reduce Echo voltage. D9/GPIO9 is a boot strapping pin, so it is not the configured Echo input. MPU6050 SDA/SCL/INT connect to D4/D5/D6; use the supported 3.3 V supply and common ground.

Use a suitable motor supply; motors must not be powered from the XIAO 3V3 output. Confirm wheel directions with the wheels raised. INVERT_LEFT and INVERT_RIGHT are preserved. PWM is clamped to 0..255. Existing flash-saved trims still override source values; startup currently falls back to 150 if no saved trim exists. Serial L/l and R/r adjust trim; S saves. Changing trim stops motion.

## Verification

Build with `pio run`. Run the embedded-page browser-event checks with `node test/web_ui.cjs`. These checks do not validate physical motor motion, supply current, or obstacle stopping clearance.

## Balancing only on STOP

The supplied PID values are reused: upright 178 degrees, Kp 21, Ki 140, Kd 0.8, gyro offsets -479/84/15 and Z acceleration offset 1638. The PID runs on fresh IMU measurements in a background task, approximately every 10 ms. No driving lean target or balance-driving commands exist. Direct arrow control uses the previous wheel trims and ramps without tilt correction, so the robot may fall while driving or during the stop ramp. This feature does not solve balancing during travel.

Balancing can start only with fresh sensor data and tilt within 20 degrees of calibrated upright. Beyond 20 degrees, invalid/stale data, backlog, or delayed control switches it off. No automatic retry after fault or MOTORS OFF. Repeated STOP while balancing does not reset the PID. STOP after a fault retries once. Motor output polarity preserves the standalone test with both current inversion flags true.

STOP balance can move the robot and does not guarantee zero speed or position hold. MOTORS OFF disables the PID and motor drive and prevents automatic reactivation. For the first test, support the robot, press STOP while upright, check live tilt/PWM, and verify wheels catch the body. Then test a short driving command, its release-to-slowdown-to-balance handoff, motor-off, fault latching, and disconnect. These transitions and actual stability require hardware validation.

Run `node test/stop_balance.cjs` for main.cpp transition checks with mocked hardware and time. Source functions are mechanically translated for the harness; this does not execute native firmware or simulate physical balancing.
