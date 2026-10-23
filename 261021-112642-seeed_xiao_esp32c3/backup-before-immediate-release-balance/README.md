# XIAO robot: two modes

## Operation

- Manual: hold arrows or WASD to move remotely.
- Distance: valid distance above 20 cm goes forward; below 5 cm goes backward; 5 through 20 cm turns left. No valid echo stops immediately.
- Both modes use requestMotion() and updateDriveMotion(). Forward, backward, and normal STOP use an S-shaped PWM ramp. Left/right turns apply immediately. Repeated commands do not restart a ramp. Releasing a turn also uses the normal STOP ramp.
- STOP exits distance mode and slows the current motion to zero. MOTORS OFF, disconnect, malformed input, and stale manual commands cut motor output immediately. Startup/reconnect never restart motion. Upright balance automatically starts when stopped with fresh IMU data and within 20 degrees of calibrated upright. It also starts from the initial stopped state. Movement commands disable balance. Normal STOP requests balancing again after the S-shaped slowdown. Safety stops disable everything and do not auto-rearm.

DRIVE_RAMP_MS and STOP_RAMP_MS in include/robot_config.h are currently 1000 ms each. The S curve shapes commanded PWM, not measured acceleration. Forward/backward reversals and normal STOP take time: check physical clearance, particularly the 5 cm reverse threshold, with the robot supported first.

## Upload and control

Open this main project in PlatformIO, build, then upload using the board's current COM port. Stop Serial Monitor before uploading. Serial Monitor is 115200 baud. Join XIAO-Robot (password robotdrive), then open http://192.168.4.1/. The page shows AP_SSID from configuration. No Internet or separate webpage upload is required. One controlling browser is allowed; heartbeats/held commands refresh every 150 ms and the timeout is 800 ms. A normal STOP ramp can finish independently of held-command age, but heartbeat loss always cuts drive.

## Wiring and speed trim

DRV8833 IN1..IN4 use D0..D3. HC-SR04 Trig is D10 and Echo is D7. All grounds are common. Echo requires level shifting from 5 V to a suitable 3.3 V input; using a 3.3 V Trig does not lower Echo voltage. MPU6050 SDA/SCL/INT use D4/D5/D6 with supported 3.3 V supply and common ground. Provide a suitable motor supply; do not power the motors from the XIAO 3V3 output. Check motor directions with wheels raised.

PWM limits are 0..255. Source defaults and inversion flags are preserved. Saved flash trims override source defaults; without saved values the source defaults now apply. Serial L/l and R/r adjust wheel trims; S saves. Changing trim stops the robot. Existing backups and the separate standalone PID project are preserved.

## Checks

Build with PlatformIO. Run node test/web_ui.cjs for the embedded browser controls and node test/motion.cjs for actual main.cpp transition functions mechanically translated into a mocked hardware/time harness. These checks do not simulate chassis physics or establish real-world stopping distance.

## Always-visible angle and target

The page displays raw PID Angle and Target (upright is 178 degrees), plus Tilt relative to upright, in manual, distance, and stopped operation. Sensor reads continue while driving; only motor correction is disabled. Missing angle data displays --, not a made-up zero. Target remains visible even if the sensor fails. Disconnected values are marked unavailable. Ultrasonic measurement is paused while balancing to avoid blocking Echo waits.

The stopped PID reuses Kp 21, Ki 140, Kd 0.8 and the supplied gyro/acceleration offsets. It runs in a background task about every 10 ms. Both inverted motor outputs are compensated with BALANCE_OUTPUT_SIGN=-1. Stop/fall cutoff is 20 degrees; failed/stale IMU data, FIFO backlog or delayed task switch balance off. A normal STOP retries after a fault or MOTORS OFF; there is no repeated automatic re-arm after activation fails. Initially waiting for fresh readings/upright posture is not a latched fault. No correction is applied during forward/backward/turning or the slowdown ramp.

MOTORS OFF, disconnect, stale manual commands, invalid echoes, and serial trim changes disable balance. Startup can balance without a browser once fresh/upright, as requested for stopped operation. Reconnecting after a safety stop does not reactivate it. Balance may move the wheels and robot; it is not position hold. Support the robot for the first hardware test and check correction direction, STOP handoff, immediate motor-off and faults. Compile and mocked tests do not verify physical stability.
