# XIAO robot: two modes

## Operation

- Manual: hold arrows or WASD to move remotely.
- Distance: valid distance above 20 cm goes forward; below 5 cm goes backward; 5 through 20 cm turns left. No valid echo stops immediately.
- Both modes use requestMotion() and updateDriveMotion(). Forward and backward use an S-shaped PWM ramp. Release/STOP cancels the drive ramp and hands control immediately to upright PID, without a stopping ramp. Left/right turns apply immediately. Repeated commands do not restart a ramp. Releasing a turn also immediately requests upright balancing.
- STOP exits distance mode, cancels ordinary motor drive, and starts upright balancing as soon as fresh IMU data/posture allow it. MOTORS OFF, disconnect, malformed input, and stale manual commands cut motor output immediately. Startup/reconnect never restart motion. Upright balance automatically starts when stopped with fresh IMU data and within 20 degrees of calibrated upright. It also starts from the initial stopped state. Movement commands disable balance. Normal release/STOP requests balancing immediately. Safety stops disable everything and do not auto-rearm.

DRIVE_RAMP_MS in include/robot_config.h is currently 2000 ms. The obsolete STOP_RAMP_MS setting has been removed. The S curve shapes commanded PWM, not measured acceleration. Forward/backward reversals take time: check physical clearance, particularly the 5 cm reverse threshold, with the robot supported first.

## Upload and control

Open this main project in PlatformIO, build, then upload using the board's current COM port. Stop Serial Monitor before uploading. Serial Monitor is 115200 baud. Join XIAO-Robot (password robotdrive), then open http://192.168.4.1/. The page shows AP_SSID from configuration. No Internet or separate webpage upload is required. One controlling browser is allowed; heartbeats/held commands refresh every 150 ms and the timeout is 800 ms. Normal release/STOP requests upright PID immediately; heartbeat loss disables both drive and balancing.

## Wiring and speed trim

DRV8833 IN1..IN4 use D0..D3. HC-SR04 Trig is D10 and Echo is D7. All grounds are common. Echo requires level shifting from 5 V to a suitable 3.3 V input; using a 3.3 V Trig does not lower Echo voltage. MPU6050 SDA/SCL/INT use D4/D5/D6 with supported 3.3 V supply and common ground. Provide a suitable motor supply; do not power the motors from the XIAO 3V3 output. Check motor directions with wheels raised.

PWM limits are 0..255. Source defaults and inversion flags are preserved. Saved flash trims override source defaults; without saved values the source defaults now apply. Serial L/l and R/r adjust wheel trims; S saves. Changing trim stops the robot. Existing backups and the separate standalone PID project are preserved.

## Checks

Build with PlatformIO. Run node test/web_ui.cjs for the embedded browser controls and node test/motion.cjs for actual main.cpp transition functions mechanically translated into a mocked hardware/time harness. These checks do not simulate chassis physics or establish real-world stopping distance.

## Always-visible angle and target

The page displays raw PID Angle and Target (upright is 178 degrees), plus Tilt relative to upright, in manual, distance, and stopped operation. Sensor reads continue while driving; only motor correction is disabled. Missing angle data displays --, not a made-up zero. Target remains visible even if the sensor fails. Disconnected values are marked unavailable. Ultrasonic measurement is paused while balancing to avoid blocking Echo waits.

The stopped PID reuses Kp 21, Ki 140, Kd 0.8 and the supplied gyro/acceleration offsets. It runs in a background task about every 10 ms. Both inverted motor outputs are compensated with BALANCE_OUTPUT_SIGN=-1. Stop/fall cutoff is 20 degrees; failed/stale IMU data, FIFO backlog or delayed task switch balance off. A normal STOP retries after a fault or MOTORS OFF; there is no repeated automatic re-arm after activation fails. Initially waiting for fresh readings/upright posture is not a latched fault. No correction is applied during forward/backward/turning while driving.

MOTORS OFF, disconnect, stale manual commands, invalid echoes, and serial trim changes disable balance. Startup can balance without a browser once fresh/upright, as requested for stopped operation. Reconnecting after a safety stop does not reactivate it. Balance may move the wheels and robot; it is not position hold. Support the robot for the first hardware test and check correction direction, STOP handoff, immediate motor-off and faults. Compile and mocked tests do not verify physical stability.

## Starting PWM

Forward/backward from STOP starts immediately at DRIVE_START_PWM (50), then follows the existing S curve to each wheel's target over DRIVE_RAMP_MS (currently 2000 ms). With target 254, the magnitude is 50 + (254 - 50) * t*t*(3 - 2*t). Backward uses negative logical values. Saved wheel trims still set the target and a target below 50 caps its starting magnitude. Repeated held commands do not reset the ramp. A direction change while already moving ramps from the currently applied PWM instead of jumping to 50. Turns and release-to-balance remain immediate.

## Wireless upright calibration

Select IMU calibration on the robot webpage. Entering this settings mode immediately disables motors, cancels movement ramps, and prevents balancing or driving. The live Angle/Target/Tilt display remains active. Hold the body in its intended upright posture, use Capture current angle to copy the current sensor reading (a single fresh snapshot, not a stability average), or enter an angle manually. Save upright angle updates the runtime target and writes it to ESP32 Preferences namespace imuCalib, key upright. Stay supported; capture and save do not prove correct mechanical posture.

On first use, DEFAULT_BALANCE_UPRIGHT_DEG is 178. Normal restart and firmware upload without flash erasure load the saved value. This calibrates only the balance upright target; it does not change gyro offsets or PID gains. Only explicit Save writes flash. Invalid/nonfinite/out-of-range angles, stale sensor readings, saves outside calibration mode, and unavailable storage are rejected. A storage write failure restores the previous runtime target and reports failure. Accepted angle range is 90..270 degrees in the pitch+180 convention. Calibration remains motors-off after Save; Remote drive resumes normal operation and stopped balancing with the saved target. STOP stays in calibration without arming; MOTORS OFF exits with output disabled. Disconnect exits to disabled manual mode.

The driving rules, PWM 50 start, two-second S curve, immediate turns, and release-to-balance outside calibration are unchanged. test/calibration.cjs covers validation, storage failure rollback, first-use fallback and loading saved settings with mocked Preferences/motor hardware; actual flash retention still requires hardware testing.
