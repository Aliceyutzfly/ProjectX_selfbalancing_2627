# XIAO robot: two modes

## Operation

- Manual: hold arrows or WASD to move remotely.
- Distance: valid distance above 20 cm goes forward; below 5 cm goes backward; 5 through 20 cm turns left. No valid echo stops immediately.
- Both modes use requestMotion() and updateDriveMotion(). Forward, backward, and normal STOP use an S-shaped PWM ramp. Left/right turns apply immediately. Repeated commands do not restart a ramp. Releasing a turn also uses the normal STOP ramp.
- STOP exits distance mode and slows the current motion to zero. MOTORS OFF, disconnect, malformed input, and stale manual commands cut motor output immediately. Startup/reconnect never restart motion. There is no balancing/PID/IMU initialization or background task.

DRIVE_RAMP_MS and STOP_RAMP_MS in include/robot_config.h are currently 1000 ms each. The S curve shapes commanded PWM, not measured acceleration. Forward/backward reversals and normal STOP take time: check physical clearance, particularly the 5 cm reverse threshold, with the robot supported first.

## Upload and control

Open this main project in PlatformIO, build, then upload using the board's current COM port. Stop Serial Monitor before uploading. Serial Monitor is 115200 baud. Join XIAO-Robot (password robotdrive), then open http://192.168.4.1/. The page shows AP_SSID from configuration. No Internet or separate webpage upload is required. One controlling browser is allowed; heartbeats/held commands refresh every 150 ms and the timeout is 800 ms. A normal STOP ramp can finish independently of held-command age, but heartbeat loss always cuts drive.

## Wiring and speed trim

DRV8833 IN1..IN4 use D0..D3. HC-SR04 Trig is D10 and Echo is D7. All grounds are common. Echo requires level shifting from 5 V to a suitable 3.3 V input; using a 3.3 V Trig does not lower Echo voltage. D4/D5/D6 are unused. Provide a suitable motor supply; do not power the motors from the XIAO 3V3 output. Check motor directions with wheels raised.

PWM limits are 0..255. Source defaults and inversion flags are preserved. Saved flash trims override source defaults; without saved values the source defaults now apply. Serial L/l and R/r adjust wheel trims; S saves. Changing trim stops the robot. Existing backups and the separate standalone PID project are preserved.

## Checks

Build with PlatformIO. Run node test/web_ui.cjs for the embedded browser controls and node test/motion.cjs for actual main.cpp transition functions mechanically translated into a mocked hardware/time harness. These checks do not simulate chassis physics or establish real-world stopping distance.
