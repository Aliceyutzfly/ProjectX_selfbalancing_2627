# XIAO two-mode robot

Modes: remote drive and ultrasonic obstacle avoidance. The previous third mode, MPU6050 initialization, background task, PID controller, settings, and webpage controls have been removed from the active project. A pre-change copy is in `backup-before-removing-balance`.

## Upload and connect

Build this project with PlatformIO, then upload using the board's current COM port. Stop Serial Monitor before uploading. Join the AP configured in `include/robot_config.h` (currently XIAO-Robot, password robotdrive) and open http://192.168.4.1/. No separate webpage upload or Internet connection is needed. The page displays the configured Wi-Fi name. One controlling browser is allowed.

## Operation

- Remote drive: hold arrows or WASD to move. Starting uses an S-shaped PWM ramp; normal release/STOP uses a linear slowdown. Durations are set by MANUAL_RAMP_MS and MANUAL_STOP_RAMP_MS (currently 1000 ms each).
- Avoid obstacles: move forward above 20 cm, otherwise turn right until clear. Missing/invalid echoes stop motor output. This mode applies its PWM immediately.
- MOTORS OFF stops drive immediately and returns to remote mode. Connection loss and stale manual commands also stop immediately. The heartbeat timeout is 800 ms. Reconnection never resumes motion.
- Distance telemetry continues in both modes. Stopping sensor triggers would not switch off the sensor's power.

## Wiring and trim

DRV8833 inputs: D0, D1, D2, D3. Ultrasonic Trig: D10; Echo: D7. All grounds must be common. HC-SR04 Echo needs level shifting from its 5 V signal to a suitable 3.3 V input; a 3.3 V Trig does not reduce Echo voltage. D9/GPIO9 is a boot strapping pin, so it is not the configured Echo input. D4/D5/D6 are unused by this firmware.

Use a suitable motor supply; motors must not be powered from the XIAO 3V3 output. Confirm wheel directions with the wheels raised. INVERT_LEFT and INVERT_RIGHT are preserved. PWM is clamped to 0..255. Existing flash-saved trims still override source values; startup currently falls back to 150 if no saved trim exists. Serial L/l and R/r adjust trim; S saves. Changing trim stops motion.

## Verification

Build with `pio run`. Run the embedded-page browser-event checks with `node test/web_ui.cjs`. These checks do not validate physical motor motion, supply current, or obstacle stopping clearance.
