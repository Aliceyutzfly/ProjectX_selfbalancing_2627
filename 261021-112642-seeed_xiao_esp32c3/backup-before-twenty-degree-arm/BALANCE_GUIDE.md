# Balance + forward/backward drive

Implemented 22 October 2026 in the existing AP robot project. This is a first hardware-test version, not a claim of stable balance on the assembled robot. Nothing has been uploaded by Codex.

## Simple control logic

```text
Fresh MPU6050 pitch -> compare with target -> PID -> same correction to both wheels

No arrow held: target = calibrated upright
Forward held: target = upright + a small lean
Backward held: target = upright - a small lean
```

The motor polarity and lean sign depend on the installed sensor and motors. The values below preserve the standalone PID test's electrical correction direction for the current two inverted wheel settings. Validate these directions while physically supporting the chassis.

There is no programmed reverse-then-forward sequence. PID decides the wheel output continuously. There is no steering in balance mode in this version. The ordinary Remote drive mode still has left/right buttons.

## Files to teach from

- `include/robot_config.h`: upright target, gains, calibration offsets, lean amount, sign settings, and cutoffs.
- `src/balance.cpp`: `readAngle()` reads the measurement; `updateBalance()` slowly moves the target and runs PID; the same PWM is applied to both wheels.
- `include/balance.h`: small interface used by `main.cpp`.
- `src/main.cpp`: existing Wi-Fi, modes, commands, telemetry, and motor driver.
- `include/web_page.h`: three-mode control page, tilt/target display, arm and disarm controls.

There is one Arduino `setup()` and one `loop()`. Do not copy the complete standalone PID sketch into this project's `src` folder: it defines a second `setup()`/`loop()`.

## Startup and operation

Boot starts in ordinary manual mode with motors off. MPU6050 initialization runs at startup. If it fails, the page shows the error and the two original modes remain usable. Correct sensor wiring and reset to retry initialization.

1. Upload from the main project folder on D:, not the separate PID project. Open Serial Monitor at 115200 baud.
2. Join the configured AP and open `http://192.168.4.1/`.
3. Support the robot near upright. Read the live tilt, shown relative to the configured upright target. Do not assume that a displayed zero proves correct sensor mounting/calibration.
4. Select **Balance + drive / Arm**. Arming needs ten consecutive recent samples, a latest sample no older than 30 ms, and measured tilt within 5 degrees of the configured upright target. A refused arm is shown on the page; hold upright and select Arm again.
5. Confirm balance-only operation before briefly pressing Forward/Backward.
6. Support the robot before pressing **MOTORS OFF / DISARM**. That cuts drive and returns to ordinary manual mode. A two-wheel robot may fall when disarmed.

| Action/event in balance mode | Behavior |
| --- | --- |
| Hold Forward/Backward | Gradually shifts the angle target by up to 2 degrees |
| Release, Space, or HOLD UPRIGHT | Target returns toward upright at 2 degrees/second; balance stays active |
| No new movement command for over 800 ms | Requests upright; task keeps balancing |
| Wi-Fi disconnect/heartbeat timeout | Requests upright; balance continues without the phone |
| Reconnect | No old held movement resumes; page reads the actual balance state |
| Motors off / Disarm | Zero motor output; exit balance mode |
| Excessive tilt, invalid/failed IMU read, data backlog, stale data, delayed control | Zero motor output; remain disarmed until explicitly re-armed |

Manual/obstacle mode buttons are disabled while balance is armed. Firmware also refuses a direct switch away from armed balance until explicit disarm. This prevents an ordinary mode selection from silently cutting balance power. After a fault, the page allows selecting another mode or explicitly re-arming once readings and posture meet the conditions.

Returning the target upright is not a guaranteed speed stop. With angle feedback alone, the robot may coast or drift. There is no wheel-speed measurement, position hold, speed limit in metres/second, autonomous obstacle avoidance while balancing, or guaranteed recovery after a large disturbance.

## What was reused from PID_test.cpp

- Angle calculation: `pitchRadians * 180 / PI + 180`.
- Upright target: 178 degrees.
- Kp = 21, Ki = 140, Kd = 0.8.
- Gyro offsets: -479, 84, 15. Z accelerometer offset: 1638.
- PID minimum sample interval: 10 ms; signed output limited to -255..255.

These are starting values copied from the user's tested sketch. They have not been experimentally retuned for Wi-Fi operation or a changed chassis. If the chassis, mounting, battery position, or sensor has changed, recheck the upright target and tuning.

The two existing wheel inversion settings remain `true`. `BALANCE_OUTPUT_SIGN = -1` compensates for those inversions so a positive standalone PID correction still energizes the same DRV8833 legs as before. If the wheel polarity changes, recheck this sign. `BALANCE_FORWARD_LEAN_SIGN` sets which target direction means physical forward. Verify balance correction first, then drive-target direction; changing one does not automatically fix the other.

## Timing and fault handling

A dedicated FreeRTOS task is scheduled about every 10 ms, independently of the main web loop. Its first job is reading fresh data, then selecting a target, computing PID, and updating both wheels. A mutex keeps mode changes/manual output from racing that motor update. This scheduling plumbing leaves the teaching controller as one PID, not cascaded speed/position controllers.

The D6 interrupt only sets a data-ready flag. It does not detect a tilt threshold, read I2C, or run PID. The task also polls FIFO availability as a fallback. It never waits in a loop for a packet: only complete packets already present are read, at most two per cycle, taking the latest complete measurement. Old backlogs are discarded and an armed controller is disarmed rather than using stale orientation.

I2C is set to 400 kHz with short transaction timeouts. Sensor read failures or invalid quaternion/pitch data cut drive. No valid fresh sample for over 100 ms cuts drive. Measured body tilt more than 20 degrees from upright cuts drive. A control gap or individual sensor-read section over 50 ms also disarms. These are software checks executed when the task runs, not an independent hardware emergency-stop circuit.

The ultrasonic sensor is not triggered/read in BALANCE mode. Its potentially blocking Echo wait is excluded. Low-rate webpage and Serial reporting continue. Wheel trim via Serial is ignored in balance mode; ordinary manual-mode trims do not scale PID output.

## Hardware test sequence

1. Confirm MPU6050 SDA D4, SCL D5, INT D6, supported supply, and common ground. Echo is D7 through suitable level shifting for the other two modes.
2. Start with the robot supported so it cannot fall or run away. Confirm sensible live pitch changes and the measured upright target before arming.
3. While supporting it, arm and gently lean forward/backward. Wheels must move to catch the body. If correction increases the fall, disarm and check `BALANCE_OUTPUT_SIGN`, sensor axis, and wheel polarity. Do not attempt free balancing with a wrong correction sign.
4. Establish stationary balance before driving. Briefly hold Forward, release, then test Backward. Start with very short commands and space to catch it; target lean does not enforce a speed cap.
5. While supported, verify release and simulated Wi-Fi loss request upright without cutting balance power. Verify explicit DISARM removes drive. Confirm fault cutoff behavior while restrained, including excessive tilt and an IMU failure test performed with appropriate power/wiring precautions.
6. If gains need adjustment, tune under supervision. The robot's mechanical behavior and actual timing under Wi-Fi load remain to be measured.

## Verification scope

PlatformIO compile/link for XIAO ESP32-C3 and automated mocked-browser control tests passed. The browser checks cover original manual/obstacle controls plus balance gating, disabled steering, hold-upright command, explicit disarm, fault clearing of held motion, and reconnect without stale movement. They do not execute the physical PID loop or simulate the chassis.

No on-device balance, sensor-fault, timing-under-load, or physical motor-direction test has been performed by Codex. The fresh-data and fault logic was reviewed in source; its real sensor/RTOS behavior still needs the hardware tests above.

## Arming diagnosis

The page explicitly shows ARMED or NOT ARMED and signed motor PWM. A changing tilt alone proves sensor updates, not that PID owns the motors. Target zero is the upright target relative to BALANCE_UPRIGHT_DEG. "Hold within 5 degrees" means arming was refused, so forward/backward remain disabled. Arm, the other modes, and motor-off remain available while connected and unarmed. Steering stays disabled in balance mode.

If the robot is physically upright but displayed tilt is outside the arming window, verify sensor mounting and the intended pitch axis. For a modest upright calibration correction, the raw measured angle equals the current BALANCE_UPRIGHT_DEG plus displayed Tilt. Measure consistently at the intended balance posture before changing the target; do not bypass the arming or fall limits.

FIFO backlog detection counts complete packets, allowing two complete readings and an incomplete third, and reads at most the two complete packets. Three complete packets still cause a reset and an armed controller is switched off.

Drive tuning trial: requested lean increased from 1 to 2 degrees after the robot received forward/backward requests but showed no useful wheel motion. Gains and the target slew rate are unchanged. This is a hardware tuning trial, not a confirmed fix. At 2 degrees per second, reaching the full 2-degree target takes about one second. Observe signed Motor PWM while testing; nonzero PWM with stationary wheels suggests checking available torque, supply, and mechanical resistance. Do not add a constant motor offset that bypasses angle control.
