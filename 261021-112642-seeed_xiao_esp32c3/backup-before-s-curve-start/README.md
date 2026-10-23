# XIAO ESP32-C3 robot - three modes over Wi-Fi AP

Updated 23 October 2026. Implemented modes: remote drive, simple 20 cm ultrasonic obstacle avoidance, and balance plus forward/backward drive. Read [BALANCE_GUIDE.md](BALANCE_GUIDE.md) before arming the new mode; it has not been physically validated by Codex.

## Connect and drive

1. Make the Echo wiring correction below before testing. Raise the wheels for the first motor test.
2. Open this project in PlatformIO. Build with `pio run`; upload with `pio run -t upload` after selecting the correct board/USB port. No filesystem upload is needed.
3. Open Serial Monitor at 115200 baud to see startup details.
4. Join **XIAO-Robot**, password **robotdrive**. Keep the connection if your phone warns that it has no Internet. If necessary, disable automatic switching to mobile data or another Wi-Fi network.
5. Open **http://192.168.4.1/** in a normal browser. This is HTTP, not HTTPS; there is no automatic captive portal.
6. Hold arrows to drive; release to stop. WASD/arrow keys also work. In ordinary drive/obstacle modes, STOP or Space ends movement and returns to remote drive. In BALANCE mode, release/Space requests upright while balance remains active; use the separate MOTORS OFF / DISARM control to cut drive.

The board creates its own Wi-Fi network. All page assets live in `include/web_page.h`; there are no external stylesheet, JavaScript, font, logo, or CDN dependencies. Only one Wi-Fi station and one controlling browser connection are allowed; close an old tab before opening another. No phone hotspot or mobile data is needed. STA can also serve a local page without Internet, but AP removes the need for an existing Wi-Fi network.

## Pin map and schematic review

The attached schematic's U6 is labelled **XIAO_esp32_s3**. This firmware targets **XIAO ESP32-C3**, matching the user's stated board and original PlatformIO configuration. Use the C3 symbol/pinout in the schematic. XIAO D-labels are board-pad names, not raw ESP GPIO numbers.

| Signal | XIAO pad | ESP32-C3 GPIO | Review |
| --- | --- | --- | --- |
| DRV8833 IN1 / AIN1 | D0 | 2 | Matches schematic; GPIO2 is a strapping pin, see below |
| DRV8833 IN2 / AIN2 | D1 | 3 | Matches schematic |
| DRV8833 IN3 / BIN1 | D2 | 4 | Matches schematic |
| DRV8833 IN4 / BIN2 | D3 | 5 | Matches schematic |
| MPU6050 SDA | D4 | 6 | Schematic correct; original code had SDA/SCL reversed |
| MPU6050 SCL | D5 | 7 | Schematic correct; used by balance.cpp |
| MPU6050 INT | D6 | 21 | Matches schematic; IMU data-ready interrupt, with USB CDC Serial |
| HC-SR04 Trig | D10 | 10 | Matches schematic |
| HC-SR04 Echo | **D7** | **20** | **Wire change: move from schematic D9 to D7, through divider** |

The original `PIN_BUTTON = D4` conflicted with SDA, and no external mode button appears in the supplied schematic. The floating input could switch modes unpredictably. The new firmware removes this button and uses the web page. D4/D5 are used for I2C; `balance.cpp` initializes the MPU6050 once at startup and reads fresh orientation in a periodic task.

### Echo voltage and boot pin — required wiring change

A standard 5 V HC-SR04 Echo output must not be connected directly to a 3.3 V ESP32 input. The supplied schematic shows no divider. If the real circuit already contains one, retain suitable level shifting and move its output to D7.

One suitable divider for a nominal regulated 5 V Echo signal is:

```text
HC-SR04 Echo ---- 2.2 kohm ----+---- XIAO D7 (GPIO20)
                             |
                          3.3 kohm
                             |
                            GND
```

The divider gives approximately `5 × 3.3 / (2.2 + 3.3) = 3.0 V`. Use the shared signal ground. Keep HC-SR04 VCC on its required 5 V supply. The 3.3 V trigger output usually works with standard HC-SR04 modules; verify your particular sensor if it does not respond.

D9 is GPIO9, also BOOT. A sensor output or divider that holds this pin low at reset can select download mode rather than normal boot. This is why the firmware uses the spare D7 pad, not merely a different numerical alias. Remove the old Echo connection from D9 entirely.

GPIO2/D0 is also a strapping pin and remains a motor input to minimize wiring changes. Check repeated cold starts with the driver connected. If an external load causes startup problems, review its reset-time level; a future layout can use spare D9/D8 only with careful strap handling, or free another non-strap pin by changing the interrupt arrangement. Do not add an arbitrary strong pull-up to a motor input: it could energize a wheel before firmware starts. Motor shutdown during reset requires appropriate hardware gating if that behavior matters.

### Items the schematic alone cannot confirm

- **DRV8833 EEP/SLEEP:** the schematic leaves it unconnected. If that pad is nSLEEP, it must be high for operation. The bare IC has an internal pull-down; modules may add their own pull-up. Check the actual module schematic/voltage. If no pull-up exists, a pull-up such as 10 kohm to 3.3 V is suitable for nSLEEP after confirming the pad identity. Leaving nFAULT unconnected is acceptable if fault reporting is not required.
- **MPU6050 breakout:** the schematic shows VCC = 3.3 V. Verify the module accepts that at its VCC pin; some breakouts have a regulator whose dropout must be considered. I2C pull-ups must resolve to 3.3 V at the ESP32. Check AD0 has a defined state, normally low/address 0x68. An onboard pull-down may already provide it. These details depend on the actual breakout.
- **Power:** the schematic puts the driver, ultrasonic sensor, and XIAO 5 V pad on the same net. It does not show the power source, available motor stall current, or all module capacitors. Do not power the motors from the XIAO 3V3 output. Use a motor-rated supply and common grounds, verify the N20 voltage/stall-current rating against the actual driver module, and provide appropriate local bypass/bulk capacitance at the driver. Avoid feeding an unprotected external 5 V source into USB VBUS; follow Seeed's external-power isolation guidance when USB is also connected.
- **Wheel identity/polarity:** code assumes bridge A is left and bridge B is right. The schematic identifies M1/M2 but cannot prove their physical positions or forward rotation. Verify with raised wheels. Change `INVERT_LEFT` or `INVERT_RIGHT` in `include/robot_config.h` if needed, or correct wiring with power removed.

## Behavior and settings

Manual startup uses quadratic ease-out: `fraction = 1 - (1 - t)^2`, where `t` is elapsed time divided by `MANUAL_RAMP_MS` (1000 ms). PWM rises quickly initially and gradually approaches the target; this is commanded PWM, not measured acceleration. Repeated held commands do not restart the ramp. New directions and resuming during slowdown ramp from the currently applied PWM. Normal stopping remains linear over `MANUAL_STOP_RAMP_MS` (1000 ms). The explicit slowdown has its own fixed deadline, so it can last one second without changing `CONTROL_TIMEOUT_MS` (800 ms). The heartbeat timeout and immediate motor-off remain active during slowdown. Repeated STOP does not restart its timer. Modes 1 and 2 do not apply balance PID output.

`include/robot_config.h` contains pins, AP credentials, wheel inversion, a 20 cm obstacle threshold. Existing saved calibration is retained under the original `servoCalib` preference namespace. Serial `L/l` changes left PWM, `R/r` changes right PWM, and `S` saves. Source default PWM is 200 of 255; trim values are clamped to 0–255. Changing trim in ordinary drive stops the robot and returns to manual mode. Serial trims are ignored in BALANCE mode, whose two wheels receive the same PID correction.

Manual mode requires refreshed drive commands. The page sends them every 150 ms while held. Firmware removes motor drive if commands stop for over 800 ms. A separate 150 ms page heartbeat keeps the controller session alive in all modes. In manual/obstacle mode, after over 800 ms without it the robot stops, returns to manual, and closes that connection. In BALANCE mode, connection or movement-command loss instead requests upright while balance stays active. Explicit DISARM or a balance fault cuts motor drive. These are loop-based timeouts, not a hard real-time guarantee; physical coast-down takes additional time. 00 on DRV8833 is coast, not active braking.

Obstacle mode samples about every 80 ms and uses a bounded Echo wait of up to 25 ms. With a valid distance greater than 20 cm, it drives forward. At 20 cm or closer, it turns on the spot using the fixed `turnRight` command until a valid reading exceeds 20 cm, then drives forward again. There is no reversing, timed escape sequence, or attempt counter. Actual turn direction depends on the physical motor assignments.

Missing Echo, readings below 2 cm, or readings above 400 cm stop autonomous movement. A valid reading can allow obstacle mode to resume. An empty space beyond the sensor's range is therefore treated as unknown rather than clear. STOP/disconnection returns to manual and requires explicit selection to restart autonomy. Manual mode intentionally allows driving regardless of ultrasonic readings.

A single front distance sensor cannot detect actual wheel stalls, stairs, or all angled/soft surfaces, and cannot guarantee escaping every corner. This simple rule may switch repeatedly near 20 cm when readings fluctuate. Test at low speed with adequate clearance.

## Hardware acceptance checklist

1. With motor power off, inspect Echo divider/D7, common grounds, driver sleep, board model, and power supply. Update the schematic to match.
2. Build/upload with wheels raised. At startup motors should remain stopped, even with no phone connected. Repeat several power cycles to expose strap/startup problems.
3. Connect to the AP with mobile data disabled. Confirm the page styles, buttons, mode display, and distance value load without Internet.
4. In remote mode, briefly hold Forward, Backward, Left, and Right. Confirm bridge A/B correspond to the correct wheel and correct inversion flags if needed. Release outside a button, cancel a touch, and use STOP; drive must stop.
5. While driving with wheels raised, close/hide the page or disconnect Wi-Fi. Confirm motor drive stops within approximately one second and does not resume on reconnect. Repeat during obstacle mode.
6. With wheels still raised, put a flat target at measured 10, 20, 30, and 50 cm distances. Confirm plausible values. Unplug sensor power with robot motor power disabled, then restore power for a no-Echo check: obstacle mode must stay stopped. Restore normal wiring before further tests.
7. On a clear flat floor, test obstacle mode at low PWM: a target beyond 20 cm should permit forward drive; at 20 cm or closer it should turn. Move the target away and confirm forward drive resumes. STOP and a missing Echo must still stop it. Adjust the threshold based on observed clearance.

## Verification completed

- One-second normal-stop deadline fix compiled successfully. Manual startup now uses a one-second quadratic ease-out ramp. Physical timing and motor response still need testing.
- Compilation/linking passed for XIAO ESP32-C3 with Espressif32 6.12.0 / Arduino ESP32 2.0.17, WebSockets 2.7.3, PID 1.2.1, and I2Cdevlib-MPU6050 1.0.0.
- Automated mocked-browser tests passed for original controls and balance-specific arming controls, disabled steering, upright requests, explicit disarm, fault clearing of held motion, and reconnect without stale motion.
- No firmware upload or physical balancing/timing test has been performed by Codex. The browser harness does not execute the IMU/PID/task code or simulate the chassis. Follow the supported hardware tests in [BALANCE_GUIDE.md](BALANCE_GUIDE.md).

## Balance plus remote drive

Implemented in `src/balance.cpp`, with settings in `include/robot_config.h` and an interface in `include/balance.h`. It uses one angle PID and a slowly changing forward/backward lean target; steering is disabled in this mode. The sensor/PID loop is separated from web handling. Ultrasonic waits are skipped in balance mode.

See [BALANCE_GUIDE.md](BALANCE_GUIDE.md) for operation, motor/lean direction checks, reused offsets/gains, fault limits, and hardware validation. Returning upright does not guarantee zero speed or position hold without encoders.

## Sources

- [Seeed XIAO ESP32-C3 pin map, strap pins, and power guidance](https://wiki.seeedstudio.com/XIAO_ESP32C3_Getting_Started/)
- [Espressif Arduino Wi-Fi API: Soft AP](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/wifi.html)
- [TI DRV8833 datasheet: input truth table, nSLEEP, and supply capacitance](https://www.ti.com/lit/gpn/drv8833)
- [Adafruit HC-SR04 wiring: reducing the 5 V Echo signal](https://learn.adafruit.com/distance-measurement-ultrasound-hcsr04/connect-the-sensor)
- [Original Breadboard Robot repository](https://github.com/Fleming-Society/Breadboard-Robot)
- [User's self-balancing reference](https://projecthub.arduino.cc/mircemk/arduino-two-weel-self-balancing-robot-edef61)
