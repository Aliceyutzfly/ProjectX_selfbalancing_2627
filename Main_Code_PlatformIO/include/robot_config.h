#pragma once

#include <Arduino.h>

// Wiring

constexpr int MOTOR_IN1 = D0;  // GPIO2, DRV8833 AIN1
constexpr int MOTOR_IN2 = D1;  // GPIO3, DRV8833 AIN2
constexpr int MOTOR_IN3 = D2;  // GPIO4, DRV8833 BIN1
constexpr int MOTOR_IN4 = D3;  // GPIO5, DRV8833 BIN2

constexpr int PIN_TRIG = D10;  // GPIO10
constexpr int PIN_ECHO = D7;   // GPIO20; level-shift the HC-SR04 Echo signal to 3.3 V.

constexpr int MPU_SDA = D4;  // GPIO6
constexpr int MPU_SCL = D5;  // GPIO7
constexpr int MPU_INT = D6;  // GPIO21
constexpr uint8_t BALANCE_IMU_ADDRESS = 0x68;

// Wi-Fi access point

constexpr char AP_SSID[] = "XIAO-Robot";
constexpr char AP_PASSWORD[] = "robotdrive";  // At least 8 characters.

// Driving

// Match wheel mounting and motor-lead polarity.
constexpr bool INVERT_LEFT = true;
constexpr bool INVERT_RIGHT = true;

// Forward/reverse use an S-ramp from this starting magnitude.
// Turns and release/STOP change the driving request immediately.
constexpr int DRIVE_START_PWM = 100;
constexpr uint32_t DRIVE_RAMP_MS = 1000;

// Obstacle avoidance

// Forward above the upper threshold; reverse below the lower threshold.
// Turn left between the thresholds, including both endpoints.
constexpr float FORWARD_DISTANCE_CM = 20.0f;
constexpr float REVERSE_DISTANCE_CM = 5.0f;
constexpr uint32_t SENSOR_INTERVAL_MS = 80;
constexpr uint32_t ECHO_TIMEOUT_US = 25000;

// Attitude control

// Angles use pitch + 180 degrees. Saved calibration overrides the default.
// These bounds limit the calibration input; they are not operating tilt cutoffs.
constexpr double DEFAULT_BALANCE_UPRIGHT_DEG = 178.0;
constexpr double MIN_UPRIGHT_DEG = 90.0;
constexpr double MAX_UPRIGHT_DEG = 270.0;

// Half-width of the angle band with no attitude correction; driving still works.
// A saved setting overrides the default. Zero disables the finite-width band.
constexpr double DEFAULT_DEAD_ZONE_DEG = 2.0;
constexpr double MAX_DEAD_ZONE_DEG = 90.0;

// Default PID gains; saved tuning overrides these values.
constexpr double BALANCE_KP = 21.0;
constexpr double BALANCE_KI = 0.0;  // Integral correction is disabled by default.
constexpr double BALANCE_KD = 0.8;

// Accepted ranges for wireless PID tuning.
constexpr double PID_MAX_KP = 200.0;
constexpr double PID_MAX_KI = 500.0;
constexpr double PID_MAX_KD = 50.0;

// Direction and maximum magnitude of the correction added to both wheels.
constexpr int BALANCE_OUTPUT_SIGN = -1;
constexpr int ATTITUDE_MAX_PWM = 255;

// Motor output

// Limit each mixed wheel command, then suppress magnitudes below the PWM dead
// zone. Exactly +/-150 passes. This is separate from the angle dead zone above.
constexpr int BALANCE_MAX_PWM = 255;
constexpr int MOTOR_PWM_DEAD_ZONE = 150;

// Advanced settings

// MPU6050 sensor offsets from the supplied PID test; not the upright target.
constexpr int BALANCE_GYRO_X_OFFSET = -479;
constexpr int BALANCE_GYRO_Y_OFFSET = 84;
constexpr int BALANCE_GYRO_Z_OFFSET = 15;
constexpr int BALANCE_ACCEL_Z_OFFSET = 1638;

// IMU task period, stale-reading timeout and maximum control-loop gap.
constexpr uint32_t BALANCE_PERIOD_MS = 10;
constexpr uint32_t BALANCE_IMU_TIMEOUT_MS = 100;
constexpr uint32_t BALANCE_MAX_LOOP_GAP_MS = 50;

constexpr uint32_t BALANCE_I2C_HZ = 400000;
constexpr uint16_t BALANCE_I2C_TIMEOUT_MS = 5;

// Watchdog for browser heartbeats, held commands and drive-request updates.
constexpr uint32_t CONTROL_TIMEOUT_MS = 800;

// Compile-time validation

static_assert(DRIVE_START_PWM >= 0 && DRIVE_START_PWM <= 255,
              "Start PWM must be 0..255");
static_assert(DRIVE_RAMP_MS > 0 && DRIVE_RAMP_MS <= 5000,
              "Drive ramp must be 1..5000 ms");
static_assert(REVERSE_DISTANCE_CM > 0 &&
                  REVERSE_DISTANCE_CM < FORWARD_DISTANCE_CM,
              "Reverse distance must be smaller than forward distance");
static_assert(MOTOR_PWM_DEAD_ZONE >= 0 && MOTOR_PWM_DEAD_ZONE <= 255,
              "Check PWM dead zone");
static_assert(ATTITUDE_MAX_PWM > 0 && ATTITUDE_MAX_PWM <= BALANCE_MAX_PWM,
              "Check correction limit");
static_assert(BALANCE_OUTPUT_SIGN == -1 || BALANCE_OUTPUT_SIGN == 1,
              "Balance sign must be -1 or 1");
static_assert(BALANCE_MAX_PWM > 0 && BALANCE_MAX_PWM <= 255,
              "Check balance limits");
static_assert(BALANCE_PERIOD_MS > 0 &&
                  BALANCE_PERIOD_MS < BALANCE_MAX_LOOP_GAP_MS &&
                  BALANCE_MAX_LOOP_GAP_MS < BALANCE_IMU_TIMEOUT_MS,
              "Check balance timing");
