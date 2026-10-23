#pragma once
#include <Arduino.h>

// Pin Configurations
constexpr int MOTOR_IN1 = D0;  // GPIO2, DRV8833 AIN1
constexpr int MOTOR_IN2 = D1;  // GPIO3, DRV8833 AIN2
constexpr int MOTOR_IN3 = D2;  // GPIO4, DRV8833 BIN1
constexpr int MOTOR_IN4 = D3;  // GPIO5, DRV8833 BIN2
constexpr int PIN_TRIG = D10;  // GPIO10

constexpr int PIN_ECHO = D7;   // GPIO20: Echo must still be level-shifted

constexpr bool INVERT_LEFT = true;
constexpr bool INVERT_RIGHT = true;

// Wifi Access Point Configuration
constexpr char AP_SSID[] = "XIAO-Robot";
constexpr char AP_PASSWORD[] = "robotdrive";  // At least 8 characters.

constexpr uint32_t CONTROL_TIMEOUT_MS = 800;

// Both modes ramp forward/backward only. Turns and release/STOP are immediate.
constexpr uint32_t DRIVE_RAMP_MS = 2000;
static_assert(DRIVE_RAMP_MS > 0 && DRIVE_RAMP_MS <= 5000, "Drive ramp must be 1..5000 ms");
constexpr uint32_t SENSOR_INTERVAL_MS = 80; // Sample about 12.5 times per second; test stopping clearance on your robot.
constexpr uint32_t ECHO_TIMEOUT_US = 25000;

// Forward above 20 cm; reverse below 5 cm; turn left from 5 through 20 cm.
constexpr float FORWARD_DISTANCE_CM = 20.0f;
constexpr float REVERSE_DISTANCE_CM = 5.0f;
static_assert(REVERSE_DISTANCE_CM > 0 && REVERSE_DISTANCE_CM < FORWARD_DISTANCE_CM,
              "Reverse distance must be smaller than forward distance");

// Upright balancing is a stopped behavior, not a third driving mode.
constexpr int MPU_SDA = D4;
constexpr int MPU_SCL = D5;
constexpr int MPU_INT = D6;
constexpr uint8_t BALANCE_IMU_ADDRESS = 0x68;
// Same angle convention, PID gains and offsets as the supplied PID test.
constexpr double BALANCE_UPRIGHT_DEG = 178.0;
constexpr double BALANCE_KP = 21.0;
constexpr double BALANCE_KI = 140.0;
constexpr double BALANCE_KD = 0.8;
constexpr int BALANCE_GYRO_X_OFFSET = -479;
constexpr int BALANCE_GYRO_Y_OFFSET = 84;
constexpr int BALANCE_GYRO_Z_OFFSET = 15;
constexpr int BALANCE_ACCEL_Z_OFFSET = 1638;
// Compensate for both inverted motor outputs to match the standalone test.
constexpr int BALANCE_OUTPUT_SIGN = -1;
constexpr double BALANCE_TILT_LIMIT_DEG = 20.0; // Both starting and fall limit.
constexpr int BALANCE_MAX_PWM = 255;
constexpr uint32_t BALANCE_PERIOD_MS = 10;
constexpr uint32_t BALANCE_IMU_TIMEOUT_MS = 100;
constexpr uint32_t BALANCE_MAX_LOOP_GAP_MS = 50;
constexpr uint32_t BALANCE_I2C_HZ = 400000;
constexpr uint16_t BALANCE_I2C_TIMEOUT_MS = 5;
static_assert(BALANCE_OUTPUT_SIGN == -1 || BALANCE_OUTPUT_SIGN == 1, "Balance sign must be -1 or 1");
static_assert(BALANCE_TILT_LIMIT_DEG > 0 && BALANCE_MAX_PWM > 0 && BALANCE_MAX_PWM <= 255, "Check balance limits");
static_assert(BALANCE_PERIOD_MS > 0 && BALANCE_PERIOD_MS < BALANCE_MAX_LOOP_GAP_MS &&
              BALANCE_MAX_LOOP_GAP_MS < BALANCE_IMU_TIMEOUT_MS, "Check balance timing");
