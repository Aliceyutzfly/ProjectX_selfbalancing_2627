#pragma once
#include <Arduino.h>

// Pin Configurations
constexpr int MOTOR_IN1 = D0;  // GPIO2, DRV8833 AIN1
constexpr int MOTOR_IN2 = D1;  // GPIO3, DRV8833 AIN2
constexpr int MOTOR_IN3 = D2;  // GPIO4, DRV8833 BIN1
constexpr int MOTOR_IN4 = D3;  // GPIO5, DRV8833 BIN2
constexpr int MPU_SDA = D4;    // GPIO6, reserved for balancing
constexpr int MPU_SCL = D5;    // GPIO7
constexpr int MPU_INT = D6;    // GPIO21; Serial must use USB CDC, not UART TX
constexpr int PIN_TRIG = D10;  // GPIO10

constexpr int PIN_ECHO = D7;   // GPIO20: Echo must still be level-shifted

constexpr bool INVERT_LEFT = true;
constexpr bool INVERT_RIGHT = true;

// Wifi Access Point Configuration
constexpr char AP_SSID[] = "XIAO-Robot";
constexpr char AP_PASSWORD[] = "robotdrive";  // At least 8 characters.

constexpr uint32_t CONTROL_TIMEOUT_MS = 800;

// Manual startup uses an S-shaped PWM rise: gentle, faster, then gentle again.
constexpr uint32_t MANUAL_RAMP_MS = 1000;
static_assert(MANUAL_RAMP_MS > 0 && MANUAL_RAMP_MS <= 5000, "Manual ramp must be 1..5000 ms");
// Normal release/STOP ramps down; emergency off and timeouts stay immediate.
constexpr uint32_t MANUAL_STOP_RAMP_MS = 1000;
static_assert(MANUAL_STOP_RAMP_MS > 0 && MANUAL_STOP_RAMP_MS <= 5000,
              "Normal slowdown must be 1..5000 ms");

constexpr uint32_t SENSOR_INTERVAL_MS = 80; // Sample about 12.5 times per second; test stopping clearance on your robot.
constexpr uint32_t ECHO_TIMEOUT_US = 25000;

// At 20 cm or closer, turn; above 20 cm, continue forward.
constexpr float OBSTACLE_CM = 20.0f;

// Balance settings copied from your standalone PID_test.cpp.
// Angle convention: pitch in degrees + 180; upright was measured as 178.
constexpr double BALANCE_UPRIGHT_DEG = 178.0;
constexpr double BALANCE_KP = 21.0;
constexpr double BALANCE_KI = 140.0;
constexpr double BALANCE_KD = 0.8;
constexpr int BALANCE_GYRO_X_OFFSET = -479;
constexpr int BALANCE_GYRO_Y_OFFSET = 84;
constexpr int BALANCE_GYRO_Z_OFFSET = 15;
constexpr int BALANCE_ACCEL_Z_OFFSET = 1638;
constexpr uint8_t BALANCE_IMU_ADDRESS = 0x68;

// The standalone test writes positive PID output directly to IN1/IN3.
// With BOTH wheel inversion flags true, -1 preserves that electrical polarity.
// Check correction direction while supporting the chassis before free balancing.
constexpr int BALANCE_OUTPUT_SIGN = -1;
constexpr int BALANCE_FORWARD_LEAN_SIGN = 1;
constexpr double BALANCE_DRIVE_LEAN_DEG = 2.0;   // First drive-tuning trial: 2-degree lean while held.
constexpr double BALANCE_TARGET_RATE_DEG_S = 2.0; // Smooth target changes.
constexpr double BALANCE_FALL_LIMIT_DEG = 20.0;
constexpr double BALANCE_ARM_WINDOW_DEG = BALANCE_FALL_LIMIT_DEG; // Same 20-degree limit for starting.
constexpr int BALANCE_MAX_PWM = 255;
constexpr uint32_t BALANCE_PERIOD_MS = 10;
constexpr uint32_t BALANCE_IMU_TIMEOUT_MS = 100;
constexpr uint32_t BALANCE_MAX_LOOP_GAP_MS = 50;
constexpr uint32_t BALANCE_I2C_HZ = 400000;
constexpr uint16_t BALANCE_I2C_TIMEOUT_MS = 5;

static_assert(BALANCE_OUTPUT_SIGN == 1 || BALANCE_OUTPUT_SIGN == -1, "Balance output sign must be +1 or -1");
static_assert(BALANCE_FORWARD_LEAN_SIGN == 1 || BALANCE_FORWARD_LEAN_SIGN == -1, "Lean sign must be +1 or -1");
static_assert(BALANCE_MAX_PWM > 0 && BALANCE_MAX_PWM <= 255, "Balance PWM must fit the driver range");
static_assert(BALANCE_DRIVE_LEAN_DEG > 0 && BALANCE_DRIVE_LEAN_DEG < BALANCE_ARM_WINDOW_DEG &&
              BALANCE_ARM_WINDOW_DEG <= BALANCE_FALL_LIMIT_DEG, "Keep drive lean < arm window <= fall limit");
static_assert(BALANCE_TARGET_RATE_DEG_S > 0, "Target slew rate must be positive");
static_assert(BALANCE_PERIOD_MS > 0 && BALANCE_PERIOD_MS < BALANCE_MAX_LOOP_GAP_MS &&
              BALANCE_MAX_LOOP_GAP_MS < BALANCE_IMU_TIMEOUT_MS, "Check balance timing limits");
