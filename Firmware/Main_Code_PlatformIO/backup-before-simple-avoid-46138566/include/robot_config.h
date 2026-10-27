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

constexpr uint32_t SENSOR_INTERVAL_MS = 80; // 80 ms = 12.5 Hz, fast enough to avoid collisions at 0.5 m/s.
constexpr uint32_t ECHO_TIMEOUT_US = 25000;

constexpr float OBSTACLE_CM = 30.0f;
constexpr uint32_t REVERSE_MS = 350;
constexpr uint32_t TURN_MS = 450;
constexpr uint32_t SETTLE_MS = 120;
