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

// Both modes share S-shaped ramps for forward/backward and normal STOP.
constexpr uint32_t DRIVE_RAMP_MS = 1000;
static_assert(DRIVE_RAMP_MS > 0 && DRIVE_RAMP_MS <= 5000, "Drive ramp must be 1..5000 ms");
// Normal release/STOP ramps down; emergency off and timeouts stay immediate.
constexpr uint32_t STOP_RAMP_MS = 1000;
static_assert(STOP_RAMP_MS > 0 && STOP_RAMP_MS <= 5000,
              "Normal slowdown must be 1..5000 ms");

constexpr uint32_t SENSOR_INTERVAL_MS = 80; // Sample about 12.5 times per second; test stopping clearance on your robot.
constexpr uint32_t ECHO_TIMEOUT_US = 25000;

// Forward above 20 cm; reverse below 5 cm; turn left from 5 through 20 cm.
constexpr float FORWARD_DISTANCE_CM = 20.0f;
constexpr float REVERSE_DISTANCE_CM = 5.0f;
static_assert(REVERSE_DISTANCE_CM > 0 && REVERSE_DISTANCE_CM < FORWARD_DISTANCE_CM,
              "Reverse distance must be smaller than forward distance");
