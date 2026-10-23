#pragma once
#include <Arduino.h>

// A copy of the balance state, safe to read from the webpage/main loop.
struct BalanceStatus {
  bool ready;       // Initialized and receiving consecutive fresh measurements.
  bool armed;       // The balance controller currently owns both motors.
  bool angleValid;
  double angle;     // Same degrees + 180 convention as PID_test.cpp.
  double target;
  int pwm;
  const char* message;
};

// Initialize the MPU6050 and its background control loop. Motors remain off.
bool balanceBegin(void (*writeMotors)(int left, int right));
// Arm only with fresh readings and the body within the upright window.
bool balanceArm();
// Explicit power-off: remove motor drive, including after a fall/fault.
void balanceDisarm();
BalanceStatus balanceStatus();
// Manual/obstacle output is ignored while the balance controller is armed.
void balanceWriteManual(int left, int right);
