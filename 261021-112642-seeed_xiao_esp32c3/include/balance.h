#pragma once
#include <Arduino.h>

// Snapshot protected by the controller mutex. pwm is the attitude correction.
struct BalanceStatus {
  bool ready, armed, angleValid;
  double angle, target;
  int pwm;
  const char* message;
  int driveLeft, driveRight, outputLeft, outputRight;
  bool limited;
  double kp, ki, kd;
  double deadZone;
};

bool balanceBegin(void (*writeMotors)(int left, int right));
bool balanceArm(); // Enable the single output controller with fresh IMU data.
void balanceDisarm(); // Immediate motor-off; normal drive never writes hardware.
void balanceSetDrive(int left, int right); // Publish base wheel requests, not PWM hardware writes.
BalanceStatus balanceStatus();
bool balanceSetUpright(double degrees); // Only while disabled.
bool balanceValidTunings(double kp, double ki, double kd);
bool balanceSetTunings(double kp, double ki, double kd); // Only while disabled; resets PID history.
bool balanceSetDeadZone(double degrees); // Only while disabled; 0..90 degrees.
