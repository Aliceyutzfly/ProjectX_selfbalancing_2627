#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <math.h>
#include "robot_config.h"
#include "balance.h"

namespace {
MPU6050 mpu(BALANCE_IMU_ADDRESS);
double input = BALANCE_UPRIGHT_DEG, output = 0, target = BALANCE_UPRIGHT_DEG;
PID pid(&input, &output, &target, BALANCE_KP, BALANCE_KI, BALANCE_KD, DIRECT);
SemaphoreHandle_t stateMutex = nullptr;
void (*motorWriter)(int, int) = nullptr;
bool initialized = false, armed = false, angleValid = false;
volatile bool imuInterrupt = false;
uint16_t packetSize = 0;
uint8_t fifoBuffer[64];
uint32_t lastSample = 0, lastCommand = 0, lastTargetUpdate = 0;
unsigned goodSamples = 0;
int requestedDirection = 0, motorPWM = 0;
const char* message = "IMU not initialized";

// Interrupt means new data is available, not that the body crossed a tilt limit.
// Sensor reads and motor updates happen in the task, never inside this ISR.
void IRAM_ATTR dataReady() { imuInterrupt = true; }

// Called with the mutex held. Faults latch motors OFF until an explicit re-arm.
void disarmLocked(const char* reason) {
  armed = false;
  requestedDirection = 0;
  target = BALANCE_UPRIGHT_DEG;
  output = 0;
  motorPWM = 0;
  pid.SetMode(MANUAL);
  if (motorWriter) motorWriter(0, 0);
  message = reason;
}

void sensorFault(const char* reason) {
  angleValid = false;
  goodSamples = 0;
  if (armed) disarmLocked(reason);
  else message = reason;
}

// Return immediately if no COMPLETE packet is ready; never wait in a FIFO loop.
bool readAngle() {
  uint8_t countBytes[2];
  if (I2Cdev::readBytes(BALANCE_IMU_ADDRESS, MPU6050_RA_FIFO_COUNTH,
                      2, countBytes, BALANCE_I2C_TIMEOUT_MS) != 2) {
    sensorFault("IMU read failed - motors off");
    return false;
  }
  const uint16_t count = (uint16_t(countBytes[0]) << 8) | countBytes[1];
  // A backlog means the data is too old for balancing. Drop it, rather than
  // repeatedly controlling motors from old samples. Require explicit re-arm.
  // Count complete packets; an incomplete next packet is not another reading.
  if (count >= 1024 || count / packetSize > 2) {
    if (armed) disarmLocked("IMU backlog - motors off");
    angleValid = false;
    goodSamples = 0;
    mpu.resetFIFO();
    return false;
  }
  if (count < packetSize) return false;
  // At most TWO packets, already known to exist; use the newest complete one.
  const uint16_t packets = count / packetSize;
  for (uint16_t i = 0; i < packets; ++i) {
    if (I2Cdev::readBytes(BALANCE_IMU_ADDRESS, MPU6050_RA_FIFO_R_W,
                        packetSize, fifoBuffer, BALANCE_I2C_TIMEOUT_MS) != packetSize) {
      sensorFault("IMU packet failed - motors off");
      return false;
    }
  }
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  const float norm = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
  if (!isfinite(norm) || norm < 0.8f || norm > 1.2f) {
    sensorFault("Invalid IMU data - motors off");
    return false;
  }
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  const double measured = ypr[1] * 180.0 / PI + 180.0;
  if (!isfinite(measured)) {
    sensorFault("Invalid angle - motors off");
    return false;
  }
  const uint32_t now = millis();
  if (now - lastSample > 30) goodSamples = 0;
  input = measured;
  lastSample = now;
  angleValid = true;
  if (goodSamples < 10) ++goodSamples;
  return true;
}

// The teaching logic: measure angle -> choose lean target -> PID -> both wheels.
void updateBalance(uint32_t now) {
  if (fabs(input - BALANCE_UPRIGHT_DEG) > BALANCE_FALL_LIMIT_DEG) {
    disarmLocked("Tilt limit - support robot and re-arm");
    return;
  }
  // This timeout lives in the balance task too, so a slow web request cannot
  // leave the robot driving on an old command. Upright balance stays enabled.
  if (now - lastCommand > CONTROL_TIMEOUT_MS) requestedDirection = 0;
  const double desired = BALANCE_UPRIGHT_DEG + requestedDirection *
                         BALANCE_FORWARD_LEAN_SIGN * BALANCE_DRIVE_LEAN_DEG;
  double step = BALANCE_TARGET_RATE_DEG_S * (now - lastTargetUpdate) / 1000.0;
  lastTargetUpdate = now;
  if (step > 0.1) step = 0.1;  // Do not jump the target after a scheduling delay.
  if (target < desired) target = fmin(target + step, desired);
  else if (target > desired) target = fmax(target - step, desired);

  if (pid.Compute()) {
    motorPWM = constrain(int(lround(output * BALANCE_OUTPUT_SIGN)),
                         -BALANCE_MAX_PWM, BALANCE_MAX_PWM);
    // No steering in this first version. Both wheels get the same correction.
    motorWriter(motorPWM, motorPWM);
  }
  message = requestedDirection > 0 ? "Balance - forward lean" :
            requestedDirection < 0 ? "Balance - backward lean" : "Balancing upright";
}

// One periodic task keeps balancing independent of HTTP and WebSocket waits.
// A mutex prevents mode changes and this task writing the motors simultaneously.
void balanceTask(void*) {
  TickType_t wakeTime = xTaskGetTickCount();
  uint32_t lastRun = millis(), lastPoll = 0;
  for (;;) {
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    const uint32_t now = millis();
    if (armed && now - lastRun > BALANCE_MAX_LOOP_GAP_MS)
      disarmLocked("Control loop delayed - motors off");
    lastRun = now;
    const bool signaled = imuInterrupt;
    imuInterrupt = false;
    // Poll as a fallback if an interrupt edge was missed. No busy waiting.
    bool fresh = false;
    if (signaled || now - lastPoll >= BALANCE_PERIOD_MS) {
      lastPoll = now;
      fresh = readAngle();
    }
    const uint32_t afterRead = millis();
    if (armed && afterRead - now > BALANCE_MAX_LOOP_GAP_MS) {
      disarmLocked("Sensor read delayed - motors off");
    }
    if (afterRead - lastSample > BALANCE_IMU_TIMEOUT_MS) {
      sensorFault("IMU timeout - motors off");
    } else if (armed && fresh) updateBalance(afterRead);
    xSemaphoreGive(stateMutex);
    vTaskDelayUntil(&wakeTime, pdMS_TO_TICKS(BALANCE_PERIOD_MS));
  }
}
} // namespace

bool balanceBegin(void (*writeMotors)(int, int)) {
  motorWriter = writeMotors;
  motorWriter(0, 0);
  stateMutex = xSemaphoreCreateMutex();
  if (!stateMutex) { message = "Balance memory allocation failed"; return false; }
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(BALANCE_I2C_HZ);
  Wire.setTimeOut(BALANCE_I2C_TIMEOUT_MS);
  I2Cdev::readTimeout = BALANCE_I2C_TIMEOUT_MS;
  mpu.initialize();
  if (!mpu.testConnection()) { message = "MPU6050 not found - check wiring"; return false; }
  if (mpu.dmpInitialize() != 0) { message = "MPU6050 DMP initialization failed"; return false; }
  mpu.setXGyroOffset(BALANCE_GYRO_X_OFFSET);
  mpu.setYGyroOffset(BALANCE_GYRO_Y_OFFSET);
  mpu.setZGyroOffset(BALANCE_GYRO_Z_OFFSET);
  mpu.setZAccelOffset(BALANCE_ACCEL_Z_OFFSET);
  packetSize = mpu.dmpGetFIFOPacketSize();
  if (!packetSize || packetSize > sizeof(fifoBuffer)) {
    message = "Unsupported IMU packet size";
    return false;
  }
  pid.SetSampleTime(BALANCE_PERIOD_MS);
  pid.SetOutputLimits(-BALANCE_MAX_PWM, BALANCE_MAX_PWM);
  pid.SetMode(MANUAL);
  pinMode(MPU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INT), dataReady, RISING);
  mpu.resetFIFO();
  mpu.setDMPEnabled(true);
  initialized = true;
  lastSample = millis();  // Give the first packet a full freshness interval.
  message = "Hold upright, then select Balance + drive";
  if (xTaskCreate(balanceTask, "balance", 6144, nullptr, 3, nullptr) != pdPASS) {
    initialized = false;
    detachInterrupt(digitalPinToInterrupt(MPU_INT));
    mpu.setDMPEnabled(false);
    message = "Could not start balance task";
    return false;
  }
  return true;
}

bool balanceArm() {
  if (!stateMutex) return false;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  bool accepted = false;
  if (!initialized) { /* Preserve the initialization failure message. */ }
  else if (!angleValid || goodSamples < 10 || millis() - lastSample > 30)
    message = "Wait for fresh IMU readings, then re-arm";
  else if (fabs(input - BALANCE_UPRIGHT_DEG) > BALANCE_ARM_WINDOW_DEG)
    message = "Hold within 20 degrees of upright to arm";
  else if (armed) { requestedDirection = 0; accepted = true; }
  else {
    target = BALANCE_UPRIGHT_DEG;
    output = 0;
    requestedDirection = 0;
    motorPWM = 0;
    lastCommand = lastTargetUpdate = millis();
    pid.SetMode(MANUAL);
    pid.SetMode(AUTOMATIC);  // Reset accumulated correction on every explicit arm.
    armed = true;
    message = "Balancing upright";
    accepted = true;
  }
  xSemaphoreGive(stateMutex);
  return accepted;
}

void balanceDisarm() {
  if (!stateMutex) { if (motorWriter) motorWriter(0, 0); return; }
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  disarmLocked(initialized ? "Balance off - motors stopped" : message);
  xSemaphoreGive(stateMutex);
}

void balanceDrive(int direction) {
  if (!stateMutex) return;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  if (armed) {
    requestedDirection = constrain(direction, -1, 1);
    lastCommand = millis();
  }
  xSemaphoreGive(stateMutex);
}

BalanceStatus balanceStatus() {
  if (stateMutex) xSemaphoreTake(stateMutex, portMAX_DELAY);
  BalanceStatus state {initialized && angleValid && goodSamples >= 10 && millis() - lastSample <= 30,
                       armed, angleValid, input, target, motorPWM, message};
  if (stateMutex) xSemaphoreGive(stateMutex);
  return state;
}

void balanceWriteManual(int left, int right) {
  if (!motorWriter) return;
  if (stateMutex) xSemaphoreTake(stateMutex, portMAX_DELAY);
  if (!armed) motorWriter(left, right);
  if (stateMutex) xSemaphoreGive(stateMutex);
}
