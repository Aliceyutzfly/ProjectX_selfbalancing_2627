// IMU sampling, PID correction and final motor output.
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
double deadZone = DEFAULT_DEAD_ZONE_DEG;
double uprightAngle = DEFAULT_BALANCE_UPRIGHT_DEG;
double input = uprightAngle, output = 0, target = uprightAngle;
PID pid(&input, &output, &target, BALANCE_KP, BALANCE_KI, BALANCE_KD, DIRECT);
SemaphoreHandle_t stateMutex = nullptr;
void (*motorWriter)(int, int) = nullptr;
bool initialized = false, armed = false, angleValid = false;
volatile bool imuInterrupt = false;
uint16_t packetSize = 0;
uint8_t fifoBuffer[64];
uint32_t lastSample = 0;
unsigned goodSamples = 0;
int motorPWM = 0;
int baseLeft = 0, baseRight = 0, actualLeft = 0, actualRight = 0;
bool outputLimited = false;
uint32_t lastDriveUpdate = 0;
const char* message = "IMU not initialized";

// Interrupt means new data is available, not that the body crossed a tilt limit.
// Sensor reads and motor updates happen in the task, never inside this ISR.
void IRAM_ATTR dataReady() {
  imuInterrupt = true;
}

// Caller holds the controller mutex (or runs before task startup).
// Automatic mode reinitializes PID history when control resumes.
void resetPIDLocked() {
  pid.SetMode(MANUAL);
  output = 0;
  motorPWM = 0;
}

// Called with the mutex held. Faults latch motors OFF until an explicit re-arm.
void disarmLocked(const char* reason) {
  armed = false;
  target = uprightAngle;
  resetPIDLocked();
  baseLeft = baseRight = actualLeft = actualRight = 0;
  outputLimited = false;
  if (motorWriter)
    motorWriter(0, 0);
  message = reason;
}

void sensorFault(const char* reason) {
  angleValid = false;
  goodSamples = 0;
  if (armed)
    disarmLocked(reason);
  else
    message = reason;
}

// Return immediately if no COMPLETE packet is ready; never wait in a FIFO loop.
bool readAngle() {
  uint8_t countBytes[2];
  if (I2Cdev::readBytes(BALANCE_IMU_ADDRESS, MPU6050_RA_FIFO_COUNTH, 2, countBytes,
                        BALANCE_I2C_TIMEOUT_MS) != 2) {
    sensorFault("IMU read failed - motors off");
    return false;
  }
  const uint16_t count = (uint16_t(countBytes[0]) << 8) | countBytes[1];
  // A backlog means the data is too old for balancing. Drop it, rather than
  // repeatedly controlling motors from old samples. Require explicit re-arm.
  // Count complete packets; an incomplete next packet is not another reading.
  if (count >= 1024 || count / packetSize > 2) {
    if (armed)
      disarmLocked("IMU backlog - motors off");
    angleValid = false;
    goodSamples = 0;
    mpu.resetFIFO();
    return false;
  }
  if (count < packetSize)
    return false;
  // At most TWO packets, already known to exist; use the newest complete one.
  const uint16_t packets = count / packetSize;
  for (uint16_t i = 0; i < packets; ++i) {
    if (I2Cdev::readBytes(BALANCE_IMU_ADDRESS, MPU6050_RA_FIFO_R_W, packetSize, fifoBuffer,
                          BALANCE_I2C_TIMEOUT_MS) != packetSize) {
      sensorFault("IMU packet failed - motors off");
      return false;
    }
  }
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  const float norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
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
  if (now - lastSample > BALANCE_FRESH_SAMPLE_MS)
    goodSamples = 0;
  input = measured;
  lastSample = now;
  angleValid = true;
  if (goodSamples < BALANCE_REQUIRED_SAMPLES)
    ++goodSamples;
  return true;
}

// Angle dead zone affects only attitude correction, never base driving.
void updateAttitudeCorrection() {
  target = uprightAngle;
  if (fabs(input - target) <= deadZone) {
    resetPIDLocked();
    return;
  }
  pid.SetMode(AUTOMATIC);
  if (pid.Compute()) {
    motorPWM =
        constrain(int(lround(output * BALANCE_OUTPUT_SIGN)), -ATTITUDE_MAX_PWM, ATTITUDE_MAX_PWM);
  }
}

// Mix first, saturate second, then suppress small final wheel outputs.
// Output-limited telemetry reports saturation; wheel telemetry includes filtering.
void writeMixedMotorOutputs() {
  const int mixedLeft = baseLeft + motorPWM;
  const int mixedRight = baseRight + motorPWM;
  actualLeft = constrain(mixedLeft, -BALANCE_MAX_PWM, BALANCE_MAX_PWM);
  actualRight = constrain(mixedRight, -BALANCE_MAX_PWM, BALANCE_MAX_PWM);
  outputLimited = actualLeft != mixedLeft || actualRight != mixedRight;
  if (abs(actualLeft) < MOTOR_PWM_DEAD_ZONE)
    actualLeft = 0;
  if (abs(actualRight) < MOTOR_PWM_DEAD_ZONE)
    actualRight = 0;
  motorWriter(actualLeft, actualRight);
}

// Called only for a fresh IMU sample while the controller is enabled.
void updateBalance(uint32_t now) {
  if (now - lastDriveUpdate > CONTROL_TIMEOUT_MS) {
    disarmLocked("Drive updates timed out - motors off");
    return;
  }
  updateAttitudeCorrection();
  writeMixedMotorOutputs();
  message = "Drive plus attitude correction";
}

// One periodic task keeps balancing independent of HTTP and WebSocket waits.
// The mutex serializes published requests, settings, and emergency shutdown.
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
    } else if (armed && fresh)
      updateBalance(afterRead);
    xSemaphoreGive(stateMutex);
    vTaskDelayUntil(&wakeTime, pdMS_TO_TICKS(BALANCE_PERIOD_MS));
  }
}

} // namespace

bool balanceBegin(void (*writeMotors)(int, int)) {
  motorWriter = writeMotors;
  motorWriter(0, 0);
  stateMutex = xSemaphoreCreateMutex();
  if (!stateMutex) {
    message = "Balance memory allocation failed";
    return false;
  }
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(BALANCE_I2C_HZ);
  Wire.setTimeOut(BALANCE_I2C_TIMEOUT_MS);
  I2Cdev::readTimeout = BALANCE_I2C_TIMEOUT_MS;
  mpu.initialize();
  if (!mpu.testConnection()) {
    message = "MPU6050 not found - check wiring";
    return false;
  }
  if (mpu.dmpInitialize() != 0) {
    message = "MPU6050 DMP initialization failed";
    return false;
  }
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
  pid.SetOutputLimits(-ATTITUDE_MAX_PWM, ATTITUDE_MAX_PWM);
  pid.SetMode(MANUAL);
  pinMode(MPU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(MPU_INT), dataReady, RISING);
  mpu.resetFIFO();
  mpu.setDMPEnabled(true);
  initialized = true;
  lastSample = millis(); // Give the first packet a full freshness interval.
  message = "Ready - drive plus attitude correction";
  if (xTaskCreate(balanceTask, "balance", BALANCE_TASK_STACK_BYTES, nullptr, BALANCE_TASK_PRIORITY,
                  nullptr) != pdPASS) {
    initialized = false;
    detachInterrupt(digitalPinToInterrupt(MPU_INT));
    mpu.setDMPEnabled(false);
    message = "Could not start balance task";
    return false;
  }
  return true;
}

bool balanceArm() {
  if (!stateMutex)
    return false;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  bool accepted = false;
  if (!initialized) { /* Preserve the initialization failure message. */
  } else if (!angleValid || goodSamples < BALANCE_REQUIRED_SAMPLES ||
             millis() - lastSample > BALANCE_FRESH_SAMPLE_MS)
    message = "Wait for fresh IMU readings, then press STOP";
  else if (armed) {
    accepted = true;
  } else {
    target = uprightAngle;
    resetPIDLocked();
    pid.SetMode(AUTOMATIC); // Reset accumulated correction on every explicit arm.
    lastDriveUpdate = millis();
    armed = true;
    message = "Drive plus attitude correction";
    accepted = true;
  }
  xSemaphoreGive(stateMutex);
  return accepted;
}

void balanceDisarm() {
  if (!stateMutex) {
    if (motorWriter)
      motorWriter(0, 0);
    return;
  }
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  disarmLocked(initialized ? "Controller off - motors stopped" : message);
  xSemaphoreGive(stateMutex);
}

BalanceStatus balanceStatus() {
  if (stateMutex)
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  BalanceStatus state{initialized && angleValid && goodSamples >= BALANCE_REQUIRED_SAMPLES &&
                          millis() - lastSample <= BALANCE_FRESH_SAMPLE_MS,
                      armed,
                      angleValid,
                      input,
                      target,
                      motorPWM,
                      message,
                      baseLeft,
                      baseRight,
                      actualLeft,
                      actualRight,
                      outputLimited,
                      pid.GetKp(),
                      pid.GetKi(),
                      pid.GetKd(),
                      deadZone};
  if (stateMutex)
    xSemaphoreGive(stateMutex);
  return state;
}

// Publishing a request never writes a motor or enables the controller.
void balanceSetDrive(int left, int right) {
  if (!stateMutex)
    return;
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  baseLeft = constrain(left, -BALANCE_MAX_PWM, BALANCE_MAX_PWM);
  baseRight = constrain(right, -BALANCE_MAX_PWM, BALANCE_MAX_PWM);
  lastDriveUpdate = millis();
  xSemaphoreGive(stateMutex);
}

bool balanceSetUpright(double degrees) {
  if (!isfinite(degrees) || degrees < MIN_UPRIGHT_DEG || degrees > MAX_UPRIGHT_DEG)
    return false;
  if (stateMutex)
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  const bool accepted = !armed;
  if (accepted) {
    uprightAngle = target = degrees;
    resetPIDLocked();
  }
  if (stateMutex)
    xSemaphoreGive(stateMutex);
  return accepted;
}

bool balanceValidTunings(double kp, double ki, double kd) {
  return isfinite(kp) && isfinite(ki) && isfinite(kd) && kp >= 0 && kp <= PID_MAX_KP && ki >= 0 &&
         ki <= PID_MAX_KI && kd >= 0 && kd <= PID_MAX_KD;
}

bool balanceSetTunings(double kp, double ki, double kd) {
  if (!balanceValidTunings(kp, ki, kd))
    return false;
  if (stateMutex)
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  const bool accepted = !armed;
  if (accepted) {
    resetPIDLocked();
    pid.SetTunings(kp, ki, kd);
  }
  if (stateMutex)
    xSemaphoreGive(stateMutex);
  return accepted;
}

bool balanceSetDeadZone(double degrees) {
  if (!isfinite(degrees) || degrees < 0 || degrees > MAX_DEAD_ZONE_DEG)
    return false;
  if (stateMutex)
    xSemaphoreTake(stateMutex, portMAX_DELAY);
  const bool accepted = !armed;
  if (accepted) {
    deadZone = degrees;
    resetPIDLocked();
  }
  if (stateMutex)
    xSemaphoreGive(stateMutex);
  return accepted;
}
