// Libraries: Arduino handles pins/timing; WiFi creates the network;
// WebServer serves the page; WebSockets carries live commands/status;
// Preferences saves wheel trims in flash. The two local headers hold
// hardware settings and the complete offline control page.
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include "robot_config.h"
#include "web_page.h"
#include "balance.h"

enum Mode { MANUAL, DISTANCE, CALIBRATION };
Mode currentMode = MANUAL;

// Create persistent-storage and network-server objects once.
// HTTP port 80 loads the page; WebSocket port 81 keeps controls responsive.
Preferences prefs;
Preferences imuPrefs;
bool imuStorageReady = false;
const char* calibrationMessage = "Hold upright, capture or enter an angle, then save";
WebServer httpServer(80);
WebSocketsServer wsServer(81);

// Shared runtime values. PWM magnitude is 0..255; each wheel has its
// own trim because two nominally identical motors may run at different speeds.
int leftPWM = 254, rightPWM = 254;
int controller = -1;  // One controlling browser at a time.
// millis() timestamps are milliseconds since boot. Subtract unsigned
// timestamps rather than comparing absolute deadlines, so wraparound works.
uint32_t lastHeartbeat = 0, lastDrive = 0, lastSensor = 0, lastStatus = 0;
float distanceCM = -1;
bool distanceValid = false;
// Both modes share one movement function and one ramp state.
enum Motion { STOPPED, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT };
Motion requestedMotion = STOPPED;
bool rampActive = false;
// Initially idle: start balancing when fresh data and upright posture allow it.
// Safety stops cancel this request; a normal STOP enables it again.
bool idleBalanceRequested = true, idleBalanceAttempted = false;
uint32_t rampStarted = 0, rampDuration = 0;
int rampStartLeft = 0, rampStartRight = 0;
int targetLeft = 0, targetRight = 0;
int appliedLeft = 0, appliedRight = 0;
const char* activity = "Stopped";

// Low-level wheel control. Positive = logical forward; negative =
// reverse; zero = no drive. The magnitude sets PWM duty cycle.
// Each DRV8833 bridge uses two inputs: PWM/0 forward, 0/PWM reverse.
// Inversion flags compensate for wheel mounting or motor-lead polarity.
void moveMotor(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  if (INVERT_LEFT) left = -left;
  if (INVERT_RIGHT) right = -right;
  // Clear the opposite leg first when changing direction.
  if (left >= 0) { analogWrite(MOTOR_IN2, 0); analogWrite(MOTOR_IN1, left); }
  else { analogWrite(MOTOR_IN1, 0); analogWrite(MOTOR_IN2, -left); }
  if (right >= 0) { analogWrite(MOTOR_IN4, 0); analogWrite(MOTOR_IN3, right); }
  else { analogWrite(MOTOR_IN3, 0); analogWrite(MOTOR_IN4, -right); }
}


// Immediate motor-off for faults, mode changes, and lost control.
// Cancelling the ramp prevents the next loop from restarting the motors.
void stopMotors() {
  rampActive = false;
  requestedMotion = STOPPED;
  rampStartLeft = rampStartRight = 0;
  targetLeft = targetRight = 0;
  appliedLeft = appliedRight = 0;
  balanceDisarm();
  balanceWriteManual(0, 0);
  idleBalanceRequested = false;
  idleBalanceAttempted = true;
  activity = "Stopped";
}

void stopAndManual() {
  stopMotors();
  currentMode = MANUAL;
}

// Enable upright PID as soon as a release/STOP cancels driving. Wait for a
// usable IMU and posture; after a successful attempt, faults never auto-rearm.
void serviceIdleBalance() {
  if (currentMode == CALIBRATION || !idleBalanceRequested || idleBalanceAttempted || rampActive ||
      requestedMotion != STOPPED) return;
  const BalanceStatus state = balanceStatus();
  if (!state.ready || fabs(state.angle - state.target) > BALANCE_TILT_LIMIT_DEG) return;
  idleBalanceAttempted = true;
  balanceArm();
}

const char* modeName() {
  if (currentMode == CALIBRATION) return "CALIBRATION";
  return currentMode == MANUAL ? "MANUAL" : "DISTANCE";
}

// JSON null means the sensor has no valid reading.
String statusJSON() {
  String json = "{\"mode\":\"";
  json += modeName();
  json += "\",\"distance_cm\":";
  json += distanceValid ? String(distanceCM, 1) : String("null");
  json += ",\"activity\":\"";
  const BalanceStatus balance = balanceStatus();
  json += balance.armed ? balance.message : activity;
  json += "\",\"balance_active\":";
  json += balance.armed ? "true" : "false";
  json += ",\"balance_ready\":";
  json += balance.ready ? "true" : "false";
  json += ",\"balance_pending\":";
  json += idleBalanceRequested && !idleBalanceAttempted ? "true" : "false";
  json += ",\"angle_deg\":";
  json += balance.angleValid ? String(balance.angle, 2) : String("null");
  json += ",\"target_deg\":";
  json += String(balance.target, 2);
  json += ",\"tilt_deg\":";
  json += balance.angleValid ? String(balance.angle - balance.target, 2) : String("null");
  json += ",\"balance_pwm\":";
  json += String(balance.pwm);
  json += ",\"balance_message\":\"";
  json += balance.message;
  json += "\",\"calibration_message\":\"";
  json += calibrationMessage;
  json += "\"}";
  return json;
}

// Shared by both modes: turns apply immediately, forward/backward ramp,
// and release/STOP hands control directly to upright balancing.
void requestMotion(Motion motion) {
  if (currentMode == CALIBRATION) return;
  if (motion == STOPPED) {
    if (balanceStatus().armed) return; // Repeated releases must not reset PID.
    stopMotors(); // Cancel any drive ramp/output before handing over ownership.
    idleBalanceRequested = true;
    idleBalanceAttempted = false;
    serviceIdleBalance(); // Starts now if IMU/posture allow it; otherwise waits off.
    return;
  }
  if (motion == requestedMotion) return; // Held commands never restart a ramp.
  if (motion != STOPPED && idleBalanceRequested) {
    balanceDisarm(); // Release PID ownership before ordinary motor control.
    idleBalanceRequested = false;
    idleBalanceAttempted = true;
  }
  const bool startingFromStop = requestedMotion == STOPPED;
  requestedMotion = motion;
  if (motion == FORWARD) { targetLeft = leftPWM; targetRight = rightPWM; activity = "Forward"; }
  else if (motion == BACKWARD) { targetLeft = -leftPWM; targetRight = -rightPWM; activity = "Reverse"; }
  else if (motion == TURN_LEFT) { targetLeft = -leftPWM; targetRight = rightPWM; activity = "Turn left"; }
  else if (motion == TURN_RIGHT) { targetLeft = leftPWM; targetRight = -rightPWM; activity = "Turn right"; }

  if (motion == TURN_LEFT || motion == TURN_RIGHT) {
    rampActive = false;
    appliedLeft = targetLeft;
    appliedRight = targetRight;
    balanceWriteManual(appliedLeft, appliedRight);
    return;
  }
  if (startingFromStop) {
    // Start forward/backward with useful drive, then S-ramp to each wheel's trim.
    // A lower saved trim must never be exceeded by the starting PWM.
    const int direction = motion == FORWARD ? 1 : -1;
    appliedLeft = direction * min(DRIVE_START_PWM, leftPWM);
    appliedRight = direction * min(DRIVE_START_PWM, rightPWM);
    balanceWriteManual(appliedLeft, appliedRight);
  }
  rampStartLeft = appliedLeft;
  rampStartRight = appliedRight;
  rampStarted = millis();
  rampDuration = DRIVE_RAMP_MS;
  rampActive = true;
}

// Named commands are accepted only by the manual WebSocket handler.
bool drive(const String& command) {
  if (command == "forward") requestMotion(FORWARD);
  else if (command == "backward") requestMotion(BACKWARD);
  else if (command == "turnLeft") requestMotion(TURN_LEFT);
  else if (command == "turnRight") requestMotion(TURN_RIGHT);
  else return false;
  return true;
}

// Smoothstep: gentle at both ends, faster in the middle. This shapes PWM;
// actual wheel acceleration also depends on the motors, power, and load.
void updateDriveMotion(uint32_t now) {
  if (!rampActive) return;
  const uint32_t elapsed = now - rampStarted;
  const float t = elapsed >= rampDuration ? 1.0f : float(elapsed) / float(rampDuration);
  const float fraction = t * t * (3.0f - 2.0f * t);
  const int left = rampStartLeft + int(lroundf((targetLeft - rampStartLeft) * fraction));
  const int right = rampStartRight + int(lroundf((targetRight - rampStartRight) * fraction));
  if (left != appliedLeft || right != appliedRight) {
    appliedLeft = left;
    appliedRight = right;
    balanceWriteManual(left, right);
  }
  if (elapsed >= rampDuration) {
    rampActive = false;
  }
}

// Button release/STOP exits autonomy and starts balancing without a slowdown.
void requestUserStop() {
  if (currentMode == CALIBRATION) { stopMotors(); return; }
  currentMode = MANUAL;
  requestMotion(STOPPED);
}

// A separate namespace keeps upright calibration independent of wheel trims.
void loadUprightCalibration() {
  imuStorageReady = imuPrefs.begin("imuCalib", false);
  const double saved = imuStorageReady ?
    imuPrefs.getDouble("upright", DEFAULT_BALANCE_UPRIGHT_DEG) : DEFAULT_BALANCE_UPRIGHT_DEG;
  if (!balanceSetUpright(saved)) balanceSetUpright(DEFAULT_BALANCE_UPRIGHT_DEG);
  if (!imuStorageReady) calibrationMessage = "Calibration storage unavailable - using default";
}

void enterCalibration() {
  stopAndManual(); // Cancels ramps and PID before enabling the settings page.
  currentMode = CALIBRATION;
  calibrationMessage = "Hold the body at its intended upright position, then capture and save";
}

void saveUprightCalibration(const String& value) {
  if (currentMode != CALIBRATION) return;
  const char* start = value.c_str();
  char* end = nullptr;
  const double degrees = strtod(start, &end);
  if (end == start || *end != '\0' || !isfinite(degrees) ||
      degrees < MIN_UPRIGHT_DEG || degrees > MAX_UPRIGHT_DEG) {
    calibrationMessage = "Enter a valid angle from 90 to 270 degrees";
    return;
  }
  const BalanceStatus state = balanceStatus();
  if (!state.ready || state.armed) {
    calibrationMessage = "Cannot save - fresh IMU readings and motors off are required";
    return;
  }
  if (!imuStorageReady) { calibrationMessage = "Cannot save - calibration storage unavailable"; return; }
  const double previous = state.target;
  if (!balanceSetUpright(degrees)) { calibrationMessage = "Cannot update upright angle"; return; }
  if (imuPrefs.putDouble("upright", degrees) != sizeof(double)) {
    balanceSetUpright(previous); // Do not claim success when flash storage failed.
    calibrationMessage = "Save failed - previous upright angle restored";
    return;
  }
  calibrationMessage = "Upright angle saved - retained after restart; motors remain off";
}

// WebSocket callback: runs when a browser connects, disconnects,
// or sends a message. Only the accepted controller may issue commands.
void webSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
  // Accept one controller, reset to a stopped state, then send its first status.
  if (type == WStype_CONNECTED) {
    if (controller >= 0) { wsServer.disconnect(client); return; }
    // Initial connection may arrive while the initially stopped robot balances.
    // Reconnect after a safety stop preserves its disabled state.
    if (requestedMotion != STOPPED || rampActive) stopAndManual();
    controller = client;
    lastHeartbeat = millis();
    String state = statusJSON();
    wsServer.sendTXT(client, state);
    return;
  }
  if (client != controller) return;
  // Disconnect immediately stops motion and returns to manual mode.
  if (type == WStype_DISCONNECTED) {
    controller = -1;
    stopAndManual();
    return;
  }
  if (type != WStype_TEXT) return;
  // Copy only the supplied bytes; do not assume a NUL-terminated payload.
  if (length == 0 || length > 32) { stopAndManual(); return; }
  String command;
  command.reserve(length);
  for (size_t i = 0; i < length; ++i) command += static_cast<char>(payload[i]); // Copy the data received from the WebSocket into a String.

  // Release/STOP requests balance immediately; motor-off disables everything.
  if (command == "stop") requestUserStop();
  else if (command == "heartbeat") lastHeartbeat = millis();
  else if (command == "motors:off") stopAndManual();
  else if (command == "mode:calibration") enterCalibration();
  else if (command.startsWith("imu:save:")) saveUprightCalibration(command.substring(9));
  else if (command == "mode:manual" || command == "mode:distance") {
    stopAndManual();
    if (command == "mode:distance") {
      currentMode = DISTANCE;
      distanceValid = false;
      lastSensor = millis() - SENSOR_INTERVAL_MS;
      activity = "Checking distance";
    } else requestUserStop();
  } else if (currentMode == MANUAL && drive(command)) {
    lastDrive = millis();
  } else if (currentMode == MANUAL) stopMotors();
  String state = statusJSON(); // Calls statusJSON() to build the robot’s status message, then stores that text in a variable called state.
  if (command != "heartbeat") wsServer.sendTXT(client, state);
}

// Read the ultrasonic sensor at the configured interval.
void sampleDistance(uint32_t now) {
  if (now - lastSensor < SENSOR_INTERVAL_MS) return;
  lastSensor = now;
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  const uint32_t duration = pulseInLong(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  //Trig pulse → sensor sends ultrasound → Echo goes HIGH → reflected sound returns → Echo goes LOW
  distanceCM = duration * 0.0343f / 2.0f;
  distanceValid = duration != 0 && distanceCM >= 2.0f && distanceCM <= 400.0f;
}

// Distance rules include exact boundaries: 5..20 cm turns left.
// No echo keeps the last motion; before any movement, default to forward.
void handleDistanceMode() {
  if (!distanceValid) {
    // Do not restart an existing ramp or replace the last distance decision.
    if (requestedMotion == STOPPED) requestMotion(FORWARD);
    return;
  }
  if (distanceCM > FORWARD_DISTANCE_CM) requestMotion(FORWARD);
  else if (distanceCM < REVERSE_DISTANCE_CM) requestMotion(BACKWARD);
  else requestMotion(TURN_LEFT);
}

// Lost heartbeat closes the browser session and immediately stops motion.
void checkControlTimeouts(uint32_t now) {
  if (controller >= 0 && now - lastHeartbeat > CONTROL_TIMEOUT_MS) {
    const int expiredClient = controller;
    controller = -1;
    stopAndManual();
    wsServer.disconnect(expiredClient);
  }
  // Held movement commands need refreshing only in remote mode.
  // Connection/command failure disables drive and balance immediately.
  if (currentMode == MANUAL && requestedMotion != STOPPED &&
      now - lastDrive > CONTROL_TIMEOUT_MS) stopMotors();
  if (controller < 0 && (requestedMotion != STOPPED || rampActive)) stopAndManual();
}

// Optional USB Serial wheel trim. L/R increase a wheel PWM; l/r
// decrease it; S saves both trims in flash. Changing a trim cancels motion.
void serialCalibration() {
  if (!Serial.available()) return;
  const char c = Serial.read();
  bool changed = false;
  switch (c) {
    case 'L': ++leftPWM; changed = true; break;
    case 'l': --leftPWM; changed = true; break;
    case 'R': ++rightPWM; changed = true; break;
    case 'r': --rightPWM; changed = true; break;
    case 'S':
      prefs.putInt("leftPWM", leftPWM);
      prefs.putInt("rightPWM", rightPWM);
      Serial.println("Calibration saved.");
      break;
  }
  if (changed) {
    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);
    stopAndManual();
    Serial.printf("PWM: L=%d R=%d\n", leftPWM, rightPWM);
  }
}


void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  moveMotor(0, 0);  // Keep motors off during startup.
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);


  Serial.begin(115200);
  // Give USB Serial a short chance to attach, but still boot without a PC.
  const uint32_t serialStarted = millis();
  while (!Serial && millis() - serialStarted < 2000) delay(10);
  Serial.println("[BOOT] XIAO Robot AP firmware starting");

  prefs.begin("servoCalib", false);  // Preserve existing wheel calibration.
  leftPWM = constrain(prefs.getInt("leftPWM", leftPWM), 0, 255);
  rightPWM = constrain(prefs.getInt("rightPWM", rightPWM), 0, 255);
  Serial.printf("PWM: L=%d R=%d. L/l and R/r adjust; S saves.\n", leftPWM, rightPWM);


  loadUprightCalibration(); // Load saved upright before starting the sensor/PID task.

  // Sensor task keeps angle telemetry live even during ordinary driving.
  // Initialization alone never writes nonzero motor output.
  balanceBegin(moveMotor);
  Serial.printf("Balance: %s\n", balanceStatus().message);

  // Fixed local address: phone connects directly to the robot, without a router.
  const IPAddress address(192, 168, 4, 1), subnet(255, 255, 255, 0);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAPConfig(address, address, subnet) ||
      !WiFi.softAP(AP_SSID, AP_PASSWORD, 1, false, 1)) {
    Serial.println("AP startup failed; motors remain stopped. Reset to retry.");
    while (true) { stopMotors(); delay(100); }
  }
  Serial.printf("Wi-Fi: %s\nPassword: %s\nOpen http://%s\n",
                AP_SSID, AP_PASSWORD, WiFi.softAPIP().toString().c_str());
  // Serve the embedded HTML from flash. No separate CSS/JS/filesystem upload
  // is required; no-store asks the browser to fetch the current firmware page.
  httpServer.on("/", []() {
    httpServer.sendHeader("Cache-Control", "no-store");
    // Show this robot's configured Wi-Fi name in the page and connection hint.
    // Escape HTML characters so names containing e.g. & or < display as text.
    String robotName = AP_SSID;
    robotName.replace("&", "&amp;");
    robotName.replace("<", "&lt;");
    robotName.replace(">", "&gt;");
    robotName.replace("\"", "&quot;");
    robotName.replace("'", "&#39;");
    String page = FPSTR(HTML_PAGE);
    page.replace("%ROBOT_SSID%", robotName);
    httpServer.send(200, "text/html; charset=utf-8", page);
  });
  httpServer.onNotFound([]() { httpServer.send(404, "text/plain", "Open http://192.168.4.1/"); });
  wsServer.begin();
  wsServer.onEvent(webSocketEvent);
  httpServer.begin();
}

void loop() {
  // Enforce timeouts before accepting queued commands and before autonomy.
  checkControlTimeouts(millis());
  httpServer.handleClient();
  wsServer.loop();
  checkControlTimeouts(millis());
  if (!balanceStatus().armed) sampleDistance(millis()); // Avoid blocking Echo waits while balancing.
  checkControlTimeouts(millis());
  if (currentMode == DISTANCE) handleDistanceMode();
  updateDriveMotion(millis());
  serviceIdleBalance();
  serialCalibration();
  // Send display updates four times per second without delaying the loop.
  if (millis() - lastStatus >= 250) {
    lastStatus = millis();
    if (controller >= 0) { String state = statusJSON(); wsServer.sendTXT(controller, state); }
  }
  // Repeat a small diagnostic so opening Serial Monitor after boot still
  // shows output. Skip if USB isn't ready or its transmit buffer is full.
  static uint32_t lastSerialReport = 0;
  if (millis() - lastSerialReport >= 2000) {
    lastSerialReport = millis();
    if (Serial && Serial.availableForWrite() >= 160) {
      Serial.printf("[Robot running] uptime=%lu s mode=%s Wi-Fi=%s URL=http://%s/\n",
                    static_cast<unsigned long>(millis() / 1000),
                    modeName(),
                    AP_SSID, WiFi.softAPIP().toString().c_str());
    }
  }
}
