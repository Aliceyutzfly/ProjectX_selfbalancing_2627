// Libraries: Arduino handles pins/timing; WiFi creates the network;
// WebServer serves the page; WebSockets carries live commands/status;
// Preferences saves wheel trims in flash. The two local headers hold
// hardware settings and the complete offline control page.
#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include "robot_config.h"
#include "web_page.h"
#include "balance.h"

// Balance mode owns motor output continuously, including between commands.
enum Mode { MANUAL, DISTANCE, BALANCE };
Mode currentMode = MANUAL;

// Create persistent-storage and network-server objects once.
// HTTP port 80 loads the page; WebSocket port 81 keeps controls responsive.
Preferences prefs;
WebServer httpServer(80);
WebSocketsServer wsServer(81);

// Shared runtime values. PWM magnitude is 0..255; each wheel has its
// own trim because two nominally identical motors may run at different speeds.
int leftPWM = 224, rightPWM = 224;
int controller = -1;  // One controlling browser at a time.
// millis() timestamps are milliseconds since boot. Subtract unsigned
// timestamps rather than comparing absolute deadlines, so wraparound works.
uint32_t lastHeartbeat = 0, lastDrive = 0, lastSensor = 0, lastStatus = 0;
float distanceCM = -1;
bool distanceValid = false;
// Track manual motion so stale movement commands can stop the motors.
bool manualMoving = false;
uint32_t manualStopStarted = 0;
int manualStartLeft = 0, manualStartRight = 0;
int manualAppliedLeft = 0, manualAppliedRight = 0;
bool manualStopping = false;
bool manualStarting = false;
uint32_t manualRampStarted = 0;
int manualTargetLeft = 0, manualTargetRight = 0;
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


void stopMotors() {
  // Clear the ramp too, so the next loop cannot restart a stopped motor.
  manualStopping = manualStarting = false;
  manualTargetLeft = manualTargetRight = 0;
  manualStartLeft = manualStartRight = 0;
  manualAppliedLeft = manualAppliedRight = 0;
  balanceWriteManual(0, 0);  // Ignored while balance owns the motors.
  manualMoving = false;
  activity = "Stopped";
}

// Explicit mode change/disarm: switch off balance and stop the other modes.
void stopAndManual() {
  balanceDisarm();
  stopMotors();
  currentMode = MANUAL;
}

// Releasing a balance-mode arrow requests upright, not motor power-off.
void stopRequestedMotion() {
  if (currentMode == BALANCE) balanceDrive(0);
  else stopAndManual();
}

const char* modeName() {
  if (currentMode == BALANCE) return "BALANCE";
  return currentMode == MANUAL ? "MANUAL" : "DISTANCE";
}

// Encode telemetry for the page. JSON null means no valid distance;
// the browser displays it as No echo rather than treating it as zero cm.
String statusJSON() {
  String json = "{\"mode\":\"";
  json += modeName();
  json += "\",\"distance_cm\":";
  json += distanceValid ? String(distanceCM, 1) : String("null");
  json += ",\"activity\":\"";
  const BalanceStatus balance = balanceStatus();
  json += currentMode == BALANCE ? balance.message : activity;
  json += "\",\"balance_ready\":";
  json += balance.ready ? "true" : "false";
  json += ",\"balance_armed\":";
  json += balance.armed ? "true" : "false";
  json += ",\"tilt_deg\":";
  json += balance.angleValid ? String(balance.angle - BALANCE_UPRIGHT_DEG, 2) : String("null");
  json += ",\"target_tilt_deg\":";
  json += String(balance.target - BALANCE_UPRIGHT_DEG, 2);
  json += ",\"balance_message\":\"";
  json += balance.message;
  json += "\"}";
  return json;
}

// Translate a named command into differential wheel speeds.
// Turning spins the wheels in opposite directions to turn on the spot.
// Return false for unknown commands so callers can handle them safely.
bool drive(const String& command) {
  int left = 0, right = 0;
  if (command == "forward") { left = leftPWM; right = rightPWM; activity = "Forward"; }
  else if (command == "backward") { left = -leftPWM; right = -rightPWM; activity = "Reverse"; }
  else if (command == "turnLeft") { left = -leftPWM; right = rightPWM; activity = "Turn left"; }
  else if (command == "turnRight") { left = leftPWM; right = -rightPWM; activity = "Turn right"; }
  else return false;

  if (currentMode == MANUAL) {
    // Repeated held commands must not restart the ramp. A new direction or
    // resuming during slowdown begins from the PWM currently being applied.
    if (!manualMoving || manualStopping ||
        left != manualTargetLeft || right != manualTargetRight) {
      manualStartLeft = manualAppliedLeft;
      manualStartRight = manualAppliedRight;
      manualTargetLeft = left;
      manualTargetRight = right;
      manualRampStarted = millis();
      manualStarting = true;
      manualStopping = false;
    }
  } else balanceWriteManual(left, right);  // Obstacle mode stays immediate.
  return true;
}

// Starting: quadratic ease-out. PWM rises quickly first, then more slowly.
// t runs from 0 to 1; fraction = 1 - (1 - t)^2 reaches exactly 1 at the end.
// This shapes commanded PWM, not measured wheel speed or acceleration.
void updateManualMotion(uint32_t now) {
  if (currentMode == MANUAL && manualMoving && manualStarting) {
    const uint32_t elapsed = now - manualRampStarted;
    const float t = elapsed >= MANUAL_RAMP_MS ? 1.0f :
                    float(elapsed) / float(MANUAL_RAMP_MS);
    const float fraction = 1.0f - (1.0f - t) * (1.0f - t);
    const int left = manualStartLeft + int(lroundf((manualTargetLeft - manualStartLeft) * fraction));
    const int right = manualStartRight + int(lroundf((manualTargetRight - manualStartRight) * fraction));
    if (left != manualAppliedLeft || right != manualAppliedRight) {
      balanceWriteManual(left, right);
      manualAppliedLeft = left;
      manualAppliedRight = right;
    }
    if (elapsed >= MANUAL_RAMP_MS) manualStarting = false;
    return;
  }
  // Normal stopping keeps its existing linear reduction.
  if (currentMode != MANUAL || !manualMoving || !manualStopping) return;
  const uint32_t elapsed = now - manualStopStarted;
  if (elapsed >= MANUAL_STOP_RAMP_MS) { stopMotors(); return; }
  const int32_t remaining = MANUAL_STOP_RAMP_MS - elapsed;
  const int left = manualStartLeft * remaining / int32_t(MANUAL_STOP_RAMP_MS);
  const int right = manualStartRight * remaining / int32_t(MANUAL_STOP_RAMP_MS);
  if (left != manualAppliedLeft || right != manualAppliedRight) {
    balanceWriteManual(left, right);
    manualAppliedLeft = left;
    manualAppliedRight = right;
  }
}

// A deliberate release/STOP in mode 1 is a smooth stop. Safety paths still
// use stopRequestedMotion()/stopAndManual() for immediate removal of drive.
void requestUserStop() {
  if (currentMode != MANUAL) { stopRequestedMotion(); return; }
  if (!manualMoving) { stopMotors(); return; }
  if (manualStopping) return;  // Repeated stop messages must not extend coasting.
  manualStartLeft = manualAppliedLeft;
  manualStartRight = manualAppliedRight;
  manualStopStarted = lastDrive = millis(); // A fresh, explicit stop request.
  manualStarting = false;
  manualStopping = true;
  activity = "Slowing down";
}

// WebSocket callback: runs when a browser connects, disconnects,
// or sends a message. Only the accepted controller may issue commands.
void webSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
  // Accept one controller, reset to a stopped state, then send its first status.
  if (type == WStype_CONNECTED) {
    if (controller >= 0) { wsServer.disconnect(client); return; }
    stopRequestedMotion();
    controller = client;
    lastHeartbeat = millis();
    String state = statusJSON();
    wsServer.sendTXT(client, state);
    return;
  }
  if (client != controller) return;
  // Disconnect stops manual/obstacle motion, or requests upright in balance mode.
  if (type == WStype_DISCONNECTED) {
    controller = -1;
    stopRequestedMotion();
    return;
  }
  if (type != WStype_TEXT) return;
  // Copy only the supplied bytes; do not assume a NUL-terminated payload.
  if (length == 0 || length > 32) { stopRequestedMotion(); return; }
  String command;
  command.reserve(length);
  for (size_t i = 0; i < length; ++i) command += static_cast<char>(payload[i]); // Copy the data received from the WebSocket into a String.

  // Commands never directly write fixed PWM in balance mode.
  if (command == "stop") requestUserStop();
  else if (command == "heartbeat") lastHeartbeat = millis();
  else if (command == "balance:off") stopAndManual();
  else if (command == "mode:manual" || command == "mode:distance") {
    // Require explicit DISARM before changing away from an armed balance mode.
    if (currentMode == BALANCE && balanceStatus().armed) balanceDrive(0);
    else {
      stopAndManual();
      if (command == "mode:distance") {
        currentMode = DISTANCE;
        distanceValid = false;
        lastSensor = millis() - SENSOR_INTERVAL_MS;
        activity = "Checking distance";
      }
    }
  } else if (command == "mode:balance") {
    if (currentMode != BALANCE) stopAndManual();
    currentMode = BALANCE;
    manualMoving = false;
    distanceValid = false;  // Ultrasonic readings are suspended in this mode.
    balanceArm();          // May refuse; the page displays the reason.
    lastHeartbeat = lastDrive = millis();
  } else if (currentMode == BALANCE) {
    if (command == "forward" || command == "backward") {
      balanceDrive(command == "forward" ? 1 : -1);
      lastDrive = millis();
    } else balanceDrive(0);  // No steering in this first balance version.
  } else if (currentMode == MANUAL && drive(command)) {
    lastDrive = millis();
    manualMoving = true;
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

// Simple obstacle avoidance: forward while farther than 20 cm; otherwise
// turn on the spot until a fresh reading shows the path is clear.
void handleDistanceMode() {
  if (!distanceValid) {
    stopMotors();
    activity = "No valid echo";
    return;
  }

  if (distanceCM > OBSTACLE_CM) drive("forward");
  else drive("turnRight");  // Fixed escape direction; no reversing or timers.
}

// Lost heartbeat closes the browser session. Manual/obstacle modes stop;
// balance keeps correcting tilt with an upright target. Its movement-command
// timeout is also checked inside the independent balance task.
void checkControlTimeouts(uint32_t now) {
  if (controller >= 0 && now - lastHeartbeat > CONTROL_TIMEOUT_MS) {
    const int expiredClient = controller;
    controller = -1;
    stopRequestedMotion();
    wsServer.disconnect(expiredClient);
  }
  // An explicit smooth stop has its own finite deadline. It need not finish
  // within the held-command timeout, but the heartbeat check above still applies.
  if (manualMoving && !manualStopping && now - lastDrive > CONTROL_TIMEOUT_MS) stopMotors();
  if (manualStopping && now - manualStopStarted >= MANUAL_STOP_RAMP_MS) stopMotors();
  if (currentMode == BALANCE && now - lastDrive > CONTROL_TIMEOUT_MS) balanceDrive(0);
  if (controller < 0) stopRequestedMotion();
}

// Optional USB Serial wheel trim. L/R increase a wheel PWM; l/r
// decrease it; S saves both trims in flash. Changing a trim cancels motion.
void serialCalibration() {
  if (!Serial.available()) return;
  const char c = Serial.read();
  if (currentMode == BALANCE) return;  // Never change wheel trim while balancing.
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
  moveMotor(0, 0);  // Motor guard is initialized later, so stop hardware directly.
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pinMode(MPU_INT, INPUT);


  Serial.begin(115200);
  // Give USB Serial a short chance to attach, but still boot without a PC.
  const uint32_t serialStarted = millis();
  while (!Serial && millis() - serialStarted < 2000) delay(10);
  Serial.println("[BOOT] XIAO Robot AP firmware starting");

  prefs.begin("servoCalib", false);  // Preserve existing wheel calibration.
  leftPWM = constrain(prefs.getInt("leftPWM", 150), 0, 255);
  rightPWM = constrain(prefs.getInt("rightPWM", 150), 0, 255);
  Serial.printf("PWM: L=%d R=%d. L/l and R/r adjust; S saves.\n", leftPWM, rightPWM);

  // Sensor/PID setup failure disables balance only; the other modes still work.
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
  if (currentMode != BALANCE) sampleDistance(millis()); // No blocking Echo waits in balance mode.
  checkControlTimeouts(millis());
  updateManualMotion(millis());
  if (currentMode == DISTANCE) handleDistanceMode();
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
