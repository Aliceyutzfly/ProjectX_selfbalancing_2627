// Open this sketch in Arduino IDE. Application: this tab; IMU/PID: Balance.ino.
// Edit robot_config.h for wiring and control settings. Libraries are bundled in src/lib.
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <WiFi.h>
#include <WebServer.h>
#include "src/lib/WebSockets/WebSocketsServer.h"
#include <Preferences.h>
#include "robot_config.h"
#include "web_page.h"
#include "balance.h"
#include "robot_types.h"

// Explicit declarations for Arduino sketch preprocessing.
bool isSettingsMode();
const char* modeName();
void moveMotor(int left, int right);
void clearDriveRequest();
void stopMotors();
void stopAndManual();
void serviceController();
void requestMotion(Motion motion);
void updateDriveMotion(uint32_t now);
bool drive(const String& command);
void requestUserStop();
void sampleDistance(uint32_t now);
void handleDistanceMode();
void checkControlTimeouts(uint32_t now);
void serialCalibration();
void loadUprightCalibration();
void enterCalibration();
void saveUprightCalibration(const String& value);
void loadPIDSettings();
bool parsePIDValues(const String& value, double gains[3]);
void updatePIDSettings(const String& value, bool save);
void updateDeadZone(const String& value, bool save);
String statusJSON();
void handleCommand(const String& command);
void webSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length);
void setup();
void loop();

// Modes

Mode currentMode = REMOTE_MODE;

// Storage and network objects

Preferences prefs;
Preferences imuPrefs;
Preferences pidPrefs;
WebServer httpServer(80);
WebSocketsServer wsServer(81);

// Movement state

// Independent wheel trims; saved values override these startup defaults.
int leftPWM = 254, rightPWM = 254;
Motion requestedMotion = STOPPED;
bool rampActive = false;
// Startup waits for fresh IMU data. Faults require an explicit new enable request.
bool armPending = true;
uint32_t rampStarted = 0;
int rampStartLeft = 0, rampStartRight = 0;
int targetLeft = 0, targetRight = 0;
int appliedLeft = 0, appliedRight = 0;
const char* activity = "Stopped";

// Connection, timing and sensor state

// -1 means no browser owns control.
int controller = -1;
// Unsigned elapsed-time subtraction handles millis() wraparound.
uint32_t lastHeartbeat = 0, lastDrive = 0, lastSensor = 0, lastStatus = 0;
float distanceCM = -1;
bool distanceValid = false;

// Stored settings and browser feedback

bool imuStorageReady = false;
const char* calibrationMessage = "Hold upright, capture or enter an angle, then save";
bool pidStorageReady = false;
// Revision counters advance on Apply/Save, including rejected requests.
uint32_t pidUpdate = 0;
const char* pidMessage = "Apply tests gains; Save also retains them after restart";
uint32_t deadZoneUpdate = 0;
const char* deadZoneMessage = "Apply is temporary; Save remembers the dead zone";

// Mode helpers

// Settings pages never enable the motor controller.
bool isSettingsMode() {
  return currentMode == CALIBRATION_MODE || currentMode == PID_TUNING_MODE;
}

const char* modeName() {
  if (currentMode == PID_TUNING_MODE)
    return "PID_TUNING";
  if (currentMode == CALIBRATION_MODE)
    return "CALIBRATION";
  return currentMode == REMOTE_MODE ? "MANUAL" : "DISTANCE";
}

// Motor output

// Positive PWM is logical forward; negative is reverse. Clear the opposite
// DRV8833 input before applying PWM. This callback is owned by the IMU task.
void moveMotor(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  if (INVERT_LEFT)
    left = -left;
  if (INVERT_RIGHT)
    right = -right;
  // Clear the opposite leg first when changing direction.
  if (left >= 0) {
    analogWrite(MOTOR_IN2, 0);
    analogWrite(MOTOR_IN1, left);
  } else {
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, -left);
  }
  if (right >= 0) {
    analogWrite(MOTOR_IN4, 0);
    analogWrite(MOTOR_IN3, right);
  } else {
    analogWrite(MOTOR_IN3, 0);
    analogWrite(MOTOR_IN4, -right);
  }
}

// Movement requests and acceleration

// Cancel both the current request and any unfinished ramp.
void clearDriveRequest() {
  rampActive = false;
  requestedMotion = STOPPED;
  rampStartLeft = rampStartRight = 0;
  targetLeft = targetRight = 0;
  appliedLeft = appliedRight = 0;
}

// Immediate shutdown: faults, connection loss and emergency stop.
void stopMotors() {
  clearDriveRequest();
  balanceDisarm();
  armPending = false;
  activity = "Stopped";
}

void stopAndManual() {
  stopMotors();
  currentMode = REMOTE_MODE;
}

// Publish base wheel requests. Try enabling once, when fresh IMU data is ready.
void serviceController() {
  balanceSetDrive(appliedLeft, appliedRight);
  if (isSettingsMode() || !armPending)
    return;
  const BalanceStatus state = balanceStatus();
  if (!state.ready)
    return;
  armPending = false;
  balanceArm();
}

// Both driving modes share this path: immediate turns, S-ramped forward/reverse.
// Repeated held commands must not reset a ramp or restart a faulted controller.
void requestMotion(Motion motion) {
  if (isSettingsMode())
    return;
  if (motion == STOPPED) {
    clearDriveRequest();
    activity = "Stopped";
    if (!balanceStatus().armed)
      armPending = true;
    serviceController(); // Zero base drive, without resetting active attitude PID.
    return;
  }
  if (motion == requestedMotion)
    return; // Repeats never reset ramps or a fault latch.
  const bool startingFromStop = requestedMotion == STOPPED;
  requestedMotion = motion;
  if (!balanceStatus().armed)
    armPending = true;
  if (motion == FORWARD) {
    targetLeft = leftPWM;
    targetRight = rightPWM;
    activity = "Forward";
  } else if (motion == BACKWARD) {
    targetLeft = -leftPWM;
    targetRight = -rightPWM;
    activity = "Reverse";
  } else if (motion == TURN_LEFT) {
    targetLeft = -leftPWM;
    targetRight = rightPWM;
    activity = "Turn left";
  } else if (motion == TURN_RIGHT) {
    targetLeft = leftPWM;
    targetRight = -rightPWM;
    activity = "Turn right";
  }

  if (motion == TURN_LEFT || motion == TURN_RIGHT) {
    rampActive = false;
    appliedLeft = targetLeft;
    appliedRight = targetRight;
  } else {
    if (startingFromStop) {
      const int direction = motion == FORWARD ? 1 : -1;
      appliedLeft = direction * min(DRIVE_START_PWM, leftPWM);
      appliedRight = direction * min(DRIVE_START_PWM, rightPWM);
    }
    rampStartLeft = appliedLeft;
    rampStartRight = appliedRight;
    rampStarted = millis();
    rampActive = true;
  }
  serviceController();
}

// Smoothstep shapes requested PWM; physical acceleration also depends on the load.
void updateDriveMotion(uint32_t now) {
  if (!rampActive)
    return;
  const uint32_t elapsed = now - rampStarted;
  const float t = elapsed >= DRIVE_RAMP_MS ? 1.0f : float(elapsed) / float(DRIVE_RAMP_MS);
  const float fraction = t * t * (3.0f - 2.0f * t);
  const int left = rampStartLeft + int(lroundf((targetLeft - rampStartLeft) * fraction));
  const int right = rampStartRight + int(lroundf((targetRight - rampStartRight) * fraction));
  if (left != appliedLeft || right != appliedRight) {
    appliedLeft = left;
    appliedRight = right;
    balanceSetDrive(left, right);
  }
  if (elapsed >= DRIVE_RAMP_MS) {
    rampActive = false;
  }
}

// Decode direction commands; the command handler enforces manual mode.
bool drive(const String& command) {
  if (command == "forward")
    requestMotion(FORWARD);
  else if (command == "backward")
    requestMotion(BACKWARD);
  else if (command == "turnLeft")
    requestMotion(TURN_LEFT);
  else if (command == "turnRight")
    requestMotion(TURN_RIGHT);
  else
    return false;
  return true;
}

// STOP exits distance mode and zeros base drive. Attitude correction stays active;
// settings pages remain disabled.
void requestUserStop() {
  if (isSettingsMode()) {
    stopMotors();
    return;
  }
  currentMode = REMOTE_MODE;
  requestMotion(STOPPED);
}

// Obstacle sensing and distance mode

// Measure at the configured interval; the IMU task continues during the Echo wait.
void sampleDistance(uint32_t now) {
  if (now - lastSensor < SENSOR_INTERVAL_MS)
    return;
  lastSensor = now;
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  const uint32_t duration = pulseInLong(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  // Echo pulse duration is the sound round-trip time.
  distanceCM = duration * 0.0343f / 2.0f;
  distanceValid = duration != 0 && distanceCM >= 2.0f && distanceCM <= 400.0f;
}

// Above 20 cm: forward. Below 5 cm: reverse. Otherwise: left.
// No echo preserves the last request, or starts forward if stopped.
void handleDistanceMode() {
  if (!distanceValid) {
    // Do not restart an existing ramp or replace the last distance decision.
    if (requestedMotion == STOPPED)
      requestMotion(FORWARD);
    return;
  }
  if (distanceCM > FORWARD_DISTANCE_CM)
    requestMotion(FORWARD);
  else if (distanceCM < REVERSE_DISTANCE_CM)
    requestMotion(BACKWARD);
  else
    requestMotion(TURN_LEFT);
}

// Control timeouts

// Lost heartbeat or stale held commands disable both drive and attitude correction.
void checkControlTimeouts(uint32_t now) {
  if (controller >= 0 && now - lastHeartbeat > CONTROL_TIMEOUT_MS) {
    const int expiredClient = controller;
    controller = -1;
    stopAndManual();
    wsServer.disconnect(expiredClient);
  }
  // Held movement commands need refreshing only in remote mode.
  // Connection/command failure disables drive and balance immediately.
  if (currentMode == REMOTE_MODE && requestedMotion != STOPPED && now - lastDrive > CONTROL_TIMEOUT_MS)
    stopMotors();
  if (controller < 0 && (requestedMotion != STOPPED || rampActive))
    stopAndManual();
}

// Wheel trim and upright calibration

// USB wheel trim: L/R increase, l/r decrease, S saves. Adjustments stop motion.
void serialCalibration() {
  if (!Serial.available())
    return;
  const char c = Serial.read();
  bool changed = false;
  switch (c) {
  case 'L':
    ++leftPWM;
    changed = true;
    break;
  case 'l':
    --leftPWM;
    changed = true;
    break;
  case 'R':
    ++rightPWM;
    changed = true;
    break;
  case 'r':
    --rightPWM;
    changed = true;
    break;
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

// The saved upright target is independent of the wheel trims and MPU offsets.
void loadUprightCalibration() {
  imuStorageReady = imuPrefs.begin("imuCalib", false);
  const double saved = imuStorageReady ? imuPrefs.getDouble("upright", DEFAULT_BALANCE_UPRIGHT_DEG)
                                       : DEFAULT_BALANCE_UPRIGHT_DEG;
  if (!balanceSetUpright(saved))
    balanceSetUpright(DEFAULT_BALANCE_UPRIGHT_DEG);
  if (!imuStorageReady)
    calibrationMessage = "Calibration storage unavailable - using default";
}

void enterCalibration() {
  stopAndManual(); // Cancels ramps and PID before enabling the settings page.
  currentMode = CALIBRATION_MODE;
  calibrationMessage = "Hold the body at its intended upright position, then capture and save";
}

void saveUprightCalibration(const String& value) {
  if (currentMode != CALIBRATION_MODE)
    return;
  const char* start = value.c_str();
  char* end = nullptr;
  const double degrees = strtod(start, &end);
  if (end == start || *end != '\0' || !isfinite(degrees) || degrees < MIN_UPRIGHT_DEG ||
      degrees > MAX_UPRIGHT_DEG) {
    calibrationMessage = "Enter a valid angle from 90 to 270 degrees";
    return;
  }
  const BalanceStatus state = balanceStatus();
  if (!state.ready || state.armed) {
    calibrationMessage = "Cannot save - fresh IMU readings and motors off are required";
    return;
  }
  if (!imuStorageReady) {
    calibrationMessage = "Cannot save - calibration storage unavailable";
    return;
  }
  const double previous = state.target;
  if (!balanceSetUpright(degrees)) {
    calibrationMessage = "Cannot update upright angle";
    return;
  }
  if (imuPrefs.putDouble("upright", degrees) != sizeof(double)) {
    balanceSetUpright(previous); // Do not claim success when flash storage failed.
    calibrationMessage = "Save failed - previous upright angle restored";
    return;
  }
  calibrationMessage = "Upright angle saved - retained after restart; motors remain off";
}

// PID and angle dead-zone settings

// Store the gain triple together; the angle dead zone has its own saved value.
void loadPIDSettings() {
  pidStorageReady = pidPrefs.begin("attitudePID", false);
  double gains[3] = {BALANCE_KP, BALANCE_KI, BALANCE_KD};
  if (pidStorageReady && pidPrefs.getBytesLength("gains") == sizeof(gains)) {
    double saved[3];
    if (pidPrefs.getBytes("gains", saved, sizeof(saved)) == sizeof(saved) &&
        balanceValidTunings(saved[0], saved[1], saved[2])) {
      for (size_t i = 0; i < 3; ++i)
        gains[i] = saved[i];
    }
  }
  balanceSetTunings(gains[0], gains[1], gains[2]);
  const double savedZone = pidStorageReady ? pidPrefs.getDouble("deadzone", DEFAULT_DEAD_ZONE_DEG)
                                           : DEFAULT_DEAD_ZONE_DEG;
  if (!balanceSetDeadZone(savedZone))
    balanceSetDeadZone(DEFAULT_DEAD_ZONE_DEG);
}

bool parsePIDValues(const String& value, double gains[3]) {
  const char* start = value.c_str();
  for (size_t i = 0; i < 3; ++i) {
    char* end = nullptr;
    gains[i] = strtod(start, &end);
    if (end == start || (i < 2 && *end != ',') || (i == 2 && *end != '\0'))
      return false;
    start = end + 1;
  }
  return balanceValidTunings(gains[0], gains[1], gains[2]);
}

// Apply changes RAM; Save also writes flash. Failed saves restore runtime values.
void updatePIDSettings(const String& value, bool save) {
  ++pidUpdate;
  double gains[3];
  if (!parsePIDValues(value, gains)) {
    pidMessage = "Invalid PID: P 0..200, I 0..500, D 0..50";
    return;
  }
  if (save && !pidStorageReady) {
    pidMessage = "Cannot save PID - storage unavailable";
    return;
  }
  const BalanceStatus previous = balanceStatus();
  const bool editingSettings = isSettingsMode();
  stopMotors(); // Settings apply with output OFF, and never restart automatically.
  if (!editingSettings)
    currentMode = REMOTE_MODE;
  if (!balanceSetTunings(gains[0], gains[1], gains[2])) {
    pidMessage = "Cannot apply PID gains";
    return;
  }
  if (save && pidPrefs.putBytes("gains", gains, sizeof(gains)) != sizeof(gains)) {
    balanceSetTunings(previous.kp, previous.ki, previous.kd);
    pidMessage = "PID save failed - previous gains restored";
    return;
  }
  pidMessage = save ? "PID saved - select Remote drive to resume"
                    : "PID applied for this session - select Remote drive to resume";
}

void updateDeadZone(const String& value, bool save) {
  ++deadZoneUpdate;
  if (currentMode != PID_TUNING_MODE) {
    deadZoneMessage = "Cannot edit dead zone outside PID settings";
    return;
  }
  const char* start = value.c_str();
  char* end = nullptr;
  const double degrees = strtod(start, &end);
  if (end == start || *end != '\0' || !isfinite(degrees) || degrees < 0 ||
      degrees > MAX_DEAD_ZONE_DEG) {
    deadZoneMessage = "Invalid dead zone - enter 0 to 90 degrees";
    return;
  }
  if (save && !pidStorageReady) {
    deadZoneMessage = "Cannot save dead zone - storage unavailable";
    return;
  }
  stopMotors();
  const double previous = balanceStatus().deadZone;
  if (!balanceSetDeadZone(degrees)) {
    deadZoneMessage = "Cannot apply dead zone";
    return;
  }
  if (save && pidPrefs.putDouble("deadzone", degrees) != sizeof(double)) {
    balanceSetDeadZone(previous);
    deadZoneMessage = "Dead zone save failed - previous value restored";
    return;
  }
  deadZoneMessage = save ? "Dead zone saved - select Remote drive to test"
                         : "Dead zone applied for this session - select Remote drive to test";
}

// Browser telemetry

// Values passed to add() are JSON literals; text() quotes fixed message strings.
String statusJSON() {
  const BalanceStatus state = balanceStatus();
  String json = "{";
  json.reserve(1024);
  auto text = [](const char* value) {
    return String("\"") + value + "\"";
  };
  auto add = [&json](const char* key, const String& value) {
    if (json.length() > 1)
      json += ',';
    json += String("\"") + key + "\":" + value;
  };
  add("mode", text(modeName()));
  add("distance_cm", distanceValid ? String(distanceCM, 1) : "null");
  add("activity", text(activity));
  add("balance_active", state.armed ? "true" : "false");
  add("balance_ready", state.ready ? "true" : "false");
  add("balance_pending", armPending ? "true" : "false");
  add("angle_deg", state.angleValid ? String(state.angle, 2) : "null");
  add("target_deg", String(state.target, 2));
  add("tilt_deg", state.angleValid ? String(state.angle - state.target, 2) : "null");
  add("balance_pwm", String(state.pwm));
  add("drive_left", String(state.driveLeft));
  add("drive_right", String(state.driveRight));
  add("output_left", String(state.outputLeft));
  add("output_right", String(state.outputRight));
  add("output_limited", state.limited ? "true" : "false");
  add("pid_p", String(state.kp, 3));
  add("pid_i", String(state.ki, 3));
  add("pid_d", String(state.kd, 3));
  add("pid_update", String(pidUpdate));
  add("pid_message", text(pidMessage));
  add("deadzone_deg", String(state.deadZone, 3));
  add("deadzone_update", String(deadZoneUpdate));
  add("deadzone_message", text(deadZoneMessage));
  add("balance_message", text(state.message));
  add("calibration_message", text(calibrationMessage));
  return json + '}';
}

// Browser commands and WebSocket sessions

// Interpret commands independently of the connection and payload handling below.
void handleCommand(const String& command) {
  // STOP zeros driving; settings remain off. Emergency stop disables everything.
  if (command == "stop")
    requestUserStop();
  else if (command == "heartbeat")
    lastHeartbeat = millis();
  else if (command == "motors:off")
    stopAndManual();
  else if (command == "mode:calibration")
    enterCalibration();
  else if (command == "mode:pid") {
    stopAndManual();
    currentMode = PID_TUNING_MODE;
    activity = "PID settings - motors off";
  } else if (command.startsWith("imu:save:"))
    saveUprightCalibration(command.substring(sizeof("imu:save:") - 1));
  else if (command.startsWith("deadzone:apply:"))
    updateDeadZone(command.substring(sizeof("deadzone:apply:") - 1), false);
  else if (command.startsWith("deadzone:save:"))
    updateDeadZone(command.substring(sizeof("deadzone:save:") - 1), true);
  else if (command.startsWith("pid:apply:"))
    updatePIDSettings(command.substring(sizeof("pid:apply:") - 1), false);
  else if (command.startsWith("pid:save:"))
    updatePIDSettings(command.substring(sizeof("pid:save:") - 1), true);
  else if (command == "mode:manual" || command == "mode:distance") {
    stopAndManual();
    if (command == "mode:distance") {
      currentMode = DISTANCE_MODE;
      distanceValid = false;
      lastSensor = millis() - SENSOR_INTERVAL_MS;
      activity = "Checking distance";
    } else
      requestUserStop();
  } else if (currentMode == REMOTE_MODE && drive(command)) {
    lastDrive = millis();
  } else if (currentMode == REMOTE_MODE)
    stopMotors();
}

// Only the accepted controller can issue commands. Disconnects shut down immediately.
void webSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
  // Accept one controller, reset to a stopped state, then send its first status.
  if (type == WStype_CONNECTED) {
    if (controller >= 0) {
      wsServer.disconnect(client);
      return;
    }
    // Initial connection may arrive while the initially stopped robot balances.
    // Reconnect after a safety stop preserves its disabled state.
    if (requestedMotion != STOPPED || rampActive)
      stopAndManual();
    controller = client;
    lastHeartbeat = millis();
    String state = statusJSON();
    wsServer.sendTXT(client, state);
    return;
  }
  if (client != controller)
    return;
  // Disconnect immediately stops motion and returns to manual mode.
  if (type == WStype_DISCONNECTED) {
    controller = -1;
    stopAndManual();
    return;
  }
  if (type != WStype_TEXT)
    return;
  // Copy only the supplied bytes; do not assume a NUL-terminated payload.
  if (length == 0 || length > 64) {
    stopAndManual();
    return;
  }
  String command;
  command.reserve(length);
  for (size_t i = 0; i < length; ++i)
    command += static_cast<char>(payload[i]);

  handleCommand(command);
  String state = statusJSON();
  if (command != "heartbeat")
    wsServer.sendTXT(client, state);
}

// Arduino entry points

// Initialization order matters: keep motors off, restore settings, then start services.
void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  moveMotor(0, 0); // Keep motors off during startup.
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);

  Serial.begin(115200);
  // Give USB Serial a short chance to attach, but still boot without a PC.
  const uint32_t serialStarted = millis();
  while (!Serial && millis() - serialStarted < 2000)
    delay(10);
  Serial.println("[BOOT] XIAO Robot AP firmware starting");

  prefs.begin("servoCalib", false); // Preserve existing wheel calibration.
  leftPWM = constrain(prefs.getInt("leftPWM", leftPWM), 0, 255);
  rightPWM = constrain(prefs.getInt("rightPWM", rightPWM), 0, 255);
  Serial.printf("PWM: L=%d R=%d. L/l and R/r adjust; S saves.\n", leftPWM, rightPWM);

  loadUprightCalibration();
  loadPIDSettings(); // Defaults or saved tuning, before the single output task starts.

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
    while (true) {
      stopMotors();
      delay(100);
    }
  }
  Serial.printf("Wi-Fi: %s\nPassword: %s\nOpen http://%s\n", AP_SSID, AP_PASSWORD,
                WiFi.softAPIP().toString().c_str());
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
  httpServer.onNotFound([]() {
    httpServer.send(404, "text/plain", "Open http://192.168.4.1/");
  });
  wsServer.begin();
  wsServer.onEvent(webSocketEvent);
  httpServer.begin();
}

// Service networking and sensors, enforce timeouts, and publish drive requests.
// The independent IMU task performs PID calculation and final motor writes.
void loop() {
  // Enforce timeouts before accepting queued commands and before autonomy.
  checkControlTimeouts(millis());
  httpServer.handleClient();
  wsServer.loop();
  checkControlTimeouts(millis());
  sampleDistance(millis()); // The independent motor task continues during Echo waits.
  checkControlTimeouts(millis());
  if (currentMode == DISTANCE_MODE)
    handleDistanceMode();
  updateDriveMotion(millis());
  serviceController();
  serialCalibration();
  // Send display updates four times per second without delaying the loop.
  if (millis() - lastStatus >= 250) {
    lastStatus = millis();
    if (controller >= 0) {
      String state = statusJSON();
      wsServer.sendTXT(controller, state);
    }
  }
  // Repeat a small diagnostic so opening Serial Monitor after boot still
  // shows output. Skip if USB isn't ready or its transmit buffer is full.
  static uint32_t lastSerialReport = 0;
  if (millis() - lastSerialReport >= 2000) {
    lastSerialReport = millis();
    if (Serial && Serial.availableForWrite() >= 160) {
      Serial.printf("[Robot running] uptime=%lu s mode=%s Wi-Fi=%s URL=http://%s/\n",
                    static_cast<unsigned long>(millis() / 1000), modeName(), AP_SSID,
                    WiFi.softAPIP().toString().c_str());
    }
  }
}
