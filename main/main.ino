// main.ino

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>

// === Wi-Fi ===
const char* SSID     = "REPLACE THIS WITH YOUR HOTSPOT NAME";
const char* PASSWORD = "REPLACE THIS WITH YOUR HOTSPOT PASSWORD";

// === Pins ===
constexpr int MOTOR_IN1        = D0;
constexpr int MOTOR_IN2        = D1;
constexpr int MOTOR_IN3        = D2;
constexpr int MOTOR_IN4        = D3;
constexpr int MPU_SCL          = D4;
constexpr int MPU_SDA          = D5;
constexpr int MPU_ACC_INIT     = D6;
constexpr int PIN_TRIG         = D10;
constexpr int PIN_ECHO         = D9;
constexpr int PIN_BUTTON       = D4;  // Active-HIGH


// === Calibration ===
Preferences prefs;
int leftPWM;
int rightPWM;


// === Modes ===
enum Mode { MANUAL, DISTANCE };
Mode currentMode = MANUAL;
const char* modeNames[] = { "MANUAL", "DISTANCE" };

// === Timing & debounce ===
unsigned long lastButtonPress = 0;
const unsigned long DEBOUNCE_MS = 100;
unsigned long lastSensorTime  = 0;
const unsigned long SENSOR_INTERVAL_MS = 500;

// === Ultrasonic ===
#define SOUND_SPEED_CM_PER_US 0.034f
#define ECHO_TIMEOUT_US       30000UL

// === Servers ===
WebServer        httpServer(80);
WebSocketsServer wsServer(81);


// === HTML ===
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Omni Directional Robot Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="stylesheet" href="https://raw.flemingsociety.com/robot/style.css">
</head>
<body>
  <div class="logo-container">
    <img src="https://raw.flemingsociety.com/logo.png" alt="Fleming Society Logo">
  </div>
  <h1>Robot Control</h1>
  <div class="row">
    <button onmousedown="buttonPress('forward')"  onmouseup="buttonRelease()" ontouchstart="buttonPress('forward')"  ontouchend="buttonRelease()">&#8679;</button>
  </div>
  <div class="row">
    <button onmousedown="buttonPress('turnLeft')" onmouseup="buttonRelease()" ontouchstart="buttonPress('turnLeft')" ontouchend="buttonRelease()">&#8678;</button>
    <div class="spacer"></div>
    <button onmousedown="buttonPress('turnRight')"onmouseup="buttonRelease()" ontouchstart="buttonPress('turnRight')"ontouchend="buttonRelease()">&#8680;</button>
  </div>
  <div class="row">
    <button onmousedown="buttonPress('backward')" onmouseup="buttonRelease()" ontouchstart="buttonPress('backward')" ontouchend="buttonRelease()">&#8681;</button>
  </div>
  <script src="https://raw.flemingsociety.com/robot/script.js"></script>
</body>
</html>
)rawliteral";

// === Prototypes ===
void stopMotors();
void moveMoto();
void spinMotor(const String& cmd);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
float readDistanceCM();
void checkModeButton();
void handleDistanceMode();


void setup() {
  Serial.begin(115200);

  // H-bridge Cofiguration
pinMode(MOTOR_IN1, OUTPUT);
pinMode(MOTOR_IN2, OUTPUT);
pinMode(MOTOR_IN3, OUTPUT);
pinMode(MOTOR_IN4, OUTPUT);
pinMode(MPU_ACC_INIT, INPUT);
  // calibration
  prefs.begin("servoCalib", false);
  leftPWM  = prefs.getInt("leftPWM", 150);
  rightPWM = prefs.getInt("rightPWM", 150);
  Serial.printf("Calibration: L=%d, R=%d\n", leftPWM, rightPWM);
  Serial.println("L/l adjust left, R/r adjust right, S to save");

  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  stopMotors();

  // Wi-Fi
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.printf("\nConnected: %s\n", WiFi.localIP().toString().c_str());

  // HTTP & WS
  httpServer.on("/", [](){ httpServer.send(200,"text/html",htmlPage); });
  httpServer.begin();
  wsServer.begin();
  wsServer.onEvent(webSocketEvent);
}

void loop() {
  httpServer.handleClient();
  wsServer.loop();
  checkModeButton();
  if (currentMode == DISTANCE) handleDistanceMode();

  // serial calibration
  if (Serial.available()) {
    char c = Serial.read();
    bool changed = false;

    switch (c) {
      case 'L': leftPWM++;  changed = true; break;
      case 'l': leftPWM--;  changed = true; break;
      case 'R': rightPWM++; changed = true; break;
      case 'r': rightPWM--; changed = true; break;

      case 'S':
        prefs.putInt("leftPWM", leftPWM);
        prefs.putInt("rightPWM", rightPWM);
        Serial.println("Saved.");
        break;
    }

    if (changed) {
      leftPWM  = constrain(leftPWM, 0, 255);
      rightPWM = constrain(rightPWM, 0, 255);
      Serial.printf("PWM: L=%d R=%d\n", leftPWM, rightPWM);
      stopMotors();
    }
  }
}

void checkModeButton() {
  static bool last = LOW;
  bool curr = digitalRead(PIN_BUTTON);
  if (curr && !last && millis()-lastButtonPress>DEBOUNCE_MS) {
    currentMode = Mode((currentMode+1)%2);
    Serial.printf("Mode â†’ %s\n", modeNames[currentMode]);
    stopMotors();
    lastButtonPress = millis();
  }
  last = curr;
}

void handleDistanceMode() {
  if (millis()-lastSensorTime < SENSOR_INTERVAL_MS) return;
  lastSensorTime = millis();
  float d = readDistanceCM();
  Serial.printf("Dist=%.1fcm\n", d);

  static bool turnLeftNext = true;
  if (d>0 && d<30.0f) {
    if (turnLeftNext) { Serial.println("Avoid: turnLeft");  spinMotor("turnLeft"); }
    else             { Serial.println("Avoid: turnRight"); spinMotor("turnRight"); }
    turnLeftNext = !turnLeftNext;
  } else {
    Serial.println("Path clear: forward");
    spinMotor("forward");
  }
}


float readDistanceCM() {
  digitalWrite(PIN_TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG,LOW);
  long dur = pulseInLong(PIN_ECHO,HIGH,ECHO_TIMEOUT_US);
  return dur==0 ? -1 : dur * SOUND_SPEED_CM_PER_US / 2.0f;
}


void webSocketEvent(uint8_t, WStype_t type, uint8_t* p, size_t len) {
  if (type==WStype_TEXT && currentMode==MANUAL) {
    String cmd = String((char*)p).substring(0,len);
    spinMotor(cmd);
  }
}


void moveMotor(int leftCommand, int rightCommand) {
  leftCommand  = constrain(leftCommand, -255, 255);
  rightCommand = constrain(rightCommand, -255, 255);

  analogWrite(MOTOR_IN1, leftCommand  > 0 ?  leftCommand  : 0);
  analogWrite(MOTOR_IN2, leftCommand  < 0 ? -leftCommand  : 0);
  analogWrite(MOTOR_IN3, rightCommand > 0 ?  rightCommand : 0);
  analogWrite(MOTOR_IN4, rightCommand < 0 ? -rightCommand : 0);
}

void spinMotor(const String& cmd) {
  if      (cmd == "forward")   moveMotor( leftPWM,  rightPWM);
  else if (cmd == "backward")  moveMotor(-leftPWM, -rightPWM);
  else if (cmd == "turnLeft")  moveMotor(-leftPWM,  rightPWM);
  else if (cmd == "turnRight") moveMotor( leftPWM, -rightPWM);
  else                        stopMotors();
}

void stopMotors() {
  moveMotor(0, 0);
}
