//==========================================================
// Fleming Society Project X 2025-26 - REFINED COMMAND CENTER
//==========================================================

#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TFT_eSPI.h> 

// --- WIFI CONFIGURATION ---
const char* AP_SSID = "Fleming_Project_X_17";
const char* AP_PASS = "128128128";

// --- LOGO ---
#include "Logo.h" 

// --- FACE CONFIGURATION ---
struct Face {
  String text;
  uint16_t color;
};

Face FACE_NEUTRAL = {"U w U", TFT_CYAN};
Face FACE_HAPPY   = {"^ v ^", TFT_GREEN};
Face FACE_FAST    = {"> o <", TFT_ORANGE};

// --- PIN DEFINITIONS ---
#define PIN_LCD_BL     38  
#define PIN_POWER_ON   15  
#define PIN_BUTTON     0   
#define PIN_KEY_BTN    14  
constexpr int PIN_SERVO_LEFT  = 1; 
constexpr int PIN_SERVO_RIGHT = 2;  
constexpr int PIN_TRIG        = 11; 
constexpr int PIN_ECHO        = 10; 

// --- GLOBALS ---
TFT_eSPI tft = TFT_eSPI(); 
Servo servoL;
Servo servoR;
WebServer server(80);

int robotState = -1; // -1: Standby, 0: WiFi, 1: Auto
bool isAutoStarted = false; 
int set_distance = 70; 
int LEFT_STOP_OFFSET = 90; 
int RIGHT_STOP_OFFSET = 90;
bool isMoving = false;
String currentFace = ""; 

// --- HTML INTERFACE ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<style>
  body { background:#121212; color:#00FF41; text-align:center; font-family:sans-serif; margin:0; padding:10px; user-select:none; -webkit-user-select:none; }
  .section { background:#1e1e1e; border-radius:10px; padding:12px; margin-bottom:12px; border:1px solid #333; }
  .dpad { display: grid; grid-template-areas: ". up ." "left . right" ". down ."; grid-gap: 10px; justify-content: center; }
  .btn { 
    width: 85px; height: 85px; background: #333; border: 3px solid #00FF41; border-radius: 12px; color: white; 
    display: flex; align-items: center; justify-content: center; touch-action: none; font-weight:bold; -webkit-touch-callout: none;
  }
  .btn:active { background: #00FF41; color: #000; }
  .up { grid-area: up; } .down { grid-area: down; } .left { grid-area: left; } .right { grid-area: right; }
  
  .ctrl-group { display: flex; justify-content: space-around; margin-top: 15px; }
  .action-btn { padding: 15px; width: 45%; border-radius: 8px; font-weight: bold; border: none; cursor: pointer; }
  .start { background: #00FF41; color: #000; }
  .stop { background: #FF3131; color: #fff; }

  .face-edit { display: flex; align-items: center; justify-content: space-between; margin-top: 10px; background: #252525; padding: 5px; border-radius: 5px; }
  input[type="text"] { background:#000; color:#00FF41; border:1px solid #444; padding:5px; width:40%; }
  input[type="color"] { background:none; border:none; width:30px; height:30px; }
  .save-btn { background:#00FF41; color:#000; border:none; padding:5px 10px; font-weight:bold; border-radius:4px; }
  label { font-size: 0.7em; color:#888; width: 20%; text-align: left; }
</style></head>
<body>
  <h3>PROJECT X COMMAND</h3>
  
  <div class="section">
    <strong>DISTANCE: <span id="dist">Sensor Off</span></strong>
    <div class="ctrl-group">
      <button class="action-btn start" onclick="f('/startAuto')">START AUTO</button>
      <button class="action-btn stop" onclick="f('/stopMotors')">STOP ALL</button>
    </div>
  </div>

  <div class="section dpad">
    <div class="btn up" ontouchstart="m(100,100)" ontouchend="s()">FWD</div>
    <div class="btn left" ontouchstart="m(-80,80)" ontouchend="s()">LFT</div>
    <div class="btn right" ontouchstart="m(80,-80)" ontouchend="s()">RGT</div>
    <div class="btn down" ontouchstart="m(-100,-100)" ontouchend="s()">BCK</div>
  </div>

  <div class="section">
    <label style="display:block; margin-bottom:5px;">SET STOP DISTANCE (CM)</label>
    <input type="number" id="distVal" value="70" style="width:70%; background:#000; color:#00FF41; border:1px solid #444; padding:8px;">
    <button class="save-btn" onclick="uD()">SET</button>
  </div>

  <div class="section">
    <strong>EDIT FACES</strong>
    <div class="face-edit"><label>NEUTRAL</label><input type="text" id="nT" value="U w U"><input type="color" id="nC" value="#00FFFF"><button class="save-btn" onclick="uF('n')">SET</button></div>
    <div class="face-edit"><label>HAPPY</label><input type="text" id="hT" value="^ v ^"><input type="color" id="hC" value="#00FF00"><button class="save-btn" onclick="uF('h')">SET</button></div>
    <div class="face-edit"><label>FAST</label><input type="text" id="fT" value="> o <"><input type="color" id="fC" value="#FFA500"><button class="save-btn" onclick="uF('f')">SET</button></div>
  </div>

<script>
  function f(path){fetch(path);}
  function m(l,r){fetch(`/control?L=${l}&R=${r}`);}
  function s(){fetch('/control?L=0&R=0');}
  function uD(){let v = document.getElementById('distVal').value; fetch(`/setDist?v=${v}`);}
  function uF(type){
    let t = document.getElementById(type+'T').value;
    let c = document.getElementById(type+'C').value.replace('#','');
    fetch(`/updateFace?type=${type}&t=${encodeURIComponent(t)}&c=${c}`);
  }
  setInterval(()=>{
    fetch('/getDist').then(r=>r.text()).then(d=>{
      document.getElementById('dist').innerText = (d == "-1") ? "Idle/Manual" : d + " cm";
    });
  }, 500);
</script></body></html>)rawliteral";

// --- HELPERS ---
uint16_t rgb888To565(uint32_t rgb) {
  uint8_t r = (rgb >> 16) & 0xFF, g = (rgb >> 8) & 0xFF, b = rgb & 0xFF;
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void setMotorSpeed(int speedL, int speedR) {
  isMoving = (speedL != 0 || speedR != 0);
  int pwmL = (speedL == 0) ? LEFT_STOP_OFFSET : (speedL > 0 ? map(speedL, 0, 100, LEFT_STOP_OFFSET + 5, 180) : map(speedL, -100, 0, 0, LEFT_STOP_OFFSET - 5));
  int pwmR = (speedR == 0) ? RIGHT_STOP_OFFSET : (speedR > 0 ? map(speedR, 0, 100, RIGHT_STOP_OFFSET - 5, 0) : map(speedR, -100, 0, 180, RIGHT_STOP_OFFSET + 5));
  servoL.write(pwmL); servoR.write(pwmR);
}

float getDistance() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long d = pulseIn(PIN_ECHO, HIGH, 25000);
  return (d == 0) ? 999 : (d * 0.034 / 2);
}

void drawFace(String type) {
  tft.fillScreen(TFT_BLACK); tft.setTextSize(5); tft.setCursor(60, 60);
  if (type == "neutral") { tft.setTextColor(FACE_NEUTRAL.color); tft.print(FACE_NEUTRAL.text); }
  else if (type == "happy") { tft.setTextColor(FACE_HAPPY.color); tft.print(FACE_HAPPY.text); }
  else if (type == "fast") { tft.setTextColor(FACE_FAST.color); tft.print(FACE_FAST.text); }
}

void showResetScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.pushImage(0, 0, 320, 170, Logo); 
  tft.setTextColor(TFT_CYAN); tft.setTextSize(2); tft.setCursor(75, 145); tft.print("> Press Boot to Start <");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_POWER_ON, OUTPUT); digitalWrite(PIN_POWER_ON, HIGH); 
  pinMode(PIN_LCD_BL, OUTPUT); digitalWrite(PIN_LCD_BL, HIGH); 
  pinMode(PIN_BUTTON, INPUT_PULLUP); pinMode(PIN_KEY_BTN, INPUT_PULLUP); 

  tft.init(); tft.setRotation(1); tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
  tft.pushImage(0, 0, 320, 170, Logo); 
  delay(2000);
  showResetScreen();

  WiFi.softAP(AP_SSID, AP_PASS);

  server.on("/", []() { server.send(200, "text/html", index_html); });
  
  server.on("/getDist", []() { 
    if(robotState == 1 && isAutoStarted) server.send(200, "text/plain", String(getDistance())); 
    else server.send(200, "text/plain", "-1");
  });

  server.on("/startAuto", []() {
    if(robotState == 1) isAutoStarted = true;
    server.send(200);
  });

  server.on("/stopMotors", []() {
    setMotorSpeed(0, 0); // Kill power to servos
    isAutoStarted = false; // Pause auto-logic
    server.send(200);
  });

  server.on("/control", []() {
    if(robotState == 0) setMotorSpeed(server.arg("L").toInt(), server.arg("R").toInt());
    server.send(200);
  });

  server.on("/updateFace", []() {
    String type = server.arg("type");
    uint16_t col = rgb888To565(strtoul(server.arg("c").c_str(), NULL, 16));
    if(type == "n") { FACE_NEUTRAL.text = server.arg("t"); FACE_NEUTRAL.color = col; }
    else if(type == "h") { FACE_HAPPY.text = server.arg("t"); FACE_HAPPY.color = col; }
    else if(type == "f") { FACE_FAST.text = server.arg("t"); FACE_FAST.color = col; }
    currentFace = ""; server.send(200);
  });

  server.begin();
  servoL.attach(PIN_SERVO_LEFT, 500, 2400); 
  servoR.attach(PIN_SERVO_RIGHT, 500, 2400); 
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
}

void loop() {
  server.handleClient();

  if (digitalRead(PIN_KEY_BTN) == LOW) {
    setMotorSpeed(0, 0); robotState = -1; isAutoStarted = false; showResetScreen(); delay(500);           
  }

  if (digitalRead(PIN_BUTTON) == LOW) {
    delay(50); 
    if (digitalRead(PIN_BUTTON) == LOW) {
      while(digitalRead(PIN_BUTTON) == LOW); 
      setMotorSpeed(0, 0);
      isAutoStarted = false;
      robotState = (robotState == -1) ? 0 : (robotState == 0 ? 1 : 0);
      tft.fillScreen(TFT_BLACK); tft.setCursor(65, 70); tft.setTextSize(3);
      tft.setTextColor(TFT_GREEN); 
      tft.println(robotState == 1 ? "Auto Mode" : "WiFi Mode");
      delay(1000); currentFace = ""; 
    }
  }

  if (robotState == -1) return; 

  String faceType = (robotState == 1) ? "fast" : (!isMoving ? "neutral" : "happy");
  if (currentFace != faceType) { currentFace = faceType; drawFace(currentFace); }

  if (robotState == 1 && isAutoStarted) {
    float d = getDistance();
    if (d > 0 && d < set_distance) { setMotorSpeed(80, -80); } 
    else { setMotorSpeed(100, 100); } 
    delay(50);
  }
}