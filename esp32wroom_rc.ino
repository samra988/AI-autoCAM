// ============================================================
// ESP32-WROOM-32 — FIXED TIMING & PIN INITIALIZATION
// ============================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

const char* WIFI_SSID = ":)";
const char* WIFI_PASS = "abcd@1234";
const int   UDP_PORT  = 4210;

#define ENA 25
#define IN1 26
#define IN2 27
#define IN3 14
#define IN4 12
#define ENB 13

// [CRITICAL OVERRIDE] Shifting to core free peripherals to avoid data collision
#define PAN_PIN  4
#define TILT_PIN 2

#define PAN_MIN   20
#define PAN_MAX  160
#define TILT_MIN  40
#define TILT_MAX 140

#define MAX_SPD 240

WiFiUDP udp;
Servo   panServo;
Servo   tiltServo;

int   currentPan  = 90;
int   currentTilt = 90;
unsigned long lastCmd = 0;
#define CMD_TIMEOUT 600   // ms

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void setMotors(int l, int r) {
  analogWrite(ENA, constrain(abs(l), 0, MAX_SPD));
  analogWrite(ENB, constrain(abs(r), 0, MAX_SPD));
  digitalWrite(IN1, l >= 0 ? HIGH : LOW);
  digitalWrite(IN2, l <  0 ? HIGH : LOW);
  digitalWrite(IN3, r >= 0 ? HIGH : LOW);
  digitalWrite(IN4, r <  0 ? HIGH : LOW);
}

void setPan(int angle) {
  currentPan = constrain(angle, PAN_MIN, PAN_MAX);
  panServo.write(currentPan);
}

void setTilt(int angle) {
  currentTilt = constrain(angle, TILT_MIN, TILT_MAX);
  tiltServo.write(currentTilt);
}

// [PREVIOUS SEPARATION IMPLEMENTATION INTEGRATED]
void handleCommand(String raw) {
  raw.trim();
  if (!raw.startsWith("CMD:")) return;

  lastCmd = millis();
  String body = raw.substring(4);

  int pi = body.indexOf("PAN:");
  int ti = body.indexOf("TILT:");

  if (pi >= 0) {
    int endIdx = body.indexOf(',', pi);
    String panStr = (endIdx > pi) ? body.substring(pi + 4, endIdx) : body.substring(pi + 4);
    setPan(panStr.toInt());
  }
  
  if (ti >= 0) {
    int endIdx = body.indexOf(',', ti);
    String tiltStr = (endIdx > ti) ? body.substring(ti + 5, endIdx) : body.substring(ti + 5);
    setTilt(tiltStr.toInt());
  }

  int c1 = body.indexOf(',');
  int c2 = body.indexOf(',', c1 + 1);
  
  if (c1 > 0) {
    String dir = body.substring(0, c1);
    int lPWM = 0;
    int rPWM = 0;

    if (c2 > c1) {
      lPWM = body.substring(c1 + 1, c2).toInt();
      int c3 = body.indexOf(',', c2 + 1);
      String rPWMStr = (c3 > c2) ? body.substring(c2 + 1, c3) : body.substring(c2 + 1);
      rPWM = rPWMStr.toInt();
    }

    if      (dir == "STOP")     stopMotors();
    else if (dir == "FORWARD")  setMotors( lPWM,  rPWM);
    else if (dir == "BACKWARD") setMotors(-lPWM, -rPWM);
    else if (dir == "LEFT")     setMotors( lPWM,  rPWM);
    else if (dir == "RIGHT")    setMotors( lPWM,  rPWM);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  Serial.print("Connecting WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\nWiFi OK — IP: %s\n", WiFi.localIP().toString().c_str());
  udp.begin(UDP_PORT);

  // Allocate safe non-camera background peripheral timers
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  
  panServo.attach(PAN_PIN, 500, 2400);
  tiltServo.attach(TILT_PIN, 500, 2400);
  
  panServo.write(90);
  tiltServo.write(90);

  Serial.println("READY AND UNLOCKED");
}

void loop() {
  int sz;
  String lastPacket = "";
  
  while ((sz = udp.parsePacket()) > 0) {
    char buf[128] = {0};
    udp.read(buf, sizeof(buf) - 1);
    lastPacket = String(buf);
  }
  
  if (lastPacket.length() > 0) {
    handleCommand(lastPacket);
  }

  if (millis() - lastCmd > CMD_TIMEOUT) {
    stopMotors();
  }
  delay(1);
}