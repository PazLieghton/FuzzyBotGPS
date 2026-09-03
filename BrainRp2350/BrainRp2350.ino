/*
   Raspberry Pi Pico 2 W (RP2350) brain for FuzzyBot
   Based on ESP32 version v0.12
   By Paz Lieghton
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <TinyGPSPlus.h>
#include <Servo.h>    // Pico core Servo library

const char* ssid     = "HotspotPax";
const char* password = "78547854";

// ----- Pin definitions -----
const int GPS_RX_PIN = 1;      // UART1 RX = GPIO 1
const int GPS_TX_PIN = 0;      // UART1 TX = GPIO 0
const int MOTOR_RX_PIN = 9;    // UART2 RX = GPIO 9
const int MOTOR_TX_PIN = 8;    // UART2 TX = GPIO 8

const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
const int SERVO_PIN = 14;
const int BUZZER_PIN = 13;

const uint32_t GPS_BAUD = 9600;
const double MIN_SPEED_FOR_COURSE_KMPH = 1.5;

// ----- Obstacle avoidance parameters -----
const float OBSTACLE_THRESHOLD_CM = 30.0;
const float SCAN_LEFT_ANGLE = 120;
const float SCAN_RIGHT_ANGLE = 60;
const float SCAN_CENTER_ANGLE = 90;
const int   AVOID_TURN_TIME_MS = 1500;

bool obstacleAvoidanceActive = false;
unsigned long avoidanceStartTime = 0;
int avoidancePhase = 0;

float currentDistanceCm = 999.0;
unsigned long lastUltrasonicRead = 0;
const unsigned long ULTRASONIC_INTERVAL_MS = 200;

int currentServoAngle = SCAN_CENTER_ANGLE;

// ----- Motor rest cycle -----
const unsigned long REST_INTERVAL_MS = 10000;
const unsigned long REST_DURATION_MS  = 2000;
unsigned long lastMoveStart = 0;
bool resting = false;

// ----- Navigation constants -----
const unsigned long NAV_CONTROL_INTERVAL_MS = 800;
const unsigned long NAV_WAYPOINT_TIMEOUT_MS = 120000;
const unsigned long AUTO_GPS_TIMEOUT_MS = 300000;
float lastDistance = -1;
unsigned long lastProgressTime = 0;
const unsigned long STUCK_TIMEOUT_MS = 10000;

// ----- Other globals -----
const int MAX_WAYPOINTS = 10;
const double WAYPOINT_RADIUS_M = 5.0;
const double TURN_THRESHOLD_DEG = 28.0;
const double TURN_TOLERANCE_DEG = 8.0;

#define MAG_ADDR 0x1D
#define MAG_X_SIGN  -1
#define MAG_Y_SIGN  1
#define MAG_Z_SIGN  1

float gyroBias = 0.0;
float biasSum = 0.0;
int   biasCount = 0;
const int BIAS_SAMPLES = 200;
const float STATIONARY_THRESHOLD = 3.0;

WebServer server(80);
MPU6050 mpu(Wire);
TinyGPSPlus gps;
Servo servo;

struct Waypoint { double lat, lon; };
Waypoint waypoints[MAX_WAYPOINTS];
int numWaypoints = 0;
int currentWaypointIndex = 0;

bool autoMode = false;
enum NavState { NAV_IDLE, NAV_MOVING, NAV_TURNING, NAV_AVOIDING };
NavState navState = NAV_IDLE;

unsigned long lastNavAction = 0;
unsigned long autoStartTime = 0;
unsigned long waypointStartTime = 0;

unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000;

float fusedHeadingDeg = 0.0;
unsigned long lastGyroUpdate = 0;
int motorSpeed = 220;

String navDebug = "Idle";
String moodEmoji = "😊";

// ----- Prototypes -----
void handleRoot();
void handleCommand();
void handleTelemetry();
void handleAddWaypoint();
void handleClearWaypoints();
void handleStartAuto();
void handleStopAuto();
void handleGetWaypoints();
void handleRecalibrate();
void handleServo();
void navigate();
void performObstacleAvoidance();
void removeCurrentWaypoint();
double bearingTo(double lat1, double lon1, double lat2, double lon2);
double distanceTo(double lat1, double lon1, double lat2, double lon2);
double angleDifference(double bearing, double heading);
void sendCommand(char c);
float wrapDeg(float d);
float getFusedHeading();
float readMagHeading();
void updateFusedHeading();
float readUltrasonicDistance();
void setServoAngle(int angle);

void playTone(int freq, int durationMs);
void beepStartup();
void beepAutoStart();
void beepObstacle();
void beepAvoid();
void beepWaypoint();
void beepMissionComplete();

// ----- Helper functions -----
float wrapDeg(float d) {
  while (d > 180)  d -= 360;
  while (d < -180) d += 360;
  return d;
}

float readMagHeading() {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return NAN;
  Wire.requestFrom(MAG_ADDR, 6);
  if (Wire.available() != 6) return NAN;
  int x = Wire.read() | Wire.read() << 8;
  int y = Wire.read() | Wire.read() << 8;
  int z = Wire.read() | Wire.read() << 8;
  x *= MAG_X_SIGN;
  y *= MAG_Y_SIGN;
  z *= MAG_Z_SIGN;
  float heading = atan2(y, x) * 180.0 / PI;
  return wrapDeg(heading);
}

void updateFusedHeading() {
  unsigned long now = millis();
  float dt = (now - lastGyroUpdate) / 1000.0;
  lastGyroUpdate = now;
  if (dt <= 0 || dt > 1.0) dt = 0.01;
  float gyroZ = -mpu.getGyroZ();

  if (abs(gyroZ) < STATIONARY_THRESHOLD) {
    biasSum += gyroZ;
    biasCount++;
    if (biasCount >= BIAS_SAMPLES) {
      gyroBias = biasSum / biasCount;
      biasSum = 0.0;
      biasCount = 0;
      Serial.print("Updated gyro bias: ");
      Serial.println(gyroBias, 6);
    }
  } else {
    biasSum = 0.0;
    biasCount = 0;
  }

  fusedHeadingDeg = wrapDeg(fusedHeadingDeg + (gyroZ - gyroBias) * dt);
}

float getFusedHeading() {
  return fusedHeadingDeg;
}

float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float dist = duration * 0.0343 / 2.0;
  if (dist <= 0 || dist > 400) dist = 999.0;
  return dist;
}

void setServoAngle(int angle) {
  angle = constrain(angle, 0, 180);
  servo.write(angle);
  currentServoAngle = angle;
}

// ----- Buzzer functions (simple square wave) -----
void playTone(int freq, int durationMs) {
  if (freq <= 0 || durationMs <= 0) {
    digitalWrite(BUZZER_PIN, LOW);
    delay(durationMs);
    return;
  }
  unsigned long period = 1000000UL / freq;
  unsigned long half = period / 2;
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(half);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(half);
  }
  digitalWrite(BUZZER_PIN, LOW);
}

void beepStartup() { playTone(1000,100); playTone(1500,100); playTone(2000,200); }
void beepAutoStart() { playTone(800,100); playTone(1000,100); playTone(1200,150); }
void beepObstacle() { playTone(2000,150); playTone(2000,150); }
void beepAvoid() { playTone(1500,100); playTone(1800,100); }
void beepWaypoint() { playTone(1200,150); playTone(1600,150); playTone(2000,200); }
void beepMissionComplete() { playTone(1000,150); playTone(1300,150); playTone(1600,150); playTone(2000,300); }

// ----- Obstacle avoidance logic -----
void performObstacleAvoidance() {
  if (!obstacleAvoidanceActive) return;

  switch (avoidancePhase) {
    case 0:
      setServoAngle(SCAN_LEFT_ANGLE);
      delay(300);
      currentDistanceCm = readUltrasonicDistance();
      avoidancePhase = 1;
      break;
    case 1: {
      setServoAngle(SCAN_RIGHT_ANGLE);
      delay(300);
      float rightDist = readUltrasonicDistance();
      if (currentDistanceCm > rightDist) sendCommand('L');
      else sendCommand('R');
      beepAvoid();
      avoidancePhase = 2;
      avoidanceStartTime = millis();
      break;
    }
    case 2:
      if (millis() - avoidanceStartTime > AVOID_TURN_TIME_MS) {
        sendCommand('X');
        setServoAngle(SCAN_CENTER_ANGLE);
        obstacleAvoidanceActive = false;
        avoidancePhase = 0;
        navState = NAV_MOVING;
      }
      break;
  }
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Pico 2 W FuzzyBot brain starting...");

  // GPS on Serial1   
  Serial1.setRX(GPS_RX_PIN);
  Serial1.setTX(GPS_TX_PIN);
  Serial1.begin(GPS_BAUD);

  // Motor communication on Serial2
  Serial2.setRX(MOTOR_RX_PIN);
  Serial2.setTX(MOTOR_TX_PIN);
  Serial2.begin(9600);

  // I2C for MPU6050 and magnetometer
  Wire.setSDA(6);
  Wire.setSCL(7);
  Wire.begin();

  byte mpuStatus = mpu.begin();
  Serial.print("MPU6050 status: "); Serial.println(mpuStatus);
  if (mpuStatus == 0) {
    Serial.println("Calibrating MPU6050 - keep still...");
    mpu.calcOffsets(true, true);
    Serial.println("MPU6050 ready.");
  } else {
    Serial.println("MPU6050 not responding.");
  }

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x09);
  Wire.write(0x01);
  if (Wire.endTransmission() != 0) {
    Serial.println("Magnetometer init failed");
  } else {
    Serial.println("QMC5883L initialized");
  }

  delay(200);
  float initMag = readMagHeading();
  if (!isnan(initMag)) {
    fusedHeadingDeg = initMag;
    Serial.print("Initial heading from magnetometer: ");
    Serial.println(fusedHeadingDeg);
  } else {
    fusedHeadingDeg = 0.0;
  }

  gyroBias = 0.0;
  biasSum = 0.0;
  biasCount = 0;
  lastGyroUpdate = millis();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);
  setServoAngle(SCAN_CENTER_ANGLE);
  currentDistanceCm = 999.0;

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  beepStartup();

  lastMoveStart = millis();
  resting = false;

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to "); Serial.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.on("/telemetry", handleTelemetry);
  server.on("/addwaypoint", handleAddWaypoint);
  server.on("/clearwaypoints", handleClearWaypoints);
  server.on("/startauto", handleStartAuto);
  server.on("/stopauto", handleStopAuto);
  server.on("/waypoints", handleGetWaypoints);
  server.on("/recalibrate", handleRecalibrate);
  server.on("/servo", handleServo);

  server.begin();
  Serial.println("HTTP server started.");
}

// ----- Main loop -----
void loop() {
  if (millis() - lastWifiCheck > WIFI_CHECK_INTERVAL) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected - reconnecting...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
  }

  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  mpu.update();
  updateFusedHeading();

  if (!obstacleAvoidanceActive && currentServoAngle == SCAN_CENTER_ANGLE) {
    if (millis() - lastUltrasonicRead > ULTRASONIC_INTERVAL_MS) {
      lastUltrasonicRead = millis();
      currentDistanceCm = readUltrasonicDistance();
    }
  }

  if (autoMode && obstacleAvoidanceActive) performObstacleAvoidance();
  if (autoMode && !obstacleAvoidanceActive) navigate();

  if (!autoMode) moodEmoji = "😊";
  else if (obstacleAvoidanceActive) moodEmoji = "⚠️";
  else if (resting) moodEmoji = "😴";
  else moodEmoji = "🚗";

  server.handleClient();
  yield();
}

// ----- Telemetry -----
void handleTelemetry() {
  bool gpsValid = gps.location.isValid();
  double lat = gpsValid ? gps.location.lat() : 0.0;
  double lon = gpsValid ? gps.location.lng() : 0.0;
  int sats = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
  double hdopVal = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;
  double speedKmph = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  bool courseValid = gps.course.isValid() && gps.speed.isValid() && speedKmph > MIN_SPEED_FOR_COURSE_KMPH;
  double courseDeg = courseValid ? gps.course.deg() : 0.0;

  float gyroHeading = getFusedHeading();
  float pitch = mpu.getAngleX();
  float roll = mpu.getAngleY();

  double targetBearing = 0.0;
  double distanceM = 0.0;
  double headingError = 0.0;

  if (gpsValid && numWaypoints > 0) {
    targetBearing = bearingTo(lat, lon, waypoints[0].lat, waypoints[0].lon);
    distanceM = distanceTo(lat, lon, waypoints[0].lat, waypoints[0].lon);
    headingError = angleDifference(targetBearing, gyroHeading);
  }

  char buf[1300];
  snprintf(buf, sizeof(buf),
    "{\"gpsValid\":%s,\"lat\":%.6f,\"lon\":%.6f,\"satellites\":%d,\"hdop\":%.1f,"
    "\"courseValid\":%s,\"courseDeg\":%.1f,\"speedKmph\":%.2f,"
    "\"gyroHeadingDeg\":%.1f,\"pitchDeg\":%.1f,\"rollDeg\":%.1f,"
    "\"targetBearingDeg\":%.1f,\"distanceToWaypointM\":%.1f,\"headingErrorDeg\":%.1f,"
    "\"autoMode\":%s,\"waypoints\":%d,\"motorSpeed\":%d,\"debug\":\"%s\","
    "\"ultrasonicCm\":%.1f,\"servoAngle\":%d,\"mood\":\"%s\"}",
    gpsValid ? "true" : "false", lat, lon, sats, hdopVal,
    courseValid ? "true" : "false", courseDeg, speedKmph,
    gyroHeading, pitch, roll,
    targetBearing, distanceM, headingError,
    autoMode ? "true" : "false", numWaypoints, motorSpeed, navDebug.c_str(),
    currentDistanceCm, currentServoAngle, moodEmoji.c_str());

  server.send(200, "application/json", buf);
}

// ----- Servo manual control endpoint -----
void handleServo() {
  if (server.hasArg("angle")) {
    int angle = server.arg("angle").toInt();
    angle = constrain(angle, 0, 180);
    setServoAngle(angle);
    server.send(200, "text/plain", "Servo angle set to " + String(angle));
  } else {
    server.send(400, "text/plain", "Missing angle parameter");
  }
}

// ----- Navigation with obstacle check and motor rest -----
void navigate() {
  if (!gps.location.isValid()) {
    navDebug = "Waiting for GPS fix";
    if (millis() - autoStartTime > AUTO_GPS_TIMEOUT_MS) {
      autoMode = false;
      navState = NAV_IDLE;
      sendCommand('X');
      Serial.println("Auto mode aborted: GPS timeout.");
    }
    return;
  }

  if (numWaypoints == 0) {
    autoMode = false;
    navState = NAV_IDLE;
    sendCommand('X');
    navDebug = "All waypoints reached";
    beepMissionComplete();
    return;
  }

  if (resting) {
    if (millis() - lastMoveStart > REST_DURATION_MS) {
      resting = false;
      lastMoveStart = millis();
      navDebug = "Rest finished, moving again";
    } else {
      sendCommand('X');
      return;
    }
  } else if (millis() - lastMoveStart > REST_INTERVAL_MS) {
    resting = true;
    lastMoveStart = millis();
    sendCommand('X');
    navDebug = "Resting motors (2s)";
    return;
  }

  if (millis() - waypointStartTime > NAV_WAYPOINT_TIMEOUT_MS) {
    navDebug = "Waypoint timeout, skipping";
    removeCurrentWaypoint();
    if (numWaypoints > 0) {
      waypointStartTime = millis();
      navState = NAV_MOVING;
    } else {
      autoMode = false;
      navState = NAV_IDLE;
      sendCommand('X');
    }
    return;
  }

  if (millis() - lastNavAction < NAV_CONTROL_INTERVAL_MS) return;
  lastNavAction = millis();

  double curLat = gps.location.lat();
  double curLon = gps.location.lng();
  double wpLat = waypoints[0].lat;
  double wpLon = waypoints[0].lon;
  double dist = distanceTo(curLat, curLon, wpLat, wpLon);

  if (lastDistance < 0) lastDistance = dist;
  if (abs(dist - lastDistance) < 0.5) {
    if (millis() - lastProgressTime > STUCK_TIMEOUT_MS) {
      navDebug = "Stuck! Executing evasion";
      sendCommand('R');
      beepObstacle();
      lastProgressTime = millis();
      lastNavAction = millis();
      lastDistance = dist;
      return;
    }
  } else {
    lastDistance = dist;
    lastProgressTime = millis();
  }

  if (dist < WAYPOINT_RADIUS_M) {
    sendCommand('X');
    navDebug = "Arrived at waypoint";
    beepWaypoint();
    removeCurrentWaypoint();
    if (numWaypoints > 0) {
      waypointStartTime = millis();
      navState = NAV_MOVING;
    } else {
      autoMode = false;
      navState = NAV_IDLE;
      beepMissionComplete();
    }
    return;
  }

  double bearing = bearingTo(curLat, curLon, wpLat, wpLon);
  double heading = getFusedHeading();
  double headingError = angleDifference(bearing, heading);

  char debugBuf[120];
  snprintf(debugBuf, sizeof(debugBuf), "Dist %.1fm, Bear %.0f, Head %.0f, Err %.0f", dist, bearing, heading, headingError);
  navDebug = String(debugBuf);

  char intendedCmd = 'X';
  if (headingError > TURN_THRESHOLD_DEG) {
    intendedCmd = 'R';
    navState = NAV_TURNING;
    navDebug += " | Turning R";
  } else if (headingError < -TURN_THRESHOLD_DEG) {
    intendedCmd = 'L';
    navState = NAV_TURNING;
    navDebug += " | Turning L";
  } else {
    intendedCmd = 'F';
    navState = NAV_MOVING;
    navDebug += " | Forward";
  }

  if (intendedCmd == 'F') {
    currentDistanceCm = readUltrasonicDistance();
    if (currentDistanceCm < OBSTACLE_THRESHOLD_CM) {
      sendCommand('X');
      obstacleAvoidanceActive = true;
      avoidancePhase = 0;
      avoidanceStartTime = millis();
      navState = NAV_AVOIDING;
      navDebug = "Obstacle detected! Avoiding...";
      beepObstacle();
      return;
    }
  }

  sendCommand(intendedCmd);
}

// ----- Add waypoint -----
void handleAddWaypoint() {
  if (server.hasArg("lat") && server.hasArg("lon")) {
    if (numWaypoints >= MAX_WAYPOINTS) {
      server.send(400, "text/plain", "Waypoint limit reached");
      return;
    }
    double lat = server.arg("lat").toDouble();
    double lon = server.arg("lon").toDouble();
    waypoints[numWaypoints].lat = lat;
    waypoints[numWaypoints].lon = lon;
    numWaypoints++;
    Serial.printf("Added waypoint %d: %.6f, %.6f\n", numWaypoints, lat, lon);
    server.send(200, "text/plain", "Waypoint added");
  } else {
    server.send(400, "text/plain", "Missing lat/lon");
  }
}

void handleClearWaypoints() {
  numWaypoints = 0;
  currentWaypointIndex = 0;
  if (autoMode) {
    autoMode = false;
    navState = NAV_IDLE;
    sendCommand('X');
  }
  server.send(200, "text/plain", "Waypoints cleared");
}

void handleStartAuto() {
  if (numWaypoints == 0) {
    server.send(400, "text/plain", "No waypoints set");
    return;
  }
  autoMode = true;
  currentWaypointIndex = 0;
  navState = NAV_MOVING;
  lastNavAction = 0;
  autoStartTime = millis();
  waypointStartTime = millis();
  sendCommand('-');
  sendCommand('-');
  lastMoveStart = millis();
  resting = false;
  beepAutoStart();
  Serial.println("Auto mode started.");
  server.send(200, "text/plain", "Auto mode started");
}

void handleStopAuto() {
  autoMode = false;
  navState = NAV_IDLE;
  sendCommand('X');
  sendCommand('+');
  sendCommand('+');
  obstacleAvoidanceActive = false;
  setServoAngle(SCAN_CENTER_ANGLE);
  Serial.println("Auto mode stopped.");
  server.send(200, "text/plain", "Auto mode stopped");
}

void handleGetWaypoints() {
  String json = "[";
  for (int i = 0; i < numWaypoints; i++) {
    if (i > 0) json += ",";
    json += "{\"lat\":" + String(waypoints[i].lat, 6) + ",\"lon\":" + String(waypoints[i].lon, 6) + "}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleRecalibrate() {
  float magSum = 0.0;
  int magCount = 0;
  for (int i = 0; i < 10; i++) {
    float h = readMagHeading();
    if (!isnan(h)) {
      magSum += h;
      magCount++;
    }
    delay(50);
  }
  if (magCount > 0) {
    fusedHeadingDeg = wrapDeg(magSum / magCount);
    Serial.print("Recalibrated heading: ");
    Serial.println(fusedHeadingDeg);
    server.send(200, "text/plain", "Recalibrated: " + String(fusedHeadingDeg));
  } else {
    server.send(500, "text/plain", "Magnetometer read failed");
  }
}

void sendCommand(char c) {
  Serial2.print(c);
  Serial.print("Sent to Arduino: ");
  Serial.println(c);
}

double bearingTo(double lat1, double lon1, double lat2, double lon2) {
  double phi1 = lat1 * PI / 180.0;
  double phi2 = lat2 * PI / 180.0;
  double lambda1 = lon1 * PI / 180.0;
  double lambda2 = lon2 * PI / 180.0;
  double y = sin(lambda2 - lambda1) * cos(phi2);
  double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(lambda2 - lambda1);
  double theta = atan2(y, x);
  return fmod((theta * 180.0 / PI + 360.0), 360.0);
}

double distanceTo(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double phi1 = lat1 * PI / 180.0;
  double phi2 = lat2 * PI / 180.0;
  double dphi = (lat2 - lat1) * PI / 180.0;
  double dlambda = (lon2 - lon1) * PI / 180.0;
  double a = sin(dphi/2) * sin(dphi/2) +
             cos(phi1) * cos(phi2) *
             sin(dlambda/2) * sin(dlambda/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

double angleDifference(double bearing, double heading) {
  double diff = bearing - heading;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}

void removeCurrentWaypoint() {
  if (numWaypoints == 0) return;
  for (int i = 0; i < numWaypoints - 1; i++) {
    waypoints[i] = waypoints[i + 1];
  }
  numWaypoints--;
  currentWaypointIndex = 0;
}

void handleCommand() {
  if (server.hasArg("c")) {
    String cmd = server.arg("c");
    if (cmd.length() == 1) {
      char c = cmd.charAt(0);
      if (autoMode && c != 'X') {
        server.send(403, "text/plain", "Auto mode active, send X to stop");
        return;
      }
      if (c == 'X') {
        autoMode = false;
        navState = NAV_IDLE;
      }
      if (c == '+') {
        motorSpeed = min(motorSpeed + 20, 255);
      } else if (c == '-') {
        motorSpeed = max(motorSpeed - 20, 0);
      }
      sendCommand(c);
      server.send(200, "text/plain", "Sent: " + cmd);
      return;
    }
  }
  server.send(400, "text/plain", "Bad command");
}

// ------------------------------ HTML ------------------------------
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <meta name="theme-color" content="#0b1220">
  <title>FuzzyBot Control</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <style>
    :root { --bg:#eef2f7; --surface:#ffffff; --surface2:#f7f9fc; --primary:#3b82f6; --primary2:#2563eb; --danger:#ef4444; --success:#10b981; --warning:#f59e0b; --text:#172033; --muted:#6b7280; --border:#dfe5ee; --shadow:0 10px 30px rgba(15,23,42,.08); }
    [data-theme=dark] { --bg:#08111f; --surface:#101a2b; --surface2:#0c1524; --primary:#60a5fa; --primary2:#3b82f6; --danger:#f87171; --success:#34d399; --warning:#fbbf24; --text:#edf4ff; --muted:#91a0b8; --border:#223149; --shadow:0 16px 34px rgba(0,0,0,.28); }
    * { box-sizing:border-box; font-family:Inter,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif; }
    html,body { height:100%; margin:0; background:var(--bg); color:var(--text); }
    body { overflow:hidden; }
    button { font:inherit; }
    header { height:64px; display:flex; justify-content:space-between; align-items:center; padding:0 22px; background:var(--surface); border-bottom:1px solid var(--border); }
    .brand { display:flex; align-items:center; gap:8px; }
    .brand h1 { margin:0; font-size:18px; font-weight:700; letter-spacing:-.02em; }
    .brand .subtitle { margin:0; font-size:10px; color:var(--muted); line-height:1.3; }
    .header-actions { display:flex; align-items:center; gap:10px; }
    .conn { display:flex; align-items:center; gap:7px; font-size:13px; color:var(--muted); }
    .dot { width:9px; height:9px; border-radius:50%; background:var(--success); box-shadow:0 0 0 4px rgba(16,185,129,.10); }
    .dot.offline { background:var(--danger); box-shadow:0 0 0 4px rgba(239,68,68,.10); }
    .icon-btn { width:36px; height:36px; border-radius:10px; border:1px solid var(--border); background:var(--surface2); color:var(--text); cursor:pointer; display:flex; align-items:center; justify-content:center; font-size:18px; }
    .main { display:flex; height:calc(100vh - 64px); gap:16px; padding:16px; }
    #map { flex:1; min-width:0; border-radius:18px; border:1px solid var(--border); box-shadow:var(--shadow); overflow:hidden; z-index:1; }
    .sidebar { width:360px; flex-shrink:0; overflow-y:auto; display:flex; flex-direction:column; gap:12px; padding-right:2px; }
    .card { background:var(--surface); border:1px solid var(--border); border-radius:16px; padding:15px; box-shadow:var(--shadow); }
    .card-head { display:flex; align-items:center; justify-content:space-between; gap:10px; margin-bottom:12px; }
    .card-title { margin:0; font-size:11px; text-transform:uppercase; letter-spacing:.09em; font-weight:750; color:var(--muted); }
    .badge { padding:5px 8px; border-radius:999px; font-size:11px; font-weight:750; background:var(--surface2); color:var(--muted); border:1px solid var(--border); }
    .badge.active { background:rgba(59,130,246,.12); color:var(--primary); border-color:rgba(59,130,246,.22); }
    .telemetry-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
    .metric { padding:10px 11px; border:1px solid var(--border); background:var(--surface2); border-radius:12px; }
    .metric span { display:block; color:var(--muted); font-size:11px; margin-bottom:4px; }
    .metric strong { font-size:18px; letter-spacing:-.03em; }
    .compass-wrap { display:flex; gap:14px; align-items:center; margin-bottom:12px; }
    .compass { width:94px; height:94px; flex:0 0 94px; border-radius:50%; border:2px solid var(--border); position:relative; background:radial-gradient(circle,var(--surface2) 0 62%,transparent 63%),var(--surface); box-shadow:inset 0 0 0 8px rgba(127,127,127,.03); }
    .compass::before { content:"N"; position:absolute; left:50%; top:5px; transform:translateX(-50%); color:var(--danger); font-weight:800; font-size:11px; }
    .compass::after { content:"W    S  E"; position:absolute; left:50%; bottom:8px; transform:translateX(-50%); color:var(--muted); font-size:8px; letter-spacing:7px; white-space:nowrap; }
    .compass-needle { position:absolute; top:50%; left:50%; width:0; height:0; border-left:7px solid transparent; border-right:7px solid transparent; border-bottom:36px solid var(--danger); transform-origin:50% 100%; margin-left:-7px; margin-top:-36px; transition:transform .18s ease-out; filter:drop-shadow(0 2px 2px rgba(0,0,0,.12)); }
    .big-heading { font-size:31px; font-weight:760; letter-spacing:-.05em; }
    .subtle { font-size:11px; color:var(--muted); }
    .nav-strip { position:relative; height:34px; border-radius:10px; background:var(--surface2); border:1px solid var(--border); overflow:hidden; margin-top:11px; }
    .nav-center { position:absolute; left:50%; top:0; bottom:0; width:2px; background:var(--text); opacity:.35; }
    .nav-target { position:absolute; top:5px; width:4px; height:24px; border-radius:6px; background:var(--danger); box-shadow:0 0 0 4px rgba(239,68,68,.10); transition:left .2s ease; }
    .error-pill { margin-top:8px; display:flex; justify-content:space-between; align-items:center; font-size:12px; }
    .error-pill strong { color:var(--primary); }
    .grid-dpad { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; }
    .grid-dpad button { height:48px; }
    .grid-2 { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
    button.ctrl { padding:11px 10px; border:0; border-radius:10px; background:var(--primary); color:#fff; font-weight:700; cursor:pointer; transition:transform .08s,opacity .15s; display:flex; align-items:center; justify-content:center; gap:4px; }
    button.ctrl:active { transform:translateY(1px) scale(.99); opacity:.82; }
    .danger { background:var(--danger)!important; }
    .secondary { background:var(--surface2)!important; color:var(--text)!important; border:1px solid var(--border)!important; }
    .mission { display:grid; grid-template-columns:auto 1fr auto; gap:10px; align-items:center; padding:11px; border-radius:12px; background:linear-gradient(135deg,rgba(59,130,246,.10),rgba(59,130,246,.03)); border:1px solid rgba(59,130,246,.16); }
    .mission-icon { width:34px; height:34px; border-radius:10px; display:grid; place-items:center; background:rgba(59,130,246,.15); color:var(--primary); font-weight:800; }
    .mission-main strong { display:block; font-size:15px; }
    .mission-main span { display:block; font-size:11px; color:var(--muted); margin-top:2px; }
    .mission-distance { font-size:18px; font-weight:760; }
    .waypoint-list { margin-top:10px; padding:9px; background:var(--surface2); border:1px solid var(--border); border-radius:10px; font-size:12px; max-height:120px; overflow:auto; }
    .waypoint-list div { padding:5px 2px; border-bottom:1px solid var(--border); }
    .waypoint-list div:last-child { border-bottom:none; }
    .bar { height:8px; background:var(--surface2); border:1px solid var(--border); border-radius:999px; overflow:hidden; }
    .bar > div { height:100%; width:0%; border-radius:999px; transition:width .25s ease; background:linear-gradient(90deg,var(--primary),#8b5cf6); }
    .split { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
    .mini-label { font-size:10px; color:var(--muted); margin-bottom:5px; text-transform:uppercase; letter-spacing:.06em; }
    .debug { margin-top:9px; background:#0b1220; color:#b9f6cf; border-radius:10px; padding:10px; font:11px/1.5 ui-monospace,SFMono-Regular,Consolas,monospace; min-height:40px; }
    [data-theme=dark] .debug { background:#050b14; }
    .footer-note { color:var(--muted); font-size:10px; line-height:1.4; margin-top:9px; }
    .dir-indicator { display:flex; align-items:center; justify-content:center; gap:6px; margin-bottom:8px; font-size:22px; font-weight:700; color:var(--primary); }
    .dir-indicator span { background:var(--surface2); border:1px solid var(--border); border-radius:8px; padding:4px 10px; }
    .robot-arrow { font-size:22px; color:var(--primary); transform-origin:50% 50%; text-shadow:0 0 4px var(--surface); transition:transform .18s ease-out; display:inline-block; }
    .leaflet-control.recenter-ctrl button { padding:8px 11px; font-size:12px; border-radius:8px; }
    @media (max-width:900px) { 
      body{overflow:auto;} 
      .main{height:calc(100vh - 64px); min-height:0; flex-direction:row; padding:8px; gap:8px;}
      #map{flex:1; min-width:0; height:auto; min-height:0;}
      .sidebar{width:48%; min-width:200px; overflow-y:auto; padding-right:0;}
    }
  </style>
</head>
<body>
<header>
  <div class="brand">
    <div><h1>&#129302; FuzzyBot</h1><p class="subtitle">Autonomous Version 0.12<br>AI-assisted UI &bull; UNSAM &bull; By Paz Lieghton &bull; CREA</p></div>
  </div>
  <div class="header-actions">
    <div class="conn"><span class="dot" id="connDot"></span><span id="connText">Connected</span></div>
    <button class="icon-btn" id="themeBtn" onclick="toggleTheme()" title="Toggle theme">&#9790;</button>
  </div>
</header>
<div class="main">
  <div id="map"></div>
  <div class="sidebar">
    <div class="card">
      <div class="card-head"><h2 class="card-title">Telemetry</h2><span class="badge" id="gpsBadge">NO FIX</span></div>
      <div style="display:flex; align-items:center; gap:8px; margin-bottom:10px;">
        <span style="font-size:28px;" id="moodEmoji">😊</span>
        <span class="subtle">Robot mood</span>
      </div>
      <div class="compass-wrap">
        <div class="compass"><div class="compass-needle" id="compassNeedle"></div></div>
        <div><div class="subtle">NORTH HEADING</div><div class="big-heading" id="headingValue">--&deg;</div><div class="subtle" id="headingSource">gyro (bias corrected)</div></div>
      </div>
      <div class="telemetry-grid">
        <div class="metric"><span>TARGET</span><strong id="targetValue">--&deg;</strong></div>
        <div class="metric"><span>ERROR</span><strong id="errorValue">--&deg;</strong></div>
        <div class="metric"><span>DISTANCE</span><strong id="distanceValue">-- m</strong></div>
        <div class="metric"><span>SPEED</span><strong id="speedValue">--</strong></div>
      </div>
      <div class="nav-strip"><div class="nav-center"></div><div class="nav-target" id="navTarget"></div></div>
      <div class="error-pill"><span>Target direction</span><strong id="directionHint">--</strong></div>
    </div>

    <div class="card">
      <div class="card-head"><h2 class="card-title">Sensors</h2><span class="badge">LIVE</span></div>
      <div class="metric"><span>ULTRASONIC DISTANCE</span><strong id="distanceCm">-- cm</strong></div>
      <div style="margin-top:10px;"><div class="mini-label">SERVO ANGLE</div>
        <input type="range" min="0" max="180" value="90" id="servoSlider" oninput="setServo(this.value)" style="width:100%;">
        <span id="servoAngleDisplay">90°</span>
      </div>
    </div>

    <div class="card">
      <div class="card-head"><h2 class="card-title">Manual control</h2><span class="badge">HOLD</span></div>
      <div class="dir-indicator"><span id="dirIndicator">&#9632;</span></div>
      <div class="grid-dpad">
        <div></div><button class="ctrl" onmousedown="startMove('F')" onmouseup="stopMove()" onmouseleave="stopMove()" ontouchstart="startMove('F')" ontouchend="stopMove()">&#8593;</button><div></div>
        <button class="ctrl" onmousedown="startMove('L')" onmouseup="stopMove()" onmouseleave="stopMove()" ontouchstart="startMove('L')" ontouchend="stopMove()">&#8592;</button>
        <button class="ctrl danger" onclick="send('X')">&#9632;</button>
        <button class="ctrl" onmousedown="startMove('R')" onmouseup="stopMove()" onmouseleave="stopMove()" ontouchstart="startMove('R')" ontouchend="stopMove()">&#8594;</button>
        <div></div><button class="ctrl" onmousedown="startMove('B')" onmouseup="stopMove()" onmouseleave="stopMove()" ontouchstart="startMove('B')" ontouchend="stopMove()">&#8595;</button><div></div>
      </div>
      <div class="grid-2" style="margin-top:10px;"><button class="ctrl secondary" onclick="send('-')">- Speed</button><button class="ctrl secondary" onclick="send('+')">+ Speed</button></div>
      <div style="margin-top:12px;"><div class="mini-label">Motor output</div><div class="bar"><div id="speedBar"></div></div></div>
    </div>

    <div class="card">
      <div class="card-head"><h2 class="card-title">Mission</h2><span class="badge" id="autoBadge">OFF</span></div>
      <div class="mission"><div class="mission-icon">&#8962;</div><div class="mission-main"><strong id="missionText">Standby</strong><span id="missionSub">Add a waypoint on the map</span></div><div class="mission-distance" id="missionDistance">--</div></div>
      <div class="grid-2" style="margin-top:10px;"><button class="ctrl" onclick="startAuto()">&#9654; Start</button><button class="ctrl danger" onclick="stopAuto()">&#9632; Stop</button></div>
      <button class="ctrl secondary" onclick="clearWaypoints()" style="width:100%;margin-top:8px;">Clear waypoints</button>
      <div class="waypoint-list" id="waypointList">No waypoints. Tap the map to add.</div>
    </div>

    <div class="card">
      <div class="card-head"><h2 class="card-title">System</h2><span class="badge" id="satBadge">SAT --</span></div>
      <div class="split">
        <div><div class="mini-label">GPS quality</div><div class="bar"><div id="gpsBar"></div></div></div>
        <div><div class="mini-label">Heading error</div><div class="bar"><div id="errorBar"></div></div></div>
      </div>
      <div class="debug" id="debugInfo">Idle</div>
      <div class="grid-2" style="margin-top:10px;"><button class="ctrl secondary" onclick="recalibrate()">Recalibrate</button><button class="ctrl secondary" onclick="recenterRobot()">Recenter</button></div>
      <div class="footer-note">Heading from gyro with automatic bias correction. Recalibrate uses magnetometer once to set absolute north.</div>
    </div>
  </div>
</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
  const savedTheme = localStorage.getItem('fuzzybot-theme');
  document.documentElement.dataset.theme = savedTheme || 'light';
  updateThemeButton();
  function toggleTheme(){ const next=(document.documentElement.dataset.theme==='dark')?'light':'dark'; document.documentElement.dataset.theme=next; localStorage.setItem('fuzzybot-theme',next); updateThemeButton(); }
  function updateThemeButton(){ document.getElementById('themeBtn').innerHTML = document.documentElement.dataset.theme==='dark' ? '&#9728;' : '&#9790;'; }

  let lastCommand = null;
  function send(c){ 
    lastCommand = c;
    updateDirIndicator(c);
    fetch('/cmd?c='+encodeURIComponent(c)).then(()=>setConnected(true)).catch(()=>setConnected(false)); 
  }
  function updateDirIndicator(c) {
    const map = {'F':'&#8593;','B':'&#8595;','L':'&#8592;','R':'&#8594;','X':'&#9632;'};
    const el = document.getElementById('dirIndicator');
    if (el) el.innerHTML = map[c] || '-';
  }
  function startMove(c){ send(c); }
  function stopMove(){ send('X'); }
  function stopAuto(){ fetch('/stopauto').then(()=>{ autoActive=false; updateAutoBadge(); }); }
  function startAuto(){ fetch('/startauto').then(()=>{ autoActive=true; updateAutoBadge(); }); }
  function recalibrate(){ if(confirm('Keep the robot stationary and press OK to recalibrate heading.')) fetch('/recalibrate').then(r=>r.text()).then(t=>alert(t)); }
  function setConnected(ok){ document.getElementById('connDot').className=ok?'dot':'dot offline'; document.getElementById('connText').innerText=ok?'Connected':'Disconnected'; }
  function setServo(angle) {
    fetch('/servo?angle='+angle).then(r=>r.text()).then(t=>{ document.getElementById('servoAngleDisplay').innerText = angle+'°'; });
  }

  let waypointMarkers=[]; let routePolyline=null; let targetMarker=null; let robotMarker=null; let robotArrowEl=null; let autoActive=false;
  const map=L.map('map').setView([0,0],2);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);
  function ensureMarker(lat,lon){ if(robotMarker)return; const icon=L.divIcon({className:'',html:'<div class="robot-arrow" id="robotArrow">&#9650;</div>',iconSize:[22,22],iconAnchor:[11,11]}); robotMarker=L.marker([lat,lon],{icon}).addTo(map); robotArrowEl=document.getElementById('robotArrow'); map.setView([lat,lon],18); }
  function recenterRobot(){ if(robotMarker) map.setView(robotMarker.getLatLng(),map.getZoom()); }
  const RecenterControl=L.Control.extend({options:{position:'topright'},onAdd:function(){const div=L.DomUtil.create('div','leaflet-bar recenter-ctrl'); div.innerHTML='<button type="button">Recenter</button>'; L.DomEvent.disableClickPropagation(div); div.querySelector('button').onclick=recenterRobot; return div;}}); map.addControl(new RecenterControl());
  function updateWaypointList(){ const d=document.getElementById('waypointList'); if(!waypointMarkers.length){d.innerHTML='No waypoints. Tap the map to add.';return;} d.innerHTML=waypointMarkers.map((m,i)=>`<div>WP${i+1} &middot; ${m.getLatLng().lat.toFixed(5)}, ${m.getLatLng().lng.toFixed(5)}</div>`).join(''); }
  function updateRouteLine(){ if(routePolyline)map.removeLayer(routePolyline); if(waypointMarkers.length){ const pts=[]; if(robotMarker) pts.push(robotMarker.getLatLng()); waypointMarkers.forEach(m=>pts.push(m.getLatLng())); if(pts.length>1) routePolyline=L.polyline(pts,{color:'#3b82f6',weight:4,dashArray:'8,8',opacity:.78}).addTo(map); }}
  function updateTargetMarker(){ if(targetMarker)map.removeLayer(targetMarker); if(waypointMarkers.length)targetMarker=L.circle(waypointMarkers[0].getLatLng(),{radius:5,color:'#ef4444',fillColor:'#ef4444',fillOpacity:.35,weight:2}).addTo(map); }
  function clearWaypoints(){ if(!confirm('Clear all waypoints?'))return; fetch('/clearwaypoints').then(()=>{waypointMarkers.forEach(m=>map.removeLayer(m)); waypointMarkers=[]; if(routePolyline)map.removeLayer(routePolyline); if(targetMarker)map.removeLayer(targetMarker); routePolyline=null; targetMarker=null; updateWaypointList(); autoActive=false; updateAutoBadge(); }); }
  function updateAutoBadge(){ const b=document.getElementById('autoBadge'); b.className=autoActive?'badge active':'badge'; b.innerText=autoActive?'ACTIVE':'OFF'; }
  map.on('click',e=>{ if(autoActive){alert('Stop auto mode before adding waypoints');return;} fetch(`/addwaypoint?lat=${e.latlng.lat}&lon=${e.latlng.lng}`).then(()=>{waypointMarkers.push(L.marker([e.latlng.lat,e.latlng.lng]).addTo(map)); updateWaypointList(); updateRouteLine(); updateTargetMarker();}); });

  function clamp(v,a,b){return Math.max(a,Math.min(b,v));}
  function directionText(err){ if(err>6)return 'Turn right'; if(err<-6)return 'Turn left'; return 'On course'; }
  setInterval(async()=>{ try{ const r=await fetch('/telemetry'); const t=await r.json(); setConnected(true);
    const h=t.gyroHeadingDeg; const err=t.headingErrorDeg||0; const dist=t.distanceToWaypointM||0; const target=t.targetBearingDeg||0;
    document.getElementById('headingValue').innerHTML=Math.round(h)+'&deg;';
    document.getElementById('targetValue').innerHTML=t.waypoints?Math.round(target)+'&deg;':'--&deg;';
    document.getElementById('errorValue').innerHTML=t.waypoints?(err>0?'+':'')+Math.round(err)+'&deg;':'--&deg;';
    document.getElementById('distanceValue').innerHTML=t.waypoints?(dist<100?dist.toFixed(1):Math.round(dist))+' m':'-- m';
    document.getElementById('speedValue').innerText=t.motorSpeed;
    document.getElementById('headingSource').innerText='gyro (bias corrected)';
    document.getElementById('directionHint').innerText=t.waypoints?directionText(err):'No target';
    document.getElementById('navTarget').style.left=(clamp((err+90)/180,0,1)*100)+'%';
    document.getElementById('speedBar').style.width=((t.motorSpeed/255)*100)+'%';
    document.getElementById('errorBar').style.width=(clamp(Math.abs(err)/90,0,1)*100)+'%';
    const gpsQuality=t.gpsValid?clamp(100-(Math.max(0,t.hdop-1)*28),12,100):0; document.getElementById('gpsBar').style.width=gpsQuality+'%';
    document.getElementById('gpsBadge').innerText=t.gpsValid?'GPS FIX':'NO FIX'; document.getElementById('satBadge').innerText='SAT '+(t.satellites||'--');
    document.getElementById('gpsBadge').className=t.gpsValid?'badge active':'badge';
    document.getElementById('satBadge').className=(t.satellites>=5)?'badge active':'badge';
    document.getElementById('debugInfo').innerText=t.debug||'Idle';
    document.getElementById('missionText').innerText=t.autoMode?(t.waypoints?'Auto navigation':'Finishing mission'):'Standby';
    document.getElementById('missionSub').innerText=t.waypoints?('Next waypoint - '+t.waypoints+' remaining'):'Add a waypoint on the map';
    document.getElementById('missionDistance').innerText=t.waypoints?(dist<100?dist.toFixed(1):Math.round(dist))+'m':'--';
    const rot='rotate('+h+'deg)'; document.getElementById('compassNeedle').style.transform=rot; if(robotArrowEl)robotArrowEl.style.transform=rot;
    if(t.gpsValid){ ensureMarker(t.lat,t.lon); robotMarker.setLatLng([t.lat,t.lon]); updateRouteLine(); }
    if(t.autoMode!==autoActive){autoActive=t.autoMode;updateAutoBadge();}
    if(t.waypoints<waypointMarkers.length){const gone=waypointMarkers.shift();if(gone)map.removeLayer(gone);updateWaypointList();updateRouteLine();updateTargetMarker();}
    // Ultrasonic display
    document.getElementById('distanceCm').innerText = (t.ultrasonicCm > 400 ? '--' : t.ultrasonicCm.toFixed(1))+' cm';
    // Servo display
    document.getElementById('servoAngleDisplay').innerText = t.servoAngle+'°';
    document.getElementById('servoSlider').value = t.servoAngle;
    // Mood emoji
    document.getElementById('moodEmoji').innerText = t.mood || '😊';
  }catch(e){setConnected(false);} },500);
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}