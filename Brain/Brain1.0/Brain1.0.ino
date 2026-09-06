/*
   ESP32 WiFi remote brain for FuzzyBot - v2.1 (FreeRTOS) - PATCHED
   Heading: gyro integration with automatic bias removal.
   Magnetometer only used once at startup and on manual Recalibrate.
   Obstacle avoidance with HC-SR04 and SG90 servo during auto mode.
   Manual control of servo and ultrasonic readout on UI.
   Motor rest cycle added: 10s move / 2s stop in auto mode.
   Ultrasonic read immediately before each forward command in auto mode.
   UI shows a mood emoji based on robot state.
   Passive buzzer on GPIO 13 plays cute sounds (simple square wave).
   By Paz Lieghton
   Refactored with FreeRTOS for non‑blocking operation.

   ---- v2.1 patch notes ----
   1. Manual drive commands (F/B/L/R) now require a heartbeat from the UI.
      If it stops arriving (dropped WiFi, backgrounded tab, browser crash)
      the robot auto-stops after MANUAL_CMD_TIMEOUT_MS instead of coasting
      forever. Frontend now re-sends the held command every 300ms.
   2. Removed the redundant direct ultrasonic trigger/read inside navigate()
      right before a forward command - it raced with taskUltrasonic hitting
      the same TRIG/ECHO pins from another task. navigate() now trusts the
      cached currentDistanceCm value instead.
   3/4. navDebug/mood/ultrasonicCm/servoAngle are now written into the
      telemetry struct from a single task (taskNavigation) under the mutex,
      using fixed-size char buffers instead of String. This removes a
      cross-core race (taskMPU used to copy navDebug without any lock) and
      the heap churn/fragmentation risk of reallocating a String ~50x/sec.
   5. Stuck-detection state (navLastDistance / navLastProgressTime) is now
      hoisted out of navigate() into globals and explicitly reset when auto
      mode starts, instead of persisting across missions as function-local
      statics.
   6. Buzzer tones are now queued to a dedicated low-priority task instead
      of busy-waiting inline inside taskNavigation, so a beep can no longer
      stall navigation/obstacle-avoidance logic for its full duration.
   7. Obstacle-avoidance turns are now heading-based (turn until
      fusedHeadingDeg changes by AVOID_TURN_DEG, capped by
      AVOID_TURN_TIMEOUT_MS as a safety net) instead of a fixed time, so the
      turn amount no longer drifts with battery voltage or surface friction.
   8. IMU/magnetometer init failures are now surfaced in telemetry
      (imuOk/magOk) and shown in the UI; auto mode is refused if the IMU
      never initialized.
   9. /addwaypoint now validates lat/lon are in a sane range before
      accepting them.
   Also removed one genuinely unused variable (lastUltrasonicRead).

   Not changed in this pass (flagged separately, bigger scope):
   - No auth on the web endpoints (fine on a private hotspot, but anyone on
     the WiFi has full drive control).
   - Waypoints/mode flags are still read/written across cores without a
     queue (works fine for a hobby project; a command-queue architecture
     would be the "correct" fix if this ever needs to be bulletproof).
   - WebServer is still synchronous; ESPAsyncWebServer + WebSockets would
     give lower-latency manual control and push-based telemetry.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <TinyGPSPlus.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

// ========== Pin definitions ==========
const char* ssid     = "HotspotPax";
const char* password = "78547854";
const char* mdnsName = "fuzzybot";

HardwareSerial gpsSerial(1);
const int GPS_RX_PIN = 25;
const int GPS_TX_PIN = 26;
const uint32_t GPS_BAUD = 9600;
const double MIN_SPEED_FOR_COURSE_KMPH = 1.5;

const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

const int SERVO_PIN = 27;
Servo servo;

const int BUZZER_PIN = 13;

const float OBSTACLE_THRESHOLD_CM = 30.0;
const float SCAN_LEFT_ANGLE = 120;
const float SCAN_RIGHT_ANGLE = 60;
const float SCAN_CENTER_ANGLE = 90;
const float AVOID_TURN_DEG = 45.0;                 // rotate until heading changes by this much...
const unsigned long AVOID_TURN_TIMEOUT_MS = 3000;  // ...or give up and resume after this long (safety net)

const unsigned long MANUAL_CMD_TIMEOUT_MS = 800;   // auto-stop if manual heartbeat lapses for this long

const unsigned long REST_INTERVAL_MS = 10000;
const unsigned long REST_DURATION_MS  = 2000;

const unsigned long NAV_CONTROL_INTERVAL_MS = 800;
const unsigned long NAV_WAYPOINT_TIMEOUT_MS = 120000;
const unsigned long AUTO_GPS_TIMEOUT_MS = 300000;
const unsigned long STUCK_TIMEOUT_MS = 10000;

const int MAX_WAYPOINTS = 10;
const double WAYPOINT_RADIUS_M = 5.0;
const double TURN_THRESHOLD_DEG = 28.0;
const double TURN_TOLERANCE_DEG = 8.0;

#define MAG_ADDR 0x1D
#define MAG_X_SIGN  -1
#define MAG_Y_SIGN  1
#define MAG_Z_SIGN  1

// ========== Global objects ==========
WebServer server(80);
MPU6050 mpu(Wire);
TinyGPSPlus gps;

// ========== Shared data with mutex ==========
struct TelemetryData {
  bool gpsValid;
  double lat, lon;
  int satellites;
  double hdop;
  bool courseValid;
  double courseDeg;
  double speedKmph;
  float gyroHeadingDeg;
  float pitchDeg, rollDeg;
  float targetBearingDeg;
  float distanceToWaypointM;
  float headingErrorDeg;
  bool autoMode;
  int numWaypoints;
  int motorSpeed;
  char debug[96];
  float ultrasonicCm;
  int servoAngle;
  char mood[12];
  bool imuOk;
  bool magOk;
} telemetry;

SemaphoreHandle_t telemetryMutex = nullptr;

// ========== Other globals ==========
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

float fusedHeadingDeg = 0.0;
unsigned long lastGyroUpdate = 0;
int motorSpeed = 220;

char navDebug[96] = "Idle";

// Navigation "stuck" detection - persists across navigate() calls, must be
// explicitly reset whenever a new mission starts (see handleStartAuto).
float navLastDistance = -1;
unsigned long navLastProgressTime = 0;

// Manual control deadman's switch
volatile bool manualMoving = false;
volatile unsigned long lastManualCmdTime = 0;

// Obstacle avoidance state
bool obstacleAvoidanceActive = false;
unsigned long avoidanceStartTime = 0;
int avoidancePhase = 0;
float currentDistanceCm = 999.0;
const unsigned long ULTRASONIC_INTERVAL_MS = 200;
int currentServoAngle = SCAN_CENTER_ANGLE;
float avoidanceStartHeading = 0.0;

// Motor rest
unsigned long lastMoveStart = 0;
bool resting = false;

// Gyro bias
float gyroBias = 0.0;
float biasSum = 0.0;
int   biasCount = 0;
const int BIAS_SAMPLES = 200;
const float STATIONARY_THRESHOLD = 3.0;

// Sensor init status - gates auto mode and is surfaced in telemetry
bool mpuOk = false;
bool magOk = false;

// Buzzer (non-blocking via dedicated task + queue)
struct ToneCmd { int freq; int durationMs; };
QueueHandle_t buzzerQueue = nullptr;

// ========== Function prototypes ==========
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

void sendCommand(char c);
float wrapDeg(float d);
void setNavDebug(const char* s);
float readMagHeading();
void updateFusedHeading();
float readUltrasonicDistance();
void setServoAngle(int angle);
void performObstacleAvoidance();
void navigate();
void removeCurrentWaypoint();
double bearingTo(double lat1, double lon1, double lat2, double lon2);
double distanceTo(double lat1, double lon1, double lat2, double lon2);
double angleDifference(double bearing, double heading);

// Buzzer
void playTone(int freq, int durationMs);
void queueTone(int freq, int durationMs);
void beepStartup();
void beepAutoStart();
void beepObstacle();
void beepAvoid();
void beepWaypoint();
void beepMissionComplete();

// FreeRTOS tasks
void taskWiFi(void *pvParameters);
void taskGPS(void *pvParameters);
void taskMPU(void *pvParameters);
void taskUltrasonic(void *pvParameters);
void taskNavigation(void *pvParameters);
void taskWebServer(void *pvParameters);
void taskBuzzer(void *pvParameters);

// ========== Helper implementations ==========
float wrapDeg(float d) {
  while (d > 180)  d -= 360;
  while (d < -180) d += 360;
  return d;
}

void setNavDebug(const char* s) {
  snprintf(navDebug, sizeof(navDebug), "%s", s);
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

void playTone(int freq, int durationMs) {
  if (freq <= 0 || durationMs <= 0) {
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(durationMs));
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

// Non-blocking front-end for playTone(): hands the tone to taskBuzzer via a
// queue instead of busy-waiting inline in whichever task calls beep*().
void queueTone(int freq, int durationMs) {
  if (!buzzerQueue) return;
  ToneCmd cmd{freq, durationMs};
  xQueueSend(buzzerQueue, &cmd, 0); // never block the caller; drop if the queue is full
}

void beepStartup() {
  queueTone(1000, 100);
  queueTone(1500, 100);
  queueTone(2000, 200);
}

void beepAutoStart() {
  queueTone(800, 100);
  queueTone(1000, 100);
  queueTone(1200, 150);
}

void beepObstacle() {
  queueTone(2000, 150);
  queueTone(2000, 150);
}

void beepAvoid() {
  queueTone(1500, 100);
  queueTone(1800, 100);
}

void beepWaypoint() {
  queueTone(1200, 150);
  queueTone(1600, 150);
  queueTone(2000, 200);
}

void beepMissionComplete() {
  queueTone(1000, 150);
  queueTone(1300, 150);
  queueTone(1600, 150);
  queueTone(2000, 300);
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

// ========== Obstacle avoidance (non‑blocking) ==========
void performObstacleAvoidance() {
  if (!obstacleAvoidanceActive) return;

  switch (avoidancePhase) {
    case 0: // scan left
      setServoAngle(SCAN_LEFT_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(300));
      currentDistanceCm = readUltrasonicDistance();
      Serial.print("Left distance: ");
      Serial.println(currentDistanceCm);
      avoidancePhase = 1;
      break;

    case 1: { // scan right
      setServoAngle(SCAN_RIGHT_ANGLE);
      vTaskDelay(pdMS_TO_TICKS(300));
      float rightDist = readUltrasonicDistance();
      Serial.print("Right distance: ");
      Serial.println(rightDist);
      avoidanceStartHeading = fusedHeadingDeg; // baseline for the heading-based turn below
      if (currentDistanceCm > rightDist) {
        sendCommand('L');
        setNavDebug("Avoiding: turning left");
      } else {
        sendCommand('R');
        setNavDebug("Avoiding: turning right");
      }
      beepAvoid();
      avoidancePhase = 2;
      avoidanceStartTime = millis();
      break;
    }

    case 2: { // turn until heading has rotated enough, or bail out on timeout
      float turned = fabs(angleDifference(fusedHeadingDeg, avoidanceStartHeading));
      bool timedOut = millis() - avoidanceStartTime > AVOID_TURN_TIMEOUT_MS;
      if (turned >= AVOID_TURN_DEG || timedOut) {
        sendCommand('X');
        setServoAngle(SCAN_CENTER_ANGLE);
        obstacleAvoidanceActive = false;
        avoidancePhase = 0;
        navState = NAV_MOVING;
        setNavDebug(timedOut ? "Avoidance turn timed out, resuming" : "Obstacle avoided, resuming");
      }
      break;
    }
  }
}

// ========== Navigation logic (non‑blocking) ==========
void navigate() {
  if (!gps.location.isValid()) {
    setNavDebug("Waiting for GPS fix");
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
    setNavDebug("All waypoints reached");
    beepMissionComplete();
    return;
  }

  // Motor rest cycle
  if (resting) {
    if (millis() - lastMoveStart > REST_DURATION_MS) {
      resting = false;
      lastMoveStart = millis();
      setNavDebug("Rest finished, moving again");
    } else {
      sendCommand('X');
      return;
    }
  } else if (millis() - lastMoveStart > REST_INTERVAL_MS) {
    resting = true;
    lastMoveStart = millis();
    sendCommand('X');
    setNavDebug("Resting motors (2s)");
    return;
  }

  if (millis() - waypointStartTime > NAV_WAYPOINT_TIMEOUT_MS) {
    setNavDebug("Waypoint timeout, skipping");
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

  if (navLastDistance < 0) navLastDistance = dist;
  if (abs(dist - navLastDistance) < 0.5) {
    if (millis() - navLastProgressTime > STUCK_TIMEOUT_MS) {
      setNavDebug("Stuck! Executing evasion");
      sendCommand('R');
      beepObstacle();
      navLastProgressTime = millis();
      lastNavAction = millis();
      navLastDistance = dist;
      return;
    }
  } else {
    navLastDistance = dist;
    navLastProgressTime = millis();
  }

  if (dist < WAYPOINT_RADIUS_M) {
    sendCommand('X');
    setNavDebug("Arrived at waypoint");
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
  float heading = fusedHeadingDeg;  // read from global
  double headingError = angleDifference(bearing, heading);

  char debugBuf[100];
  snprintf(debugBuf, sizeof(debugBuf), "Dist %.1fm, Bear %.0f, Head %.0f, Err %.0f", dist, bearing, heading, headingError);

  char intendedCmd = 'X';
  const char* suffix;
  if (headingError > TURN_THRESHOLD_DEG) {
    intendedCmd = 'R';
    navState = NAV_TURNING;
    suffix = " | Turning R";
  } else if (headingError < -TURN_THRESHOLD_DEG) {
    intendedCmd = 'L';
    navState = NAV_TURNING;
    suffix = " | Turning L";
  } else {
    intendedCmd = 'F';
    navState = NAV_MOVING;
    suffix = " | Forward";
  }
  snprintf(navDebug, sizeof(navDebug), "%s%s", debugBuf, suffix);

  if (intendedCmd == 'F' && currentDistanceCm < OBSTACLE_THRESHOLD_CM) {
    // Trust the ultrasonic task's cached reading instead of triggering the
    // sensor again here - taskUltrasonic already refreshes it every
    // ULTRASONIC_INTERVAL_MS, and firing TRIG/ECHO from two tasks at once
    // could corrupt either reading.
    sendCommand('X');
    obstacleAvoidanceActive = true;
    avoidancePhase = 0;
    avoidanceStartTime = millis();
    navState = NAV_AVOIDING;
    setNavDebug("Obstacle detected! Avoiding...");
    beepObstacle();
    return;
  }

  sendCommand(intendedCmd);
}

// ========== Web handlers ==========
void handleTelemetry() {
  TelemetryData data;
  if (xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
    data = telemetry;
    xSemaphoreGive(telemetryMutex);
  }

  char buf[1400];
  snprintf(buf, sizeof(buf),
    "{\"gpsValid\":%s,\"lat\":%.6f,\"lon\":%.6f,\"satellites\":%d,\"hdop\":%.1f,"
    "\"courseValid\":%s,\"courseDeg\":%.1f,\"speedKmph\":%.2f,"
    "\"gyroHeadingDeg\":%.1f,\"pitchDeg\":%.1f,\"rollDeg\":%.1f,"
    "\"targetBearingDeg\":%.1f,\"distanceToWaypointM\":%.1f,\"headingErrorDeg\":%.1f,"
    "\"autoMode\":%s,\"waypoints\":%d,\"motorSpeed\":%d,\"debug\":\"%s\","
    "\"ultrasonicCm\":%.1f,\"servoAngle\":%d,\"mood\":\"%s\","
    "\"imuOk\":%s,\"magOk\":%s}",
    data.gpsValid ? "true" : "false", data.lat, data.lon, data.satellites, data.hdop,
    data.courseValid ? "true" : "false", data.courseDeg, data.speedKmph,
    data.gyroHeadingDeg, data.pitchDeg, data.rollDeg,
    data.targetBearingDeg, data.distanceToWaypointM, data.headingErrorDeg,
    data.autoMode ? "true" : "false", data.numWaypoints, data.motorSpeed, data.debug,
    data.ultrasonicCm, data.servoAngle, data.mood,
    data.imuOk ? "true" : "false", data.magOk ? "true" : "false");

  server.send(200, "application/json", buf);
}

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

void handleAddWaypoint() {
  if (server.hasArg("lat") && server.hasArg("lon")) {
    if (numWaypoints >= MAX_WAYPOINTS) {
      server.send(400, "text/plain", "Waypoint limit reached");
      return;
    }
    double lat = server.arg("lat").toDouble();
    double lon = server.arg("lon").toDouble();
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0 || (lat == 0.0 && lon == 0.0)) {
      server.send(400, "text/plain", "Invalid lat/lon");
      return;
    }
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
  if (!mpuOk) {
    server.send(400, "text/plain", "IMU not initialized - cannot start auto mode");
    return;
  }
  autoMode = true;
  currentWaypointIndex = 0;
  navState = NAV_MOVING;
  lastNavAction = 0;
  autoStartTime = millis();
  waypointStartTime = millis();
  navLastDistance = -1;              // reset stuck-detection state for this mission
  navLastProgressTime = millis();
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
  manualMoving = false;
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
    vTaskDelay(pdMS_TO_TICKS(50));
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
        manualMoving = false;
      } else if (c == 'F' || c == 'B' || c == 'L' || c == 'R') {
        // Deadman's switch: the UI must keep re-sending this command
        // (see the frontend heartbeat in handleRoot's JS) or taskNavigation
        // will auto-stop the robot after MANUAL_CMD_TIMEOUT_MS.
        manualMoving = true;
        lastManualCmdTime = millis();
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

void handleRoot() {
  // [HTML unchanged - keep the same as original to save space]
  // For brevity, we include a placeholder. In practice, paste the full HTML from the original.
  String html = "<head>\n"
"  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n"
"  <meta name=\"theme-color\" content=\"#0b1220\">\n"
"  <title>FuzzyBot Control</title>\n"
"  <link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.css\" />\n"
"  <style>\n"
"    :root { --bg:#eef2f7; --surface:#ffffff; --surface2:#f7f9fc; --primary:#3b82f6; --primary2:#2563eb; --danger:#ef4444; --success:#10b981; --warning:#f59e0b; --text:#172033; --muted:#6b7280; --border:#dfe5ee; --shadow:0 10px 30px rgba(15,23,42,.08); }\n"
"    [data-theme=dark] { --bg:#08111f; --surface:#101a2b; --surface2:#0c1524; --primary:#60a5fa; --primary2:#3b82f6; --danger:#f87171; --success:#34d399; --warning:#fbbf24; --text:#edf4ff; --muted:#91a0b8; --border:#223149; --shadow:0 16px 34px rgba(0,0,0,.28); }\n"
"    * { box-sizing:border-box; font-family:Inter,system-ui,-apple-system,BlinkMacSystemFont,\"Segoe UI\",sans-serif; }\n"
"    html,body { height:100%; margin:0; background:var(--bg); color:var(--text); }\n"
"    body { overflow:hidden; }\n"
"    button { font:inherit; }\n"
"    header { height:64px; display:flex; justify-content:space-between; align-items:center; padding:0 22px; background:var(--surface); border-bottom:1px solid var(--border); }\n"
"    .brand { display:flex; align-items:center; gap:8px; }\n"
"    .brand h1 { margin:0; font-size:18px; font-weight:700; letter-spacing:-.02em; }\n"
"    .brand .subtitle { margin:0; font-size:10px; color:var(--muted); line-height:1.3; }\n"
"    .header-actions { display:flex; align-items:center; gap:10px; }\n"
"    .conn { display:flex; align-items:center; gap:7px; font-size:13px; color:var(--muted); }\n"
"    .dot { width:9px; height:9px; border-radius:50%; background:var(--success); box-shadow:0 0 0 4px rgba(16,185,129,.10); }\n"
"    .dot.offline { background:var(--danger); box-shadow:0 0 0 4px rgba(239,68,68,.10); }\n"
"    .icon-btn { width:36px; height:36px; border-radius:10px; border:1px solid var(--border); background:var(--surface2); color:var(--text); cursor:pointer; display:flex; align-items:center; justify-content:center; font-size:18px; }\n"
"    .main { display:flex; height:calc(100vh - 64px); gap:16px; padding:16px; }\n"
"    #map { flex:1; min-width:0; border-radius:18px; border:1px solid var(--border); box-shadow:var(--shadow); overflow:hidden; z-index:1; }\n"
"    .sidebar { width:360px; flex-shrink:0; overflow-y:auto; display:flex; flex-direction:column; gap:12px; padding-right:2px; }\n"
"    .card { background:var(--surface); border:1px solid var(--border); border-radius:16px; padding:15px; box-shadow:var(--shadow); }\n"
"    .card-head { display:flex; align-items:center; justify-content:space-between; gap:10px; margin-bottom:12px; }\n"
"    .card-title { margin:0; font-size:11px; text-transform:uppercase; letter-spacing:.09em; font-weight:750; color:var(--muted); }\n"
"    .badge { padding:5px 8px; border-radius:999px; font-size:11px; font-weight:750; background:var(--surface2); color:var(--muted); border:1px solid var(--border); }\n"
"    .badge.active { background:rgba(59,130,246,.12); color:var(--primary); border-color:rgba(59,130,246,.22); }\n"
"    .badge.fault { background:rgba(239,68,68,.12); color:var(--danger); border-color:rgba(239,68,68,.22); }\n"
"    .telemetry-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; }\n"
"    .metric { padding:10px 11px; border:1px solid var(--border); background:var(--surface2); border-radius:12px; }\n"
"    .metric span { display:block; color:var(--muted); font-size:11px; margin-bottom:4px; }\n"
"    .metric strong { font-size:18px; letter-spacing:-.03em; }\n"
"    .compass-wrap { display:flex; gap:14px; align-items:center; margin-bottom:12px; }\n"
"    .compass { width:94px; height:94px; flex:0 0 94px; border-radius:50%; border:2px solid var(--border); position:relative; background:radial-gradient(circle,var(--surface2) 0 62%,transparent 63%),var(--surface); box-shadow:inset 0 0 0 8px rgba(127,127,127,.03); }\n"
"    .compass::before { content:\"N\"; position:absolute; left:50%; top:5px; transform:translateX(-50%); color:var(--danger); font-weight:800; font-size:11px; }\n"
"    .compass::after { content:\"W    S  E\"; position:absolute; left:50%; bottom:8px; transform:translateX(-50%); color:var(--muted); font-size:8px; letter-spacing:7px; white-space:nowrap; }\n"
"    .compass-needle { position:absolute; top:50%; left:50%; width:0; height:0; border-left:7px solid transparent; border-right:7px solid transparent; border-bottom:36px solid var(--danger); transform-origin:50% 100%; margin-left:-7px; margin-top:-36px; transition:transform .18s ease-out; filter:drop-shadow(0 2px 2px rgba(0,0,0,.12)); }\n"
"    .big-heading { font-size:31px; font-weight:760; letter-spacing:-.05em; }\n"
"    .subtle { font-size:11px; color:var(--muted); }\n"
"    .nav-strip { position:relative; height:34px; border-radius:10px; background:var(--surface2); border:1px solid var(--border); overflow:hidden; margin-top:11px; }\n"
"    .nav-center { position:absolute; left:50%; top:0; bottom:0; width:2px; background:var(--text); opacity:.35; }\n"
"    .nav-target { position:absolute; top:5px; width:4px; height:24px; border-radius:6px; background:var(--danger); box-shadow:0 0 0 4px rgba(239,68,68,.10); transition:left .2s ease; }\n"
"    .error-pill { margin-top:8px; display:flex; justify-content:space-between; align-items:center; font-size:12px; }\n"
"    .error-pill strong { color:var(--primary); }\n"
"    .grid-dpad { display:grid; grid-template-columns:repeat(3,1fr); gap:8px; }\n"
"    .grid-dpad button { height:48px; }\n"
"    .grid-2 { display:grid; grid-template-columns:1fr 1fr; gap:8px; }\n"
"    button.ctrl { padding:11px 10px; border:0; border-radius:10px; background:var(--primary); color:#fff; font-weight:700; cursor:pointer; transition:transform .08s,opacity .15s; display:flex; align-items:center; justify-content:center; gap:4px; }\n"
"    button.ctrl:active { transform:translateY(1px) scale(.99); opacity:.82; }\n"
"    .danger { background:var(--danger)!important; }\n"
"    .secondary { background:var(--surface2)!important; color:var(--text)!important; border:1px solid var(--border)!important; }\n"
"    .mission { display:grid; grid-template-columns:auto 1fr auto; gap:10px; align-items:center; padding:11px; border-radius:12px; background:linear-gradient(135deg,rgba(59,130,246,.10),rgba(59,130,246,.03)); border:1px solid rgba(59,130,246,.16); }\n"
"    .mission-icon { width:34px; height:34px; border-radius:10px; display:grid; place-items:center; background:rgba(59,130,246,.15); color:var(--primary); font-weight:800; }\n"
"    .mission-main strong { display:block; font-size:15px; }\n"
"    .mission-main span { display:block; font-size:11px; color:var(--muted); margin-top:2px; }\n"
"    .mission-distance { font-size:18px; font-weight:760; }\n"
"    .waypoint-list { margin-top:10px; padding:9px; background:var(--surface2); border:1px solid var(--border); border-radius:10px; font-size:12px; max-height:120px; overflow:auto; }\n"
"    .waypoint-list div { padding:5px 2px; border-bottom:1px solid var(--border); }\n"
"    .waypoint-list div:last-child { border-bottom:none; }\n"
"    .bar { height:8px; background:var(--surface2); border:1px solid var(--border); border-radius:999px; overflow:hidden; }\n"
"    .bar > div { height:100%; width:0%; border-radius:999px; transition:width .25s ease; background:linear-gradient(90deg,var(--primary),#8b5cf6); }\n"
"    .split { display:grid; grid-template-columns:1fr 1fr; gap:10px; }\n"
"    .mini-label { font-size:10px; color:var(--muted); margin-bottom:5px; text-transform:uppercase; letter-spacing:.06em; }\n"
"    .debug { margin-top:9px; background:#0b1220; color:#b9f6cf; border-radius:10px; padding:10px; font:11px/1.5 ui-monospace,SFMono-Regular,Consolas,monospace; min-height:40px; }\n"
"    [data-theme=dark] .debug { background:#050b14; }\n"
"    .footer-note { color:var(--muted); font-size:10px; line-height:1.4; margin-top:9px; }\n"
"    .dir-indicator { display:flex; align-items:center; justify-content:center; gap:6px; margin-bottom:8px; font-size:22px; font-weight:700; color:var(--primary); }\n"
"    .dir-indicator span { background:var(--surface2); border:1px solid var(--border); border-radius:8px; padding:4px 10px; }\n"
"    .robot-arrow { font-size:22px; color:var(--primary); transform-origin:50% 50%; text-shadow:0 0 4px var(--surface); transition:transform .18s ease-out; display:inline-block; }\n"
"    .leaflet-control.recenter-ctrl button { padding:8px 11px; font-size:12px; border-radius:8px; }\n"
"    @media (max-width:900px) { \n"
"      body{overflow:auto;} \n"
"      .main{height:calc(100vh - 64px); min-height:0; flex-direction:row; padding:8px; gap:8px;}\n"
"      #map{flex:1; min-width:0; height:auto; min-height:0;}\n"
"      .sidebar{width:48%; min-width:200px; overflow-y:auto; padding-right:0;}\n"
"    }\n"
"  </style>\n"
"</head>\n"
"<body>\n"
"<header>\n"
"  <div class=\"brand\">\n"
"    <div><h1>&#129302; FuzzyBot</h1><p class=\"subtitle\">Autonomous Version 0.12<br>AI-assisted UI &bull; UNSAM &bull; By Paz Lieghton &bull; CREA</p></div>\n"
"  </div>\n"
"  <div class=\"header-actions\">\n"
"    <div class=\"conn\"><span class=\"dot\" id=\"connDot\"></span><span id=\"connText\">Connected</span></div>\n"
"    <button class=\"icon-btn\" id=\"themeBtn\" onclick=\"toggleTheme()\" title=\"Toggle theme\">&#9790;</button>\n"
"  </div>\n"
"</header>\n"
"<div class=\"main\">\n"
"  <div id=\"map\"></div>\n"
"  <div class=\"sidebar\">\n"
"    <div class=\"card\">\n"
"      <div class=\"card-head\"><h2 class=\"card-title\">Telemetry</h2><span class=\"badge\" id=\"gpsBadge\">NO FIX</span></div>\n"
"      <div style=\"display:flex; align-items:center; gap:8px; margin-bottom:10px;\">\n"
"        <span style=\"font-size:28px;\" id=\"moodEmoji\">😊</span>\n"
"        <span class=\"subtle\">Robot mood</span>\n"
"      </div>\n"
"      <div class=\"compass-wrap\">\n"
"        <div class=\"compass\"><div class=\"compass-needle\" id=\"compassNeedle\"></div></div>\n"
"        <div><div class=\"subtle\">NORTH HEADING</div><div class=\"big-heading\" id=\"headingValue\">--&deg;</div><div class=\"subtle\" id=\"headingSource\">gyro (bias corrected)</div></div>\n"
"      </div>\n"
"      <div class=\"telemetry-grid\">\n"
"        <div class=\"metric\"><span>TARGET</span><strong id=\"targetValue\">--&deg;</strong></div>\n"
"        <div class=\"metric\"><span>ERROR</span><strong id=\"errorValue\">--&deg;</strong></div>\n"
"        <div class=\"metric\"><span>DISTANCE</span><strong id=\"distanceValue\">-- m</strong></div>\n"
"        <div class=\"metric\"><span>SPEED</span><strong id=\"speedValue\">--</strong></div>\n"
"      </div>\n"
"      <div class=\"nav-strip\"><div class=\"nav-center\"></div><div class=\"nav-target\" id=\"navTarget\"></div></div>\n"
"      <div class=\"error-pill\"><span>Target direction</span><strong id=\"directionHint\">--</strong></div>\n"
"    </div>\n"
"\n"
"    <div class=\"card\">\n"
"      <div class=\"card-head\"><h2 class=\"card-title\">Sensors</h2><span class=\"badge\">LIVE</span></div>\n"
"      <div class=\"metric\"><span>ULTRASONIC DISTANCE</span><strong id=\"distanceCm\">-- cm</strong></div>\n"
"      <div style=\"margin-top:10px;\"><div class=\"mini-label\">SERVO ANGLE</div>\n"
"        <input type=\"range\" min=\"0\" max=\"180\" value=\"90\" id=\"servoSlider\" oninput=\"setServo(this.value)\" style=\"width:100%;\">\n"
"        <span id=\"servoAngleDisplay\">90°</span>\n"
"      </div>\n"
"    </div>\n"
"\n"
"    <div class=\"card\">\n"
"      <div class=\"card-head\"><h2 class=\"card-title\">Manual control</h2><span class=\"badge\">HOLD</span></div>\n"
"      <div class=\"dir-indicator\"><span id=\"dirIndicator\">&#9632;</span></div>\n"
"      <div class=\"grid-dpad\">\n"
"        <div></div><button class=\"ctrl\" onmousedown=\"startMove('F')\" onmouseup=\"stopMove()\" onmouseleave=\"stopMove()\" ontouchstart=\"startMove('F')\" ontouchend=\"stopMove()\">&#8593;</button><div></div>\n"
"        <button class=\"ctrl\" onmousedown=\"startMove('L')\" onmouseup=\"stopMove()\" onmouseleave=\"stopMove()\" ontouchstart=\"startMove('L')\" ontouchend=\"stopMove()\">&#8592;</button>\n"
"        <button class=\"ctrl danger\" onclick=\"send('X')\">&#9632;</button>\n"
"        <button class=\"ctrl\" onmousedown=\"startMove('R')\" onmouseup=\"stopMove()\" onmouseleave=\"stopMove()\" ontouchstart=\"startMove('R')\" ontouchend=\"stopMove()\">&#8594;</button>\n"
"        <div></div><button class=\"ctrl\" onmousedown=\"startMove('B')\" onmouseup=\"stopMove()\" onmouseleave=\"stopMove()\" ontouchstart=\"startMove('B')\" ontouchend=\"stopMove()\">&#8595;</button><div></div>\n"
"      </div>\n"
"      <div class=\"grid-2\" style=\"margin-top:10px;\"><button class=\"ctrl secondary\" onclick=\"send('-')\">- Speed</button><button class=\"ctrl secondary\" onclick=\"send('+')\">+ Speed</button></div>\n"
"      <div style=\"margin-top:12px;\"><div class=\"mini-label\">Motor output</div><div class=\"bar\"><div id=\"speedBar\"></div></div></div>\n"
"    </div>\n"
"\n"
"    <div class=\"card\">\n"
"      <div class=\"card-head\"><h2 class=\"card-title\">Mission</h2><span class=\"badge\" id=\"autoBadge\">OFF</span></div>\n"
"      <div class=\"mission\"><div class=\"mission-icon\">&#8962;</div><div class=\"mission-main\"><strong id=\"missionText\">Standby</strong><span id=\"missionSub\">Add a waypoint on the map</span></div><div class=\"mission-distance\" id=\"missionDistance\">--</div></div>\n"
"      <div class=\"grid-2\" style=\"margin-top:10px;\"><button class=\"ctrl\" onclick=\"startAuto()\">&#9654; Start</button><button class=\"ctrl danger\" onclick=\"stopAuto()\">&#9632; Stop</button></div>\n"
"      <button class=\"ctrl secondary\" onclick=\"clearWaypoints()\" style=\"width:100%;margin-top:8px;\">Clear waypoints</button>\n"
"      <div class=\"waypoint-list\" id=\"waypointList\">No waypoints. Tap the map to add.</div>\n"
"    </div>\n"
"\n"
"    <div class=\"card\">\n"
"      <div class=\"card-head\"><h2 class=\"card-title\">System</h2><div style=\"display:flex;gap:6px;\"><span class=\"badge\" id=\"imuBadge\">IMU --</span><span class=\"badge\" id=\"satBadge\">SAT --</span></div></div>\n"
"      <div class=\"split\">\n"
"        <div><div class=\"mini-label\">GPS quality</div><div class=\"bar\"><div id=\"gpsBar\"></div></div></div>\n"
"        <div><div class=\"mini-label\">Heading error</div><div class=\"bar\"><div id=\"errorBar\"></div></div></div>\n"
"      </div>\n"
"      <div class=\"debug\" id=\"debugInfo\">Idle</div>\n"
"      <div class=\"grid-2\" style=\"margin-top:10px;\"><button class=\"ctrl secondary\" onclick=\"recalibrate()\">Recalibrate</button><button class=\"ctrl secondary\" onclick=\"recenterRobot()\">Recenter</button></div>\n"
"      <div class=\"footer-note\">Heading from gyro with automatic bias correction. Recalibrate uses magnetometer once to set absolute north.</div>\n"
"    </div>\n"
"  </div>\n"
"</div>\n"
"<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>\n"
"<script>\n"
"  const savedTheme = localStorage.getItem('fuzzybot-theme');\n"
"  document.documentElement.dataset.theme = savedTheme || 'light';\n"
"  updateThemeButton();\n"
"  function toggleTheme(){ const next=(document.documentElement.dataset.theme==='dark')?'light':'dark'; document.documentElement.dataset.theme=next; localStorage.setItem('fuzzybot-theme',next); updateThemeButton(); }\n"
"  function updateThemeButton(){ document.getElementById('themeBtn').innerHTML = document.documentElement.dataset.theme==='dark' ? '&#9728;' : '&#9790;'; }\n"
"\n"
"  let lastCommand = null;\n"
"  function send(c){ \n"
"    lastCommand = c;\n"
"    updateDirIndicator(c);\n"
"    fetch('/cmd?c='+encodeURIComponent(c)).then(()=>setConnected(true)).catch(()=>setConnected(false)); \n"
"  }\n"
"  function updateDirIndicator(c) {\n"
"    const map = {'F':'&#8593;','B':'&#8595;','L':'&#8592;','R':'&#8594;','X':'&#9632;'};\n"
"    const el = document.getElementById('dirIndicator');\n"
"    if (el) el.innerHTML = map[c] || '-';\n"
"  }\n"
"  let moveInterval=null;\n"
"  function startMove(c){ send(c); if(moveInterval)clearInterval(moveInterval); moveInterval=setInterval(()=>send(c),300); }\n"
"  function stopMove(){ if(moveInterval){clearInterval(moveInterval); moveInterval=null;} send('X'); }\n"
"  function stopAuto(){ fetch('/stopauto').then(()=>{ autoActive=false; updateAutoBadge(); }); }\n"
"  function startAuto(){ fetch('/startauto').then(()=>{ autoActive=true; updateAutoBadge(); }); }\n"
"  function recalibrate(){ if(confirm('Keep the robot stationary and press OK to recalibrate heading.')) fetch('/recalibrate').then(r=>r.text()).then(t=>alert(t)); }\n"
"  function setConnected(ok){ document.getElementById('connDot').className=ok?'dot':'dot offline'; document.getElementById('connText').innerText=ok?'Connected':'Disconnected'; }\n"
"  function setServo(angle) {\n"
"    fetch('/servo?angle='+angle).then(r=>r.text()).then(t=>{ document.getElementById('servoAngleDisplay').innerText = angle+'°'; });\n"
"  }\n"
"\n"
"  let waypointMarkers=[]; let routePolyline=null; let targetMarker=null; let robotMarker=null; let robotArrowEl=null; let autoActive=false;\n"
"  const map=L.map('map').setView([0,0],2);\n"
"  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);\n"
"  function ensureMarker(lat,lon){ if(robotMarker)return; const icon=L.divIcon({className:'',html:'<div class=\"robot-arrow\" id=\"robotArrow\">&#9650;</div>',iconSize:[22,22],iconAnchor:[11,11]}); robotMarker=L.marker([lat,lon],{icon}).addTo(map); robotArrowEl=document.getElementById('robotArrow'); map.setView([lat,lon],18); }\n"
"  function recenterRobot(){ if(robotMarker) map.setView(robotMarker.getLatLng(),map.getZoom()); }\n"
"  const RecenterControl=L.Control.extend({options:{position:'topright'},onAdd:function(){const div=L.DomUtil.create('div','leaflet-bar recenter-ctrl'); div.innerHTML='<button type=\"button\">Recenter</button>'; L.DomEvent.disableClickPropagation(div); div.querySelector('button').onclick=recenterRobot; return div;}}); map.addControl(new RecenterControl());\n"
"  function updateWaypointList(){ const d=document.getElementById('waypointList'); if(!waypointMarkers.length){d.innerHTML='No waypoints. Tap the map to add.';return;} d.innerHTML=waypointMarkers.map((m,i)=>`<div>WP${i+1} &middot; ${m.getLatLng().lat.toFixed(5)}, ${m.getLatLng().lng.toFixed(5)}</div>`).join(''); }\n"
"  function updateRouteLine(){ if(routePolyline)map.removeLayer(routePolyline); if(waypointMarkers.length){ const pts=[]; if(robotMarker) pts.push(robotMarker.getLatLng()); waypointMarkers.forEach(m=>pts.push(m.getLatLng())); if(pts.length>1) routePolyline=L.polyline(pts,{color:'#3b82f6',weight:4,dashArray:'8,8',opacity:.78}).addTo(map); }}\n"
"  function updateTargetMarker(){ if(targetMarker)map.removeLayer(targetMarker); if(waypointMarkers.length)targetMarker=L.circle(waypointMarkers[0].getLatLng(),{radius:5,color:'#ef4444',fillColor:'#ef4444',fillOpacity:.35,weight:2}).addTo(map); }\n"
"  function clearWaypoints(){ if(!confirm('Clear all waypoints?'))return; fetch('/clearwaypoints').then(()=>{waypointMarkers.forEach(m=>map.removeLayer(m)); waypointMarkers=[]; if(routePolyline)map.removeLayer(routePolyline); if(targetMarker)map.removeLayer(targetMarker); routePolyline=null; targetMarker=null; updateWaypointList(); autoActive=false; updateAutoBadge(); }); }\n"
"  function updateAutoBadge(){ const b=document.getElementById('autoBadge'); b.className=autoActive?'badge active':'badge'; b.innerText=autoActive?'ACTIVE':'OFF'; }\n"
"  map.on('click',e=>{ if(autoActive){alert('Stop auto mode before adding waypoints');return;} fetch(`/addwaypoint?lat=${e.latlng.lat}&lon=${e.latlng.lng}`).then(()=>{waypointMarkers.push(L.marker([e.latlng.lat,e.latlng.lng]).addTo(map)); updateWaypointList(); updateRouteLine(); updateTargetMarker();}); });\n"
"\n"
"  function clamp(v,a,b){return Math.max(a,Math.min(b,v));}\n"
"  function directionText(err){ if(err>6)return 'Turn right'; if(err<-6)return 'Turn left'; return 'On course'; }\n"
"  setInterval(async()=>{ try{ const r=await fetch('/telemetry'); const t=await r.json(); setConnected(true);\n"
"    const h=t.gyroHeadingDeg; const err=t.headingErrorDeg||0; const dist=t.distanceToWaypointM||0; const target=t.targetBearingDeg||0;\n"
"    document.getElementById('headingValue').innerHTML=Math.round(h)+'&deg;';\n"
"    document.getElementById('targetValue').innerHTML=t.waypoints?Math.round(target)+'&deg;':'--&deg;';\n"
"    document.getElementById('errorValue').innerHTML=t.waypoints?(err>0?'+':'')+Math.round(err)+'&deg;':'--&deg;';\n"
"    document.getElementById('distanceValue').innerHTML=t.waypoints?(dist<100?dist.toFixed(1):Math.round(dist))+' m':'-- m';\n"
"    document.getElementById('speedValue').innerText=t.motorSpeed;\n"
"    document.getElementById('headingSource').innerText='gyro (bias corrected)';\n"
"    document.getElementById('directionHint').innerText=t.waypoints?directionText(err):'No target';\n"
"    document.getElementById('navTarget').style.left=(clamp((err+90)/180,0,1)*100)+'%';\n"
"    document.getElementById('speedBar').style.width=((t.motorSpeed/255)*100)+'%';\n"
"    document.getElementById('errorBar').style.width=(clamp(Math.abs(err)/90,0,1)*100)+'%';\n"
"    const gpsQuality=t.gpsValid?clamp(100-(Math.max(0,t.hdop-1)*28),12,100):0; document.getElementById('gpsBar').style.width=gpsQuality+'%';\n"
"    document.getElementById('gpsBadge').innerText=t.gpsValid?'GPS FIX':'NO FIX'; document.getElementById('satBadge').innerText='SAT '+(t.satellites||'--');\n"
"    document.getElementById('gpsBadge').className=t.gpsValid?'badge active':'badge';\n"
"    document.getElementById('satBadge').className=(t.satellites>=5)?'badge active':'badge';\n"
"    document.getElementById('imuBadge').innerText=(t.imuOk===false)?'IMU FAULT':'IMU OK';\n"
"    document.getElementById('imuBadge').className=(t.imuOk===false)?'badge fault':'badge active';\n"
"    document.getElementById('debugInfo').innerText=t.debug||'Idle';\n"
"    document.getElementById('missionText').innerText=t.autoMode?(t.waypoints?'Auto navigation':'Finishing mission'):'Standby';\n"
"    document.getElementById('missionSub').innerText=t.waypoints?('Next waypoint - '+t.waypoints+' remaining'):'Add a waypoint on the map';\n"
"    document.getElementById('missionDistance').innerText=t.waypoints?(dist<100?dist.toFixed(1):Math.round(dist))+'m':'--';\n"
"    const rot='rotate('+h+'deg)'; document.getElementById('compassNeedle').style.transform=rot; if(robotArrowEl)robotArrowEl.style.transform=rot;\n"
"    if(t.gpsValid){ ensureMarker(t.lat,t.lon); robotMarker.setLatLng([t.lat,t.lon]); updateRouteLine(); }\n"
"    if(t.autoMode!==autoActive){autoActive=t.autoMode;updateAutoBadge();}\n"
"    if(t.waypoints<waypointMarkers.length){const gone=waypointMarkers.shift();if(gone)map.removeLayer(gone);updateWaypointList();updateRouteLine();updateTargetMarker();}\n"
"    // Ultrasonic display\n"
"    document.getElementById('distanceCm').innerText = (t.ultrasonicCm > 400 ? '--' : t.ultrasonicCm.toFixed(1))+' cm';\n"
"    // Servo display\n"
"    document.getElementById('servoAngleDisplay').innerText = t.servoAngle+'°';\n"
"    document.getElementById('servoSlider').value = t.servoAngle;\n"
"    // Mood emoji\n"
"    document.getElementById('moodEmoji').innerText = t.mood || '😊';\n"
"  }catch(e){setConnected(false);} },500);\n"
"</script>\n"
"</body>\n"
"</html>";
  
  server.send(200, "text/html", html);
}

// ========== FreeRTOS tasks ==========

void taskWiFi(void *pvParameters) {
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected - attempting reconnect...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      // Wait up to 10 seconds
      unsigned long start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Reconnected! IP: ");
        Serial.println(WiFi.localIP());
      } else {
        Serial.println("WiFi reconnect failed.");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void taskGPS(void *pvParameters) {
  while (1) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void taskMPU(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50 Hz

  while (1) {
    if (mpuOk) {
      mpu.update();
      updateFusedHeading();
    }

    // Update telemetry struct (protected). Note: debug/mood/ultrasonicCm/
    // servoAngle are intentionally NOT touched here anymore - taskNavigation
    // owns those now (single writer, see taskNavigation below).
    if (xSemaphoreTake(telemetryMutex, portMAX_DELAY) == pdTRUE) {
      telemetry.gyroHeadingDeg = fusedHeadingDeg;
      telemetry.pitchDeg = mpu.getAngleX();
      telemetry.rollDeg = mpu.getAngleY();
      // Also copy GPS data from gps object (handled by GPS task, but we read it here)
      telemetry.gpsValid = gps.location.isValid();
      if (telemetry.gpsValid) {
        telemetry.lat = gps.location.lat();
        telemetry.lon = gps.location.lng();
      }
      telemetry.satellites = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
      telemetry.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;
      double speed = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
      telemetry.speedKmph = speed;
      telemetry.courseValid = gps.course.isValid() && speed > MIN_SPEED_FOR_COURSE_KMPH;
      telemetry.courseDeg = telemetry.courseValid ? gps.course.deg() : 0.0;

      // Waypoint info
      if (telemetry.gpsValid && numWaypoints > 0) {
        telemetry.targetBearingDeg = bearingTo(telemetry.lat, telemetry.lon,
                                                waypoints[0].lat, waypoints[0].lon);
        telemetry.distanceToWaypointM = distanceTo(telemetry.lat, telemetry.lon,
                                                    waypoints[0].lat, waypoints[0].lon);
        telemetry.headingErrorDeg = angleDifference(telemetry.targetBearingDeg, fusedHeadingDeg);
      } else {
        telemetry.targetBearingDeg = 0;
        telemetry.distanceToWaypointM = 0;
        telemetry.headingErrorDeg = 0;
      }
      telemetry.autoMode = autoMode;
      telemetry.numWaypoints = numWaypoints;
      telemetry.motorSpeed = motorSpeed;
      telemetry.imuOk = mpuOk;
      telemetry.magOk = magOk;

      xSemaphoreGive(telemetryMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void taskUltrasonic(void *pvParameters) {
  while (1) {
    // Only read if servo at center and not in avoidance
    if (!obstacleAvoidanceActive && currentServoAngle == SCAN_CENTER_ANGLE) {
      currentDistanceCm = readUltrasonicDistance();
    }
    vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_INTERVAL_MS));
  }
}

void taskNavigation(void *pvParameters) {
  while (1) {
    if (autoMode && obstacleAvoidanceActive) {
      performObstacleAvoidance();
    } else if (autoMode) {
      navigate();
    } else {
      // Manual/idle mode: deadman's switch for manual drive commands. If
      // the UI stops sending F/B/L/R heartbeats, stop the motors instead of
      // trusting the last command indefinitely.
      if (manualMoving && millis() - lastManualCmdTime > MANUAL_CMD_TIMEOUT_MS) {
        sendCommand('X');
        manualMoving = false;
        setNavDebug("Manual command timeout - stopped");
      } else if (!manualMoving) {
        setNavDebug("Idle");
      }
    }

    // taskNavigation is the single writer for debug/mood/ultrasonic/servo
    // telemetry fields - avoids the cross-core String race that used to
    // exist between this task and taskMPU, and uses fixed buffers so there
    // is no heap allocation on this hot path.
    const char* mood;
    if (!autoMode) mood = manualMoving ? "🚗" : "😊";
    else if (obstacleAvoidanceActive) mood = "⚠️";
    else if (resting) mood = "😴";
    else mood = "🚗";

    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      snprintf(telemetry.debug, sizeof(telemetry.debug), "%s", navDebug);
      snprintf(telemetry.mood, sizeof(telemetry.mood), "%s", mood);
      telemetry.ultrasonicCm = currentDistanceCm;
      telemetry.servoAngle = currentServoAngle;
      xSemaphoreGive(telemetryMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // ~20 Hz
  }
}

void taskWebServer(void *pvParameters) {
  while (1) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(1)); // yield to other tasks
  }
}

void taskBuzzer(void *pvParameters) {
  ToneCmd cmd;
  for (;;) {
    if (xQueueReceive(buzzerQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      playTone(cmd.freq, cmd.durationMs); // OK to block here - this task has nothing else to do
    }
  }
}

// ========== Setup ==========
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  Wire.begin();
  byte mpuStatus = mpu.begin();
  Serial.print("MPU6050 status: "); Serial.println(mpuStatus);
  if (mpuStatus == 0) {
    Serial.println("Calibrating MPU6050 - keep still...");
    mpu.calcOffsets(true, true);
    Serial.println("MPU6050 ready.");
    mpuOk = true;
  } else {
    Serial.println("MPU6050 not responding.");
    mpuOk = false;
  }

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x09);
  Wire.write(0x01);
  if (Wire.endTransmission() != 0) {
    Serial.println("Magnetometer init failed");
    magOk = false;
  } else {
    Serial.println("QMC5883L initialized");
    magOk = true;
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

  // Buzzer queue + task must exist before we start queuing tones below.
  buzzerQueue = xQueueCreate(8, sizeof(ToneCmd));
  xTaskCreatePinnedToCore(taskBuzzer, "Buzzer", 2048, NULL, 1, NULL, 1);

  beepStartup();

  lastMoveStart = millis();
  resting = false;

  // WiFi initial connect (blocking for a while)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to "); Serial.print(ssid);
  unsigned long attemptStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - attemptStart < 20000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Couldn't connect within 20s - will retry in background.");
  }

  if (MDNS.begin(mdnsName)) {
    Serial.print("mDNS ready - browse to http://");
    Serial.print(mdnsName);
    Serial.println(".local/");
  } else {
    Serial.println("mDNS failed - use IP address.");
  }

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

  // Create mutex
  telemetryMutex = xSemaphoreCreateMutex();

  // Create tasks
  xTaskCreatePinnedToCore(taskWiFi, "WiFi", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskGPS, "GPS", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskMPU, "MPU", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskUltrasonic, "Ultrasonic", 2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskNavigation, "Nav", 8192, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskWebServer, "Web", 8192, NULL, 1, NULL, 0); // lower priority, run on core 0

  // The loop() below will be idle, but we can delete it or put a dummy.
}

void loop() {
  // Empty – all work is done by tasks.
  vTaskDelay(pdMS_TO_TICKS(1000));
}