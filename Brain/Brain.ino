/*
   ESP32 WiFi remote brain for FuzzyBot - v2.0 ver 0.2
   Simplified heading: gyro only, manual North calibration.
   Immediate manual control: hold-to-move, no heartbeat latency.
   Auto mode waits for GPS, deletes reached waypoints, draws lines on map.
   By Paz Lieghton
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <TinyGPSPlus.h>

const char* ssid     ="HotspotPax";
const char* password = "78547854";
const char* mdnsName = "fuzzybot";

HardwareSerial gpsSerial(1);
const int GPS_RX_PIN = 25;
const int GPS_TX_PIN = 26;
const uint32_t GPS_BAUD = 9600;
const double MIN_SPEED_FOR_COURSE_KMPH = 1.5;

const int MAX_WAYPOINTS = 10;
const double WAYPOINT_RADIUS_M = 5.0;
const double TURN_THRESHOLD_DEG = 20.0;
const double TURN_TOLERANCE_DEG = 8.0;
   // when to stop turning

  const float YAW_SIGN = -1.0;

WebServer server(80);
MPU6050 mpu(Wire);
TinyGPSPlus gps;

struct Waypoint {
  double lat, lon;
};
Waypoint waypoints[MAX_WAYPOINTS];
int numWaypoints = 0;
int currentWaypointIndex = 0;

bool autoMode = false;
enum NavState { NAV_IDLE,
                NAV_MOVING,
                NAV_TURNING };
NavState navState = NAV_IDLE;

unsigned long lastNavAction = 0;
unsigned long autoStartTime = 0;
unsigned long waypointStartTime = 0;

unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 30000;

float headingCorrectionDeg = 0.0;

int motorSpeed = 220;
   // matches Arduino default

  // Debug info string (sent in telemetry)
  String navDebug = "Idle";

// Prototypes
void handleRoot();
void handleCommand();
void handleTelemetry();
void handleAddWaypoint();
void handleClearWaypoints();
void handleStartAuto();
void handleStopAuto();
void handleGetWaypoints();
void handleCalibrateNorth();
void navigate();
void removeCurrentWaypoint();
double bearingTo(double lat1, double lon1, double lat2, double lon2);
double distanceTo(double lat1, double lon1, double lat2, double lon2);
double angleDifference(double bearing, double heading);
void sendCommand(char c);
float wrapDeg(float d);
float getFusedHeading();
void calibrateToNorth();

float wrapDeg(float d) {
    while (d > 180)  d -= 360;
    while (d < -180) d += 360;
    return d;
}

float getFusedHeading() {
    return wrapDeg(YAW_SIGN * mpu.getAngleZ() + headingCorrectionDeg);
}

void calibrateToNorth() {
    headingCorrectionDeg = wrapDeg(0.0 - YAW_SIGN * mpu.getAngleZ());
    Serial.println("Calibrated: current heading set to North (0 deg).");
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    Wire.begin();
    byte mpuStatus = mpu.begin();
    Serial.print("MPU6050 status: ");
  Serial.println(mpuStatus);
    if (mpuStatus == 0) {
        Serial.println("Calibrating MPU6050 - keep still...");
        mpu.calcOffsets(true, true);
        Serial.println("MPU6050 ready.");
     
  }
  else {
        Serial.println("MPU6050 not responding.");
     
  }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to ");
  Serial.print(ssid);
    unsigned long attemptStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - attemptStart < 20000) {
        delay(250);
        Serial.print(".");
     
  }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected! IP address: ");
        Serial.println(WiFi.localIP());
     
  }
  else {
        Serial.println("Couldn't connect within 20s - will retry.");
     
  }

    if (MDNS.begin(mdnsName)) {
        Serial.print("mDNS ready - browse to http://");
        Serial.print(mdnsName);
        Serial.println(".local/");
     
  }
  else {
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
    server.on("/calibrateNorth", handleCalibrateNorth);

    server.begin();
    Serial.println("HTTP server started.");
}

void loop() {
    if (millis() - lastWifiCheck > WIFI_CHECK_INTERVAL) {
        lastWifiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost - reconnecting...");
            WiFi.reconnect();
         
    }
     
  }

    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
     
  }

    mpu.update();

    if (autoMode) {
        navigate();
     
  }

    server.handleClient();
}

void handleTelemetry() {
    bool gpsValid   = gps.location.isValid();
    double lat      = gpsValid ? gps.location.lat() : 0.0;
    double lon      = gpsValid ? gps.location.lng() : 0.0;
    int sats        = gps.satellites.isValid() ? (int)gps.satellites.value() : 0;
    double hdopVal  = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0;
    double speedKmph = gps.speed.isValid() ? gps.speed.kmph() : 0.0;

    bool courseValid = gps.course.isValid() && gps.speed.isValid() && speedKmph > MIN_SPEED_FOR_COURSE_KMPH;
    double courseDeg = courseValid ? gps.course.deg() : 0.0;

    float gyroHeading = getFusedHeading();
    float pitch = mpu.getAngleX();
    float roll  = mpu.getAngleY();

    String debug = navDebug;  // copy

    char buf[600];
    snprintf(buf, sizeof(buf),
    "{\"gpsValid\":%s,\"lat\":%.6f,\"lon\":%.6f,\"satellites\":%d,\"hdop\":%.1f,"
    "\"courseValid\":%s,\"courseDeg\":%.1f,\"speedKmph\":%.2f,"
    "\"gyroHeadingDeg\":%.1f,\"pitchDeg\":%.1f,\"rollDeg\":%.1f,"
    "\"autoMode\":%s,\"waypoints\":%d,\"motorSpeed\":%d,\"debug\":\"%s\"}",
    gpsValid ? "true" : "false", lat, lon, sats, hdopVal,
    courseValid ? "true" : "false", courseDeg, speedKmph,
    gyroHeading, pitch, roll,
    autoMode ? "true" : "false", numWaypoints, motorSpeed, debug.c_str());

    server.send(200, "application/json", buf);
}

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
     
  }
  else {
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
    Serial.println("Auto mode started.");
    server.send(200, "text/plain", "Auto mode started");
}

void handleStopAuto() {
    autoMode = false;
    navState = NAV_IDLE;
    sendCommand('X');
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

void handleCalibrateNorth() {
    calibrateToNorth();
    server.send(200, "text/plain", "Calibrated to North");
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

    double a = sin(dphi / 2) * sin(dphi / 2) +
             cos(phi1) * cos(phi2) *
             sin(dlambda / 2) * sin(dlambda / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

double angleDifference(double bearing, double heading) {
    double diff = bearing - heading;
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    return diff;
}

const unsigned long NAV_CONTROL_INTERVAL_MS = 800;
const unsigned long NAV_WAYPOINT_TIMEOUT_MS = 120000;
const unsigned long AUTO_GPS_TIMEOUT_MS = 300000;

// Stuck detection
float lastDistance = -1;
unsigned long lastProgressTime = 0;
const unsigned long STUCK_TIMEOUT_MS = 10000;
   // if no progress in 10s, do something

  void
  navigate() {
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
        Serial.println("All waypoints reached. Auto mode stopped.");
        return;
     
  }

    if (millis() - waypointStartTime > NAV_WAYPOINT_TIMEOUT_MS) {
        navDebug = "Waypoint timeout, skipping";
        Serial.println("Waypoint timed out - skipping to next.");
        removeCurrentWaypoint();
        if (numWaypoints > 0) {
            waypointStartTime = millis();
            navState = NAV_MOVING;
         
    }
    else {
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

     // Stuck detection: if distance hasn't decreased significantly after timeout, maybe reverse or turn?
  if (lastDistance < 0) lastDistance = dist;
    if (abs(dist - lastDistance) < 0.5) {
       // not moving much
    if (millis() - lastProgressTime > STUCK_TIMEOUT_MS) {
            navDebug = "Stuck! Executing evasion";
             // Simple evasion: turn right for a bit
      sendCommand('R');
            lastProgressTime = millis();
            lastNavAction = millis();
            lastDistance = dist;
          // reset
      return;
         
    }
     
  }
  else {
        lastDistance = dist;
        lastProgressTime = millis();
     
  }

    if (dist < WAYPOINT_RADIUS_M) {
        sendCommand('X');
        navDebug = "Arrived at waypoint";
        Serial.println("Arrived at waypoint.");
        removeCurrentWaypoint();
        if (numWaypoints > 0) {
            waypointStartTime = millis();
            navState = NAV_MOVING;
         
    }
    else {
            autoMode = false;
            navState = NAV_IDLE;
            Serial.println("All waypoints reached. Auto mode stopped.");
         
    }
        return;
     
  }

    double bearing = bearingTo(curLat, curLon, wpLat, wpLon);
    double heading = getFusedHeading();
    double headingError = angleDifference(bearing, heading);

    char debugBuf[120];
    snprintf(debugBuf, sizeof(debugBuf), "Dist %.1fm, Bear %.0f, Head %.0f, Err %.0f", dist, bearing, heading, headingError);
    navDebug = String(debugBuf);

    if (headingError > TURN_THRESHOLD_DEG) {
        sendCommand('R');
        navState = NAV_TURNING;
        navDebug += " | Turning R";
     
  }
  else if (headingError < -TURN_THRESHOLD_DEG) {
        sendCommand('L');
        navState = NAV_TURNING;
        navDebug += " | Turning L";
     
  }
  else {
      sendCommand('F');
        navState = NAV_MOVING;
      navDebug += " | Forward";
  }
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
         
    }
      else if (c == '-') {
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
  String html =
    "<!DOCTYPE html>\n"
    "<html>\n"
    "<head>\n"
    "   <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n"
    "   <title>FuzzyBot 2.0</title>\n"
    "   <link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.css\" />\n"
    "   <style>\n"
    "     :root { --bg: #f3f4f6; --surface: #ffffff; --primary: #3b82f6; --danger: #ef4444; --text: #1f2937; --text-muted: #6b7280; --border: #e5e7eb; }\n"
    "     * { box-sizing: border-box; font-family: system-ui, -apple-system, sans-serif; }\n"
    "     html, body { height: 100%; margin: 0; background: var(--bg); color: var(--text); }\n"
    "     header { display: flex; justify-content: space-between; align-items: center; background: var(--surface); padding: 16px 24px; border-bottom: 1px solid var(--border); box-shadow: 0 1px 3px rgba(0,0,0,0.05); }\n"
    "     header h1 { font-size: 20px; margin: 0; font-weight: 600; }\n"
    "     #conn { font-size: 14px; font-weight: 500; display: flex; align-items: center; gap: 6px; }\n"
    "     .dot { width: 10px; height: 10px; border-radius: 50%; background: #10b981; }\n"
    "     .dot.offline { background: var(--danger); }\n"
    "     .main { display: flex; height: calc(100vh - 65px); padding: 16px; gap: 16px; flex-wrap: wrap; }\n"
    "     #map { flex: 1; min-width: 300px; border-radius: 16px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.1); border: 1px solid var(--border); z-index: 1; }\n"
    "     .sidebar { width: 340px; flex-shrink: 0; display: flex; flex-direction: column; gap: 16px; overflow-y: auto; padding-right: 8px; }\n"
    "     @media (max-width: 768px) { .sidebar { width: 100%; } .main { padding: 12px; flex-direction: column; } #map { min-height: 40vh; } }\n"
    "     .card { background: var(--surface); border-radius: 16px; padding: 16px; box-shadow: 0 4px 6px -1px rgba(0,0,0,0.05); border: 1px solid var(--border); }\n"
    "     .card-title { font-size: 12px; text-transform: uppercase; letter-spacing: 0.05em; color: var(--text-muted); font-weight: 600; margin: 0 0 12px 0; }\n"
    "     button { padding: 12px; font-size: 16px; font-weight: 600; border: none; border-radius: 10px; background: var(--primary); color: white; cursor: pointer; transition: opacity 0.2s; display: flex; justify-content: center; align-items: center; gap: 8px; }\n"
    "     button:active { opacity: 0.8; }\n"
    "     .btn-danger { background: var(--danger); }\n"
    "     .btn-secondary { background: #f3f4f6; color: var(--text); border: 1px solid var(--border); }\n"
    "     .grid-dpad { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }\n"
    "     .grid-dpad button { height: 50px; }\n"
    "     .grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }\n"
    "     .stat-row { display: flex; justify-content: space-between; align-items: center; padding: 8px 0; border-bottom: 1px solid var(--border); font-size: 14px; }\n"
    "     .stat-row:last-child { border-bottom: none; padding-bottom: 0; }\n"
    "     .stat-label { color: var(--text-muted); }\n"
    "     .stat-val { font-weight: 600; font-family: monospace; font-size: 15px; }\n"
    "     .compass-container { display: flex; justify-content: center; margin-bottom: 16px; }\n"
    "     .compass { width: 80px; height: 80px; border-radius: 50%; border: 4px solid var(--border); position: relative; background: #f9fafb; }\n"
    "     .compass-needle { position: absolute; top: 50%; left: 50%; width: 0; height: 0; border-left: 6px solid transparent; border-right: 6px solid transparent; border-bottom: 30px solid var(--danger); transform-origin: 50% 100%; margin-left: -6px; margin-top: -30px; transition: transform 0.2s ease-out; }\n"
    "     .robot-arrow { font-size: 22px; color: var(--primary); transform-origin: 50% 50%; text-shadow: 0 0 4px white; transition: transform 0.2s ease-out; display: inline-block; }\n"
    "     .waypoint-list { margin-top: 12px; padding: 12px; background: #f9fafb; border-radius: 8px; font-size: 13px; max-height: 120px; overflow-y: auto; border: 1px solid var(--border); color: var(--text-muted); }\n"
    "     .waypoint-list div { padding: 4px 0; border-bottom: 1px solid var(--border); }\n"
    "     .waypoint-list div:last-child { border-bottom: none; }\n"
    "     #debugInfo { background: #1f2937; color: #10b981; padding: 12px; border-radius: 8px; font-family: monospace; font-size: 12px; margin-top: 8px; overflow-x: auto; }\n"
    "     .badge { padding: 4px 8px; border-radius: 20px; font-size: 12px; font-weight: 600; background: #e5e7eb; color: var(--text-muted); }\n"
    "     .badge.active { background: #dbeafe; color: #1d4ed8; }\n"
    "   </style>\n"
    "</head>\n"
    "<body>\n"
    "   <header>\n"
    "     <h1>FuzzyBot 2.0</h1>\n"
    "     <div id=\"conn\"><div class=\"dot\" id=\"connDot\"></div> <span id=\"connText\">Connected</span></div>\n"
    "   </header>\n"
    "\n"
    "   <div class=\"main\">\n"
    "     <div id=\"map\"></div>\n"
    "     \n"
    "     <div class=\"sidebar\">\n"
    "       <!-- Telemetry Card -->\n"
    "       <div class=\"card\">\n"
    "         <h2 class=\"card-title\">Telemetry & Status</h2>\n"
    "         <div class=\"compass-container\">\n"
    "           <div class=\"compass\"><div class=\"compass-needle\" id=\"compassNeedle\"></div></div>\n"
    "         </div>\n"
    "         <div class=\"stat-row\">\n"
    "           <span class=\"stat-label\">Heading (Gyro)</span>\n"
    "           <span class=\"stat-val\" id=\"headingValue\">--&deg;</span>\n"
    "         </div>\n"
    "         <div class=\"stat-row\">\n"
    "           <span class=\"stat-label\">GPS Fix</span>\n"
    "           <span class=\"stat-val\" id=\"gpsStatus\">Waiting...</span>\n"
    "         </div>\n"
    "         <div class=\"stat-row\">\n"
    "           <span class=\"stat-label\">Satellites</span>\n"
    "           <span class=\"stat-val\" id=\"satCount\">--</span>\n"
    "         </div>\n"
    "         <div class=\"stat-row\">\n"
    "           <span class=\"stat-label\">Speed Cap</span>\n"
    "           <span class=\"stat-val\" id=\"speedValue\">--</span>\n"
    "         </div>\n"
    "       </div>\n"
    "\n"
    "       <!-- Manual Control Card -->\n"
    "       <div class=\"card\">\n"
    "         <h2 class=\"card-title\">Manual Control</h2>\n"
    "         <div class=\"grid-dpad\">\n"
    "           <div></div>\n"
    "           <button onmousedown=\"startMove('F')\" onmouseup=\"stopMove()\" ontouchstart=\"startMove('F')\" ontouchend=\"stopMove()\">&#x2B06;</button>\n"
    "           <div></div>\n"
    "           <button onmousedown=\"startMove('L')\" onmouseup=\"stopMove()\" ontouchstart=\"startMove('L')\" ontouchend=\"stopMove()\">&#x2B05;</button>\n"
    "           <button class=\"btn-danger\" onclick=\"send('X')\">&#x23F9;</button>\n"
    "           <button onmousedown=\"startMove('R')\" onmouseup=\"stopMove()\" ontouchstart=\"startMove('R')\" ontouchend=\"stopMove()\">&#x27A1;</button>\n"
    "           <div></div>\n"
    "           <button onmousedown=\"startMove('B')\" onmouseup=\"stopMove()\" ontouchstart=\"startMove('B')\" ontouchend=\"stopMove()\">&#x2B07;</button>\n"
    "           <div></div>\n"
    "         </div>\n"
    "         <div class=\"grid-2\" style=\"margin-top: 12px;\">\n"
    "           <button class=\"btn-secondary\" onclick=\"send('-')\">- Speed</button>\n"
    "           <button class=\"btn-secondary\" onclick=\"send('+')\">+ Speed</button>\n"
    "         </div>\n"
    "       </div>\n"
    "\n"
    "       <!-- Auto Navigation Card -->\n"
    "       <div class=\"card\">\n"
    "         <div style=\"display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px;\">\n"
    "           <h2 class=\"card-title\" style=\"margin:0;\">Auto Navigation</h2>\n"
    "           <span class=\"badge\" id=\"autoBadge\">OFF</span>\n"
    "         </div>\n"
    "         <div class=\"grid-2\">\n"
    "           <button onclick=\"startAuto()\">&#x25B6; Start</button>\n"
    "           <button class=\"btn-danger\" onclick=\"stopAuto()\">&#x23F9; Stop</button>\n"
    "         </div>\n"
    "         <button class=\"btn-secondary\" onclick=\"clearWaypoints()\" style=\"width: 100%; margin-top: 8px;\">&#x1F5D1; Clear Waypoints</button>\n"
    "         <div class=\"waypoint-list\" id=\"waypointList\">No waypoints. Tap the map to add.</div>\n"
    "       </div>\n"
    "\n"
    "       <!-- Calibration & Debug Card -->\n"
    "       <div class=\"card\">\n"
    "         <h2 class=\"card-title\">System</h2>\n"
    "         <button class=\"btn-secondary\" onclick=\"calibrateNorth()\" style=\"width: 100%;\">&#x1F9ED; Calibrate North</button>\n"
    "         <div id=\"debugInfo\">Idle</div>\n"
    "       </div>\n"
    "     </div>\n"
    "   </div>\n"
    "\n"
    "   <script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>\n"
    "   <script>\n"
    "     // ---------- System Helpers ----------\n"
    "     function send(c) {\n"
    "       fetch('/cmd?c=' + encodeURIComponent(c))\n"
    "         .then(r => { setConnected(true); })\n"
    "         .catch(() => setConnected(false));\n"
    "     }\n"
    "     function startMove(c) { send(c); }\n"
    "     function stopMove() { send('X'); }\n"
    "     function setConnected(ok) {\n"
    "       document.getElementById('connDot').className = ok ? 'dot' : 'dot offline';\n"
    "       document.getElementById('connText').innerText = ok ? 'Connected' : 'Disconnected';\n"
    "     }\n"
    "     function calibrateNorth() {\n"
    "       if (confirm('Point the robot at true North, then press OK.')) { fetch('/calibrateNorth'); }\n"
    "     }\n"
    "\n"
    "     // ---------- Auto & Waypoints ----------\n"
    "     let waypointMarkers = [];\n"
    "     let routePolyline = null;\n"
    "     let targetMarker = null;\n"
    "     let autoActive = false;\n"
    "\n"
    "     // FIX: Added the missing startAuto() and stopAuto() functions\n"
    "     function startAuto() {\n"
    "       fetch('/startauto').then(() => { autoActive = true; updateAutoBadge(); });\n"
    "     }\n"
    "     function stopAuto() {\n"
    "       fetch('/stopauto').then(() => { autoActive = false; updateAutoBadge(); });\n"
    "     }\n"
    "     function clearWaypoints() {\n"
    "       if (confirm('Clear all waypoints?')) {\n"
    "         fetch('/clearwaypoints').then(() => {\n"
    "           waypointMarkers.forEach(m => map.removeLayer(m));\n"
    "           waypointMarkers = [];\n"
    "           if (routePolyline) { map.removeLayer(routePolyline); routePolyline = null; }\n"
    "           if (targetMarker) { map.removeLayer(targetMarker); targetMarker = null; }\n"
    "           updateWaypointList();\n"
    "           autoActive = false;\n"
    "           updateAutoBadge();\n"
    "         });\n"
    "       }\n"
    "     }\n"
    "     function updateAutoBadge() {\n"
    "       const badge = document.getElementById('autoBadge');\n"
    "       badge.className = autoActive ? 'badge active' : 'badge';\n"
    "       badge.innerText = autoActive ? 'ACTIVE' : 'OFF';\n"
    "     }\n"
    "     function updateWaypointList() {\n"
    "       const div = document.getElementById('waypointList');\n"
    "       if (waypointMarkers.length === 0) div.innerHTML = 'No waypoints. Tap the map to add.';\n"
    "       else div.innerHTML = waypointMarkers.map((m, i) => `<div>WP${i+1}: ${m.getLatLng().lat.toFixed(5)}, ${m.getLatLng().lng.toFixed(5)}</div>`).join('');\n"
    "     }\n"
    "     function updateRouteLine() {\n"
    "       if (routePolyline) map.removeLayer(routePolyline);\n"
    "       if (robotMarker && waypointMarkers.length > 0) {\n"
    "         const points = [robotMarker.getLatLng(), ...waypointMarkers.map(m => m.getLatLng())];\n"
    "         routePolyline = L.polyline(points, { color: '#3b82f6', weight: 4, dashArray: '8, 8' }).addTo(map);\n"
    "       }\n"
    "     }\n"
    "     function updateTargetMarker() {\n"
    "       if (targetMarker) map.removeLayer(targetMarker);\n"
    "       if (waypointMarkers.length > 0) {\n"
    "         targetMarker = L.circle(waypointMarkers[0].getLatLng(), { radius: 3, color: '#ef4444', fillColor: '#ef4444', fillOpacity: 0.5 }).addTo(map);\n"
    "       }\n"
    "     }\n"
    "\n"
    "     // ---------- Map ----------\n"
    "     const map = L.map('map').setView([0, 0], 2);\n"
    "     L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(map);\n"
    "     let robotMarker = null;\n"
    "     let robotArrowEl = null;\n"
    "\n"
    "     function ensureMarker(lat, lon) {\n"
    "       if (robotMarker) return;\n"
    "       const icon = L.divIcon({ className: '', html: '<div class=\"robot-arrow\" id=\"robotArrow\">&#x25B2;</div>', iconSize: [22, 22], iconAnchor: [11, 11] });\n"
    "       robotMarker = L.marker([lat, lon], { icon }).addTo(map);\n"
    "       robotArrowEl = document.getElementById('robotArrow');\n"
    "       map.setView([lat, lon], 18);\n"
    "     }\n"
    "\n"
    "     const RecenterControl = L.Control.extend({\n"
    "       options: { position: 'topright' },\n"
    "       onAdd: function() {\n"
    "         const div = L.DomUtil.create('div', 'leaflet-bar');\n"
    "         div.innerHTML = '<a href=\"#\" style=\"width:34px;height:34px;line-height:34px;text-align:center;text-decoration:none;color:black;background:white;display:block;font-weight:bold;\" title=\"Recenter\">&#x2316;</a>';\n"
    "         div.onclick = (e) => { e.preventDefault(); if (robotMarker) map.setView(robotMarker.getLatLng(), map.getZoom()); };\n"
    "         return div;\n"
    "       }\n"
    "     });\n"
    "     map.addControl(new RecenterControl());\n"
    "\n"
    "     map.on('click', function(e) {\n"
    "       if (autoActive) return alert('Stop auto mode before adding waypoints');\n"
    "       fetch(`/addwaypoint?lat=${e.latlng.lat}&lon=${e.latlng.lng}`)\n"
    "         .then(() => {\n"
    "           waypointMarkers.push(L.marker([e.latlng.lat, e.latlng.lng]).addTo(map));\n"
    "           updateWaypointList();\n"
    "           updateRouteLine();\n"
    "           updateTargetMarker();\n"
    "         });\n"
    "     });\n"
    "\n"
    "     // ---------- Telemetry Loop ----------\n"
    "     setInterval(async () => {\n"
    "       try {\n"
    "         const r = await fetch('/telemetry');\n"
    "         const t = await r.json();\n"
    "         \n"
    "         document.getElementById('headingValue').innerText = Math.round(t.gyroHeadingDeg) + '\\u00B0';\n"
    "         const rot = 'rotate(' + t.gyroHeadingDeg + 'deg)';\n"
    "         document.getElementById('compassNeedle').style.transform = rot;\n"
    "         if (robotArrowEl) robotArrowEl.style.transform = rot;\n"
    "         \n"
    "         document.getElementById('speedValue').innerText = t.motorSpeed;\n"
    "         document.getElementById('debugInfo').innerText = t.debug || 'Idle';\n"
    "\n"
    "         if (t.gpsValid) {\n"
    "           document.getElementById('gpsStatus').innerHTML = `<span style=\"color:#10b981\">3D Fix (HDOP: ${t.hdop.toFixed(1)})</span>`;\n"
    "           document.getElementById('satCount').innerText = t.satellites;\n"
    "           ensureMarker(t.lat, t.lon);\n"
    "           robotMarker.setLatLng([t.lat, t.lon]);\n"
    "           updateRouteLine();\n"
    "         } else {\n"
    "           document.getElementById('gpsStatus').innerText = 'Waiting for fix...';\n"
    "           document.getElementById('satCount').innerText = '--';\n"
    "         }\n"
    "\n"
    "         if (t.autoMode !== autoActive) {\n"
    "           autoActive = t.autoMode;\n"
    "           updateAutoBadge();\n"
    "         }\n"
    "\n"
    "         if (t.waypoints < waypointMarkers.length) {\n"
    "           const removed = waypointMarkers.shift();\n"
    "           if (removed) map.removeLayer(removed);\n"
    "           updateWaypointList();\n"
    "           updateRouteLine();\n"
    "           updateTargetMarker();\n"
    "         }\n"
    "         setConnected(true);\n"
    "       } catch (e) { setConnected(false); }\n"
    "     }, 1000);\n"
    "   </script>\n"
    "</body>\n"
    "</html>\n";

    server.send(200, "text/html", html);
}
