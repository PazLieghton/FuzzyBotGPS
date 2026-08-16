/*
   GPS_Debug_Test - TEMPORARY diagnostic sketch, not the robot code.

   Upload this by itself, open the Serial Monitor at 115200 baud, and
   watch what shows up. This bypasses TinyGPS++ completely and just
   dumps whatever raw bytes arrive on the GPS wire straight to the
   console, so you can see exactly what the ESP32 is (or isn't)
   receiving from the module.

   A working NEO-6M sends NMEA sentences once per second REGARDLESS of
   whether it has a satellite fix - fix status is just one field inside
   each sentence, not a gate on whether it talks at all. So:

   - You see lines starting with $GPGGA / $GPRMC / $GPGSV etc, once a
     second, even with empty/zero coordinate fields:
       -> Wiring and baud rate are correct. The problem is genuinely
          "no satellites yet" - check antenna placement/orientation,
          power stability, and that you're actually under open sky
          (not just outdoors near buildings/trees/a metal chassis).

   - You see nothing at all, ever:
       -> No data is reaching the ESP32. Check the wiring is CROSSED
          (GPS TX -> ESP32 GPIO25, GPS RX -> ESP32 GPIO26 - a very
          common mistake is wiring TX-to-TX / RX-to-RX instead), check
          the module is actually getting power (look for its power
          LED), and confirm the baud rate (9600 is the NEO-6M default,
          but try 4800 if a previous owner may have reconfigured it).

   - You see garbled/random characters instead of readable text:
       -> Baud rate mismatch. Try 4800 instead of 9600 below.

   Once you've confirmed real NMEA sentences are flowing, re-upload the
   actual Brain.ino - this file is throwaway, just for isolating the
   problem.
*/

HardwareSerial gpsSerial(1);
const int GPS_RX_PIN = 25; // ESP32 RX <- GPS TX
const int GPS_TX_PIN = 26; // ESP32 TX -> GPS RX
const uint32_t GPS_BAUD = 9600; // try 4800 here if you see garbage instead

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("Listening for raw GPS data on GPIO25/26...");
  Serial.println("(nothing below this line for 5+ seconds = no data arriving)");
}

void loop() {
  while (gpsSerial.available() > 0) {
    Serial.write(gpsSerial.read());
  }
}