/*
   FuzzyBot Motor Driver - powerful, safe, simple
   Listens to ESP32 on A0 via SoftwareSerial.
   Drives 4 DC motors through an Adafruit Motor Shield (AFMotor).

   - Increased default speeds so the robot can actually move.
   - Soft-start ramp for forward/backward/spin to prevent sudden current
     spikes and make motion smoother.
   - Direct turn commands (L/R) for quick in-place rotation.
   - 3-second watchdog stops motors if ESP32 connection is lost.
*/

#include <AFMotor.h>
#include <SoftwareSerial.h>

// Listen to ESP32 on A0 (RX), A1 unused (TX)
SoftwareSerial espSerial(A0, A1);

AF_DCMotor motorFL(1);
AF_DCMotor motorFR(2);
AF_DCMotor motorBL(3);
AF_DCMotor motorBR(4);

// ---------------- Tunables ----------------
int motorSpeed        = 220;   // cruise speed for F/B (0-255)
const int TURN_SPEED  = 180;   // speed for L/R (in-place turn)
const int SPIN_SPEED  = 220;   // speed for S (manual spin)
const int SPEED_STEP  = 20;    // amount +/- changes cruise speed
const int SPEED_MAX   = 255;
const int SPEED_MIN   = 0;

// Soft-start ramp (only for F/B/S)
const int RAMP_STEP_SIZE     = 15;   // PWM increase per step
const unsigned long RAMP_INTERVAL = 20; // ms between ramp steps

const unsigned long COMMAND_TIMEOUT = 3000; // ms of silence before auto-stop
// -------------------------------------------

// Current commanded state
bool haveTarget = false;          // true when we should be moving
int  desiredDirLeft  = RELEASE;
int  desiredSpeedLeft = 0;
int  desiredDirRight = RELEASE;
int  desiredSpeedRight = 0;

// Current motor state (for ramp)
int  activeDirLeft  = RELEASE;
int  activeSpeedLeft = 0;
int  activeDirRight = RELEASE;
int  activeSpeedRight = 0;

unsigned long lastRampStep = 0;
unsigned long lastCommandTime = 0;

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Start all motors released
  releaseAll();
  lastCommandTime = millis();
  lastRampStep = millis();

  Serial.println("Motor driver ready. Listening on A0...");
}

void loop() {
  // Watchdog: if no command for 3 seconds, stop safely
  if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
    if (haveTarget) {
      Serial.println("WATCHDOG: no command - stopping.");
      releaseAll();
      haveTarget = false;
      digitalWrite(13, LOW);
    }
  }

  // Process incoming commands
  if (espSerial.available() > 0) {
    char cmd = espSerial.read();
    if (cmd != '\n' && cmd != '\r') {
      lastCommandTime = millis();
      digitalWrite(13, HIGH);
      Serial.print("Command: ");
      Serial.println(cmd);

      switch (cmd) {
        case 'F': setTarget(FORWARD, motorSpeed, FORWARD, motorSpeed, true); break;
        case 'B': setTarget(BACKWARD, motorSpeed, BACKWARD, motorSpeed, true); break;
        case 'L': setTarget(BACKWARD, TURN_SPEED, FORWARD, TURN_SPEED, false); break;
        case 'R': setTarget(FORWARD, TURN_SPEED, BACKWARD, TURN_SPEED, false); break;
        case 'S': setTarget(FORWARD, SPIN_SPEED, BACKWARD, SPIN_SPEED, true); break;
        case 'X': releaseAll(); haveTarget = false; digitalWrite(13, LOW); break;
        case '+': increaseSpeed(); break;
        case '-': decreaseSpeed(); break;
        default:   Serial.println("Unknown command"); break;
      }
    }
  }

  // Update motor speeds with ramp (non-blocking)
  updateMotors();
}

// ------------------------------------------------------------
// Set desired movement direction and speed for both sides.
// If useRamp is true, the new target will be approached gradually.
// If false (turns), it applies immediately.
// ------------------------------------------------------------
void setTarget(int dirL, int spdL, int dirR, int spdR, bool useRamp) {
  desiredDirLeft   = dirL;
  desiredSpeedLeft  = spdL;
  desiredDirRight  = dirR;
  desiredSpeedRight = spdR;
  haveTarget = true;

  if (!useRamp) {
    // Direct set for turns (L/R) – instant response
    applyMotorState(desiredDirLeft, desiredSpeedLeft,
                    desiredDirRight, desiredSpeedRight);
  }
  // If useRamp is true, updateMotors() will gradually ramp to target
}

// ------------------------------------------------------------
// Gradually adjust motor speeds to match desired target.
// Called every loop iteration, but only steps every RAMP_INTERVAL ms.
// ------------------------------------------------------------
void updateMotors() {
  if (!haveTarget) return;   // already stopped

  if (millis() - lastRampStep < RAMP_INTERVAL) return;
  lastRampStep = millis();

  // Adjust left side
  adjustSide(activeDirLeft, activeSpeedLeft, desiredDirLeft, desiredSpeedLeft,
             motorFL, motorBL);
  // Adjust right side
  adjustSide(activeDirRight, activeSpeedRight, desiredDirRight, desiredSpeedRight,
             motorFR, motorBR);
}

void adjustSide(int &actDir, int &actSpd, int desDir, int desSpd,
                AF_DCMotor &motorA, AF_DCMotor &motorB) {
  // If direction needs to change, first ramp down speed to 0,
  // then change direction and ramp up.
  if (actDir != desDir) {
    if (actSpd > 0) {
      actSpd = max(0, actSpd - RAMP_STEP_SIZE);
      motorA.setSpeed(actSpd);
      motorB.setSpeed(actSpd);
      return;
    } else {
      // Direction can be changed now
      motorA.run(desDir);
      motorB.run(desDir);
      actDir = desDir;
    }
  }

  // Adjust speed toward target
  if (actSpd < desSpd) {
    actSpd = min(desSpd, actSpd + RAMP_STEP_SIZE);
    motorA.setSpeed(actSpd);
    motorB.setSpeed(actSpd);
  } else if (actSpd > desSpd) {
    actSpd = max(desSpd, actSpd - RAMP_STEP_SIZE);
    motorA.setSpeed(actSpd);
    motorB.setSpeed(actSpd);
  }

  // If speed is zero and direction is not RELEASE, we may want to keep
  // direction active but at zero speed; that's fine.
}

// ------------------------------------------------------------
// Immediate motor state set (used for direct turns and stop)
// ------------------------------------------------------------
void applyMotorState(int dirL, int spdL, int dirR, int spdR) {
  motorFL.setSpeed(spdL);
  motorBL.setSpeed(spdL);
  motorFL.run(dirL);
  motorBL.run(dirL);

  motorFR.setSpeed(spdR);
  motorBR.setSpeed(spdR);
  motorFR.run(dirR);
  motorBR.run(dirR);

  activeDirLeft = dirL;
  activeSpeedLeft = spdL;
  activeDirRight = dirR;
  activeSpeedRight = spdR;
}

// ------------------------------------------------------------
// Stop all motors immediately (no ramp)
// ------------------------------------------------------------
void releaseAll() {
  motorFL.setSpeed(0); motorBL.setSpeed(0);
  motorFR.setSpeed(0); motorBR.setSpeed(0);
  motorFL.run(RELEASE); motorBL.run(RELEASE);
  motorFR.run(RELEASE); motorBR.run(RELEASE);

  activeDirLeft = RELEASE;
  activeSpeedLeft = 0;
  activeDirRight = RELEASE;
  activeSpeedRight = 0;
}

// ------------------------------------------------------------
// Increase / decrease cruise speed (affects F/B only)
// ------------------------------------------------------------
void increaseSpeed() {
  motorSpeed = min(motorSpeed + SPEED_STEP, SPEED_MAX);
  Serial.print("Speed: ");
  Serial.println(motorSpeed);

  // If currently moving forward/backward, update target speed
  if (haveTarget && activeDirLeft == FORWARD && activeDirRight == FORWARD) {
    desiredSpeedLeft = motorSpeed;
    desiredSpeedRight = motorSpeed;
  }
}

void decreaseSpeed() {
  motorSpeed = max(motorSpeed - SPEED_STEP, SPEED_MIN);
  Serial.print("Speed: ");
  Serial.println(motorSpeed);

  if (haveTarget && activeDirLeft == FORWARD && activeDirRight == FORWARD) {
    desiredSpeedLeft = motorSpeed;
    desiredSpeedRight = motorSpeed;
  }
}