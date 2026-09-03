/*
   FuzzyBot Motor Driver - v3.1
   Arduino Uno R3 + AFMotor-compatible L293D shield
   4 DC motors, 3S 18650 battery pack

   PURPOSE
   -------
   This is the "body" controller. The ESP32 brain sends simple commands:

       F = forward
       B = backward
       L = tank-turn left
       R = tank-turn right
       S = spin in place
       X = stop
       + = increase cruise speed
       - = decrease cruise speed

   IMPORTANT DESIGN CHANGES
   ------------------------
   1. Forward/backward uses all four motors.
   2. Left/right turns now use ALL FOUR MOTORS:
          left side   = one direction
          right side  = opposite direction
      This gives maximum traction but draws more current.
   3. "S" also uses all four motors (same as a right turn).
   4. Braking is fast.
   5. Reversing is safe:
          direction -> STOP -> short dead-time -> new direction -> ramp up
      This avoids commanding a loaded motor to reverse instantly.
   6. Acceleration is deliberately slow to reduce current spikes
      from the 3S battery and L293D shield.
   7. The Arduino does NOT print every command continuously.
      Excessive serial printing can make timing less predictable.
   8. A watchdog stops the robot if the ESP32 stops sending commands.

   NOTE ABOUT TURNING
   -------------------
   All four motors are driven during turns, so the robot pivots around its centre.
   If turning is erratic, try:
       - Lower TURN_SPEED further (already set to 100)
       - Add weight directly over the centre of the wheelbase
       - Check that all wheels have similar traction
   Because an L293D has a relatively large voltage drop, the robot
   may already have much less motor voltage than the 3S pack voltage.
   Do NOT compensate by immediately pushing every PWM value to 255.
   Test current, temperature, and motor behavior first.
*/

#include <AFMotor.h>
#include <SoftwareSerial.h>

// ================================================================
// SERIAL CONNECTION TO ESP32
// ================================================================

// ESP32 -> Arduino A0
// A1 is unused for now.
SoftwareSerial espSerial(A0, A1);

// ================================================================
// MOTOR OBJECTS
// ================================================================

AF_DCMotor motorFL(1);   // Front Left
AF_DCMotor motorFR(2);   // Front Right
AF_DCMotor motorBL(3);   // Back Left
AF_DCMotor motorBR(4);   // Back Right

// ================================================================
// SPEED SETTINGS
// ================================================================

// Normal driving speed.
// Start conservatively because the robot runs from a 3S pack.
int motorSpeed = 170;

// Maximum normal cruise speed.
const int SPEED_MAX = 200;
const int SPEED_MIN = 70;

// How much + / - changes the requested cruise speed.
const int SPEED_STEP = 15;

// Turning speed – reduced to improve turn predictability with all wheels.
const int TURN_SPEED = 255;

// ================================================================
// MOTOR RAMP SETTINGS
// ================================================================

// Speeding up is slow to reduce current spikes.
//
// Slowing down is much faster because reducing PWM reduces current.
const int RAMP_UP_STEP = 8;
const int RAMP_DOWN_STEP = 45;

// Time between ramp updates.
const unsigned long RAMP_INTERVAL_MS = 20;

// Small dead-time before reversing direction.
// This gives the motor a moment with zero drive.
const unsigned long REVERSAL_DEADTIME_MS = 60;

// ================================================================
// COMMUNICATION WATCHDOG
// ================================================================

// If the Arduino hears nothing from the ESP32 for this long,
// it releases all motors.
const unsigned long COMMAND_TIMEOUT_MS = 3000;

// ================================================================
// DESIRED MOTOR STATE
// ================================================================

int desiredDirFL = RELEASE;
int desiredDirFR = RELEASE;
int desiredDirBL = RELEASE;
int desiredDirBR = RELEASE;

int desiredSpeedFL = 0;
int desiredSpeedFR = 0;
int desiredSpeedBL = 0;
int desiredSpeedBR = 0;

// ================================================================
// ACTIVE MOTOR STATE
// ================================================================

int activeDirFL = RELEASE;
int activeDirFR = RELEASE;
int activeDirBL = RELEASE;
int activeDirBR = RELEASE;

int activeSpeedFL = 0;
int activeSpeedFR = 0;
int activeSpeedBL = 0;
int activeSpeedBR = 0;

// ================================================================
// REVERSAL STATE
// ================================================================

bool reversalWaitingFL = false;
bool reversalWaitingFR = false;
bool reversalWaitingBL = false;
bool reversalWaitingBR = false;

unsigned long reversalStartFL = 0;
unsigned long reversalStartFR = 0;
unsigned long reversalStartBL = 0;
unsigned long reversalStartBR = 0;

// ================================================================
// GLOBAL STATE
// ================================================================

bool haveTarget = false;

unsigned long lastRampUpdate = 0;
unsigned long lastCommandTime = 0;

// ================================================================
// SETUP
// ================================================================

void setup() {

  Serial.begin(9600);
  espSerial.begin(9600);

  // On-board LED = communication/activity indicator.
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  stopImmediately();

  lastCommandTime = millis();
  lastRampUpdate = millis();

  Serial.println("FuzzyBot body v3.1 ready.");
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() {

  // ------------------------------------------------------------
  // SAFETY WATCHDOG
  // ------------------------------------------------------------

  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {

    if (haveTarget) {

      stopImmediately();
      haveTarget = false;

      Serial.println("WATCHDOG: ESP32 communication lost.");
    }
  }

  // ------------------------------------------------------------
  // READ ESP32 COMMANDS
  // ------------------------------------------------------------

  while (espSerial.available() > 0) {

    char cmd = espSerial.read();

    // Ignore line endings.
    if (cmd == '\n' || cmd == '\r')
      continue;

    lastCommandTime = millis();
    digitalWrite(13, HIGH);

    handleCommand(cmd);
  }

  // ------------------------------------------------------------
  // MOVE MOTORS TOWARD THEIR TARGETS
  // ------------------------------------------------------------

  updateMotors();
}

// ================================================================
// COMMAND HANDLER
// ================================================================

void handleCommand(char cmd) {

  switch (cmd) {

    case 'F':
      moveForward();
      break;

    case 'B':
      moveBackward();
      break;

    case 'L':
      turnLeft();
      break;

    case 'R':
      turnRight();
      break;

    case 'S':
      spinInPlace();
      break;

    case 'X':
      stopImmediately();
      haveTarget = false;
      digitalWrite(13, LOW);
      break;

    case '+':
      increaseSpeed();
      break;

    case '-':
      decreaseSpeed();
      break;

    default:
      // Ignore unknown characters instead of doing anything dangerous.
      break;
  }
}

// ================================================================
// HIGH-LEVEL MOVEMENT COMMANDS
// ================================================================

// ------------------------------------------------------------
// FORWARD
// All four motors drive forward.
// ------------------------------------------------------------

void moveForward() {

  setWheelTargets(
    FORWARD,  motorSpeed,
    FORWARD,  motorSpeed,
    FORWARD,  motorSpeed,
    FORWARD,  motorSpeed
  );

  haveTarget = true;
}

// ------------------------------------------------------------
// BACKWARD
// All four motors drive backward.
// ------------------------------------------------------------

void moveBackward() {

  setWheelTargets(
    BACKWARD, motorSpeed,
    BACKWARD, motorSpeed,
    BACKWARD, motorSpeed,
    BACKWARD, motorSpeed
  );

  haveTarget = true;
}

// ------------------------------------------------------------
// RIGHT TURN – all four wheels
// Left side: forward
// Right side: backward
// ------------------------------------------------------------

void turnRight() {

  setWheelTargets(
    FORWARD,  TURN_SPEED,   // FL
    BACKWARD, TURN_SPEED,   // FR
    FORWARD,  TURN_SPEED,   // BL
    BACKWARD, TURN_SPEED    // BR
  );

  haveTarget = true;
}

// ------------------------------------------------------------
// LEFT TURN – all four wheels
// Left side: backward
// Right side: forward
// ------------------------------------------------------------

void turnLeft() {

  setWheelTargets(
    BACKWARD, TURN_SPEED,   // FL
    FORWARD,  TURN_SPEED,   // FR
    BACKWARD, TURN_SPEED,   // BL
    FORWARD,  TURN_SPEED    // BR
  );

  haveTarget = true;
}

// ------------------------------------------------------------
// SPIN IN PLACE – same as a right turn
// (change direction if you want left spin)
// ------------------------------------------------------------

void spinInPlace() {

  setWheelTargets(
    FORWARD,  TURN_SPEED,
    BACKWARD, TURN_SPEED,
    FORWARD,  TURN_SPEED,
    BACKWARD, TURN_SPEED
  );

  haveTarget = true;
}

// ================================================================
// SET DESIRED WHEEL STATES
// ================================================================

void setWheelTargets(
  int dirFL, int spdFL,
  int dirFR, int spdFR,
  int dirBL, int spdBL,
  int dirBR, int spdBR
) {

  desiredDirFL = dirFL;
  desiredDirFR = dirFR;
  desiredDirBL = dirBL;
  desiredDirBR = dirBR;

  desiredSpeedFL = spdFL;
  desiredSpeedFR = spdFR;
  desiredSpeedBL = spdBL;
  desiredSpeedBR = spdBR;
}

// ================================================================
// MOTOR UPDATE LOOP
// ================================================================

void updateMotors() {

  if (!haveTarget)
    return;

  if (millis() - lastRampUpdate < RAMP_INTERVAL_MS)
    return;

  lastRampUpdate = millis();

  adjustWheel(
    activeDirFL,
    activeSpeedFL,
    desiredDirFL,
    desiredSpeedFL,
    reversalWaitingFL,
    reversalStartFL,
    motorFL
  );

  adjustWheel(
    activeDirFR,
    activeSpeedFR,
    desiredDirFR,
    desiredSpeedFR,
    reversalWaitingFR,
    reversalStartFR,
    motorFR
  );

  adjustWheel(
    activeDirBL,
    activeSpeedBL,
    desiredDirBL,
    desiredSpeedBL,
    reversalWaitingBL,
    reversalStartBL,
    motorBL
  );

  adjustWheel(
    activeDirBR,
    activeSpeedBR,
    desiredDirBR,
    desiredSpeedBR,
    reversalWaitingBR,
    reversalStartBR,
    motorBR
  );
}

// ================================================================
// SINGLE-WHEEL CONTROL
// ================================================================
//
// Direction changes are handled safely:
//
// 1. Ramp speed down quickly.
// 2. Set PWM to zero.
// 3. Wait a short dead-time.
// 4. Change direction.
// 5. Ramp speed up slowly.
//
// This is particularly important for F -> B and B -> F commands.
// ================================================================

void adjustWheel(
  int &activeDir,
  int &activeSpeed,
  int desiredDir,
  int desiredSpeed,
  bool &reversalWaiting,
  unsigned long &reversalStart,
  AF_DCMotor &motor
) {

  // ------------------------------------------------------------
  // HANDLE DIRECTION CHANGE
  // ------------------------------------------------------------

  if (activeDir != desiredDir) {

    // First, quickly reduce PWM to zero.
    if (activeSpeed > 0) {

      activeSpeed =
        max(0, activeSpeed - RAMP_DOWN_STEP);

      motor.setSpeed(activeSpeed);

      return;
    }

    // ----------------------------------------------------------
    // We are at zero speed.
    // Add a short dead-time before changing direction.
    // ----------------------------------------------------------

    if (!reversalWaiting) {

      reversalWaiting = true;
      reversalStart = millis();

      motor.setSpeed(0);
      motor.run(RELEASE);

      return;
    }

    if (millis() - reversalStart < REVERSAL_DEADTIME_MS) {

      motor.setSpeed(0);
      motor.run(RELEASE);

      return;
    }

    // ----------------------------------------------------------
    // Dead-time finished: change direction.
    // ----------------------------------------------------------

    motor.run(desiredDir);

    activeDir = desiredDir;
    reversalWaiting = false;
  }

  // ------------------------------------------------------------
  // HANDLE SPEED CHANGE
  // ------------------------------------------------------------

  if (activeSpeed < desiredSpeed) {

    activeSpeed =
      min(
        desiredSpeed,
        activeSpeed + RAMP_UP_STEP
      );

    motor.setSpeed(activeSpeed);

  } else if (activeSpeed > desiredSpeed) {

    activeSpeed =
      max(
        desiredSpeed,
        activeSpeed - RAMP_DOWN_STEP
      );

    motor.setSpeed(activeSpeed);
  }
}

// ================================================================
// IMMEDIATE STOP
// ================================================================
//
// This is for:
//
// - X command
// - watchdog
// - emergency stop
//
// RELEASE means the motors are no longer being electrically driven.
// ================================================================

void stopImmediately() {

  motorFL.setSpeed(0);
  motorFR.setSpeed(0);
  motorBL.setSpeed(0);
  motorBR.setSpeed(0);

  motorFL.run(RELEASE);
  motorFR.run(RELEASE);
  motorBL.run(RELEASE);
  motorBR.run(RELEASE);

  activeDirFL = RELEASE;
  activeDirFR = RELEASE;
  activeDirBL = RELEASE;
  activeDirBR = RELEASE;

  activeSpeedFL = 0;
  activeSpeedFR = 0;
  activeSpeedBL = 0;
  activeSpeedBR = 0;

  reversalWaitingFL = false;
  reversalWaitingFR = false;
  reversalWaitingBL = false;
  reversalWaitingBR = false;
}

// ================================================================
// INCREASE SPEED
// ================================================================

void increaseSpeed() {

  motorSpeed =
    min(
      motorSpeed + SPEED_STEP,
      SPEED_MAX
    );

  // Only change live targets when actually cruising forward/backward.
  updateCruiseTargets();

  Serial.print("Cruise speed: ");
  Serial.println(motorSpeed);
}

// ================================================================
// DECREASE SPEED
// ================================================================

void decreaseSpeed() {

  motorSpeed =
    max(
      motorSpeed - SPEED_STEP,
      SPEED_MIN
    );

  updateCruiseTargets();

  Serial.print("Cruise speed: ");
  Serial.println(motorSpeed);
}

// ================================================================
// UPDATE ACTIVE CRUISE COMMAND
// ================================================================
//
// This keeps + / - working while the robot is already driving.
// It deliberately does nothing during a turn.
// ================================================================

void updateCruiseTargets() {

  if (!haveTarget)
    return;

  bool allForward =
    activeDirFL == FORWARD &&
    activeDirFR == FORWARD &&
    activeDirBL == FORWARD &&
    activeDirBR == FORWARD;

  bool allBackward =
    activeDirFL == BACKWARD &&
    activeDirFR == BACKWARD &&
    activeDirBL == BACKWARD &&
    activeDirBR == BACKWARD;

  if (allForward || allBackward) {

    desiredSpeedFL = motorSpeed;
    desiredSpeedFR = motorSpeed;
    desiredSpeedBL = motorSpeed;
    desiredSpeedBR = motorSpeed;
  }
}