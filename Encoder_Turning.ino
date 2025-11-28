#include <AFMotor.h>
#include <Servo.h>

// ============================================================
// MOTORS
// ============================================================
AF_DCMotor rightFront(4);
AF_DCMotor rightBack(3);
AF_DCMotor leftBack(2);
AF_DCMotor leftFront(1);

// ============================================================
// ULTRASONIC + SERVO
// ============================================================
#define SERVO_PIN 23
#define TRIG_PIN 35
#define ECHO_PIN 37
Servo servoLook;

// ============================================================
// ENCODERS
// ============================================================
#define ENC1_A 18
#define ENC1_B 40
#define ENC2_A 19
#define ENC2_B 44
#define ENC3_A 20
#define ENC3_B 28
#define ENC4_A 21
#define ENC4_B 22

volatile long encoderCount[4] = {0,0,0,0};

// Encoder ISRs
void encoder1A_ISR() { encoderCount[0] += digitalRead(ENC1_B) ? 1 : -1; }
void encoder2A_ISR() { encoderCount[1] += digitalRead(ENC2_B) ? 1 : -1; }
void encoder3A_ISR() { encoderCount[2] += digitalRead(ENC3_B) ? 1 : -1; }
void encoder4A_ISR() { encoderCount[3] += digitalRead(ENC4_B) ? 1 : -1; }

// ============================================================
// SETTINGS
// ============================================================
int motorSpeed = 150;
int motorOffset = 6;
int reverseOffset = 55;

// *** ENCODER TURNING VALUE ***
// Adjust after testing
const long TURN_90_TICKS = 420;

// ============================================================
// STATE MACHINE
// ============================================================
enum State { MOVING_FORWARD, OBSTACLE_DETECTED, SCANNING, TURNING };
State currentState = MOVING_FORWARD;

bool turningLeft = false;
long turnStartLeftTicks = 0;
long turnStartRightTicks = 0;

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(9600);

  servoLook.attach(SERVO_PIN);
  servoLook.write(90);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP);
  pinMode(ENC4_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), encoder3A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), encoder4A_ISR, CHANGE);

  stopMove();
  Serial.println("Ready.");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {

  // Exit turn when encoder count reached
  if (currentState == TURNING) {
    if (checkTurnComplete()) {
      stopMove();
      currentState = MOVING_FORWARD;
    }
  }

  switch (currentState) {

    case MOVING_FORWARD: {
      moveForward();
      int front = getDistance();

      if (front > 0 && front < 18) {
        stopMove();
        currentState = OBSTACLE_DETECTED;
      }
      break;
    }

    case OBSTACLE_DETECTED:
      Reverse();
      currentState = SCANNING;
      break;

    case SCANNING:
      handleScanning();
      break;

    case TURNING:
      break;
  }
}

// ============================================================
// BASIC MOVEMENT
// ============================================================
void moveForward() {
  rightFront.setSpeed(motorSpeed + motorOffset);
  rightBack.setSpeed(motorSpeed + motorOffset);
  leftFront.setSpeed(motorSpeed);
  leftBack.setSpeed(motorSpeed);

  rightFront.run(FORWARD);
  rightBack.run(FORWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
}

void stopMove() {
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void Reverse() {
  rightFront.setSpeed(motorSpeed - reverseOffset);
  rightBack.setSpeed(motorSpeed - reverseOffset);
  leftFront.setSpeed(motorSpeed);
  leftBack.setSpeed(motorSpeed);

  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);

  delay(3500);
  stopMove();
}

// ============================================================
// *** ENCODER TURNING ***
// ============================================================
void turnLeft_Encoder() {
  turningLeft = true;
  turnStartLeftTicks = encoderCount[2];
  turnStartRightTicks = encoderCount[0];

  // Right wheels forward, left wheels stop
  rightFront.setSpeed(motorSpeed + 50);
  rightBack.setSpeed(motorSpeed + 50);
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);

  leftFront.run(RELEASE);
  leftBack.run(RELEASE);

  currentState = TURNING;
}

void turnRight_Encoder() {
  turningLeft = false;
  turnStartLeftTicks = encoderCount[2];
  turnStartRightTicks = encoderCount[0];

  // Left wheels forward, right wheels stop
  leftFront.setSpeed(motorSpeed + 50);
  leftBack.setSpeed(motorSpeed + 50);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);

  rightFront.run(RELEASE);
  rightBack.run(RELEASE);

  currentState = TURNING;
}

bool checkTurnComplete() {
  long leftDelta  = labs(encoderCount[2] - turnStartLeftTicks);
  long rightDelta = labs(encoderCount[0] - turnStartRightTicks);

  long rotationTicks = leftDelta + rightDelta;  // combined motion

  return rotationTicks >= TURN_90_TICKS;
}

// ============================================================
// SCANNING AND TURN DECISION
// ============================================================
void handleScanning() {
  stopMove();

  servoLook.write(90);
  delay(250);

  // LEFT side
  servoLook.write(160);
  delay(400);
  int leftDist = getDistance();

  // RIGHT side
  servoLook.write(20);
  delay(400);
  int rightDist = getDistance();

  servoLook.write(90);
  delay(200);

  // Decision
  if (rightDist < leftDist) {
    turnLeft_Encoder();
    return;
  }

  if (rightDist > leftDist) {
    turnRight_Encoder();
    return;
  }

  // Equal and blocked → reverse
  if (rightDist == leftDist && leftDist < 18) {
    Reverse();
    currentState = SCANNING;
    return;
  }

  // Equal and clear → turn left
  turnLeft_Encoder();
}

// ============================================================
// ULTRASONIC
// ============================================================
int getDistance() {
  long durationSum = 0;
  int samples = 3;

  for (int i = 0; i < samples; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 25000);
    if (duration == 0) duration = 30000;

    durationSum += duration;
    delay(10);
  }

  long duration = durationSum / samples;
  int cm = duration * 0.034 / 2;

  if (cm < 2 || cm > 80) return 999;
  return cm;
}
