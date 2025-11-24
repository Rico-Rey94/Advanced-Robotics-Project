#include <AFMotor.h>
#include <Servo.h>

// ---------------- Motors ----------------
AF_DCMotor rightFront(1);
AF_DCMotor rightBack(2);
AF_DCMotor leftBack(3);
AF_DCMotor leftFront(4);

// ---------------- Ultrasonic + Servo ----------------
#define SERVO_PIN 23
#define TRIG_PIN 35
#define ECHO_PIN 37
Servo servoLook;

// ---------------- Settings ----------------
int motorSpeed = 150;
int motorOffset = 6;          // forward straightening
int reverseOffset = 10;       // reverse straightening

enum State { MOVING_FORWARD, OBSTACLE_DETECTED, SCANNING, TURNING };
State currentState = MOVING_FORWARD;

unsigned long motionStartTime = 0;
unsigned long motionDuration = 0;
bool motionActive = false;

// ---------------- Setup ----------------
void setup() {
  Serial.begin(9600);
  Serial.println("Robot booting...");

  servoLook.attach(SERVO_PIN);
  servoLook.write(90);

  rightBack.setSpeed(motorSpeed);
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed);
  leftBack.setSpeed(motorSpeed);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMove();
  Serial.println("Ready.");
}

// ---------------- Main Loop ----------------
void loop() {

  // ---------- TURN EXIT HANDLER ----------
  if (motionActive && millis() - motionStartTime >= motionDuration) {
    stopMove();
    motionActive = false;
    currentState = MOVING_FORWARD;
  }

  switch (currentState) {

    case MOVING_FORWARD: {
      moveForward();
      int front = getDistance();

      if (front > 0 && front < 35) {
        Serial.print("Obstacle detected at ");
        Serial.print(front);
        Serial.println(" cm");
        stopMove();
        currentState = OBSTACLE_DETECTED;
      }
      break;
    }

    case OBSTACLE_DETECTED: {
      Reverse();
      currentState = SCANNING;
      break;
    }

    case SCANNING: {
      handleScanning();
      break;
    }

    case TURNING:
      // Motion handled by timing above
      break;
  }
}

// ---------------- Movement ----------------
void moveForward() {
  Serial.println("Moving forward");
  rightFront.setSpeed(motorSpeed - motorOffset);
  rightBack.setSpeed(motorSpeed - motorOffset);
  leftFront.setSpeed(motorSpeed + motorOffset);
  leftBack.setSpeed(motorSpeed + motorOffset);

  rightFront.run(FORWARD);
  rightBack.run(FORWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
}

void stopMove() {
  Serial.println("Stop");
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void Reverse() {
  // balanced reverse
  Serial.println("Reversing");
  rightFront.setSpeed(motorSpeed - reverseOffset);
  rightBack.setSpeed(motorSpeed - reverseOffset);
  leftFront.setSpeed(motorSpeed + reverseOffset);
  leftBack.setSpeed(motorSpeed + reverseOffset);

  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);

  delay(450);   // short controlled reverse
  stopMove();
}

// ---------------- Turning ----------------
void turnLeft(unsigned long duration) {
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);

  motionStartTime = millis();
  motionDuration = duration;
  motionActive = true;
  currentState = TURNING;
}

void turnRight(unsigned long duration) {
  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);

  motionStartTime = millis();
  motionDuration = duration;
  motionActive = true;
  currentState = TURNING;
}

// ---------------- SCANNING ----------------
void handleScanning() {
  stopMove();

  servoLook.write(90);
  delay(250);

  // Scan RIGHT
  servoLook.write(160);
  delay(400);
  int rightDist = getDistance();

  // Scan LEFT
  servoLook.write(20);
  delay(400);
  int leftDist = getDistance();

  // Return servo to center
  servoLook.write(90);
  delay(200);

  Serial.print("Left = ");
  Serial.print(leftDist);
  Serial.print(" | Right = ");
  Serial.println(rightDist);

  // ---------- DECISION LOGIC ----------
  if (rightDist < leftDist) {
    Serial.println("Turning LEFT");
    turnLeft(480);
    return;
  }

  if (rightDist > leftDist) {
    Serial.println("Turning RIGHT");
    turnRight(480);
    return;
  }

  // Equal distances but not 999 (both detect obstacle)
  if (rightDist == leftDist && leftDist != 999) {
    Serial.println("Distances equal -> reversing");
    Reverse();
    currentState = MOVING_FORWARD;
    return;
  }

  // Both 999 → wide open space
  Serial.println("Both sides clear, turning left.");
  turnLeft(900);
}

// ---------------- Ultrasonic ----------------
// Small averaging filter to stabilize readings
int getDistance() {
  long durationSum = 0;
  int samples = 3;

  for (int i = 0; i < samples; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 35000);
    if (duration == 0) duration = 60000; // treat as far away

    durationSum += duration;
    delay(10);
  }

  long duration = durationSum / samples;
  int cm = duration * 0.034 / 2;

  if (cm < 2 || cm > 80) return 999;
  return cm;
}
