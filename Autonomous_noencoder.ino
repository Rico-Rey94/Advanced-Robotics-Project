#include <AFMotor.h>
#include <Servo.h>

// ---------------- Motors ----------------
AF_DCMotor rightFront(4);
AF_DCMotor rightBack(3);
AF_DCMotor leftBack(2);
AF_DCMotor leftFront(2);

// ---------------- Ultrasonic + Servo ----------------
#define SERVO_PIN 23
#define TRIG_PIN 35
#define ECHO_PIN 37
Servo servoLook;

// ---------------- Settings ----------------
int motorSpeed = 150;
int motorOffset = 6;          // forward straightening
int reverseOffset = 55;       // reverse straightening

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
      //Serial.println("Moving forward");
      moveForward();
      int front = getDistance();

      if (front > 0 && front < 18) {
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
  //Serial.println("Stop");
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void Reverse() {
  // balanced reverse
  //Serial.println("Reversing");
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

// ---------------- Turning ----------------
void turnLeft(unsigned long duration) {
  rightFront.setSpeed(motorSpeed + 50);
  rightBack.setSpeed(motorSpeed + 50);
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);

  motionStartTime = millis();
  motionDuration = duration;
  motionActive = true;
  currentState = TURNING;
}

void turnRight(unsigned long duration) {
  leftFront.setSpeed(motorSpeed + 50);
  leftBack.setSpeed(motorSpeed + 50);
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
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
  int leftDist = getDistance();

  // Scan LEFT
  servoLook.write(20);
  delay(400);
  int rightDist = getDistance();

  // Return servo to center
  servoLook.write(90);
  delay(200);

  Serial.print("Left = ");
  Serial.print(leftDist);
  Serial.print(" | Right = ");
  Serial.println(rightDist);

  // ---------- DECISION LOGIC ----------
  if (rightDist < leftDist) {
    //Serial.println("Turning LEFT");
    turnLeft(5200);
    return;
  }

  if (rightDist > leftDist) {
    //Serial.println("Turning RIGHT");
    turnRight(5200);
    return;
  }

  // Equal distances less than 18cm (both detect obstacle)
  if (rightDist == leftDist && leftDist < 18) {
    Serial.println("Distances equal -> reversing");
    Reverse();
    currentState = SCANNING;
    return;
  }

  // Both 999 → wide open space
  Serial.println("Both sides clear, turning left.");
  turnLeft(5200);
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
    if (duration == 0) duration = 30000; // treat as far away

    durationSum += duration;
    delay(10);
  }

  long duration = durationSum / samples;
  int cm = duration * 0.034 / 2;

  if (cm < 2 || cm > 80) return 999;
  return cm;
}
