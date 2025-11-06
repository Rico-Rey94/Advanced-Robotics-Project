#include <AFMotor.h>
#include <PinChangeInt.h>
#include <Servo.h>

// ==========================
// Motor Objects
// ==========================
AF_DCMotor rightFront(1);
AF_DCMotor rightBack(2);
AF_DCMotor leftBack(3);
AF_DCMotor leftFront(4);

// ==========================
// Encoder Pins
// ==========================
#define ENC1_A 42
#define ENC1_B 40
#define ENC2_A 52
#define ENC2_B 50
#define ENC3_A 30
#define ENC3_B 28
#define ENC4_A 24
#define ENC4_B 22

volatile long encoderCount[4] = {0, 0, 0, 0};

// ==========================
// Servo and Sensor Pins
// ==========================
#define SERVO_PIN 23
#define TRIG_PIN 35
#define ECHO_PIN 37
#define OBSTACLE_PIN A9   // VM330 output pin
#define ENABLE_PIN A10    // VM330 enable pin

Servo servoLook;

// ==========================
// Global Variables
// ==========================
volatile uint8_t obstacleDetected = 0;

byte maxDist = 200;
byte stopDist = 20;
unsigned long timeOut = maxDist * 58 * 2; // µs

byte motorSpeed = 140;
int motorOffset = 10;
int turnSpeed = 70;

// correction tuning
float correctionGain = 0.25;  // try between 0.1–0.4

int rightAngle = 45;
int leftAngle = 135;

// ==========================
// Navigation States
// ==========================
enum NavigationState {
  MOVING_FORWARD,
  OBSTACLE_DETECTED,
  SCANNING,
  TURNING,
  REACHED_TARGET
};

NavigationState currentState = MOVING_FORWARD;

// ==========================
// Encoder ISRs
// ==========================
void encoder1A_ISR() {
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) encoderCount[0]++;
  else encoderCount[0]--;
}
void encoder1B_ISR() {
  if (digitalRead(ENC1_A) != digitalRead(ENC1_B)) encoderCount[0]++;
  else encoderCount[0]--;
}
void encoder2A_ISR() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B)) encoderCount[1]++;
  else encoderCount[1]--;
}
void encoder2B_ISR() {
  if (digitalRead(ENC2_A) != digitalRead(ENC2_B)) encoderCount[1]++;
  else encoderCount[1]--;
}
void encoder3A_ISR() {
  if (digitalRead(ENC3_A) == digitalRead(ENC3_B)) encoderCount[2]++;
  else encoderCount[2]--;
}
void encoder3B_ISR() {
  if (digitalRead(ENC3_A) != digitalRead(ENC3_B)) encoderCount[2]++;
  else encoderCount[2]--;
}
void encoder4A_ISR() {
  if (digitalRead(ENC4_A) == digitalRead(ENC4_B)) encoderCount[3]++;
  else encoderCount[3]--;
}
void encoder4B_ISR() {
  if (digitalRead(ENC4_A) != digitalRead(ENC4_B)) encoderCount[3]++;
  else encoderCount[3]--;
}

// ==========================
// Interrupts
// ==========================
void obstacleISR() {
  obstacleDetected = true;
}

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(9600);

  // Initialize motors
  rightBack.setSpeed(motorSpeed);
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed + motorOffset);
  leftBack.setSpeed(motorSpeed + motorOffset);
  stopMove();

  // Servo
  servoLook.attach(SERVO_PIN);
  servoLook.write(90);

  // VM330
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);
  pinMode(OBSTACLE_PIN, INPUT);
  PCintPort::attachInterrupt(OBSTACLE_PIN, obstacleISR, RISING);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Encoder pins
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP);
  pinMode(ENC4_B, INPUT_PULLUP);

  // Attach encoder interrupts
  PCintPort::attachInterrupt(ENC1_A, encoder1A_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC1_B, encoder1B_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC2_A, encoder2A_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC2_B, encoder2B_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC3_A, encoder3A_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC3_B, encoder3B_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC4_A, encoder4A_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC4_B, encoder4B_ISR, CHANGE);

  Serial.println("Robot initialized. Encoder-based speed correction active.");
}

// ==========================
// Loop
// ==========================
void loop() {
  // Handle obstacle interrupt
  if (obstacleDetected) {
    obstacleDetected = false;
    Serial.println("VM330 Obstacle detected!");
    stopMove();
    servoLook.write(45);
    delay(200);
    servoLook.write(90);
    delay(200);
    currentState = OBSTACLE_DETECTED;
  }

  switch (currentState) {
    case MOVING_FORWARD:
      handleForwardMovement();
      break;
    case OBSTACLE_DETECTED:
      handleObstacleDetection();
      break;
    case SCANNING:
      handleScanning();
      break;
    case TURNING:
      handleTurning();
      break;
    case REACHED_TARGET:
      stopMove();
      break;
  }
}

// ==========================
// State Handlers
// ==========================
void handleForwardMovement() {
  servoLook.write(90);
  delay(10);
  int frontDistance = getDistance();
  Serial.print("Distance: "); Serial.println(frontDistance);

  if (frontDistance < stopDist) {
    stopMove();
    currentState = OBSTACLE_DETECTED;
  } else {
    moveForward();
    adjustMotorBalance(); // <-- encoder-based speed correction
  }
}

void handleObstacleDetection() {
  stopMove();
  Serial.println("Handling Obstacle...");
  delay(200);
  currentState = SCANNING;
}

void handleScanning() {
  Serial.println("Scanning left and right...");
  servoLook.write(rightAngle);
  delay(400);
  int rightDist = getDistance();

  servoLook.write(leftAngle);
  delay(400);
  int leftDist = getDistance();

  servoLook.write(90);
  delay(200);

  if (rightDist > leftDist) {
    turnRight(450);
  } else {
    turnLeft(450);
  }
  currentState = MOVING_FORWARD;
}

void handleTurning() {
  currentState = MOVING_FORWARD;
}

// ==========================
// Motor + Correction
// ==========================
void moveForward() {
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
}

void stopMove() {
  rightBack.run(RELEASE);
  rightFront.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void turnLeft(int duration) {
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  delay(duration);
  stopMove();
}

void turnRight(int duration) {
  rightBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  delay(duration);
  stopMove();
}

void adjustMotorBalance() {
  static long prevLeft = 0, prevRight = 0;

  noInterrupts();
  long leftNow = encoderCount[2] + encoderCount[3];
  long rightNow = encoderCount[0] + encoderCount[1];
  interrupts();

  long leftDelta = leftNow - prevLeft;
  long rightDelta = rightNow - prevRight;
  prevLeft = leftNow;
  prevRight = rightNow;

  long diff = leftDelta - rightDelta;

  int correction = diff * correctionGain; // proportional correction

  int leftSpeed = constrain(motorSpeed - correction, 0, 255);
  int rightSpeed = constrain(motorSpeed + correction, 0, 255);

  leftFront.setSpeed(leftSpeed + motorOffset);
  leftBack.setSpeed(leftSpeed + motorOffset);
  rightFront.setSpeed(rightSpeed);
  rightBack.setSpeed(rightSpeed);

  Serial.print("LΔ: "); Serial.print(leftDelta);
  Serial.print("  RΔ: "); Serial.print(rightDelta);
  Serial.print("  Corr: "); Serial.println(correction);
}

// ==========================
// Utilities
// ==========================
int getDistance() {
  long sum = 0;
  int valid = 0;
  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    unsigned long pulseTime = pulseIn(ECHO_PIN, HIGH, timeOut);
    if (pulseTime > 0) {
      sum += pulseTime;
      valid++;
    }
  }
  if (valid == 0) return maxDist;
  int distance = (sum / valid) * 0.034 / 2;
  return distance;
}
