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
#define ENC1_A 18
#define ENC1_B 40
#define ENC2_A 19
#define ENC2_B 44
#define ENC3_A 20
#define ENC3_B 28
#define ENC4_A 21
#define ENC4_B 22

volatile long encoderCount[4] = {0,0,0,0};

// ==========================
// Servo and Sensor Pins
// ==========================
#define SERVO_PIN 23
#define TRIG_PIN 35
#define ECHO_PIN 37
#define OBSTACLE_PIN 51  // VM330 output

Servo servoLook;

// ==========================
// Global Variables
// ==========================
volatile uint8_t obstacleDetected = 0;

byte maxDist = 200;
byte stopDist = 20;
unsigned long timeOut = maxDist * 58 * 2; // µs

int motorSpeed = 140;
int motorOffset = 10;

// ==========================
// Speed Control Variables
// ==========================
unsigned long lastSpeedCheck = 0;
const unsigned long speedCheckInterval = 100; // ms
long prevEncoderCount[4] = {0,0,0,0};
float wheelSpeed[4] = {0,0,0,0}; // ticks/sec

float targetSpeed = 500.0; // nominal encoder ticks per second
float Kp = 0.05;            // proportional gain (tune this)

// ==========================
// Navigation State Machine
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
// Timing Control for Turns
// ==========================
unsigned long motionStartTime = 0;
unsigned long motionDuration = 0;
bool motionActive = false;

// ==========================
// Encoder ISRs (Hardware Interrupts)
// ==========================
void encoder1A_ISR() {
  if (digitalRead(ENC1_B)) encoderCount[0]++;
  else encoderCount[0]--;
}
void encoder2A_ISR() {
  if (digitalRead(ENC2_B)) encoderCount[1]++;
  else encoderCount[1]--;
}
void encoder3A_ISR() {
  if (digitalRead(ENC3_B)) encoderCount[2]++;
  else encoderCount[2]--;
}
void encoder4A_ISR() {
  if (digitalRead(ENC4_B)) encoderCount[3]++;
  else encoderCount[3]--;
}

// ==========================
// Obstacle ISR (Software Interrupt)
// ==========================
void obstacleISR() {
  obstacleDetected = 1;
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

  // Attach hardware interrupts (ENCx_A)
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), encoder3A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), encoder4A_ISR, CHANGE);

  // Attach software interrupt for obstacle (PinChangeInt)
  pinMode(OBSTACLE_PIN, INPUT);
  PCintPort::attachInterrupt(OBSTACLE_PIN, obstacleISR, RISING);

  delay(10);
  Serial.println("Robot initialized. Starting navigation...");
}

// ==========================
// Main Loop
// ==========================
void loop() {
  unsigned long now = millis();

  if (motionActive && now - motionStartTime >= motionDuration) {
    stopMove();
    motionActive = false;
    currentState = MOVING_FORWARD;
  }

  if (obstacleDetected) {
    obstacleDetected = 0;
    Serial.println("VM330 Obstacle detected via interrupt!");
    stopMove();
    currentState = OBSTACLE_DETECTED;
  }

  speedCorrection();

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
      break;
    case REACHED_TARGET:
      stopMove();
      break;
  }
}

// ==========================
// Motion + Logic Functions
// ==========================
void handleForwardMovement() {
  int frontDistance = getDistance();
  Serial.print("Distance: "); Serial.println(frontDistance);

  if (frontDistance < stopDist) {
    stopMove();
    currentState = OBSTACLE_DETECTED;
  } else {
    moveForward();
  }
}

void handleObstacleDetection() {
  stopMove();
  Serial.println("Handling Obstacle...");
  currentState = SCANNING;
}

void handleScanning() {
  Serial.println("Scanning left and right...");
  int rightDist = getDistance();
  int leftDist = getDistance();

  if (rightDist > leftDist) {
    turnRight(450);
  } else {
    turnLeft(450);
  }
}

// ==========================
// Motion Control
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

void turnLeft(unsigned long duration) {
  rightBack.run(FORWARD);
  rightFront.run(FORWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  motionStartTime = millis();
  motionDuration = duration;
  motionActive = true;
  currentState = TURNING;
}

void turnRight(unsigned long duration) {
  rightBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  motionStartTime = millis();
  motionDuration = duration;
  motionActive = true;
  currentState = TURNING;
}

// ==========================
// Speed Feedback + Correction
// ==========================
void updateWheelSpeeds() {
  unsigned long now = millis();
  if (now - lastSpeedCheck < speedCheckInterval) return;
  lastSpeedCheck = now;

  noInterrupts();
  long currentCount[4];
  for (int i = 0; i < 4; i++) currentCount[i] = encoderCount[i];
  interrupts();

  for (int i = 0; i < 4; i++) {
    wheelSpeed[i] = (currentCount[i] - prevEncoderCount[i]) * (1000.0 / speedCheckInterval);
    prevEncoderCount[i] = currentCount[i];
  }

  Serial.print("Speeds: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(wheelSpeed[i]); Serial.print(" ");
  }
  Serial.println();
}

void speedCorrection() {
  updateWheelSpeeds();

  float rightAvg = (wheelSpeed[0] + wheelSpeed[1]) / 2.0;
  float leftAvg  = (wheelSpeed[2] + wheelSpeed[3]) / 2.0;
  float error = leftAvg - rightAvg;

  int rightAdj = motorSpeed + Kp * error;
  int leftAdj  = motorSpeed - Kp * error;

  rightAdj = constrain(rightAdj, 0, 255);
  leftAdj  = constrain(leftAdj, 0, 255);

  rightFront.setSpeed(rightAdj);
  rightBack.setSpeed(rightAdj);
  leftFront.setSpeed(leftAdj);
  leftBack.setSpeed(leftAdj);
}

// ==========================
// Ultrasonic Utility
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
