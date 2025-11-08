#include <AFMotor.h>
#include <Servo.h>

// ==========================
// Motor Objects (Adafruit L293D Shield)
// ==========================
AF_DCMotor rightFront(1); // M1
AF_DCMotor rightBack(2);  // M2
AF_DCMotor leftBack(3);   // M3
AF_DCMotor leftFront(4);  // M4

// ==========================
// Encoder Pins (Hardware Interrupts)
// ==========================
#define ENC1_A 18 // INT5
#define ENC1_B 40
#define ENC2_A 19 // INT4
#define ENC2_B 44
#define ENC3_A 20 // INT3
#define ENC3_B 28
#define ENC4_A 21 // INT2
#define ENC4_B 22

volatile long encoderCount[4] = {0, 0, 0, 0};

// ==========================
// Servo + Ultrasonic + Obstacle Sensor
// ==========================
#define SERVO_PIN 23 //servo signal
#define TRIG_PIN 35
#define ECHO_PIN 37
#define OBSTACLE_PIN 51 // VM330 output, polled in loop

Servo servoLook;
volatile bool obstacleDetected = false;

// ==========================
// Motion Variables
// ==========================
int motorSpeed = 140;
int motorOffset = 10;

// ==========================
// Speed Control Variables
// ==========================
unsigned long lastSpeedCheck = 0;
const unsigned long speedCheckInterval = 100; // ms
long prevEncoderCount[4] = {0, 0, 0, 0};
float wheelSpeed[4] = {0, 0, 0, 0}; // ticks/sec

float targetSpeed = 500.0; // nominal ticks/sec
float Kp = 0.05;           // proportional gain

// ==========================
// Navigation State
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
// Timing for Turns
// ==========================
unsigned long motionStartTime = 0;
unsigned long motionDuration = 0;
bool motionActive = false;

// ==========================
// Encoder ISRs (A channel only)
// ==========================
void encoder1A_ISR() {
  bool b = digitalRead(ENC1_B);
  encoderCount[0] += b ? 1 : -1;
}
void encoder2A_ISR() {
  bool b = digitalRead(ENC2_B);
  encoderCount[1] += b ? 1 : -1;
}
void encoder3A_ISR() {
  bool b = digitalRead(ENC3_B);
  encoderCount[2] += b ? 1 : -1;
}
void encoder4A_ISR() {
  bool b = digitalRead(ENC4_B);
  encoderCount[3] += b ? 1 : -1;
}

// ==========================
// Obstacle ISR (Pin 51)
// ==========================

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(9600);
  Serial.println("Initializing robot...");

  // Servo setup
  servoLook.attach(SERVO_PIN);
  servoLook.write(90); // Center position

  // Motor setup
  rightBack.setSpeed(motorSpeed);
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed + motorOffset);
  leftBack.setSpeed(motorSpeed + motorOffset);
  stopMove();

  // Ultrasonic sensor
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

  // Attach hardware interrupts (Mega: 2–5, pins 21–18)
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), encoder3A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), encoder4A_ISR, CHANGE);

  // Obstacle sensor
  pinMode(OBSTACLE_PIN, INPUT);
  
  delay(10);
  Serial.println("Robot initialized. Starting navigation...");
}

// ==========================
// Main Loop
// ==========================
void loop() {
  unsigned long now = millis();

if (digitalRead(OBSTACLE_PIN) == HIGH) {
  obstacleDetected = true;
}

  if (motionActive && now - motionStartTime >= motionDuration) {
    stopMove();
    motionActive = false;
    currentState = MOVING_FORWARD;
  }

  if (obstacleDetected) {
    obstacleDetected = false;
    Serial.println("Obstacle detected!");
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
// Motion + Logic
// ==========================
void handleForwardMovement() {
  int frontDistance = getDistance();
  if (frontDistance < 20) {
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
  Serial.println("Scanning surroundings...");
  Reverse();

  int rightDist = 0, leftDist = 0;

  // Scan right
  for (int angle = 60; angle <= 120; angle += 15) {
    servoLook.write(angle);
    delay(250);
    rightDist = getDistance();
  }

  // Scan left
  for (int angle = 120; angle >= 60; angle -= 15) {
    servoLook.write(angle);
    delay(250);
    leftDist = getDistance();
  }
  Serial.print("left distance: ", leftDist, " right distance: ", rightDist);
  servoLook.write(90);

  if (rightDist > leftDist) {
    turnRight(450);
    moveForward();
  } else {
    turnLeft(450);
    moveForward();
  }
}


// ==========================
// Motor Control
// ==========================
void moveForward() {
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
  Serial.println("Reversing...");
  noInterrupts();
  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  interrupts();
  delay(600);
  stopMove();
}

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
// Ultrasonic Distance
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
    unsigned long pulseTime = pulseIn(ECHO_PIN, HIGH, 15000);
    if (pulseTime > 0) {
      sum += pulseTime;
      valid++;
    }
  }
  if (valid == 0) return 200;
  int distance = (int)((sum / valid) * 0.034 / 2.0);
  return distance;
}
