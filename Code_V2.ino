#include <AFMotor.h>
#include <PinChangeInt.h>
#include <ServoTimer2.h>

// ==========================
// Motor Objects
// ==========================
AF_DCMotor rightFront(1);
AF_DCMotor rightBack(2);
AF_DCMotor leftBack(3);
AF_DCMotor leftFront(4);

// ==========================
// Encoder Objects
// ==========================
#define ENC1_A 42
#define ENC1_B 40
#define ENC2_A 52
#define ENC2_B 50
#define ENC3_A 30
#define ENC3_B 28
#define ENC4_A 24
#define ENC4_B 22

volatile long encoderCount[4] = {0,0,0,0};

// ==========================
// Servo and Sensor Pins
// ==========================
#define SERVO_PIN 23
#define TRIG_PIN 35
#define ECHO_PIN 37
#define OBSTACLE_PIN 47  // VM330 output

ServoTimer2 servoLook;

// ==========================
// Global Variables
// ==========================
volatile uint8_t obstacleDetected = 0;

byte maxDist = 200;
byte stopDist = 20;
unsigned long timeOut = maxDist * 58 * 2; // µs

byte motorSpeed = 140;
int motorOffset = 10;

const float wheelDiameter = 3.0;
const float wheelCircumference = PI * wheelDiameter;
const int ticksPerRevolution = 360;

long previousEncoderPositions[4] = {0,0,0,0};
float distanceTraveled = 0;
float totalDistance = 0;

int rightAngle = 45;
int leftAngle = 135;

unsigned long lastDistanceUpdate = 0;
const unsigned long updateInterval = 100; // ms

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
// Encoder ISRs (A channel only)
// ==========================
void encoder1A_ISR() {
  if (digitalRead(ENC1_B) == HIGH) encoderCount[0]++;
  else encoderCount[0]--;
}

void encoder2A_ISR() {
  if (digitalRead(ENC2_B) == HIGH) encoderCount[1]++;
  else encoderCount[1]--;
}

void encoder3A_ISR() {
  if (digitalRead(ENC3_B) == HIGH) encoderCount[2]++;
  else encoderCount[2]--;
}

void encoder4A_ISR() {
  if (digitalRead(ENC4_B) == HIGH) encoderCount[3]++;
  else encoderCount[3]--;
}

// Obstacle ISR
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

  // Initialize servo
  //servoLook.attach(SERVO_PIN);
  //servoLook.write(90);

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

  // Attach interrupts (A channels only)
  PCintPort::attachInterrupt(ENC1_A, encoder1A_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC2_A, encoder2A_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC3_A, encoder3A_ISR, CHANGE);
  PCintPort::attachInterrupt(ENC4_A, encoder4A_ISR, CHANGE);

  // Obstacle interrupt
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

  // Handle timed turns
  if (motionActive && now - motionStartTime >= motionDuration) {
    stopMove();
    motionActive = false;
    currentState = MOVING_FORWARD;
  }

  // Periodic distance update
  if (now - lastDistanceUpdate >= updateInterval) {
    updateDistanceTraveled();
    lastDistanceUpdate = now;
  }

  // Obstacle interrupt
  if (obstacleDetected) {
    obstacleDetected = false;
    Serial.println("VM330 Obstacle detected via interrupt!");

    stopMove();
    //servoLook.write(45);
    //delay(10);
    //servoLook.write(90);
    //delay(10);

    currentState = OBSTACLE_DETECTED;
  }

  // Stop if reached target
  if (totalDistance >= 100.0) {
    currentState = REACHED_TARGET;
  }

  // --- State Machine ---
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
      // Turning handled by motion timer above
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
  //servoLook.write(90);
  //delay(10);

  int frontDistance = getDistance();
  Serial.print("Distance: "); Serial.println(frontDistance);

  if (frontDistance < stopDist) {
    stopMove();
    currentState = OBSTACLE_DETECTED;
  } else {
    moveForward();
    updateDistanceTraveled();
  }
}

void handleObstacleDetection() {
  stopMove();
  Serial.println("Handling Obstacle...");
  //delay(10);
  currentState = SCANNING;
}

void handleScanning() {
  Serial.println("Scanning left and right...");
  //servoLook.write(rightAngle);
  //delay(10);
  int rightDist = getDistance();

  //servoLook.write(leftAngle);
  //delay(10);
  int leftDist = getDistance();

  //servoLook.write(90);
  //delay(10);

  if (rightDist > leftDist) {
    turnRight(450);
  } else {
    turnLeft(450);
  }

  //currentState = MOVING_FORWARD; // motion timer handles it
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
// Utility Functions
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

void updateDistanceTraveled() {
  long currentEncoderPositions[4];
  noInterrupts();
  for (int i=0;i<4;i++) currentEncoderPositions[i]=encoderCount[i];
  interrupts();

  long averageEncoderTicks=0;
  for(int i=0;i<4;i++){
    averageEncoderTicks += abs(currentEncoderPositions[i]-previousEncoderPositions[i]);
    previousEncoderPositions[i] = currentEncoderPositions[i];
  }
  averageEncoderTicks /=4;

  float revolutionsTraveled = averageEncoderTicks / (float)ticksPerRevolution;
  float distanceIncrement = revolutionsTraveled * wheelCircumference;

  distanceTraveled += distanceIncrement;
  totalDistance += distanceIncrement;
}
