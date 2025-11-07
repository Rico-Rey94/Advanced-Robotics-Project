// Arduino Libraries
#include <AFMotor.h>
#include <PinChangeInt.h>
#include <Servo.h>
// [Image of Arduino Mega with Motor Shield and Sensors]

// ==========================
// Motor Objects and Speed Setup
// ==========================
AF_DCMotor rightFront(1);
AF_DCMotor rightBack(2);
AF_DCMotor leftBack(3);
AF_DCMotor leftFront(4);

// Motor control parameters
const int MOTOR_SPEED = 140;
const int MOTOR_OFFSET = 10; // For differential steering correction

// ==========================
// Encoder Pins (Using External Interrupt Pins on Mega where available)
// ==========================
#define ENC1_A 18 // D18 (External Interrupt 5)
#define ENC2_A 19 // D19 (External Interrupt 4)
#define ENC3_A 20 // D20 (External Interrupt 3)
#define ENC4_A 21 // D21 (External Interrupt 2)

// B-Channel pins used for direction check inside ISRs
#define ENC1_B 40
#define ENC2_B 50
#define ENC3_B 28
#define ENC4_B 22

// Shared variable for encoder counts (must be volatile)
volatile long encoderCount[4] = {0,0,0,0};

// ==========================
// Servo and Sensor Pins
// ==========================
#define SERVO_PIN 9
#define TRIG_PIN 35 // Ultrasonic Trigger Pin (MUST be OUTPUT)
#define ECHO_PIN 37 // Ultrasonic Echo Pin (MUST be INPUT)
#define OBSTACLE_PIN 47 // VM330 output

Servo servoLook;

// ==========================
// Global Variables
// ==========================
volatile bool obstacleDetected = false;

const int MAX_DIST_CM = 200;
const int STOP_DIST_CM = 20; // Stop robot if obstacle is closer than 20cm

// Time calculation: Distance (cm) * 2 / Speed of Sound (0.0343 cm/µs)
// Max timeout is 200cm * 2 / 0.0343 cm/µs ≈ 11661 µs
const unsigned long ULTRASONIC_TIMEOUT = 12000UL;

// Encoder/Odometry Constants
const float WHEEL_DIAMETER_INCH = 3.0;
const float WHEEL_CIRCUMFERENCE_INCH = PI * WHEEL_DIAMETER_INCH;
const int TICKS_PER_REVOLUTION = 360; // Assuming 360 CPR

long previousEncoderPositions[4] = {0,0,0,0};
float totalDistanceTraveled = 0.0; // Total distance in inches

// Servo Scan Angles
const int CENTER_ANGLE = 90;
const int RIGHT_ANGLE = 45;
const int LEFT_ANGLE = 135;

// ==========================
// State Machine & Timing Control
// ==========================
enum NavigationState {
  MOVING_FORWARD,
  OBSTACLE_FOUND,
  SCANNING,
  TURNING_LEFT,
  TURNING_RIGHT,
  REACHED_TARGET,
  IDLE
};
NavigationState currentState = MOVING_FORWARD;

unsigned long motionStartTime = 0;
unsigned long motionDuration = 0;
bool motionActive = false;

// ==========================
// Encoder ISRs (A channel only - CHANGE)
// ==========================
void encoder1A_ISR() {
  // Check B channel state to determine direction
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

// VM330 Obstacle ISR (Falling edge indicates detection)
void obstacleISR() {
  obstacleDetected = true;
}

// ==========================
// Setup
// ==========================
void setup() {
  Serial.begin(9600);

  // --- Motor Initialization ---
  rightBack.setSpeed(MOTOR_SPEED);
  rightFront.setSpeed(MOTOR_SPEED);
  leftFront.setSpeed(MOTOR_SPEED + MOTOR_OFFSET); // Apply offset
  leftBack.setSpeed(MOTOR_SPEED + MOTOR_OFFSET);  // Apply offset
  stopMove();

  // --- Servo Initialization ---
  servoLook.attach(SERVO_PIN);
  servoLook.write(CENTER_ANGLE);

  // --- Ultrasonic Pin Setup ---
  pinMode(TRIG_PIN, INPUT);
  pinMode(ECHO_PIN, OUTPUT);

  // --- Encoder Pin Modes ---
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP);
  pinMode(ENC4_B, INPUT_PULLUP);

  // --- Attach External Interrupts (for A channels) ---
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), encoder3A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), encoder4A_ISR, CHANGE);

  // --- Obstacle Pin Change Interrupt ---
  pinMode(OBSTACLE_PIN, INPUT_PULLUP);
  PCintPort::attachInterrupt(OBSTACLE_PIN, obstacleISR, FALLING);

  Serial.println("Robot initialized. Starting navigation...");
}

// ==========================
// Main Loop
// ==========================
void loop() {
  unsigned long now = millis();

  // 1. --- Handle Timed Motion (Non-Blocking) ---
  if (motionActive && now - motionStartTime >= motionDuration) {
    stopMove();
    motionActive = false;
    // After a turn, immediately resume moving forward
    if (currentState == TURNING_LEFT || currentState == TURNING_RIGHT) {
      currentState = MOVING_FORWARD;
      Serial.println("Turn complete. Resuming forward movement.");
    }
  }

  // 2. --- Handle High-Priority Obstacle Interrupt ---
  if (obstacleDetected) {
    obstacleDetected = false; // Acknowledge and reset flag
    stopMove();
    Serial.println("VM330 Obstacle detected via interrupt! Stopping.");
    // Transition to the detection state
    currentState = OBSTACLE_FOUND;
  }

  // 3. --- Odometry Update (Runs periodically) ---
  // The original code had a fixed 100ms interval for distance updates, which is reasonable.
  static unsigned long lastDistanceUpdate = 0;
  const unsigned long updateInterval = 100; // ms

  if (now - lastDistanceUpdate >= updateInterval) {
    updateDistanceTraveled();
    // Check if target reached (Target set to 100 inches in the original code)
    if (totalDistanceTraveled >= 100.0) {
      currentState = REACHED_TARGET;
    }
    lastDistanceUpdate = now;
  }

  // 4. --- State Machine Execution ---
  switch (currentState) {
    case MOVING_FORWARD:
      handleForwardMovement();
      break;
    case OBSTACLE_FOUND:
      // Immediately transition to scanning (stop is handled by interrupt)
      currentState = SCANNING;
      Serial.println("Starting obstacle avoidance routine.");
      break;
    case SCANNING:
      handleScanning();
      break;
    case TURNING_LEFT:
    case TURNING_RIGHT:
      // Motion is active, wait for the timer to expire
      break;
    case REACHED_TARGET:
      stopMove();
      Serial.println("Target distance reached! Stopping.");
      currentState = IDLE;
      break;
    case IDLE:
      // Do nothing, wait for reset or command
      break;
  }
}

// ==========================
// Navigation Logic
// ==========================

// Function to get ultrasonic distance in cm
int getDistance() {
  // Ensure the trigger pin is low initially
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Send a 10µs pulse to trigger
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the pulse duration on the echo pin
  unsigned long pulseTime = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);

  // If pulseTime is 0, the object is too far (or timeout occurred)
  if (pulseTime == 0) return MAX_DIST_CM;

  // CRITICAL FIX: Ensure floating point math is used for the division.
  // Distance (cm) = (time in µs * speed of sound in cm/µs) / 2
  // Speed of sound ≈ 0.0343 cm/µs
  float distanceCM = (float)pulseTime * 0.0343 / 2.0;

  // Cap the distance at the defined maximum
  if (distanceCM > MAX_DIST_CM) return MAX_DIST_CM;

  return (int)distanceCM;
}

void handleForwardMovement() {
  // Always look straight when moving forward
  servoLook.write(CENTER_ANGLE);

  // Non-blocking distance check
  int frontDistance = getDistance();
  Serial.print("Total Dist: "); Serial.print(totalDistanceTraveled);
  Serial.print(" in. | Front Dist: "); Serial.print(frontDistance); Serial.println(" cm");

  if (frontDistance < STOP_DIST_CM) {
    stopMove();
    currentState = OBSTACLE_FOUND;
  } else {
    moveForward();
  }
}

void handleScanning() {
  Serial.println("Scanning left and right...");
  
  // Look right and get distance
  servoLook.write(RIGHT_ANGLE);
  delay(300); // Small delay for servo to move
  int rightDist = getDistance();

  // Look left and get distance
  servoLook.write(LEFT_ANGLE);
  delay(300); // Small delay for servo to move
  int leftDist = getDistance();

  // Center servo before starting the turn
  servoLook.write(CENTER_ANGLE);
  delay(100);

  // Simple avoidance strategy: turn toward the side with more space
  const unsigned long TURN_DURATION_MS = 600; // Time in milliseconds to turn

  if (rightDist > leftDist) {
    Serial.print("Right side clear ("); Serial.print(rightDist); Serial.println(" cm). Turning right.");
    turnRight(TURN_DURATION_MS);
  } else if (leftDist > rightDist) {
    Serial.print("Left side clear ("); Serial.print(leftDist); Serial.println(" cm). Turning left.");
    turnLeft(TURN_DURATION_MS);
  } else {
    // If distances are equal or very small, turn right by default
    Serial.println("Sides equal or blocked. Turning right by default.");
    turnRight(TURN_DURATION_MS);
  }

  // State will be TURNING_LEFT/RIGHT, and the timer will handle transition back to MOVING_FORWARD
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
  rightBack.run(FORWARD);  // Right side forward
  rightFront.run(FORWARD);
  leftFront.run(BACKWARD); // Left side backward
  leftBack.run(BACKWARD);
  motionStartTime = millis();
  motionDuration = duration;
  motionActive = true;
  currentState = TURNING_LEFT;
}

void turnRight(unsigned long duration) {
  rightBack.run(BACKWARD); // Right side backward
  rightFront.run(BACKWARD);
  leftFront.run(FORWARD);  // Left side forward
  leftBack.run(FORWARD);
  motionStartTime = millis();
  motionDuration = duration;
  motionActive = true;
  currentState = TURNING_RIGHT;
}

// ==========================
// Odometry (Distance Tracking)
// ==========================
void updateDistanceTraveled() {
  long currentEncoderPositions[4];

  // Protect access to volatile variables from interrupts
  noInterrupts();
  for (int i=0; i<4; i++) {
    currentEncoderPositions[i] = encoderCount[i];
  }
  interrupts();

  long averageEncoderTicks = 0;
  for (int i=0; i<4; i++){
    // Calculate the movement since the last update
    long deltaTicks = abs(currentEncoderPositions[i] - previousEncoderPositions[i]);
    averageEncoderTicks += deltaTicks;
    previousEncoderPositions[i] = currentEncoderPositions[i];
  }

  // Get the average movement across all four wheels
  averageEncoderTicks /= 4;

  // Convert average ticks to distance traveled (in inches)
  float revolutionsTraveled = (float)averageEncoderTicks / (float)TICKS_PER_REVOLUTION;
  float distanceIncrement = revolutionsTraveled * WHEEL_CIRCUMFERENCE_INCH;

  // Add increment to the total distance
  totalDistanceTraveled += distanceIncrement;
}