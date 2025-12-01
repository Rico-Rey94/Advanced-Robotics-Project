#include <AFMotor.h>
#include <Servo.h>

// ============================================================
// MOTORS
// ============================================================
// Assuming the motor mapping is:
// 4: Right Front (RF)
// 3: Right Back (RB)
// 2: Left Back (LB)
// 1: Left Front (LF)
AF_DCMotor rightFront(4);
AF_DCMotor rightBack(3);
AF_DCMotor leftBack(2);
AF_DCMotor leftFront(2);

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
// Assuming mapping:
// 0: ENC1_A/B (Right Front) - Used for tracking 'right' side
// 1: ENC2_A/B (Right Back)
// 2: ENC3_A/B (Left Back) - Used for tracking 'left' side
// 3: ENC4_A/B (Left Front)

#define ENC1_A 18
#define ENC1_B 40
#define ENC2_A 19
#define ENC2_B 44
#define ENC3_A 20
#define ENC3_B 28
#define ENC4_A 21
#define ENC4_B 22

volatile long encoderCount[4] = {0,0,0,0};

// Encoder ISRs: Using 2X counting (CHANGE on A, reading B for direction)
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

// *** CALCULATED ENCODER TURNING VALUE ***
// Calculated based on:
// Wheel Diameter = 72mm (Circumference: 226.2mm)
// Wheel-to-Wheel Distance = 9in (228.6mm)
// JGA25 Motor Gear Ratio = 1:34 (Assumed)
// Encoder Resolution = 11 PPR (2X counting used: 748 Ticks/Wheel Rev)
// Result for 90-degree pivot turn: ~595 Ticks
const long TURN_90_TICKS = 595; // <-- **CORRECTED VALUE**

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
  Serial.println("Starting Setup...");

  servoLook.attach(SERVO_PIN);
  servoLook.write(90);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set up encoder pins with internal pull-up resistors
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP);
  pinMode(ENC4_B, INPUT_PULLUP);

  // Attach interrupts to the A channels on both edges (CHANGE)
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), encoder3A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), encoder4A_ISR, CHANGE);

  stopMove();
  Serial.println("Ready. Calculated TURN_90_TICKS: 595");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {

  // Exit turn when encoder count reached
  if (currentState == TURNING) {
    if (checkTurnComplete()) {
      Serial.println("Turn complete. Stopping motors.");
      stopMove();
      currentState = MOVING_FORWARD;
    }
  }

  switch (currentState) {

    case MOVING_FORWARD: {
      moveForward();
      int front = getDistance();
      if (front > 0 && front < 18) {
        Serial.print("Object detected: ");
        Serial.print(front);
        Serial.println(" cm.");
        stopMove();
        currentState = OBSTACLE_DETECTED;
      }
      break;
    }

    case OBSTACLE_DETECTED:
      // Reverse and then transition to scanning
      Reverse();
      currentState = SCANNING;
      break;

    case SCANNING:
      handleScanning();
      break;

    case TURNING:
      // Motors are running, waiting for checkTurnComplete()
      break;
  }
}

// ============================================================
// BASIC MOVEMENT
// ============================================================
void moveForward() {
  // Serial.println("moving foward"); // Commented out to reduce serial spam
  // Set speeds slightly differently for forward motion to account for potential imbalance
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
  // Serial.println("stop"); // Commented out to reduce serial spam
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void Reverse() {
  Serial.println("reverse");
  // Set speeds for reversing. Note: motors 1-4 are DC motors and speeds/offsets need tuning.
  rightFront.setSpeed(motorSpeed - reverseOffset);
  rightBack.setSpeed(motorSpeed - reverseOffset);
  leftFront.setSpeed(motorSpeed); // Keeping left at base speed for now
  leftBack.setSpeed(motorSpeed);

  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);

  delay(3500);
  stopMove();
}

// ============================================================
// ENCODER TURNING
// ============================================================

void turnLeft_Encoder() {
  turningLeft = true;
  Serial.print("Starting TURN LEFT. Target Ticks: ");
  Serial.println(TURN_90_TICKS);

  // Record starting ticks from the primary tracking wheels:
  // ENC3 (index 2) is Left Back, ENC1 (index 0) is Right Front
  turnStartLeftTicks  = encoderCount[2]; 
  turnStartRightTicks = encoderCount[0]; 

  // Left wheels backward (pivot)
  leftFront.setSpeed(motorSpeed + 40);
  leftBack.setSpeed(motorSpeed + 40);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  
  // Right wheels forward
  rightFront.setSpeed(motorSpeed + 40);
  rightBack.setSpeed(motorSpeed + 40);
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);

  currentState = TURNING;
}

void turnRight_Encoder() {
  turningLeft = false;
  Serial.print("Starting TURN RIGHT. Target Ticks: ");
  Serial.println(TURN_90_TICKS);

  // Record starting ticks
  turnStartLeftTicks  = encoderCount[2];
  turnStartRightTicks = encoderCount[0];

  // Right wheels backward (pivot)
  rightFront.setSpeed(motorSpeed + 40);
  rightBack.setSpeed(motorSpeed + 40);
  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  
  // Left wheels forward
  leftFront.setSpeed(motorSpeed + 40);
  leftBack.setSpeed(motorSpeed + 40);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);

  currentState = TURNING;
}

// ============================================================
// FIXED encoder turn completion
// ============================================================
bool checkTurnComplete() {
  // Use the absolute value of the change in ticks for the respective tracking wheels
  long leftDelta  = labs(encoderCount[2] - turnStartLeftTicks);
  long rightDelta = labs(encoderCount[0] - turnStartRightTicks);

  // Since it's a pivot turn, both wheels should travel roughly the same distance (number of ticks).
  // We use the maximum delta to ensure the turn finishes when the faster wheel completes its travel.
  long maxDelta = max(leftDelta, rightDelta);

  // Exit when the required number of encoder ticks has been reached
  return maxDelta >= TURN_90_TICKS;
}


// ============================================================
// SCANNING AND TURN DECISION
// ============================================================
void handleScanning() {
  Serial.println("Scanning for clearance...");
  stopMove();

  servoLook.write(90);
  delay(250);

  // LEFT side
  servoLook.write(160);
  delay(500);
  int leftDist = getDistance();
  Serial.print("Left Distance: ");
  Serial.print(leftDist);
  Serial.println(" cm");

  // RIGHT side
  servoLook.write(20);
  delay(500);
  int rightDist = getDistance();
  Serial.print("Right Distance: ");
  Serial.print(rightDist);
  Serial.println(" cm");

  servoLook.write(90);
  delay(200);

  // Decision Logic
  if (rightDist > leftDist && rightDist > 18) {
    turnRight_Encoder();
    return;
  }

  if (leftDist > rightDist && leftDist > 18) {
    turnLeft_Encoder();
    return;
  }
  
  // If distances are equal OR both are blocked (< 18cm)
  if (rightDist == leftDist) {
     if (leftDist < 18) {
      Serial.println("Blocked on both sides. Reversing...");
      Reverse(); // Reverse further if still stuck
      currentState = SCANNING;
      return;
    } else {
      // If equal and clear, default to a consistent turn (e.g., left)
      Serial.println("Equal clearance. Defaulting to left turn.");
      turnLeft_Encoder();
      return;
    }
  }

  // Fallback to a turn if only one side is clear but the logic above didn't catch it
  // This case should not be hit if the logic is robust, but for safety:
  turnLeft_Encoder(); // Default turn
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

    long duration = pulseIn(ECHO_PIN, HIGH, 35000);
    // If pulseIn times out (0), assume a large distance (far away)
    if (duration == 0) duration = 30000; 

    durationSum += duration;
    delay(10);
  }

  long duration = durationSum / samples;
  // Convert duration to distance (cm) using speed of sound (0.034 cm/µs)
  int cm = duration * 0.034 / 2;

  // Filter out noise/bad readings
  if (cm < 2 || cm > 80) return 999;
  return cm;
}
