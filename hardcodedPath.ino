#include <AFMotor.h>
#include <Arduino.h> 

// ============================================================
// CONSTANTS & SETTINGS
// ============================================================
int motorSpeed = 150;
int motorOffset = 6;
int turnSpeed = motorSpeed + 40; 

const long TURN_90_TICKS = 595; 


// ============================================================
// STATE MACHINE FOR PATH FOLLOWING
// ============================================================
enum PathState { 
  READY, 
  STEP_1_MOVE, STEP_2_TURN, STEP_3_MOVE, STEP_4_TURN, STEP_5_MOVE, STEP_6_TURN, 
  STEP_7_MOVE, STEP_8_TURN, STEP_9_MOVE, STEP_10_TURN, STEP_11_MOVE, STEP_12_TURN,
  STEP_13_MOVE, STEP_14_TURN, STEP_15_MOVE, STEP_16_TURN, STEP_17_MOVE, STEP_18_TURN,
  STEP_19_MOVE, 
  FINISHED, 
  MOVING, 
  TURNING_LEFT, 
  TURNING_RIGHT 
};

// State tracking variables
enum PathState currentPathState = READY;
enum PathState nextPathState = FINISHED; 

// TIME-BASED MOVEMENT VARIABLES
unsigned long moveStartTime = 0;
long moveDurationMs = 0; 

// ENCODER-BASED TURNING VARIABLES
long turnStartLeftTicks = 0;
long turnStartRightTicks = 0;

// ENCODERS 
#define ENC1_A 18 // Left Front 
#define ENC1_B 40
#define ENC2_A 19 // Left Rear
#define ENC2_B 44
#define ENC3_A 20 // Right Rear 
#define ENC3_B 28
#define ENC4_A 21 // Right Front
#define ENC4_B 22

volatile long encoderCount[4] = {0,0,0,0}; 


// ============================================================
// FIXED encoder turn completion 
// ============================================================
bool checkTurnComplete() {
  // Note: The primary tracking encoders are 0 (LF) and 2 (RR)
  long leftDelta  = labs(encoderCount[0] - turnStartLeftTicks);
  long rightDelta = labs(encoderCount[2] - turnStartRightTicks);
  long maxDelta = max(leftDelta, rightDelta);

  return maxDelta >= TURN_90_TICKS;
}

bool checkTimeComplete() {
    // Check if the current time minus the start time is greater than or equal to the desired duration.
    return (millis() - moveStartTime) >= moveDurationMs;
}


// ============================================================
// MOTORS (CRITICAL FIX: BOTH LEFT MOTORS MAPPED TO M2)
// ============================================================
// 4: Right Front (RF)
AF_DCMotor rightFront(4);
// 3: Right Back (RB)
AF_DCMotor rightBack(3);
// 2: Left Front (LF) <-- M2
AF_DCMotor leftFront(2);
// 1: Left Back (LB)  <-- M2 (Shares port with Left Front)
AF_DCMotor leftBack(2);


// ============================================================
// ENCODER ISRS
// ============================================================

// Encoder ISRs: Using 2X counting
void encoder1A_ISR() { encoderCount[0] += digitalRead(ENC1_B) ? 1 : -1; }
void encoder2A_ISR() { encoderCount[1] += digitalRead(ENC2_B) ? 1 : -1; }

// RIGHT SIDE - Reversed signs for correction
void encoder3A_ISR() { encoderCount[2] += digitalRead(ENC3_B) ? -1 : 1; } 
void encoder4A_ISR() { encoderCount[3] += digitalRead(ENC4_B) ? -1 : 1; } 


// ============================================================
// BASIC MOVEMENT UTILITIES
// ============================================================
void stopMove() {
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void resetEncoders() {
  encoderCount[0] = 0; 
  encoderCount[2] = 0; 
}

/**
 * **NON-BLOCKING moveForward()**
 * Starts the forward movement, sets the duration, and immediately transitions 
 * to the MOVING state so that the loop() can continue.
 */
void moveForward(long durationMs, PathState nextState) {
    // CRITICAL: Only run if we are transitioning from a step state, not MOVING.
    if (currentPathState == MOVING) {
        return;
    }

    Serial.print("Starting Move Forward for: ");
    Serial.print(durationMs);
    Serial.println("ms (Non-Blocking)");

    // 1. Configure the motors
    leftFront.setSpeed(motorSpeed);
    leftBack.setSpeed(motorSpeed);
    rightFront.setSpeed(motorSpeed + motorOffset);
    rightBack.setSpeed(motorSpeed + motorOffset);

    leftFront.run(FORWARD); // Activates M2 (Left Front and Back)
    leftBack.run(FORWARD);  // Redundantly activates M2 again (ensures all code paths hit the working motor)
    rightFront.run(FORWARD);
    rightBack.run(FORWARD);

    // 2. Store timing and state data for the state machine
    nextPathState = nextState;
    moveDurationMs = durationMs; // <--- Sets the duration
    moveStartTime = millis();     // <--- Sets the start time

    // 3. Transition to the MOVING state
    currentPathState = MOVING;
}


// ============================================================
// ENCODER TURNING 
// ============================================================

void turnLeft_Encoder(PathState nextState) {
  Serial.print("Starting TURN LEFT. Target Ticks: ");
  Serial.println(TURN_90_TICKS);

  // Store the next state
  nextPathState = nextState;

  // Record starting ticks from the primary tracking wheels:
  turnStartLeftTicks  = encoderCount[0]; 
  turnStartRightTicks = encoderCount[2]; 

  // Left wheels backward (pivot) - Both will run M2 backward
  leftFront.setSpeed(turnSpeed);
  leftBack.setSpeed(turnSpeed);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  
  // Right wheels forward
  rightFront.setSpeed(turnSpeed);
  rightBack.setSpeed(turnSpeed);
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);

  currentPathState = TURNING_LEFT;
}

void turnRight_Encoder(PathState nextState) {
  Serial.print("Starting TURN RIGHT. Target Ticks: ");
  Serial.println(TURN_90_TICKS);

  // Store the next state
  nextPathState = nextState;

  // Record starting ticks
  turnStartLeftTicks  = encoderCount[0]; 
  turnStartRightTicks = encoderCount[2]; 

  // Right wheels backward (pivot)
  rightFront.setSpeed(turnSpeed);
  rightBack.setSpeed(turnSpeed);
  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  
  // Left wheels forward - Both will run M2 forward
  leftFront.setSpeed(turnSpeed);
  leftBack.setSpeed(turnSpeed);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);

  currentPathState = TURNING_RIGHT;
}


// ============================================================
// PATH HANDLER (TIME DIVIDED BY 100)
// ============================================================
void handlePath() {
  switch (currentPathState) {
    case READY:
      resetEncoders();
      currentPathState = STEP_1_MOVE;
      break;


    case STEP_1_MOVE:
      moveForward(8, STEP_2_TURN); 
      break;
    case STEP_2_TURN:
      turnRight_Encoder(STEP_3_MOVE); 
      break;

    case STEP_3_MOVE:
      moveForward(24, STEP_4_TURN); 
      break;
    case STEP_4_TURN:
      turnLeft_Encoder(STEP_5_MOVE); 
      break;

    case STEP_5_MOVE:
      moveForward(16, STEP_6_TURN); 
      break;
    case STEP_6_TURN:
      turnLeft_Encoder(STEP_7_MOVE); 
      break;

    case STEP_7_MOVE:
      moveForward(24, STEP_8_TURN); 
      break;
    case STEP_8_TURN:
      turnRight_Encoder(STEP_9_MOVE); 
      break;

    case STEP_9_MOVE:
      moveForward(24, STEP_10_TURN); 
      break;
    case STEP_10_TURN:
      turnRight_Encoder(STEP_11_MOVE); 
      break;

    case STEP_11_MOVE:
      moveForward(16, STEP_12_TURN); 
      break;
    case STEP_12_TURN:
      turnRight_Encoder(STEP_13_MOVE);
      break;

    case STEP_13_MOVE:
      moveForward(8, STEP_14_TURN); 
      break;
    case STEP_14_TURN:
      turnLeft_Encoder(STEP_15_MOVE);
      break;

    case STEP_15_MOVE:
      moveForward(16, STEP_16_TURN);
      break;
    case STEP_16_TURN:
      turnRight_Encoder(STEP_17_MOVE); 
      break;

    case STEP_17_MOVE:
      moveForward(16, STEP_18_TURN);
      break;
    case STEP_18_TURN:
      turnLeft_Encoder(STEP_19_MOVE);
      break;

    case STEP_19_MOVE:
      moveForward(20, FINISHED);
      break;

    case FINISHED:
      stopMove();
      Serial.println("Path sequence FINISHED!");
      break;

    // --- HANDLERS FOR RUNNING STATES ---

    case MOVING:
      if (checkTimeComplete()) { 
        stopMove();
        Serial.println("Move (Time) complete. Advancing state.");
        currentPathState = nextPathState; 
      }
      break;

    case TURNING_LEFT:
    case TURNING_RIGHT:
      if (checkTurnComplete()) {
        stopMove();
        Serial.println("Turn (Encoder) complete. Advancing state.");
        currentPathState = nextPathState; 
        resetEncoders(); 
      }
      break;
  }
}

// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Starting Time-Based Path Follower Setup...");

  // Set up encoder pins with internal pull-up resistors
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP);
  pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP);
  pinMode(ENC4_B, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), encoder3A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), encoder4A_ISR, CHANGE);

  stopMove();
  Serial.println("Path Follower Ready.");
}

void loop() {
  handlePath();
  delay(10); 
}
