#include <AFMotor.h>

// ============================================================
// MOTOR DEFINITIONS (4WD)
// ============================================================
AF_DCMotor leftFront(1);   // M1
AF_DCMotor leftBack(2);    // M2
AF_DCMotor rightRear(3);   // M3
AF_DCMotor rightFront(4);  // M4

// ============================================================
// CONSTANTS & SETTINGS
// ============================================================
int spd = 150;                  
const int STRAIGHT_SPEED = 150; 
// FIX: Boost for the slower right side motors to stop drift.
const int RIGHT_SIDE_BOOST = 30; 

// Left side speed correction for right turns
const int LEFT_TURN_CORRECTION = 26; 
// Right side speed correction for left turns
const int RIGHT_TURN_CORRECTION = 23;
// PIVOT TURN TIMING
const unsigned long TURN_DURATION_MS = 1600; 

// ============================================================
// DISTANCE TIMINGS (REDUCED BY HALF)
// ============================================================
// Original Value * (1/2)
const unsigned long DIST_1_FOOT_MS = 334; 


// ============================================================
// HELPER FUNCTIONS FOR MOTOR GROUPING (4WD)
// ============================================================
void setLeftSpeed(int s) {
  leftFront.setSpeed(s);
  leftBack.setSpeed(s);
}

void setRightSpeed(int s) {
  rightFront.setSpeed(s);
  rightRear.setSpeed(s);
}

void runLeftMotors(uint8_t dir) {
  leftFront.run(dir);
  leftBack.run(dir);
}

void runRightMotors(uint8_t dir) {
  rightFront.run(dir);
  rightRear.run(dir);
}

// ============================================================
// STOP MOVEMENT (Using RELEASE)
// ============================================================
void stopMove() {
  runLeftMotors(RELEASE); 
  runRightMotors(RELEASE); 
  delay(200);
}

// ============================================================
// FORWARD / BACKWARD (Using RIGHT_SIDE_BOOST)
// ============================================================
void moveForward(unsigned long t) {
  // Left side runs at base speed
  setLeftSpeed(STRAIGHT_SPEED);  
  // Right side runs at base speed PLUS boost for correction
  setRightSpeed(min(STRAIGHT_SPEED + RIGHT_SIDE_BOOST, 255));

  runLeftMotors(FORWARD);
  runRightMotors(FORWARD);
  t = t * DIST_1_FOOT_MS;
  delay(t);
  stopMove();
}

void moveBackward(unsigned long t) {
  // Left side runs at base speed

  setLeftSpeed(STRAIGHT_SPEED);
  // Right side runs at base speed PLUS boost for correction
  setRightSpeed(min(STRAIGHT_SPEED + RIGHT_SIDE_BOOST, 255));

  runLeftMotors(BACKWARD);
  runRightMotors(BACKWARD);

  delay(t);
  stopMove();
}

// ============================================================
// PIVOT TURNS
// ============================================================
void turnLeftPivot() {
  // Set speed for spin turn
  setRightSpeed(spd - RIGHT_TURN_CORRECTION);
  setLeftSpeed(spd); 

  // Run pivot (Left BACKWARD, Right FORWARD for spin turn)
  runRightMotors(FORWARD);
  runLeftMotors(BACKWARD); 
  
  delay(TURN_DURATION_MS);

  stopMove();
}

void turnRightPivot() {
  // Set speed for spin turn
  setRightSpeed(spd); 
    // FIXED: Left speed decreased by 26 for correction
  setLeftSpeed(spd - LEFT_TURN_CORRECTION); 
  
  // Run pivot (Left FORWARD, Right BACKWARD for spin turn)
  runLeftMotors(FORWARD);
  runRightMotors(BACKWARD); 
  
  delay(TURN_DURATION_MS); // 900ms

  stopMove();
}

// ============================================================
// PATH FOLLOWING SEQUENCE
// ============================================================
void followPath() {
  Serial.println("Starting Path...");

  moveForward(5.5); 
  turnLeftPivot();

  moveForward(3);
  turnLeftPivot();

  moveForward(4.5);
  turnRightPivot();

  moveForward(5.5); 
  turnRightPivot();

  moveForward(3.5); //5th fixed
  turnRightPivot();

  moveForward(3.4); 
  turnLeftPivot();

  moveForward(4.5);
  turnLeftPivot();

  moveForward(3);
  

  stopMove();
  Serial.println("Path Finished!");
}

// ============================================================
// SETUP FUNCTION (Startup Fix Retained)
// ============================================================
void setup() {
  Serial.begin(9600);
  
  // Retain FIX: Initialize speeds AND explicitly stop motors BEFORE any delay
  setLeftSpeed(STRAIGHT_SPEED); 
  setRightSpeed(STRAIGHT_SPEED);
  
  runLeftMotors(RELEASE); 
  runRightMotors(RELEASE); 

  delay(1000);
  
  // Now safely start the path
  followPath();
}

void loop() {
  // Path runs only once inside setup()
}