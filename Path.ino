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
int spd = 220;                  
const int LEFT_OFFSET = 40;     

// PIVOT TURN TIMING
const unsigned long SWING_DELAY_MS = 6000; 
const unsigned long KICK_START_MS = 10;

// ============================================================
// DISTANCE TIMINGS (CALIBRATION REQUIRED)
// ============================================================
const unsigned long DIST_1_FOOT_MS    = 1000;
const unsigned long DIST_1_5_FOOT_MS  = 1500;
const unsigned long DIST_2_FOOT_MS    = 2000;
const unsigned long DIST_2_5_FOOT_MS  = 2500;
const unsigned long DIST_3_FOOT_MS    = 3000;
const unsigned long DIST_3_5_FOOT_MS  = 3500;
const unsigned long DIST_4_5_FOOT_MS  = 4500;

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
// STOP MOVEMENT
// ============================================================
void stopMove() {
  runLeftMotors(BRAKE);
  runRightMotors(BRAKE);
  delay(200);
}

// ============================================================
// FORWARD / BACKWARD
// ============================================================
void moveForward(unsigned long t) {
  setLeftSpeed(min(spd + LEFT_OFFSET, 255));  
  setRightSpeed(spd);

  runLeftMotors(FORWARD);
  runRightMotors(FORWARD);

  delay(t);
  stopMove();
}

void moveBackward(unsigned long t) {
  setLeftSpeed(min(spd + LEFT_OFFSET, 255));
  setRightSpeed(spd);

  runLeftMotors(BACKWARD);
  runRightMotors(BACKWARD);

  delay(t);
  stopMove();
}

// ============================================================
// PIVOT TURNS (TIME-BASED)
//  - Left pivot: Left wheels BRAKE, Right wheels FORWARD
//  - Right pivot: Right wheels BRAKE, Left wheels FORWARD
// ============================================================
void turnLeftPivot() {
  // Kick start on right side
  setRightSpeed(255);
  runRightMotors(FORWARD);
  
  runLeftMotors(BRAKE);
  delay(KICK_START_MS);

  // Normal pivot
  setRightSpeed(spd);
  delay(SWING_DELAY_MS - KICK_START_MS);

  stopMove();
}

void turnRightPivot() {
  // Kick start on left side
  setLeftSpeed(255);
  runLeftMotors(FORWARD);

  runRightMotors(BRAKE);
  delay(KICK_START_MS);

  // Normal pivot
  setLeftSpeed(spd);
  delay(SWING_DELAY_MS - KICK_START_MS);

  stopMove();
}

// ============================================================
// PATH FOLLOWING SEQUENCE
// ============================================================
void followPath() {
  Serial.println("Starting Path...");

  moveForward(DIST_1_FOOT_MS); 
  turnRightPivot();

  moveForward(DIST_4_5_FOOT_MS);
  turnLeftPivot();

  moveForward(DIST_2_5_FOOT_MS);
  turnLeftPivot();

  moveForward(DIST_3_5_FOOT_MS);
  turnRightPivot();

  moveForward(DIST_3_FOOT_MS);
  turnRightPivot();

  moveForward(DIST_2_5_FOOT_MS);
  turnRightPivot();

  moveForward(DIST_1_5_FOOT_MS);
  turnLeftPivot();

  moveForward(DIST_2_FOOT_MS);
  turnRightPivot();

  moveForward(DIST_1_FOOT_MS);
  turnLeftPivot();

  moveForward(DIST_1_5_FOOT_MS);
  turnLeftPivot();

  moveForward(DIST_2_5_FOOT_MS);

  stopMove();
  Serial.println("Path Finished!");
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  followPath();
}

void loop() {
  // Path runs only once inside setup()
}
