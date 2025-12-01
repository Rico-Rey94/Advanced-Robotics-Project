#include <AFMotor.h>

// Motors
AF_DCMotor rightFront(4);
AF_DCMotor rightBack(3);
AF_DCMotor leftBack(2);
AF_DCMotor leftFront(1);

int spd = 150;

// =========================
// BASIC MOVEMENT FUNCTIONS
// =========================

void moveForward(unsigned long t) {
  leftFront.setSpeed(spd);
  leftBack.setSpeed(spd);
  rightFront.setSpeed(spd);
  rightBack.setSpeed(spd);

  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);

  delay(t);
  stopMove();
}

void moveBackward(unsigned long t) {
  leftFront.setSpeed(spd);
  leftBack.setSpeed(spd);
  rightFront.setSpeed(spd);
  rightBack.setSpeed(spd);

  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);

  delay(t);
  stopMove();
}

void stopMove() {
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
  delay(200);
}

// =========================
// PIVOT TURNS
// =========================

void turnLeftPivot(unsigned long t) {
  leftFront.setSpeed(spd);
  leftBack.setSpeed(spd);
  rightFront.setSpeed(spd);
  rightBack.setSpeed(spd);

  // left wheels backward, right wheels forward
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);

  delay(t);
  stopMove();
}

void turnRightPivot(unsigned long t) {
  leftFront.setSpeed(spd);
  leftBack.setSpeed(spd);
  rightFront.setSpeed(spd);
  rightBack.setSpeed(spd);

  // right wheels backward, left wheels forward
  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);

  delay(t);
  stopMove();
}

// =========================
// PATH FOLLOWING SEQUENCE
// =========================

void followPath() {

  // ==== Tune These Delays For Your Robot ====
  unsigned long long_forward1 = 4000;
  unsigned long up1 = 3500;
  unsigned long right1 = 3000;
  unsigned long up2 = 3500;
  unsigned long right2 = 2500;

  // 1) Move right along bottom
  moveForward(long_forward1);

  // 2) Turn UP
  turnLeftPivot(500);

  // 3) Move UP
  moveForward(up1);

  // 4) Turn RIGHT
  turnRightPivot(500);

  // 5) Move RIGHT (middle section)
  moveForward(right1);

  // 6) Turn UP
  turnLeftPivot(500);

  // 7) Move UP (final section)
  moveForward(up2);

  // 8) Turn RIGHT (toward target)
  turnRightPivot(500);

  // 9) Final approach to target
  moveForward(right2);

  stopMove();
}

void setup() {
  delay(1000);
  followPath();
}

void loop() {}
