#include <AFMotor.h>

// Motors
AF_DCMotor rightFront(4);
AF_DCMotor rightBack(3);
AF_DCMotor leftBack(2);
AF_DCMotor leftFront(1);

int speed = 150;
int offset = 6;
int reverseOffset = 55;

// =========================
// BASIC MOVEMENT FUNCTIONS
// =========================

void moveForward(unsigned long t) {
  leftFront.setSpeed(speed);
  leftBack.setSpeed(speed);
  rightFront.setSpeed(speed + offset);
  rightBack.setSpeed(speed + offset);

  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);

  delay(t);
  stopMove();
}

void moveBackward() {
  leftFront.setSpeed(speed);
  leftBack.setSpeed(speed);
  rightFront.setSpeed(speed - reverseOffset);
  rightBack.setSpeed(speed - reverseOffset);

  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);

  delay(3500);
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

void turnLeft() {
  leftFront.setSpeed(speed + 50);
  leftBack.setSpeed(speed + 50);
  rightFront.setSpeed(speed + 50);
  rightBack.setSpeed(speed + 50);

  // left wheels backward, right wheels forward
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);

  delay(2150);
  stopMove();
}

void turnRight() {
  leftFront.setSpeed(speed + 50);
  leftBack.setSpeed(speed + 50);
  rightFront.setSpeed(speed + 50);
  rightBack.setSpeed(speed + 50);

  // right wheels backward, left wheels forward
  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);

  delay(2150);
  stopMove();
}

// =========================
// PATH FOLLOWING SEQUENCE
// =========================

void followPath() {

  // 1) Move right along bottom
  moveForward(1000);

  // 2) Turn Right
  turnRight();

  // 3) Move UP
  //moveForward(5000);

  // 4) Turn Left
  //turnLeft();

  // 5) Move RIGHT (middle section)
  //moveForward(1000);

  // 6) Turn UP
  //turnLeft();

  // 7) Move UP (final section)
  //moveForward(200);

  // 8) Turn RIGHT (toward target)
  //turnRight();

  // 9) Final approach to target
  //moveForward(200);

  stopMove();
}

void setup() {
  delay(1000);
  followPath();
}

void loop() {}
