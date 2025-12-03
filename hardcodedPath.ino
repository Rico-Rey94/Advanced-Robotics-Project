#include <AFMotor.h>
#include <Servo.h>

// ============================================================
// MOTOR SETUP (4WD, driven as left pair + right pair)
// ============================================================
AF_DCMotor rightFront(4);
AF_DCMotor rightBack(3);
AF_DCMotor leftBack(2);
AF_DCMotor leftFront(1);

const int motorSpeed = 120;      
const int pivotSpeed = 150;      
const int PIVOT_BOOST = 50;      

// ============================================================
// ENCODER INPUTS
// ============================================================
int leftCount = 0;
int rightCount = 0;

// LEFT wheel encoder
const int leftEnc_A = 18;
const int leftEnc_B = 19;

// RIGHT wheel encoder
const int rightEnc_A = 20;
const int rightEnc_B = 21;

// ISR (A channels only)
void leftISR()  { leftCount++; }
void rightISR() { rightCount++; }

// ============================================================
// Stop all motors
// ============================================================
void allStop() {
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
}

// ============================================================
// Drive straight with encoder correction (NO ANTISTALL)
// ============================================================
void driveForwardCounts(long targetCounts) {
  leftCount = 0;
  rightCount = 0;

  // Start motors at normal speed
  leftFront.setSpeed(motorSpeed);
  leftBack.setSpeed(motorSpeed);
  rightFront.setSpeed(motorSpeed);
  rightBack.setSpeed(motorSpeed);

  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);

  while ((leftCount + rightCount) / 2 < targetCounts) {
    long diff = leftCount - rightCount;

    int L = motorSpeed;
    int R = motorSpeed;

    if (diff > 5) {
      R += 5;
    } else if (diff < -5) {
      L += 5;
    }

    L = constrain(L, 0, 255);
    R = constrain(R, 0, 255);

    leftFront.setSpeed(L);
    leftBack.setSpeed(L);
    rightFront.setSpeed(R);
    rightBack.setSpeed(R);
  }

  allStop();
  delay(200);
}

// ============================================================
// TIMED PIVOT TURNS
// ============================================================
const int TURN_DELAY_MS = 4000; 
const int PRINT_INTERVAL = 50;   

void pivotLeft() {
  leftCount = 0;
  rightCount = 0;

  leftFront.run(BACKWARD);
  leftBack.run(RELEASE);
  rightFront.run(FORWARD);
  rightBack.run(RELEASE);

  leftFront.setSpeed(pivotSpeed + PIVOT_BOOST);
  rightFront.setSpeed(pivotSpeed);

  unsigned long start = millis();
  unsigned long lastPrint = 0;

  while (millis() - start < TURN_DELAY_MS) {
    if (millis() - lastPrint >= PRINT_INTERVAL) {
      lastPrint = millis();
      Serial.print("L:");
      Serial.print(leftCount);
      Serial.print(" | R:");
      Serial.println(rightCount);
    }
  }

  allStop();
  delay(200);
}

void pivotRight() {
  leftCount = 0;
  rightCount = 0;

  leftFront.run(FORWARD);
  leftBack.run(RELEASE);
  rightFront.run(BACKWARD);
  rightBack.run(RELEASE);

  leftFront.setSpeed(pivotSpeed);
  rightFront.setSpeed(pivotSpeed);

  unsigned long start = millis();
  unsigned long lastPrint = 0;

  while (millis() - start < TURN_DELAY_MS) {
    if (millis() - lastPrint >= PRINT_INTERVAL) {
      lastPrint = millis();
      Serial.print("L:");
      Serial.print(leftCount);
      Serial.print(" | R:");
      Serial.println(rightCount);
    }
  }

  allStop();
  delay(200);
}

// ============================================================
// Convert FEET → ENCODER COUNTS
// ============================================================
long feetToCounts(float feet) {
  const float wheelFeet = 0.742f;
  float revs = feet / wheelFeet;
  return (long)(revs * 360.0f);
}

// ============================================================
// HARD-CODED PATH
// ============================================================
void runPath() {

  driveForwardCounts(feetToCounts(1.0));
  pivotRight();

  driveForwardCounts(feetToCounts(4.5));
  pivotLeft();

  driveForwardCounts(feetToCounts(2.5));
  pivotLeft();

  driveForwardCounts(feetToCounts(3.5));
  pivotRight();

  driveForwardCounts(feetToCounts(3.0));
  pivotRight();

  driveForwardCounts(feetToCounts(2.5));
  pivotRight();

  driveForwardCounts(feetToCounts(1.5));
  pivotLeft();

  driveForwardCounts(feetToCounts(2.0));
  pivotRight();

  driveForwardCounts(feetToCounts(1.0));
  pivotLeft();

  driveForwardCounts(feetToCounts(1.5));
  pivotLeft();

  driveForwardCounts(feetToCounts(2.5));

  allStop();
  Serial.println("PATH COMPLETE");
}

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(leftEnc_A, INPUT_PULLUP);
  pinMode(leftEnc_B, INPUT_PULLUP);
  pinMode(rightEnc_A, INPUT_PULLUP);
  pinMode(rightEnc_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEnc_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEnc_A), rightISR, RISING);

  allStop();
  delay(1000);

  Serial.println("Robot Ready");

  runPath();   // still auto-starting, can remove if you want manual control
}

void loop() {}
