#include <AFMotor.h>
#include <Servo.h>

// ----------------- Motors -----------------
AF_DCMotor motor1(1, MOTOR12_8KHZ); // Left Front
AF_DCMotor motor2(2, MOTOR12_8KHZ); // Left Back
AF_DCMotor motor3(3, MOTOR12_8KHZ); // Right Back
AF_DCMotor motor4(4, MOTOR12_8KHZ); // Right Front

// ----------------- Ultrasonic -----------------
const int trigPin = 35;
const int echoPin = 37;

// ----------------- Servo -----------------
Servo scanServo;
const int SERVO_PIN = 23;
int servoForward = 90;

// ***** MUCH FASTER SWEEP *****
int sweepPos  = 90;
int sweepStep = 4;        // was 1 → now much faster
unsigned long lastServoMove = 0;
const int sweepInterval = 8;  // was 20 → now fast response
const int sweepMin = 60;      // was 20
const int sweepMax = 120;     // was 160
bool sweeping = true;

// ----------------- Encoders -----------------
const int leftEnc_A  = 18;
const int rightEnc_A = 20;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

void leftISR()  { leftTicks++; }
void rightISR() { rightTicks++; }

// ----------------- Robot Params -----------------
const float SAFE_STOP_DIST = 15.0;
uint8_t currentFwdSpeed = 120;
const uint8_t FWD_SPEED = 120;
const uint8_t TURN_SPEED = 200;
const uint8_t REV_SPEED = 120;

// =======================================================
//                    Helper Functions
// =======================================================

// FAST servo sweep
void updateServoSweep() {
  if (!sweeping) return;

  unsigned long now = millis();
  if (now - lastServoMove < sweepInterval) return;

  sweepPos += sweepStep;

  if (sweepPos >= sweepMax || sweepPos <= sweepMin)
    sweepStep = -sweepStep;

  scanServo.write(sweepPos);
  lastServoMove = now;
}

// Ultrasonic distance
float readDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return -1;

  return duration / 58.0;
}

// Stop motors
void stopAll() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// =======================================================
//      STRONG ENCODER CORRECTION (forward & reverse)
// =======================================================
void applyEncoderCorrection(bool movingForward) {

  long L = leftTicks;
  long R = rightTicks;
  long diff = L - R;

  const int strongGain = 5;   // slightly higher to keep perfect straightness
  const int baseBoost  = 18;
  const int maxCorr    = 90;

  int correction = strongGain * diff;

  if (diff > 0) correction = max(correction, baseBoost);
  if (diff < 0) correction = min(correction, -baseBoost);

  correction = constrain(correction, -maxCorr, maxCorr);

  int Ls = currentFwdSpeed - correction;
  int Rs = currentFwdSpeed + correction;

  Ls = constrain(Ls, 60, 255);
  Rs = constrain(Rs, 60, 255);

  if (movingForward) {
    motor1.setSpeed(Ls); motor1.run(FORWARD);
    motor2.setSpeed(Ls); motor2.run(FORWARD);
    motor3.setSpeed(Rs); motor3.run(FORWARD);
    motor4.setSpeed(Rs); motor4.run(FORWARD);
  } else {
    motor1.setSpeed(Ls); motor1.run(BACKWARD);
    motor2.setSpeed(Ls); motor2.run(BACKWARD);
    motor3.setSpeed(Rs); motor3.run(BACKWARD);
    motor4.setSpeed(Rs); motor4.run(BACKWARD);
  }
}

// =======================================================
//              Motion Functions
// =======================================================
void forwardCorrected() {
  applyEncoderCorrection(true);
}

void reverseShort() {
  motor1.setSpeed(REV_SPEED); motor1.run(BACKWARD);
  motor2.setSpeed(REV_SPEED); motor2.run(BACKWARD);
  motor3.setSpeed(REV_SPEED); motor3.run(BACKWARD);
  motor4.setSpeed(REV_SPEED); motor4.run(BACKWARD);

  delay(250);
  stopAll();
}

// Pivot turns
void pivotTurnRight() {
  motor1.setSpeed(TURN_SPEED); motor1.run(FORWARD);
  motor2.setSpeed(TURN_SPEED); motor2.run(FORWARD);
  motor3.setSpeed(TURN_SPEED); motor3.run(BACKWARD);
  motor4.setSpeed(TURN_SPEED); motor4.run(BACKWARD);

  delay(550);  // reduced for faster scanning
  stopAll();
}

void pivotTurnLeft() {
  motor1.setSpeed(TURN_SPEED); motor1.run(BACKWARD);
  motor2.setSpeed(TURN_SPEED); motor2.run(BACKWARD);
  motor3.setSpeed(TURN_SPEED); motor3.run(FORWARD);
  motor4.setSpeed(TURN_SPEED); motor4.run(FORWARD);

  delay(550);
  stopAll();
}

// =======================================================
//                        Setup
// =======================================================
void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(90);

  pinMode(leftEnc_A, INPUT_PULLUP);
  pinMode(rightEnc_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEnc_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEnc_A), rightISR, RISING);

  sweeping = true;
  Serial.println("Robot starting...");
}

// =======================================================
//                         Loop
// =======================================================
void loop() {

  float dist = readDistance();

  if (dist > 0 && dist < SAFE_STOP_DIST) {

    stopAll();
    Serial.println("*** Obstacle ***");

    reverseShort();

    bool cleared = false;
    int attempts = 0;

    sweeping = true;

    while (!cleared && attempts < 5) {
      attempts++;
      updateServoSweep();

      if (random(0,2)==0) pivotTurnLeft();
      else pivotTurnRight();

      float nd = readDistance();

      if (nd > SAFE_STOP_DIST || nd < 0) {
        cleared = true;
        leftTicks = rightTicks = 0;
      }
    }

    if (!cleared) {
      stopAll();
      Serial.println("*** STUCK ***");
      return;
    }
  }

  forwardCorrected();
  updateServoSweep();
}