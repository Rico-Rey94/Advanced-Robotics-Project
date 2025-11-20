#include <AFMotor.h>
#include <Servo.h>

// ---------------- Motors ----------------
AF_DCMotor rightFront(1);
AF_DCMotor rightBack(2);
AF_DCMotor leftBack(3);
AF_DCMotor leftFront(4);

// ---------------- Encoders ----------------
#define ENC1_A 18
#define ENC1_B 40
#define ENC2_A 19
#define ENC2_B 44
#define ENC3_A 20
#define ENC3_B 28
#define ENC4_A 21
#define ENC4_B 22

volatile long encoderCount[4] = {0,0,0,0};

// ---------------- Ultrasonic + Servo ----------------
#define SERVO_PIN 23
#define TRIG_PIN 35
#define ECHO_PIN 37
Servo servoLook;

// ---------------- Movement settings ----------------
int motorSpeed = 150;
int motorOffset = 6;

float Kp = 0.05;

unsigned long lastSpeedCheck = 0;
const unsigned long speedCheckInterval = 100;
long prevEncoderCount[4] = {0,0,0,0};
float wheelSpeed[4] = {0,0,0,0};

// ---------------- Navigation State ----------------
enum NavigationState {
  MOVING_FORWARD,
  OBSTACLE_DETECTED,
  SCANNING,
  TURNING
};
NavigationState currentState = MOVING_FORWARD;

// turn timing
unsigned long motionStartTime = 0;
unsigned long motionDuration = 0;
bool motionActive = false;

// cooldown after turn
unsigned long turnCooldownUntil = 0;

// ---------------- Loop timing ----------------
unsigned long lastLoop = 0;
const unsigned long loopInterval = 80;

// ---------------- Encoder ISRs ----------------
void encoder1A_ISR() { encoderCount[0] += digitalRead(ENC1_B) ? 1 : -1; }
void encoder2A_ISR() { encoderCount[1] += digitalRead(ENC2_B) ? 1 : -1; }
void encoder3A_ISR() { encoderCount[2] += digitalRead(ENC3_B) ? 1 : -1; }
void encoder4A_ISR() { encoderCount[3] += digitalRead(ENC4_B) ? 1 : -1; }

void setup() {
  Serial.begin(9600);
  Serial.println("Robot booting...");

  servoLook.attach(SERVO_PIN);
  servoLook.write(90);

  rightBack.setSpeed(motorSpeed);
  rightFront.setSpeed(motorSpeed);
  leftFront.setSpeed(motorSpeed);
  leftBack.setSpeed(motorSpeed);

  stopMove();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP); pinMode(ENC2_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP); pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC4_A, INPUT_PULLUP); pinMode(ENC4_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoder1A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoder2A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_A), encoder3A_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_A), encoder4A_ISR, CHANGE);

  Serial.println("Ready.");
}

void loop() {
  if (millis() - lastLoop < loopInterval) return;
  lastLoop = millis();

  unsigned long now = millis();

  // timed turn handler
  if (motionActive && now - motionStartTime >= motionDuration) {
    stopMove();
    motionActive = false;
    turnCooldownUntil = millis() + 400;  // ignore false triggers
    currentState = MOVING_FORWARD;
  }

  speedCorrection();

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
      break;
  }
}

// ---------------- Movement Logic ----------------
void handleForwardMovement() {
  moveForward();

  int front = getDistance();
  if (front < 35) {
    Serial.print("Object detected at ");
    Serial.print(front);
    Serial.println(" cm");
    stopMove();
    currentState = OBSTACLE_DETECTED;
  }
}

void handleObstacleDetection() {
  Reverse();
  delay(200);
  currentState = SCANNING;
}

void handleScanning() {
  stopMove();

  // look right
  servoLook.write(150);
  delay(2500);
  int rightDist = getDistance();

  // look left
  servoLook.write(30);
  delay(2500);
  int leftDist = getDistance();

  // center again
  servoLook.write(90);
  delay(2000);

  Serial.print("Scan -> R: ");
  Serial.print(rightDist);
  Serial.print("  L: ");
  Serial.println(leftDist);

  if (rightDist > leftDist) {
    turnRight(1000);
  } else if (leftDist > rightDist) {
    turnLeft(1000);
  } else {
    Reverse();
    currentState = MOVING_FORWARD;
  }
}

// ---------------- Motor Controls ----------------
void moveForward() {
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);
}

void stopMove() {
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
}

void Reverse() {
  rightFront.setSpeed(motorSpeed + 30);
  rightBack.setSpeed(motorSpeed + 30);
  leftFront.setSpeed(motorSpeed + 30);
  leftBack.setSpeed(motorSpeed + 30);

  rightFront.run(BACKWARD);
  rightBack.run(BACKWARD);
  leftFront.run(BACKWARD);
  leftBack.run(BACKWARD);

  delay(350);
  stopMove();
}

// Corrected turn directions
void turnLeft(unsigned long duration) {
  // left wheels stop, right wheels forward
  rightFront.run(FORWARD);
  rightBack.run(FORWARD);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);

  motionStartTime = millis();
  motionDuration = duration;
  motionActive = true;
  currentState = TURNING;
}

void turnRight(unsigned long duration) {
  // right wheels stop, left wheels forward
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
  leftFront.run(FORWARD);
  leftBack.run(FORWARD);

  motionStartTime = millis();
  motionDuration = duration;
  motionActive = true;
  currentState = TURNING;
}

// ---------------- Speed Correction ----------------
void updateWheelSpeeds() {
  unsigned long now = millis();
  if (now - lastSpeedCheck < speedCheckInterval) return;
  lastSpeedCheck = now;

  noInterrupts();
  long c[4];
  for (int i = 0; i < 4; i++) c[i] = encoderCount[i];
  interrupts();

  for (int i = 0; i < 4; i++) {
    wheelSpeed[i] = (c[i] - prevEncoderCount[i]) * 10;
    prevEncoderCount[i] = c[i];
  }
}

void speedCorrection() {
  updateWheelSpeeds();

  float rightAvg = (wheelSpeed[0] + wheelSpeed[1]) * 0.5;
  float leftAvg  = (wheelSpeed[2] + wheelSpeed[3]) * 0.5;
  float error = leftAvg - rightAvg;

  int rightAdj = motorSpeed + (int)(Kp * error);
  int leftAdj  = motorSpeed - (int)(Kp * error);

  rightAdj = constrain(rightAdj, 0, 255);
  leftAdj  = constrain(leftAdj, 0, 255);

  rightFront.setSpeed(rightAdj);
  rightBack.setSpeed(rightAdj);
  leftFront.setSpeed(leftAdj);
  leftBack.setSpeed(leftAdj);
}

// ---------------- Ultrasonic ----------------
int getDistance() {
  long total = 0;
  int valid = 0;

  for (int i = 0; i < 4; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 35000);
    if (duration > 0) {
      int cm = duration * 0.034 / 2;

      // Accept much wider range (fix!)
      if (cm >= 2 && cm <= 200) {
        total += cm;
        valid++;
      }
    }
    delay(5);
  }

  if (valid == 0) return 200; // treat unknown as far
  return total / valid;
}
