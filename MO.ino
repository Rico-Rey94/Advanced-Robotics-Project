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

// ***** SLOWED DOWN SWEEP *****
int sweepPos  = 90;
int sweepStep = 4;        
unsigned long lastServoMove = 0;
// Increased from 4 to 12 for slower, more reliable scans.
const int sweepInterval = 12;   
const int sweepMin = 60;      
const int sweepMax = 120;     
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
// FORWARD SPEED REDUCED TO 85
uint8_t currentFwdSpeed = 85; 
const uint8_t FWD_SPEED = 85;
const uint8_t TURN_SPEED = 200;
const uint8_t REV_SPEED = 100;
const float CLEARANCE_MARGIN = 5.0; // Margin to consider one side 'clearer'

// --- TURN TIMING CONSTANTS ---
const int PIVOT_TURN_90_DELAY = 900; // Standard delay for approx 90 degrees
const int TURN_180_DELAY     = PIVOT_TURN_90_DELAY * 2; // 1100ms for 180 degrees fail-safe

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
  const int maxCorr = 90;

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

// New function for short forward movement
void moveForwardShort() {
  motor1.setSpeed(FWD_SPEED); motor1.run(FORWARD);
  motor2.setSpeed(FWD_SPEED); motor2.run(FORWARD);
  motor3.setSpeed(FWD_SPEED); motor3.run(FORWARD);
  motor4.setSpeed(FWD_SPEED); motor4.run(FORWARD);

  delay(500); // Move forward for half a second
  stopAll();
}

void reverseShort() {
  motor1.setSpeed(REV_SPEED); motor1.run(BACKWARD);
  motor2.setSpeed(REV_SPEED); motor2.run(BACKWARD);
  motor3.setSpeed(REV_SPEED); motor3.run(BACKWARD);
  motor4.setSpeed(REV_SPEED); motor4.run(BACKWARD);

  delay(250);
  stopAll();
}

// Pivot turns (~90 degrees)
void pivotTurnRight() {
  motor1.setSpeed(TURN_SPEED); motor1.run(FORWARD);
  motor2.setSpeed(TURN_SPEED); motor2.run(FORWARD);
  motor3.setSpeed(TURN_SPEED); motor3.run(BACKWARD);
  motor4.setSpeed(TURN_SPEED); motor4.run(BACKWARD);

  delay(PIVOT_TURN_90_DELAY); 
  stopAll();
}

void pivotTurnLeft() {
  motor1.setSpeed(TURN_SPEED); motor1.run(BACKWARD);
  motor2.setSpeed(TURN_SPEED); motor2.run(BACKWARD);
  motor3.setSpeed(TURN_SPEED); motor3.run(FORWARD);
  motor4.setSpeed(TURN_SPEED); motor4.run(FORWARD);

  delay(PIVOT_TURN_90_DELAY);
  stopAll();
}

// New function for 180-degree turn (Fail-safe maneuver)
void pivotTurn180() {
  motor1.setSpeed(TURN_SPEED); motor1.run(FORWARD);
  motor2.setSpeed(TURN_SPEED); motor2.run(FORWARD);
  motor3.setSpeed(TURN_SPEED); motor3.run(BACKWARD);
  motor4.setSpeed(TURN_SPEED); motor4.run(BACKWARD);

  delay(TURN_180_DELAY); 
  stopAll();
}


// =======================================================
//                     NEW SCAN FUNCTION (Refined)
// =======================================================
// Performs a focused scan and returns the open direction:
// 1 = Left, -1 = Right, 0 = Blocked
int performFullScan() {
  sweeping = false; // Stop the continuous sweep during the forced scan
  float distLeft = 0;
  float distRight = 0;
  
  // 1. Scan Left Extreme
  scanServo.write(sweepMin); 
  delay(150); // Allow time for servo to move
  distLeft = readDistance();
  
  // 2. Scan Right Extreme
  scanServo.write(sweepMax);
  delay(150);
  distRight = readDistance();

  // 3. Return to center
  scanServo.write(servoForward);
  delay(150);
  sweeping = true; // Resume continuous sweep

  // Decision logic
  Serial.print("Scan L:"); Serial.print(distLeft); 
  Serial.print(" | R:"); Serial.println(distRight);

  // Check if one side is significantly clearer than the other
  if (distLeft > SAFE_STOP_DIST && distLeft > (distRight + CLEARANCE_MARGIN)) return 1;  // Go Left (and it's much clearer than right)
  if (distRight > SAFE_STOP_DIST && distRight > (distLeft + CLEARANCE_MARGIN)) return -1; // Go Right (and it's much clearer than left)
  if (distLeft > SAFE_STOP_DIST && distRight > SAFE_STOP_DIST) return (random(0,2) == 0 ? 1 : -1); // Both clear, pick randomly
  if (distLeft > SAFE_STOP_DIST) return 1; // Only left is clear
  if (distRight > SAFE_STOP_DIST) return -1; // Only right is clear
  
  return 0; // Completely blocked
}


// =======================================================
//                        Setup
// =======================================================
void setup() {
  Serial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(servoForward); // Set to 90

  pinMode(leftEnc_A, INPUT_PULLUP);
  pinMode(rightEnc_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEnc_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEnc_A), rightISR, RISING);

  sweeping = true;
  Serial.println("Robot starting...");
}

// =======================================================
//                         Loop (Refined Escape Logic)
// =======================================================
void loop() {

  // Sweep must run continuously unless a full scan is needed
  updateServoSweep(); 
  
  float dist = readDistance();

  if (dist > 0 && dist < SAFE_STOP_DIST) {

    // *** STOP MOTORS IMMEDIATELY ***
    stopAll();
    Serial.println("*** Obstacle detected, beginning escape routine. ***");

    bool cleared = false;
    int attempts = 0;

    while (!cleared && attempts < 5) {
      attempts++;
      
      // 1. REVERSE to gain space for turn
      reverseShort();
      
      // 2. PERFORM FULL SCAN TO DETERMINE TURN DIRECTION
      int turnDir = performFullScan(); 
      
      if (turnDir == 1) { // Left is clearer
        pivotTurnLeft();
      } else if (turnDir == -1) { // Right is clearer
        pivotTurnRight();
      } else { // Both sides blocked or unclear, try a small random turn to wiggle out
        Serial.println("No clear direction found, wiggling...");
        if (random(0,2)==0) pivotTurnLeft();
        else pivotTurnRight();
      }

      // 3. CHECK DISTANCE after turn/wiggle
      float nd = readDistance(); 

      if (nd > SAFE_STOP_DIST || nd < 0) {
        cleared = true;
        leftTicks = rightTicks = 0;
        Serial.println("Path cleared, resuming forward movement.");
      } else {
          Serial.print("Path still blocked after turn ");
          Serial.print(attempts);
          Serial.println(", re-scanning.");
      }
    }

    if (!cleared) {
      // If attempts reach 5 and path is still blocked
      Serial.println("*** STUCK - Executing 180-degree fail-safe turn and short move ***");
      pivotTurn180();  // Execute the decisive 180-degree turn
      moveForwardShort(); // Move forward to try and clear the conflict area
    }
  }

  // Only move forward if not blocked
  forwardCorrected();
}