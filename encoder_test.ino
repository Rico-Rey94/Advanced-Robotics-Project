#include <AFMotor.h>
#include <Arduino.h> 

// ============================================================
// CONSTANTS & SETTINGS
// ============================================================
int motorSpeed = 200; 
int motorOffset = 6;
int turnSpeed = 240; 
const long TURN_90_TICKS = 800; // Your last calibrated value

// ============================================================
// ENCODERS & COUNTS
// ============================================================
#define ENC_LEFT_A 18  
#define ENC_LEFT_B 40
#define ENC_RIGHT_A 20 
#define ENC_RIGHT_B 28

volatile long encoderCount[2] = {0,0}; 

// ============================================================
// MOTORS 
// ============================================================
AF_DCMotor rightFront(4);
AF_DCMotor rightBack(3);
AF_DCMotor leftFront(2);
AF_DCMotor leftBack(2);

// ============================================================
// ENCODER ISRS (Using the latest signs, which were: L: 1/-1, R: -1/1)
// If movement is inconsistent, these are the first place to adjust!
// ============================================================
void encoderLeft_ISR() { 
    // Left Encoder: Check if FORWARD increases this count.
    encoderCount[0] += digitalRead(ENC_LEFT_B) ? 1 : -1; 
}

void encoderRight_ISR() { 
    // Right Encoder: Check if FORWARD increases this count.
    encoderCount[1] += digitalRead(ENC_RIGHT_B) ? -1 : 1; 
} 

// ============================================================
// UTILITY FUNCTIONS
// ============================================================
void resetEncoders() {
  encoderCount[0] = 0; 
  encoderCount[1] = 0; 
  Serial.println("Encoders Reset to 0, 0.");
}

void stopMove() {
  rightFront.run(RELEASE);
  rightBack.run(RELEASE);
  leftFront.run(RELEASE);
  leftBack.run(RELEASE);
  Serial.println("Motors Stopped.");
}

void moveForward() {
    stopMove(); // Ensure motors stop before new command
    resetEncoders();
    leftFront.setSpeed(motorSpeed);
    leftBack.setSpeed(motorSpeed);
    rightFront.setSpeed(motorSpeed + motorOffset);
    rightBack.setSpeed(motorSpeed + motorOffset);

    leftFront.run(FORWARD); 
    leftBack.run(FORWARD);  
    rightFront.run(FORWARD);
    rightBack.run(FORWARD);
    Serial.println("Moving FORWARD (Type 's' to stop)...");
}

void moveBackward() {
    stopMove();
    resetEncoders();
    leftFront.setSpeed(motorSpeed);
    leftBack.setSpeed(motorSpeed);
    rightFront.setSpeed(motorSpeed + motorOffset);
    rightBack.setSpeed(motorSpeed + motorOffset);

    leftFront.run(BACKWARD); 
    leftBack.run(BACKWARD);  
    rightFront.run(BACKWARD);
    rightBack.run(BACKWARD);
    Serial.println("Moving BACKWARD (Type 's' to stop)...");
}

void turnLeft() {
    stopMove();
    resetEncoders();
    
    // Left wheels backward (pivot) 
    leftFront.setSpeed(turnSpeed);
    leftBack.setSpeed(turnSpeed);
    leftFront.run(BACKWARD);
    leftBack.run(BACKWARD);
    
    // Right wheels forward
    rightFront.setSpeed(turnSpeed);
    rightBack.setSpeed(turnSpeed);
    rightFront.run(FORWARD);
    rightBack.run(FORWARD);
    Serial.println("Turning LEFT (Type 's' to stop)...");
}

void turnRight() {
    stopMove();
    resetEncoders();

    // Right wheels backward (pivot)
    rightFront.setSpeed(turnSpeed);
    rightBack.setSpeed(turnSpeed);
    rightFront.run(BACKWARD);
    rightBack.run(BACKWARD);
    
    // Left wheels forward 
    leftFront.setSpeed(turnSpeed);
    leftBack.setSpeed(turnSpeed);
    leftFront.run(FORWARD);
    leftBack.run(FORWARD);
    Serial.println("Turning RIGHT (Type 's' to stop)...");
}


// ============================================================
// SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Encoder Test Ready.");
  Serial.println("Commands: f (forward), b (backward), l (turn left), r (turn right), s (stop)");

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  // Attach interrupts 
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), encoderLeft_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encoderRight_ISR, CHANGE);

  stopMove();
}

void loop() {
    // Check for serial input commands
    if (Serial.available()) {
        char command = Serial.read();
        if (command == 'f') moveForward();
        else if (command == 'b') moveBackward();
        else if (command == 'l') turnLeft();
        else if (command == 'r') turnRight();
        else if (command == 's') stopMove();
    }

    // Print encoder counts every 100 milliseconds
    Serial.print("Left: ");
    Serial.print(encoderCount[0]);
    Serial.print(" | Right: ");
    Serial.println(encoderCount[1]);
    delay(100); 
}
