
#include <AFMotor.h>  // Library for L293D Motor Shield

// Create motor objects for ports M1-M4 (up to 4 DC motors)
AF_DCMotor motor1(1);  // Motor on M1
AF_DCMotor motor2(2);  // Motor on M2
AF_DCMotor motor3(3);  // Motor on M3
AF_DCMotor motor4(4);  // Motor on M4

void setup() {
  // Initialize serial for debugging
  Serial.begin(9600);
  Serial.println("L293D Motor Shield: Starting motors forward...");

  // Set speed for all motors (0-255; 150 = moderate)
  motor1.setSpeed(150);
  motor2.setSpeed(150);
  motor3.setSpeed(150);
  motor4.setSpeed(150);

  // Run all motors forward
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  Serial.println("All motors running forward!");
}

void loop() {
  // Motors continue running; add logic here if needed (e.g., stop after time)
  delay(1000);  // Small delay to avoid overwhelming serial output
}
