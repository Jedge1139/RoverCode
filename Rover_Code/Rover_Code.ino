// Rover_Code.ino
// Authors: Jacob Edge

//-------------------------------

#include <Servo.h>

// Motor left
int enA = 5;
int in1 = 4; // left backword
int in2 = 7; // left forward

// Motor right
int enB = 6;
int in3 = 2; // right backword
int in4 = 8; // right forward

// Ultrasonic sensor
int trigPin = A1;
int echoPin = A0;

// Servo motor
Servo servoMotor;
int servoPin = A2;

void setup() {
// Motor setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

// Ultrasonic sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  servoMotor.attach(servoPin);

  Serial.begin(9600);
}

unsigned long obstacleDetectionStartTime = 0;
const unsigned long obstacleDetectionDelay = 3000; // 3 seconds

void loop() {
// Move forward if no obstacles
  if (!isObstacle()) {
    moveForward();
  } else {
   if (millis() - obstacleDetectionStartTime >= obstacleDetectionDelay) {
// Obstacle detected, stop and check right
    stopMotors();
    delay(1000);

    servoMotor.write(0); // Point the ultrasonic sensor to the right
    delay(500);

    if (!isObstacle()) {
  // No obstacle on the right, turn right
      turn90Degrees(1);
      delay(1000);
    } else {
// Obstacle on the right, check left
    servoMotor.write(180); // Point the ultrasonic sensor to the left
    delay(500);

    if (!isObstacle()) {
  // No obstacle on the left, turn left
      turn90Degrees(-1);
      delay(1000);
    } else {
  // Obstacle on both sides, back up
      moveBackward();
      delay(1000);
  }
}

// Move forward after turning
  moveForward();
  delay(1000);

// Move the servo back to the center
  servoMotor.write(90);
// Reset the obstacle detection start time
  obstacleDetectionStartTime = millis();
    }
  }
}

void moveForward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 200);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 200);
}

void moveBackward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 200);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 200);
}

void turnLeft(int degrees) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 90);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 120);
  delay(degrees * 10);
  stopMotors();
}

void turnRight(int degrees) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 120);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 90);
  delay(degrees * 10); 
  stopMotors();
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void turn90Degrees(int direction) {
  if (direction == 1) {
  // Turn right
    turnRight(90);
  } else {
    // Turn left
    turnLeft(90);
  }
}

bool isObstacle() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  // Calculate distance in cm
  int distance = duration / 58;

  return distance < 10;
}