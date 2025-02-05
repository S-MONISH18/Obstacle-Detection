// Motor and sensor pin definitions
const int leftMotorForward = 2;  // Pin for left motor forward
const int leftMotorBackward = 3;  // Pin for left motor backward
const int rightMotorForward = 4;  // Pin for right motor forward
const int rightMotorBackward = 5;  // Pin for right motor backward

const int trigPin = A0;  // Pin for ultrasonic sensor trigger (front)
const int echoPin = A1;  // Pin for ultrasonic sensor echo (front)

const int trigPinL = A2;  // Pin for left ultrasonic sensor trigger
const int echoPinL = A3;  // Pin for left ultrasonic sensor echo

const int trigPinR = A4;  // Pin for right ultrasonic sensor trigger
const int echoPinR = A5;  // Pin for right ultrasonic sensor echo

const int maxDistance = 20;  // Maximum distance to detect obstacles (in cm)

void setup() {
  // Initialize motor control pins
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);

  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  int distanceR = readDistance(trigPinR, echoPinR);
  int distanceL = readDistance(trigPinL, echoPinL);
  int distance = readDistance(trigPin, echoPin);

  // Print distances to Serial Monitor
  Serial.print("Front Distance: ");
  Serial.print(distance);
  Serial.print(" cm\t");

  Serial.print("Left Distance: ");
  Serial.print(distanceL);
  Serial.print(" cm\t");

  Serial.print("Right Distance: ");
  Serial.print(distanceR);
  Serial.println(" cm");

  if (distance < maxDistance) {
    // Obstacle detected, stop and turn
    stopMotors();
    delay(250);

    if (distanceL < maxDistance && distanceR < maxDistance) {
      // Obstacle detected on both sides, move backward
      moveBackward();
      delay(500);
    } else if (distanceL < maxDistance) {
      // Obstacle detected on the left, turn right
      turnRight();
    } else if (distanceR < maxDistance) {
      // Obstacle detected on the right, turn left
      turnLeft();
    } else {
      // No clear direction to turn, try moving backward
      moveBackward();
      delay(500);
    }
  } else {
    // No obstacle, move forward
    moveForward();
  }
  delay(50);
}

int readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  int duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

void stopMotors() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

void moveForward() {
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void moveBackward() {
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void turnLeft() {
  // Turn left by moving right motor forward and left motor backward
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
  delay(500); // Adjust delay to control turn angle
  stopMotors();
}

void turnRight() {
  // Turn right by moving left motor forward and right motor backward
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
  delay(500); // Adjust delay to control turn angle
  stopMotors();
}
