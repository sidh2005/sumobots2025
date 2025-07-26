#include <Arduino.h>

// Motor pins (LEFT and RIGHT sides only)
const int IN1_L = 6;
const int IN2_L = 7;

const int IN1_R = 8;
const int IN2_R = 9;

// Ultrasonic pins
const int trigPins[3] = {51, 41, 31};  
const int echoPins[3] = {53, 39, 33};
const int numSensors = 3; 
const int detectionThreshold = 30; // cm

// Motor speeds
int motorSpeed = 255;       // Full speed for forward

unsigned long lastActionTime = 0;
unsigned long cooldown = 500;

void setup() {
  Serial.begin(9600);

  // Set motor pins as outputs
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);

  // Set ultrasonic pins
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  stopAll();
}

void loop() {
  unsigned long now = millis();
  if (now - lastActionTime < cooldown) return;

  long distances[3];
  for (int i = 0; i < numSensors; i++) {
    distances[i] = readUltrasonic(trigPins[i], echoPins[i]);
  }

  if (distances[1] > 0 && distances[1] < detectionThreshold) {
    moveForward();
  } else if (distances[0] > 0 && distances[0] < detectionThreshold) {
    turnUntilCenterSeesOpponent(moveRight);
  } else if (distances[2] > 0 && distances[2] < detectionThreshold) {
    turnUntilCenterSeesOpponent(moveLeft);
  } else {
    stopAll();
  }

  lastActionTime = millis();
  delay(100);
}

void turnUntilCenterSeesOpponent(void (*turnFunc)()) {
  unsigned long start = millis();
  while (true) {
    turnFunc();
    long centerDist = readUltrasonic(trigPins[1], echoPins[1]);
    if (centerDist > 0 && centerDist < detectionThreshold) break;
    if (millis() - start > 2000) break;
    delay(50);
  }
  stopAll();
  delay(100);
}

void moveForward() {
  // LEFT side forward
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, LOW);

  // RIGHT side forward
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
}

void moveRight() {
  // LEFT side forward
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, LOW);

  // RIGHT side backward
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);
}

void moveLeft() {
  // LEFT side backward
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, HIGH);

  // RIGHT side forward
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
}

void stopAll() {
  // Active brake both sides
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);

  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
}

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}
