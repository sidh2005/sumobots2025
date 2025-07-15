#include <Arduino.h>

// pin_init_left
const int IN1_FL = 2;
const int IN2_FL = 3;

const int PWM_FL = 4;

const int IN1_BL = 5;
const int IN2_BL = 6;
const int PWM_BL = 7;

// pin_init_right
const int IN1_FR = 8;
const int IN2_FR = 9;
const int PWM_FR = 10;

const int IN1_BR = 11;
const int IN2_BR = 12;
const int PWM_BR = 13; 

// ultrasonic_init
const int trigPins[3] = {A1, A3, A5};  
const int echoPins[3] = {A2, A4, A6};
const int numSensors = 3;
const int detectionThreshold = 30; // cm

// motor_init
int motorSpeed = 200; // 0–255

unsigned long lastActionTime = 0;
unsigned long cooldown = 500;

void setup() {
  Serial.begin(9600);

  // Set motor pins as output
  int motorPins[] = {
    IN1_FL, IN2_FL, PWM_FL,
    IN1_BL, IN2_BL, PWM_BL,
    IN1_FR, IN2_FR, PWM_FR,
    IN1_BR, IN2_BR, PWM_BR
  };

  for (int i = 0; i < 12; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  // Ultrasonic
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
  // front_left
  digitalWrite(IN1_FL, HIGH);
  digitalWrite(IN2_FL, LOW);
  analogWrite(PWM_FL, motorSpeed);

  // back_left
  digitalWrite(IN1_BL, HIGH);
  digitalWrite(IN2_BL, LOW);
  analogWrite(PWM_BL, motorSpeed);

  // front_right
  digitalWrite(IN1_FR, HIGH);
  digitalWrite(IN2_FR, LOW);
  analogWrite(PWM_FR, motorSpeed);

  // back_right
  digitalWrite(IN1_BR, HIGH);
  digitalWrite(IN2_BR, LOW);
  analogWrite(PWM_BR, motorSpeed);
}

void moveRight() {
  // left_ahead
  digitalWrite(IN1_FL, HIGH);
  digitalWrite(IN2_FL, LOW);
  analogWrite(PWM_FL, motorSpeed);

  digitalWrite(IN1_BL, HIGH);
  digitalWrite(IN2_BL, LOW);
  analogWrite(PWM_BL, motorSpeed);

  // right_spins_back
  digitalWrite(IN1_FR, LOW);
  digitalWrite(IN2_FR, HIGH);
  analogWrite(PWM_FR, motorSpeed);

  digitalWrite(IN1_BR, LOW);
  digitalWrite(IN2_BR, HIGH);
  analogWrite(PWM_BR, motorSpeed);
}

void moveLeft() {
  // left_back
  digitalWrite(IN1_FL, LOW);
  digitalWrite(IN2_FL, HIGH);
  analogWrite(PWM_FL, motorSpeed);

  digitalWrite(IN1_BL, LOW);
  digitalWrite(IN2_BL, HIGH);
  analogWrite(PWM_BL, motorSpeed);

  // right_spins_forward
  digitalWrite(IN1_FR, HIGH);
  digitalWrite(IN2_FR, LOW);
  analogWrite(PWM_FR, motorSpeed);

  digitalWrite(IN1_BR, HIGH);
  digitalWrite(IN2_BR, LOW);
  analogWrite(PWM_BR, motorSpeed);
}

void stopAll() {
  analogWrite(PWM_FL, 0);
  analogWrite(PWM_BL, 0);
  analogWrite(PWM_FR, 0);
  analogWrite(PWM_BR, 0);
}

// ultrasonic_read
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}
