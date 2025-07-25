#include <Arduino.h>

// Motor pin definitions
const int IN1_FL = 2;
const int IN2_FL = 3;
const int PWM_FL = 4;

const int IN1_BL = 5;
const int IN2_BL = 6;
const int PWM_BL = 7;

const int IN1_FR = 8;
const int IN2_FR = 9;
const int PWM_FR = 10;

const int IN1_BR = 11;
const int IN2_BR = 12;
const int PWM_BR = 13;

int motorSpeed = 200; // Range: 0–255

void setup() {
  // Initialize motor pins
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

  Serial.begin(9600);
}

void loop() {
  moveForward();
  delay(2000);

  stopAll();
  delay(1000);

  moveLeft();
  delay(2000);

  stopAll();
  delay(1000);

  moveRight();
  delay(2000);

  stopAll();
  delay(1000);
}

// Movement functions
void moveForward() {
  // All wheels forward
  digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW); analogWrite(PWM_FL, motorSpeed);
  digitalWrite(IN1_BL, HIGH); digitalWrite(IN2_BL, LOW); analogWrite(PWM_BL, motorSpeed);
  digitalWrite(IN1_FR, HIGH); digitalWrite(IN2_FR, LOW); analogWrite(PWM_FR, motorSpeed);
  digitalWrite(IN1_BR, HIGH); digitalWrite(IN2_BR, LOW); analogWrite(PWM_BR, motorSpeed);
}

void moveLeft() {
  // Left wheels backward, right wheels forward
  digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, HIGH); analogWrite(PWM_FL, motorSpeed);
  digitalWrite(IN1_BL, LOW); digitalWrite(IN2_BL, HIGH); analogWrite(PWM_BL, motorSpeed);
  digitalWrite(IN1_FR, HIGH); digitalWrite(IN2_FR, LOW); analogWrite(PWM_FR, motorSpeed);
  digitalWrite(IN1_BR, HIGH); digitalWrite(IN2_BR, LOW); analogWrite(PWM_BR, motorSpeed);
}

void moveRight() {
  // Left wheels forward, right wheels backward
  digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW); analogWrite(PWM_FL, motorSpeed);
  digitalWrite(IN1_BL, HIGH); digitalWrite(IN2_BL, LOW); analogWrite(PWM_BL, motorSpeed);
  digitalWrite(IN1_FR, LOW); digitalWrite(IN2_FR, HIGH); analogWrite(PWM_FR, motorSpeed);
  digitalWrite(IN1_BR, LOW); digitalWrite(IN2_BR, HIGH); analogWrite(PWM_BR, motorSpeed);
}

void stopAll() {
  analogWrite(PWM_FL, 0);
  analogWrite(PWM_BL, 0);
  analogWrite(PWM_FR, 0);
  analogWrite(PWM_BR, 0);
}
