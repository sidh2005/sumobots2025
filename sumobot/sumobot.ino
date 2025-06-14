// ========== Motor Pin Setup ==========
// Left side (Motor A & B)
const int IN1_FL = 2;
const int IN2_FL = 3;
const int PWM_FL = 4;

const int IN1_BL = 5;
const int IN2_BL = 6;
const int PWM_BL = 7;

// Right side (Motor C & D)
const int IN1_FR = 8;
const int IN2_FR = 9;
const int PWM_FR = 10;

const int IN1_BR = 11;
const int IN2_BR = 12;
const int PWM_BR = 13;

// ========== Sensor Setup ==========
const int irPin = A0; // IR Sensor pin

// Ultrasonic
const int trigPins[3] = {A1, A3, A5};
const int echoPins[3] = {A2, A4, A6};
const int numSensors = 3;
const int detectionThreshold = 30; // cm

// ========== Movement Settings ==========
int motorSpeed = 200; // 0–255

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

  // IR Sensor
  pinMode(irPin, INPUT);

  // Ultrasonic
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  stopAll();
}

void loop() {
  if (digitalRead(irPin) == HIGH) {
    moveRight();
    delay(300);
    stopAll();
    return;
  }

  bool opponentDetected = false;
  for (int i = 0; i < numSensors; i++) {
    long dist = readUltrasonic(trigPins[i], echoPins[i]);
    if (dist > 0 && dist < detectionThreshold) {
      opponentDetected = true;
      break;
    }
  }

  if (opponentDetected) {
    moveForward();
  } else {
    stopAll();
  }

  delay(100);
}

// ======= Movement Functions =======

void moveForward() {
  // FL
  digitalWrite(IN1_FL, HIGH);
  digitalWrite(IN2_FL, LOW);
  analogWrite(PWM_FL, motorSpeed);

  // BL
  digitalWrite(IN1_BL, HIGH);
  digitalWrite(IN2_BL, LOW);
  analogWrite(PWM_BL, motorSpeed);

  // FR
  digitalWrite(IN1_FR, HIGH);
  digitalWrite(IN2_FR, LOW);
  analogWrite(PWM_FR, motorSpeed);

  // BR
  digitalWrite(IN1_BR, HIGH);
  digitalWrite(IN2_BR, LOW);
  analogWrite(PWM_BR, motorSpeed);
}

void moveRight() {
  // Left wheels forward
  digitalWrite(IN1_FL, HIGH);
  digitalWrite(IN2_FL, LOW);
  analogWrite(PWM_FL, motorSpeed);

  digitalWrite(IN1_BL, HIGH);
  digitalWrite(IN2_BL, LOW);
  analogWrite(PWM_BL, motorSpeed);

  // Right wheels backward
  digitalWrite(IN1_FR, LOW);
  digitalWrite(IN2_FR, HIGH);
  analogWrite(PWM_FR, motorSpeed);

  digitalWrite(IN1_BR, LOW);
  digitalWrite(IN2_BR, HIGH);
  analogWrite(PWM_BR, motorSpeed);
}

void stopAll() {
  analogWrite(PWM_FL, 0);
  analogWrite(PWM_BL, 0);
  analogWrite(PWM_FR, 0);
  analogWrite(PWM_BR, 0);
}

// ======= Ultrasonic Function =======

long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration * 0.034 / 2;
}
