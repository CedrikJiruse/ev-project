#include <Arduino.h>

// esp32 pins
#define throttlePin 26
#define steeringPin 13
#define leftMotorPin 22
#define rightMotorPin 23
#define speedPin 12
#define brakePin 33
#define motorLowPin 35
#define motorHighPin 34

// motor range
int pwmMin = 0, pwmMax = 4095;

// pwm settings
int pwmFreq = 10000;
int pwmBitRes = 12;
int leftMotorChannel = 1;
int rightMotorChannel = 2;

volatile int microSpeed = 0;
volatile int prev_time = 0;

void setup() {
  Serial.begin(115200);

  pinMode(speedPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(brakePin, INPUT);
  pinMode(motorLowPin, INPUT);
  pinMode(motorHighPin, INPUT);
  pinMode(steeringPin, INPUT);

  pinMode(leftMotorPin, OUTPUT);
  pinMode(rightMotorPin, OUTPUT);

  // setup pwm
  // analogWrite does not exist for the ESP32, ledcx is the replacement
  ledcSetup(leftMotorChannel, pwmFreq, pwmBitRes);
  ledcAttachPin(leftMotorPin, leftMotorChannel);
  ledcSetup(rightMotorChannel, pwmFreq, pwmBitRes);
  ledcAttachPin(rightMotorPin, rightMotorChannel);

  attachInterrupt(speedPin, rising, RISING);
}

void loop() {
  int motorPWM;
  int motorHigh = digitalRead(motorHighPin);
  int motorLow = digitalRead(motorLowPin);
  int brake = digitalRead(brakePin);

  // set the duty cycle based on throttle position and sensors
  if (motorHigh == HIGH && motorLow == LOW && brake == LOW) {
    motorPWM = pwmMax;
  }
  if (brake == LOW && motorLow == LOW && motorHigh == LOW) {
    motorPWM = map(analogRead(throttlePin), 0, 4095, pwmMin, pwmMax);
  }

  // differential
  int steering = analogRead(steeringPin);
  int leftPWM;
  int rightPWM;

  if (steering < 2047) { // going left, rightPWM HIGH
    rightPWM = motorPWM;
    leftPWM = map(steering, 0, 2047, pwmMin, motorPWM);
  }
  else { // going right, leftPWM HIGH
    leftPWM = motorPWM;
    rightPWM = map(steering, 2048, 4095, motorPWM, pwmMin);
  }

  // no power to both motors if brake/m.low is pressed
  if (motorLow == HIGH || brake == HIGH || motorPWM <= pwmMin) {
    leftPWM = 0;
    rightPWM = 0;
  }

  // write pwm values to both motors
  ledcWrite(leftMotorChannel, leftPWM);
  ledcWrite(rightMotorChannel, rightPWM);

  Serial.print("M-Low: ");
  Serial.print(motorLow);
  Serial.print("  M-High: ");
  Serial.print(motorHigh);
  Serial.print("   Brake: ");
  Serial.print(brake);
  Serial.print("  |  Steering: ");
  Serial.print(steering);
  Serial.print("\t");
  Serial.print("   PWM: ");
  Serial.print(motorPWM);
  Serial.print("\t");
  Serial.print("   L-PWM: ");
  Serial.print(leftPWM);
  Serial.print("\t");
  Serial.print("   R-PWM: ");
  Serial.print(rightPWM);
  Serial.print("\t");
  Serial.print("   Speed: ");
  Serial.print(microSpeed);
  Serial.print("\n");
}

void rising() {
  attachInterrupt(speedPin, falling, RISING);
  prev_time = micros();
}

void falling() {
  attachInterrupt(speedPin, rising, RISING);
  microSpeed = (micros() - prev_time);
}