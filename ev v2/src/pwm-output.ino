#include <Arduino.h>

#define throttlePin 26
#define steeringPin 13
#define leftMotorPin 22
#define rightMotorPin 23
#define speedPin 12
#define brakePin 33
#define motorLowPin 35
#define motorHighPin 34

int leftMotorChannel = 1;
int rightMotorChannel = 2;
int pwmMin = 50, pwmMax = 4095;

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

  ledcSetup(leftMotorChannel, 500, 12);
  ledcAttachPin(leftMotorPin, leftMotorChannel);
  ledcSetup(rightMotorChannel, 500, 12);
  ledcAttachPin(rightMotorPin, rightMotorChannel);

  attachInterrupt(speedPin, rising, RISING);
}

void loop() {
  // Motor
  int motorPWM;
  int motorHigh = digitalRead(motorHighPin);
  int motorLow = digitalRead(motorLowPin);
  int brake = digitalRead(brakePin);

  // set the duty cycle based on throttle position and sensors

  if (motorHigh == HIGH && motorLow == LOW && brake == LOW) {
    motorPWM = pwmMax;
  }
  if (brake == LOW && motorLow == LOW && motorHigh == LOW) {
    motorPWM = analogRead(throttlePin);
  }

  // // Steering
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

  if (motorLow == HIGH || brake == HIGH || motorPWM <= pwmMin) { // change BRAKE from high to low when connecting to car
    leftPWM = 0;                          // this is done so that we dont have to press the brake every time we want to test duty cycle
    rightPWM = 0;
  }

  // set pwm to motors
  ledcWrite(leftMotorChannel, leftPWM);
  ledcWrite(rightMotorChannel, rightPWM);

  Serial.print("M-Low: ");
  Serial.print(motorLow);
  Serial.print("   M-High: ");
  Serial.print(motorHigh);
  Serial.print("   Brake: ");
  Serial.print(brake);
  Serial.print("  |  Steering: ");
  Serial.print(steering);
  Serial.print("   PWM: ");
  Serial.print(motorPWM);
  Serial.print("   L-PWM: ");
  Serial.print(leftPWM);
  Serial.print("   R-PWM: ");
  Serial.print(rightPWM);
  Serial.print("  |  Speed: ");
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