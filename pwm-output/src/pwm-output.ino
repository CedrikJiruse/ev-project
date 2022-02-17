#include <Arduino.h>

#define throttlePin 26
#define steeringPin 13
#define motor1Pin 23
// #define motor2Pin 22
#define speedPin 12
#define brakePin 33
#define motorLowPin 35
#define motorHighPin 34

//Left & right diff
int LeftPWM;
// int RightPWM;

int leftMotorChannel = 1;
// int rightMotorChannel = 2;
int pwmMin = 0, pwmMax = 4095;
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

  pinMode(motor1Pin, OUTPUT);
  // pinMode(motor2Pin, OUTPUT);

  ledcSetup(leftMotorChannel, 50, 12);
  ledcAttachPin(motor1Pin, leftMotorChannel);
  // ledcSetup(rightMotorChannel, 50, 12);
  // ledcAttachPin(motor2Pin, rightMotorChannel);

  attachInterrupt(speedPin, rising, RISING);
}

void loop() {
  // Motor
  int motor1Duty;
  // int motor2Duty;
  int motorHigh = digitalRead(motorHighPin);
  int motorLow = digitalRead(motorLowPin);
  int brake = digitalRead(brakePin);

  // set the duty cycle based on throttle position and sensors
  if (motorHigh == HIGH && motorLow == LOW && brake == LOW) {
    motor1Duty = pwmMax;
  }
  if (brake == LOW && motorLow == LOW && motorHigh == LOW) {
    motor1Duty = map(analogRead(throttlePin), 0, 4095, pwmMin, pwmMax);
  }
  if (motorLow == HIGH || brake == HIGH) { // change BRAKE from high to low when connecting to car
    motor1Duty = 0;                        // this is done so that we dont have to press the brake every time we want to test duty cycle
  }

  // // Steering
  int steering = map(analogRead(steeringPin), 0, 4095, -2048, 2047);

  // // Differential
  // RightPWM = steering;

  // LeftPWM = (steering / 2048) * motor1Duty;


  ledcWrite(leftMotorChannel, motor1Duty);
  // ledcWrite(rightMotorChannel, RightPWM);

  Serial.print("Motor Low: ");
  Serial.print(motorLow);
  Serial.print(" Motor High: ");
  Serial.print(motorHigh);
  Serial.print(" Brake: ");
  Serial.print(brake);
  Serial.print(" Steering");
  Serial.print(steering);
  Serial.print("\t");
  // Serial.print("Left Duty Cycle: ");
  // Serial.print(LeftPWM);
  // Serial.print("       ");
  // Serial.print("Right Duty Cycle: ");
  // Serial.print(RightPWM);
  // Serial.print("       ");
  Serial.print("Speed: ");
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