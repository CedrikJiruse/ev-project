#include <Arduino.h>

#define throttlePin 26
#define steeringPin 13
#define motorPin 23
#define speedPin 12
#define brakePin 33
#define motorLowPin 35
#define motorHighPin 34

int pwmChannel = 0;
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

  pinMode(motorPin, OUTPUT);

  ledcSetup(pwmChannel, 50, 12);
  ledcAttachPin(motorPin, pwmChannel);

  attachInterrupt(speedPin, rising, RISING);
}

void loop() {
  // Motor
  int dutyCycle;
  int motorHigh = digitalRead(motorHighPin);
  int motorLow = digitalRead(motorLowPin);
  int brake = digitalRead(brakePin);

  // set the duty cycle based on throttle position and sensors
  if (motorHigh == HIGH && motorLow == LOW && brake == LOW) {
    dutyCycle = pwmMax;
  }
  if (brake == LOW && motorLow == LOW && motorHigh == LOW) {
    dutyCycle = map(analogRead(throttlePin), 0, 4095, pwmMin, pwmMax);
  }
  if (motorLow == HIGH || brake == HIGH) { // change BRAKE from high to low when connecting to car
    dutyCycle = 0;                         // this is done so that we dont have to press the brake every time we want to test duty cycle
  }

  // Steering
  int steering = analogRead(steeringPin);

  // Differential


  ledcWrite(pwmChannel, dutyCycle);

  Serial.print("Motor Low: ");
  Serial.print(motorLow);
  Serial.print("\t");
  Serial.print("Motor High: ");
  Serial.print(motorHigh);
  Serial.print("\t");
  Serial.print("Brake: ");
  Serial.print(brake);
  Serial.print("\t");
  Serial.print("Steering");
  // Serial.print(steering);
  Serial.print("\t");
  Serial.print("Duty Cycle: ");
  Serial.print(dutyCycle);
  Serial.print("       ");
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