#include <Arduino.h>

#define throttlePin 14
#define motorPin 23
#define speedPin 12
#define brakePin 33
#define motorLowPin 35
#define motorHighPin 34

int pwmChannel = 0;
int pwmMin = 700, pwmMax = 2400;
volatile int microSpeed = 0;
volatile int prev_time = 0;

void setup() {
  Serial.begin(115200);

  pinMode(speedPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(brakePin, INPUT);
  pinMode(motorLowPin, INPUT);
  pinMode(motorHighPin, INPUT);

  pinMode(motorPin, OUTPUT);

  ledcSetup(pwmChannel, 20000, 12);
  ledcAttachPin(motorPin, pwmChannel);

  attachInterrupt(speedPin, rising, RISING);
}

void loop() {
  int dutyCycle;
  int motorHigh = digitalRead(motorHighPin);
  int motorLow = digitalRead(motorLowPin);
  int brake = digitalRead(brakePin);

  if (motorHigh == LOW && motorLow == HIGH && brake == HIGH) {
    dutyCycle = pwmMax;
  }
  if (brake == HIGH && motorLow == HIGH && motorHigh == HIGH) {
    dutyCycle = map(analogRead(throttlePin), 0, 4095, pwmMin, pwmMax);
  }
  if (motorLow == LOW || brake == LOW) {
    dutyCycle = 0;
  }

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