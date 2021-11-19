#include <Arduino.h>

int throttlePin = 4;
int motorPin = 16;
int speedPin = 15;

volatile int pwm_value = 0;
volatile int prev_time = 0;

// setting PWM properties
const int freq = 200000, ledChannel = 0, resolution = 12;

void setup() {
  Serial.begin(115200);

  pinMode(speedPin, INPUT); 

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(motorPin, ledChannel);
  ledcWrite(ledChannel, 4090);
  delay(500);

  attachInterrupt(speedPin, rising, RISING);
}

void loop() {
  int dutyCycle = map(analogRead(throttlePin), 0, 4095, 0, 4095);
  ledcWrite(ledChannel, dutyCycle);

  // Serial.print(dutyCycle);
  // Serial.print("\t");
}

void rising() {
  attachInterrupt(speedPin, falling, RISING);
  prev_time = micros();
}

void falling() {
  attachInterrupt(speedPin, rising, RISING);
  pwm_value = (micros()-prev_time);
  Serial.println(pwm_value);
}