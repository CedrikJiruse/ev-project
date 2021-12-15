#include <Arduino.h>

int throttlePin = 32;
int motorPin = 23;
int speedPin = 12;

volatile int pwm_value = 0;
volatile int prev_time = 0;

// setting PWM properties
const int freq = 20000, ledChannel = 0, resolution = 12;

void setup() {
  Serial.begin(115200);

  pinMode(speedPin, INPUT); 
  pinMode(throttlePin, INPUT); 
  pinMode(motorPin, OUTPUT); 

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(motorPin, ledChannel);
  ledcWrite(ledChannel, 4090);
  delay(500);

  attachInterrupt(speedPin, rising, RISING);
}

void loop() {
  int dutyCycle = map(analogRead(throttlePin), 0, 4095, 0, 4095);
  ledcWrite(ledChannel, dutyCycle);

  Serial.print(dutyCycle);
  Serial.print("\t");
  Serial.print(pwm_value);
  Serial.print("\n");
}

void rising() {
  attachInterrupt(speedPin, falling, RISING);
  prev_time = micros();
}

void falling() {
  attachInterrupt(speedPin, rising, RISING);
  pwm_value = (micros()-prev_time);
}