#include <Arduino.h>
#include <analogWrite.h>

int throttlePin = 4;
int motorPin = 16;

// setting PWM properties
const int freq = 200000;
const int ledChannel = 0;
const int resolution = 12;

void setup() {
	ledcSetup(ledChannel, freq, resolution);
	ledcAttachPin(motorPin, ledChannel);

	Serial.begin(115200);

	ledcWrite(ledChannel, 4090);
	delay(500);
}

void loop() {

	int dutyCycle = map(analogRead(throttlePin), 0, 4095, 2050, 4095);

	ledcWrite(ledChannel, dutyCycle);

	Serial.print("Throttle In: ");
	Serial.println(dutyCycle);
}
