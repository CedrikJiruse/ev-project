int throttlePin = 14;
int pwmOutPin = 5;

void setup()  { 
  Serial.begin(9600);
} 

void loop()  { 
  int potValue= analogRead(throttlePin);
  int pwmOutput = map(potValue, 0, 1023, 0 , 255);
  analogWrite(pwmOutPin, pwmOutput);
  Serial.print("Pot Value ");
  Serial.print(potValue);
  Serial.print(" | PWM Output ");
  Serial.println(pwmOutput);
  delay(100);
}
