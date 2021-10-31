int throttlePin = 6;
int motorPin = 14;

void setup()
{
  pinMode(throttlePin, INPUT);
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int pwmOut = map(analogRead(throttlePin), 0, 1023, 0, 255);
  analogWrite(motorPin, pwmOut);
  
  Serial.print("Motor Out: ");
  Serial.println(pwmOut);
}