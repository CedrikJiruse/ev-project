int throttlePin = 6;
int pwmOutPin = 14;

void setup()  // setup loop
{
  pinMode(pwmOutPin, OUTPUT); // declares pin 12 as output
  pinMode(throttlePin, INPUT);  // declares pin A0 as input
  Serial.begin(9600);
}

void loop() {
  int pwmOut = map(analogRead(throttlePin), 0, 1023, 0, 255);
  analogWrite(pwmOutPin, pwmOut);
  
  Serial.print("Pot Value ");
  Serial.println(analogRead(throttlePin));
}
