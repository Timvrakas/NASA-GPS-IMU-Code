int greenLED = 12;
int redLED = 6;

void setup() {
  Serial1.begin(4800);
  delay(2000);

  //Factory Reset
  //Serial1.write("$PSRF101,0,0,0,0,0,0,12,8*1C\r\n");

  //Cold Reset
  //Serial1.write("$PSRF101,0,0,0,0,0,0,12,4*10\r\n");

  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
}

void loop() {
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  delay(100);
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, HIGH);
  delay(100);
}
