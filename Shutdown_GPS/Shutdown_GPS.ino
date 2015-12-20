int greenLED = 5;
int redLED = 6;

void setup() {
  Serial1.begin(4800);
  Serial1.write("$PSRF117,16*0B\r\n");
  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
}

void loop() {
  Serial1.write("$PSRF117,16*0B\r\n");
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  delay(100);
  digitalWrite(redLED, LOW);
  digitalWrite(greenLED, HIGH);
  delay(100);
}
