void setup() {
  	pinMode(27, OUTPUT);
}
void loop() {
  	digitalWrite(27, LOW);
  	delay(500);
  	digitalWrite(27, HIGH);
  	delay(500);
}
