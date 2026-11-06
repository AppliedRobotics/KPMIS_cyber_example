long t1 = 0;
bool s1 = false;
long t2 = 0;
bool s2 = false;
void setup() {
  	board_init();
}
void loop() {
  	if (((millis() - t1) >= 500)) {
      	t1 = (long)millis();
      	s1 = (bool)!s1;
      	digitalWrite(27, s1);
  }
  	if (((millis() - t2) >= 130)) {
      	t2 = (long)millis();
      	s2 = (bool)!s2;
      	digitalWrite(13, s2);
  }
  	digitalWrite(2, (digitalRead(34) == 0));
}

void board_init() {
  	pinMode(34, INPUT);
  	pinMode(35, INPUT);
  	pinMode(25, INPUT);
  	pinMode(26, INPUT);
  	pinMode(27, OUTPUT);
  	pinMode(14, OUTPUT);
  	pinMode(12, OUTPUT);
  	pinMode(13, OUTPUT);
  	pinMode(15, OUTPUT);
  	pinMode(2, OUTPUT);
  	pinMode(18, OUTPUT);
  	pinMode(19, OUTPUT);
  	leds_off();
}

void leds_off() {
  	digitalWrite(27, HIGH);
  	digitalWrite(14, HIGH);
  	digitalWrite(12, HIGH);
  	digitalWrite(13, HIGH);
  	digitalWrite(15, HIGH);
  	digitalWrite(2, HIGH);
  	digitalWrite(18, HIGH);
}
