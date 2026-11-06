int note = 0;
int last_note = 0;
void setup() {
  	board_init();
}
void loop() {
  	note = (int)0;
  	if (digitalRead(26)) {
      	note = (int)523;
  } else if (digitalRead(25)) {
      	note = (int)659;
  } else if (digitalRead(35)) {
      	note = (int)784;
  } else if (digitalRead(34)) {
      	note = (int)1047;
  }
  	if ((note != last_note)) {
      	if ((note == 0)) {
          	noTone(19);
      } else {
          	tone(19, note, 0);
      }
      	last_note = (int)note;
  }
  	digitalWrite(27, (digitalRead(34) == 0));
  	digitalWrite(14, (digitalRead(35) == 0));
  	digitalWrite(12, (digitalRead(25) == 0));
  	digitalWrite(13, (digitalRead(26) == 0));
  	delay(10);
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
