void setup() {
  	board_init();
}
void loop() {
  	for (int i = 0; i < 255; i += 5){
     	analogWrite(27, (255 - i));
    	analogWrite(18, (255 - i));
    	delay(15);
  }
  	for (int i = 255; i > 0; i -= 5){
     	analogWrite(27, (255 - i));
    	analogWrite(18, (255 - i));
    	delay(15);
  }
  	delay(300);
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
