void setup() {
  	board_init();
}
void loop() {
  	digitalWrite(15, LOW);
  	digitalWrite(2, HIGH);
  	digitalWrite(18, HIGH);
  	delay(3000);
  	digitalWrite(15, LOW);
  	digitalWrite(2, LOW);
  	digitalWrite(18, HIGH);
  	delay(1000);
  	digitalWrite(15, HIGH);
  	digitalWrite(2, LOW);
  	digitalWrite(18, HIGH);
  	for(int acr_iter = 0; acr_iter < 6; acr_iter++){
    	tone(19, 1500, 20);
    	delay(500);

  }
  	for(int acr_iter2 = 0; acr_iter2 < 3; acr_iter2++){
    	digitalWrite(15, HIGH);
    	digitalWrite(2, HIGH);
    	digitalWrite(18, HIGH);
    	delay(300);
    	digitalWrite(15, HIGH);
    	digitalWrite(2, LOW);
    	digitalWrite(18, HIGH);
    	delay(300);

  }
  	digitalWrite(15, LOW);
  	digitalWrite(2, LOW);
  	digitalWrite(18, HIGH);
  	delay(1000);
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
