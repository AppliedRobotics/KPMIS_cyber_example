int step = 0;
void setup() {
  	board_init();
}
void loop() {
  	leds_off();
  	switch (step){
  	case 0:{
  	    	digitalWrite(13, LOW);
  	break;
  }
  	case 1:{
  	    	digitalWrite(12, LOW);
  	break;
  }
  	case 2:{
  	    	digitalWrite(14, LOW);
  	break;
  }
  	case 3:{
  	    	digitalWrite(27, LOW);
  	break;
  }
  }
  	step++;
  	if ((step > 3)) {
      	step = (int)0;
  }
  	delay(200);
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
