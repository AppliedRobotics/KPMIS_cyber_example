int leds = 0;
int s1 = 0;
int s2 = 0;
int buttons = 0;
void setup() {
  	board_init();
  	pinMode(3, INPUT);
  	pinMode(4, INPUT);
  	Serial.begin(115200);
}
void loop() {
  	s1 = (int)digitalRead(3);
  	s2 = (int)digitalRead(4);
  	leds = (int)(s1 + (s2 * 2));
  	if (((s1 == 0) && (s2 == 0))) {
      	leds = (int)(leds | 32);
  } else if (((s1 == 1) && (s2 == 1))) {
      	leds = (int)(leds | 64);
  } else {
      	leds = (int)(leds | 96);
  }
  	leds_update();
  	Serial.println(String("Line_S1:") + String(s1) + String(",Line_S2:") + String(s2));
  	delay(50);
}

void board_init() {
  	pinMode(8, OUTPUT);
  	pinMode(9, OUTPUT);
  	pinMode(10, OUTPUT);
  	pinMode(11, OUTPUT);
  	pinMode(13, OUTPUT);
  	pinMode(12, INPUT);
  	digitalWrite(9, HIGH);
  	digitalWrite(10, HIGH);
  	leds = (int)0;
  	leds_update();
}

void leds_update() {
  	for (int i = 7; i > -1; i -= 1){
     	digitalWrite(11, (((leds >> i) & 1) == 0));
    	digitalWrite(13, HIGH);
    	digitalWrite(13, LOW);
  }
  	digitalWrite(8, LOW);
  	digitalWrite(8, HIGH);
}

void buttons_read() {
  	buttons = (int)0;
  	digitalWrite(9, LOW);
  delayMicroseconds(5);
  	digitalWrite(9, HIGH);
  	digitalWrite(10, LOW);
  	for (int i = 7; i > -1; i -= 1){
     	buttons = (int)(buttons | (digitalRead(12) << i));
    	digitalWrite(13, HIGH);
    	digitalWrite(13, LOW);
  }
  	digitalWrite(10, HIGH);
}
