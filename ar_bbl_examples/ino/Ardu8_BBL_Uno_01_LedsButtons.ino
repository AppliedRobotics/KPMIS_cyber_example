int leds = 0;
int buttons = 0;
int last_buttons = 0;
void setup() {
  	board_init();
  	Serial.begin(115200);
  	Serial.println("Нажимайте кнопки верхней группы");
}
void loop() {
  	buttons_read();
  	leds = (int)buttons;
  	if ((buttons == 0)) {
      	leds = (int)(leds | 64);
  } else {
      	leds = (int)(leds | 32);
  }
  	leds_update();
  	if ((buttons != last_buttons)) {
      	Serial.println(String("Кнопки: ") + String(buttons));
      	last_buttons = (int)buttons;
  }
  	delay(20);
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
