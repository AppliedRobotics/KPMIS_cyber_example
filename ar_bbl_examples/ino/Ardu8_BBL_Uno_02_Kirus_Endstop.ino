int prev = 0;
int leds = 0;
int now = 0;
int count = 0;
int buttons = 0;
void setup() {
  	board_init();
  	pinMode(7, INPUT);
  	prev = (int)1;
  	Serial.begin(115200);
  	Serial.println("Модуль «Концевой выключатель»: нажимайте рычаг, BUT1 — сброс");
}
void loop() {
  	now = (int)digitalRead(7);
  	if (((now == 0) && (prev == 1))) {
      	count++;
      	if ((count > 15)) {
          	count = (int)0;
      }
      	Serial.println(String("Нажатий: ") + String(count));
  }
  	prev = (int)now;
  	buttons_read();
  	if (((buttons & 1) != 0)) {
      	count = (int)0;
  }
  	leds = (int)count;
  	if ((now == 0)) {
      	leds = (int)(leds | 32);
  } else {
      	leds = (int)(leds | 16);
  }
  	leds_update();
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
