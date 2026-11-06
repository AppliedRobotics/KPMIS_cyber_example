int now = 0;
int prev = 0;
int count = 0;
void setup() {
  	board_init();
  	Serial.begin(115200);
  	Serial.println("Счётчик нажатий: BUT1 — +1, BUT4 — сброс");
}
void loop() {
  	now = (int)digitalRead(34);
  	if (((now == 1) && (prev == 0))) {
      	count++;
      	if ((count > 15)) {
          	count = (int)0;
      }
      	tone(19, 2000, 30);
      	Serial.println(String("Счётчик: ") + String(count));
  }
  	prev = (int)now;
  	if (digitalRead(26)) {
      	count = (int)0;
      	Serial.println("Сброс");
  }
  	digitalWrite(27, ((count & 1) == 0));
  	digitalWrite(14, ((count & 2) == 0));
  	digitalWrite(12, ((count & 4) == 0));
  	digitalWrite(13, ((count & 8) == 0));
  	delay(20);
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
