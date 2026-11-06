String cmd = "";
void setup() {
  	board_init();
  	Serial.begin(115200);
  	Serial.println("Команды: red, green, blue, white, off, beep");
}
void loop() {
  	if (Serial.available()) {
      	cmd = String(Serial.readStringUntil('\n'));
      	cmd.trim();
      	if ((cmd == "red")) {
          	leds_off();
          	digitalWrite(15, LOW);
          	digitalWrite(2, HIGH);
          	digitalWrite(18, HIGH);
          	Serial.println("Красный");
      } else if ((cmd == "green")) {
          	leds_off();
          	digitalWrite(15, HIGH);
          	digitalWrite(2, LOW);
          	digitalWrite(18, HIGH);
          	Serial.println("Зелёный");
      } else if ((cmd == "blue")) {
          	leds_off();
          	digitalWrite(15, HIGH);
          	digitalWrite(2, HIGH);
          	digitalWrite(18, LOW);
          	Serial.println("Синий");
      } else if ((cmd == "white")) {
          	digitalWrite(15, LOW);
          	digitalWrite(2, LOW);
          	digitalWrite(18, LOW);
          	Serial.println("Белый");
      } else if ((cmd == "off")) {
          	leds_off();
          	Serial.println("Выключено");
      } else if ((cmd == "beep")) {
          	tone(19, 1000, 200);
          	Serial.println("Бип!");
      } else {
          	Serial.println(String("Неизвестная команда: ") + String(cmd));
      }
  }
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
