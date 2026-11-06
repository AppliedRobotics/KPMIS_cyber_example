long t_start = 0;
long reaction = 0;
void setup() {
  	board_init();
  	Serial.begin(115200);
  	Serial.println("Игра «Реакция»: нажмите BUT1, как только загорится зелёный");
}
void loop() {
  	leds_off();
  	Serial.println("Приготовьтесь...");
  	delay(random(1500, 4000));
  	if (digitalRead(34)) {
      	digitalWrite(15, LOW);
      	digitalWrite(2, HIGH);
      	digitalWrite(18, HIGH);
      	tone(19, 200, 500);
      	Serial.println("Фальстарт!");
  } else {
      	digitalWrite(15, HIGH);
      	digitalWrite(2, LOW);
      	digitalWrite(18, HIGH);
      	t_start = (long)millis();
      	while (!digitalRead(34)){
      	}
      	reaction = (long)(millis() - t_start);
      	digitalWrite(15, HIGH);
      	digitalWrite(2, HIGH);
      	digitalWrite(18, HIGH);
      	tone(19, 2000, 100);
      	Serial.println(String("Время реакции: ") + String(reaction) + String(" мс"));
      	digitalWrite(27, LOW);
      	if ((reaction < 450)) {
          	digitalWrite(14, LOW);
      }
      	if ((reaction < 350)) {
          	digitalWrite(12, LOW);
      }
      	if ((reaction < 250)) {
          	digitalWrite(13, LOW);
      }
  }
  	while (digitalRead(34)){
    	delay(10);
  	}
  	delay(2000);
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
