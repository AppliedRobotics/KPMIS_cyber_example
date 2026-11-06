long duration = 0;
int distance = 0;
int leds = 0;
int buttons = 0;
void setup() {
  	board_init();
  	pinMode(5, OUTPUT);
  	pinMode(6, INPUT);
  	Serial.begin(115200);
}
void loop() {
  	measure_distance();
  	if ((distance < 10)) {
      	leds = (int)79;
  } else if ((distance < 20)) {
      	leds = (int)103;
  } else if ((distance < 40)) {
      	leds = (int)35;
  } else if ((distance < 80)) {
      	leds = (int)33;
  } else {
      	leds = (int)32;
  }
  	leds_update();
  	Serial.println(String("Distance_cm:") + String(distance));
  	delay(100);
}

void measure_distance() {
  	digitalWrite(5, LOW);
  delayMicroseconds(2);
  	digitalWrite(5, HIGH);
  delayMicroseconds(10);
  	digitalWrite(5, LOW);
  	duration = (long)pulseIn(6, HIGH);
  ;
  	distance = (int)(duration / 58);
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
