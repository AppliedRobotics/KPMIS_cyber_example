long duration = 0;
int distance = 0;
int s1 = 0;
int s2 = 0;
int endstop = 0;
int leds = 0;
int buttons = 0;
void setup() {
  	board_init();
  	pinMode(5, OUTPUT);
  	pinMode(6, INPUT);
  	pinMode(3, INPUT);
  	pinMode(4, INPUT);
  	pinMode(7, INPUT);
  	Serial.begin(115200);
}
void loop() {
  	measure_distance();
  	s1 = (int)digitalRead(3);
  	s2 = (int)digitalRead(4);
  	endstop = (int)(1 - digitalRead(7));
  	leds = (int)((s1 + (s2 * 2)) + (endstop * 8));
  	if ((distance < 15)) {
      	leds = (int)(leds | 64);
  } else if ((distance < 30)) {
      	leds = (int)(leds | 96);
  } else {
      	leds = (int)(leds | 32);
  }
  	leds_update();
  	Serial.println(String("Distance_cm:") + String(distance) + String(",Line_S1:") + String(s1) + String(",Line_S2:") + String(s2) + String(",Endstop:") + String(endstop));
  	delay(50);
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
