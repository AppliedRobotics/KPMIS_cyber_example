#include "DxlMaster2.h"
int left = 0;
int right = 0;
long duration = 0;
int distance = 0;
int buttons = 0;
bool running = false;
int s1 = 0;
int s2 = 0;
int leds = 0;
void setup() {
  	board_init();
  	pinMode(5, OUTPUT);
  	pinMode(6, INPUT);
  	pinMode(3, INPUT);
  	pinMode(4, INPUT);
  	pinMode(7, INPUT);
  	DxlMaster.begin(57600);
  	left = (int)0;
  	right = (int)0;
  	drive();
}
void loop() {
  	buttons_read();
  	if (((buttons & 1) != 0)) {
      	running = (bool)true;
  }
  	if (((buttons & 2) != 0)) {
      	running = (bool)false;
  }
  	measure_distance();
  	s1 = (int)digitalRead(3);
  	s2 = (int)digitalRead(4);
  	if (!running) {
      	left = (int)0;
      	right = (int)0;
      	leds = (int)16;
  } else if (((digitalRead(7) == 0) || ((distance > 0) && (distance < 15)))) {
      	left = (int)0;
      	right = (int)0;
      	leds = (int)72;
  } else if (((s1 == 1) && (s2 == 0))) {
      	left = (int)0;
      	right = (int)300;
      	leds = (int)33;
  } else if (((s1 == 0) && (s2 == 1))) {
      	left = (int)300;
      	right = (int)0;
      	leds = (int)34;
  } else {
      	left = (int)300;
      	right = (int)300;
      	leds = (int)35;
  }
  	drive();
  	leds_update();
  	delay(20);
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

void drive() {
  	DxlMaster.write(1, 71, 28, (uint16_t)left);
  	DxlMaster.write(1, 71, 30, (uint16_t)left);
  	DxlMaster.write(1, 72, 28, (uint16_t)right);
  	DxlMaster.write(1, 72, 30, (uint16_t)right);
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
