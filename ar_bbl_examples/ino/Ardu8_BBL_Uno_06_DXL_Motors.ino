#include "DxlMaster2.h"
int left = 0;
int right = 0;
int buttons = 0;
int leds = 0;
void setup() {
  	board_init();
  	DxlMaster.begin(57600);
  	left = (int)0;
  	right = (int)0;
  	drive();
}
void loop() {
  	buttons_read();
  	if (((buttons & 1) != 0)) {
      	left = (int)400;
      	right = (int)400;
      	leds = (int)33;
  } else if (((buttons & 2) != 0)) {
      	left = (int)-400;
      	right = (int)-400;
      	leds = (int)66;
  } else if (((buttons & 4) != 0)) {
      	left = (int)-300;
      	right = (int)300;
      	leds = (int)100;
  } else if (((buttons & 8) != 0)) {
      	left = (int)300;
      	right = (int)-300;
      	leds = (int)104;
  } else {
      	left = (int)0;
      	right = (int)0;
      	leds = (int)16;
  }
  	drive();
  	leds_update();
  	delay(50);
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
