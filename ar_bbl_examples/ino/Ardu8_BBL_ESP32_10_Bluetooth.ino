#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
String cmd = "";
int now = 0;
int prev = 0;
void setup() {
  	board_init();
  	SerialBT.begin("Kiber-BT");
}
void loop() {
  	if (SerialBT.available()) {
      	cmd = String(SerialBT.readString());
      	cmd.trim();
      	if ((cmd == "1")) {
          	digitalWrite(27, LOW);
          	SerialBT.println("LED1 включён");
      } else if ((cmd == "0")) {
          	digitalWrite(27, HIGH);
          	SerialBT.println("LED1 выключен");
      } else if ((cmd == "r")) {
          	digitalWrite(15, LOW);
          	digitalWrite(2, HIGH);
          	digitalWrite(18, HIGH);
          	SerialBT.println("Красный");
      } else if ((cmd == "g")) {
          	digitalWrite(15, HIGH);
          	digitalWrite(2, LOW);
          	digitalWrite(18, HIGH);
          	SerialBT.println("Зелёный");
      } else if ((cmd == "b")) {
          	digitalWrite(15, HIGH);
          	digitalWrite(2, HIGH);
          	digitalWrite(18, LOW);
          	SerialBT.println("Синий");
      } else if ((cmd == "x")) {
          	leds_off();
          	SerialBT.println("Всё выключено");
      } else if ((cmd == "s")) {
          	tone(19, 1200, 200);
          	SerialBT.println("Бип");
      } else {
          	SerialBT.println(String("Не понял: ") + String(cmd));
      }
  }
  	now = (int)digitalRead(34);
  	if (((now == 1) && (prev == 0))) {
      	SerialBT.println("BUT1");
  }
  	prev = (int)now;
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
