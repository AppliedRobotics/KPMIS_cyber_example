/*
  Робототехнический контроллер «Кибернетический»: кнопки через сдвиговый регистр

  Кнопки BUT1–BUT4 (слева направо) читаются через регистр 74HC165.
  Нажатая кнопка зажигает светодиод под тем же номером, байт кнопок
  печатается в мониторе порта (115200) в двоичном виде.

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF, DIP5 = ON (светодиоды).
*/

#include <SPI.h>

const int LATCH_595 = 8;   // защёлка регистра светодиодов
const int LOAD_165  = 9;   // «фотография» кнопок
const int EN_165    = 10;  // LOW — регистр кнопок подключён к линии приёма

void showLeds(byte leds) {
  digitalWrite(LATCH_595, LOW);
  SPI.transfer(~leds);
  digitalWrite(LATCH_595, HIGH);
}

byte readButtons() {
  digitalWrite(LOAD_165, LOW);     // защёлкнуть состояние кнопок
  delayMicroseconds(5);
  digitalWrite(LOAD_165, HIGH);
  digitalWrite(EN_165, LOW);       // подключить регистр
  byte b = SPI.transfer(0);        // принять байт
  digitalWrite(EN_165, HIGH);      // отключить
  return b & 0x0F;                 // биты 0–3 — кнопки BUT1–BUT4, 1 = нажата
}

byte lastButtons = 255;

void setup() {
  pinMode(LATCH_595, OUTPUT);
  pinMode(LOAD_165, OUTPUT);
  pinMode(EN_165, OUTPUT);
  digitalWrite(LOAD_165, HIGH);
  digitalWrite(EN_165, HIGH);
  SPI.begin();
  Serial.begin(115200);
}

void loop() {
  byte b = readButtons();
  showLeds(b);                     // биты кнопок совпадают с битами LED1–LED4
  if (b != lastButtons) {          // печатаем только при изменении
    Serial.println(b, BIN);
    lastButtons = b;
  }
  delay(20);
}
