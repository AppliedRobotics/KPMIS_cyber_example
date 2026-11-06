/*
  Робототехнический контроллер «Кибернетический»: анимация из кадров

  Анимация хранится в массиве: каждый байт — один кадр (какие светодиоды горят).
  Добавьте свои кадры в массив frames.

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF, DIP5 = ON (светодиоды).
*/

#include <SPI.h>

const int LATCH_595 = 8;

// Кадры: 1 в бите = горит. Биты: 1 LED1, 2 LED2, 4 LED3, 8 LED4, 16 синий, 32 зелёный, 64 красный
byte frames[] = {
  0b0001, 0b0011, 0b0111, 0b1111,      // заполнение
  0b1110, 0b1100, 0b1000, 0b0000,      // убывание
  0b1001, 0b0110, 0b1001, 0b0110,      // «мигалка»
  0b1111 | 64, 0b1111 | 32, 0b1111 | 16 // все зелёные + разные цвета RGB
};
const int FRAME_COUNT = sizeof(frames);  // сколько кадров в массиве

void showLeds(byte leds) {
  digitalWrite(LATCH_595, LOW);
  SPI.transfer(~leds);
  digitalWrite(LATCH_595, HIGH);
}

void setup() {
  pinMode(LATCH_595, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  SPI.begin();
}

void loop() {
  for (int i = 0; i < FRAME_COUNT; i++) {
    showLeds(frames[i]);
    delay(250);
  }
}
