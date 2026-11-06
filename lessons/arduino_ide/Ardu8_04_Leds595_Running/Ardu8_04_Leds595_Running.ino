/*
  Робототехнический контроллер «Кибернетический»: бегущий огонь на сдвиговом регистре

  Светодиоды LED1–LED4 загораются по одному: сначала слева направо, затем обратно.
  Байт отправляется в регистр 74HC595 по шине SPI, импульс на D8 выводит его на светодиоды.
  У регистра обратная логика: 0 — светодиод горит, 1 — погашен.

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF, DIP5 = ON (светодиоды).
*/

#include <SPI.h>

const int LATCH_595 = 8;    // защёлка регистра

void showByte(byte b) {
  digitalWrite(LATCH_595, LOW);
  SPI.transfer(b);                 // отправить 8 бит
  digitalWrite(LATCH_595, HIGH);   // вывести их на светодиоды
}

void setup() {
  pinMode(LATCH_595, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);          // D10 = HIGH: регистр кнопок не мешает
  SPI.begin();
  showByte(0xFF);                  // 0xFF = 11111111 — всё погашено
}

void loop() {
  for (int n = 0; n < 4; n++) {    // вперёд: LED1, LED2, LED3, LED4
    showByte(~(1 << n));           // 1 << n — единица в бите n, ~ — инверсия
    delay(200);
  }
  for (int n = 2; n > 0; n--) {    // обратно: LED3, LED2
    showByte(~(1 << n));
    delay(200);
  }
}
