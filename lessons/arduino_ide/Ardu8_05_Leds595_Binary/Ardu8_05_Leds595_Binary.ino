/*
  Робототехнический контроллер «Кибернетический»: двоичный счётчик

  Числа от 0 до 15 показываются на LED1–LED4 в двоичном виде (LED1 — младший разряд)
  и печатаются в мониторе порта (115200).

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF, DIP5 = ON (светодиоды).
*/

#include <SPI.h>

const int LATCH_595 = 8;

void showLeds(byte leds) {         // 1 в бите = светодиод горит
  digitalWrite(LATCH_595, LOW);
  SPI.transfer(~leds);             // регистр горит от 0 — инвертируем
  digitalWrite(LATCH_595, HIGH);
}

void setup() {
  pinMode(LATCH_595, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  SPI.begin();
  Serial.begin(115200);
}

void loop() {
  for (int n = 0; n <= 15; n++) {
    showLeds(n);                   // число выводится прямо на 4 светодиода
    Serial.print(n);
    Serial.print(" = ");
    Serial.println(n, BIN);        // то же число в двоичном виде
    delay(500);
  }
}
