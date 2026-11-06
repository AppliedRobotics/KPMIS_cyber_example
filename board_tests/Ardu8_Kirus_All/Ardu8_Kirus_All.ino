/*
  Робототехнический контроллер «Кибернетический» (Ardu8) —
  три модуля датчиков одновременно.

  Подключение (3-пиновые кабели, контакты модулей: 1 GND, 2 5V, 3 сигнал):
    «Датчик линии»               X4 (S1)   → D3 (XP17)
                                 X2 (S2)   → D4 (XP18)
    «Ультразвуковой дальномер»   X2 (TRIG) → D5 (XP19)
                                 X4 (ECHO) → D6 (XP20)
    «Концевой выключатель»       X2 или X4 → D7 (XP21)
  D2 оставлен свободным: его занимает библиотека DxlMaster (моторы).
  Свободными остаются A0..A2 — туда можно добавить ещё модули,
  например второй концевой выключатель.

  Перед запуском:
    - DIP5 = ON, DIP1 и DIP4 = OFF
    - Плата "Arduino Uno", верхний USB-C
    - Откройте «Плоттер по последовательному соединению» (115200):
      все величины видны на одном графике.

  Индикация:
    LED1 — S1 видит чёрное, LED2 — S2 видит чёрное,
    LED3 — концевой выключатель нажат, LED4 — препятствие ближе 20 см.
    RGB: красный — нажат концевой выключатель или препятствие ближе 10 см,
    иначе зелёный.
*/

#include <SPI.h>

const uint8_t PIN_LINE_S1 = 3;
const uint8_t PIN_LINE_S2 = 4;
const uint8_t PIN_TRIG    = 5;
const uint8_t PIN_ECHO    = 6;
const uint8_t PIN_USW     = 7;

// ---------- Светодиоды контроллера ----------
const uint8_t PIN_595_LATCH = 8;
const uint8_t PIN_165_LOAD  = 9;
const uint8_t PIN_165_EN    = 10;
const SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE0);

const uint8_t LED1 = 0, LED2 = 1, LED3 = 2, LED4 = 3;
const uint8_t RGB_BLUE = 4, RGB_GREEN = 5, RGB_RED = 6;

uint8_t ledState = 0;

void ledsUpdate() {
  digitalWrite(PIN_165_EN, HIGH);
  SPI.beginTransaction(spiCfg);
  SPI.transfer((uint8_t)~ledState);
  SPI.endTransaction();
  digitalWrite(PIN_595_LATCH, HIGH);
  digitalWrite(PIN_595_LATCH, LOW);
}

void led(uint8_t bit, bool on) {
  if (on) ledState |= (1 << bit);
  else    ledState &= ~(1 << bit);
}

void rgb(bool r, bool g, bool b) {
  led(RGB_RED, r); led(RGB_GREEN, g); led(RGB_BLUE, b);
}

void boardInit() {
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_165_LOAD, OUTPUT);
  pinMode(PIN_165_EN, OUTPUT);
  digitalWrite(PIN_595_LATCH, LOW);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, HIGH);
  SPI.begin();
  ledsUpdate();
}

// ---------- Модули датчиков ----------
uint16_t distanceCm() {               // 0 = нет эха
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, 25000UL) / 58;
}

void setup() {
  boardInit();
  pinMode(PIN_LINE_S1, INPUT);
  pinMode(PIN_LINE_S2, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_USW, INPUT);
  Serial.begin(115200);
}

void loop() {
  bool s1      = digitalRead(PIN_LINE_S1) == HIGH;   // чёрное
  bool s2      = digitalRead(PIN_LINE_S2) == HIGH;
  bool pressed = digitalRead(PIN_USW) == LOW;
  uint16_t cm  = distanceCm();
  bool near20  = cm > 0 && cm < 20;
  bool near10  = cm > 0 && cm < 10;

  // Формат «имя:значение» понимает плоттер Arduino IDE.
  // Логические сигналы умножены, чтобы их было видно рядом с расстоянием.
  Serial.print(F("Distance_cm:")); Serial.print(cm);
  Serial.print(F(" Line_S1:"));    Serial.print(s1 ? 10 : 0);
  Serial.print(F(" Line_S2:"));    Serial.print(s2 ? 20 : 0);
  Serial.print(F(" Endstop:"));    Serial.println(pressed ? 30 : 0);

  led(LED1, s1);
  led(LED2, s2);
  led(LED3, pressed);
  led(LED4, near20);
  bool alarm = pressed || near10;
  rgb(alarm, !alarm, false);
  ledsUpdate();

  delay(60);   // HC-SR04: не чаще ~1 раза в 50–60 мс
}
