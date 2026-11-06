/*
  Ardu8 — тест светодиодов (74HC595) и кнопок (74HC165) на стороне Arduino.

  Перед запуском:
    - DIP5 = ON  (иначе выходы 74HC595 выключены и светодиоды не горят)
    - DIP1 и DIP4 = OFF (чтобы Serial не конфликтовал с ESP32 и шиной DXL)
    - Плата в Arduino IDE: "Arduino Uno", USB — верхний USB-C на левом краю
    - Монитор порта: 115200

  Что делает:
    - по очереди зажигает LED1..LED7 (Q0..Q6), в мониторе пишет номер бита —
      так можно определить цвета каналов RGB (Q4, Q5, Q6);
    - при изменении состояния кнопок печатает байт из 74HC165 в двоичном виде.
*/

#include <SPI.h>

const uint8_t PIN_595_LATCH = 8;   // STCP 74HC595
const uint8_t PIN_165_LOAD  = 9;   // PL 74HC165 (активный LOW)
const uint8_t PIN_165_EN    = 10;  // SS: LOW — выход 165 подключён к MISO

const SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE0);

// Светодиоды горят при 0 в бите
void write595(uint8_t value) {
  digitalWrite(PIN_165_EN, HIGH);          // 165 не мешает на MISO
  SPI.beginTransaction(spiCfg);
  SPI.transfer(value);
  SPI.endTransaction();
  digitalWrite(PIN_595_LATCH, HIGH);       // защёлкнуть на выходы
  digitalWrite(PIN_595_LATCH, LOW);
}

uint8_t read165() {
  digitalWrite(PIN_165_LOAD, LOW);         // загрузить состояние кнопок
  delayMicroseconds(5);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, LOW);
  SPI.beginTransaction(spiCfg);
  uint8_t v = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite(PIN_165_EN, HIGH);
  return v;
}

void printBin(uint8_t v) {
  for (int8_t i = 7; i >= 0; i--) Serial.print((v >> i) & 1);
}

void setup() {
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_165_LOAD, OUTPUT);
  pinMode(PIN_165_EN, OUTPUT);
  digitalWrite(PIN_595_LATCH, LOW);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, HIGH);

  SPI.begin();
  Serial.begin(115200);
  Serial.println(F("Ardu8 test: LEDs + buttons"));

  write595(0xFF);  // всё погасить
}

uint8_t ledIndex = 0;
uint32_t lastLedMs = 0;
uint8_t lastButtons = 0xFF;  // заведомо «другое» значение, чтобы напечатать первое

void loop() {
  // Бегущий огонь: Q0..Q6, раз в 700 мс
  if (millis() - lastLedMs >= 700) {
    lastLedMs = millis();
    write595(~(1 << ledIndex));            // 0 только в нужном бите
    Serial.print(F("LED")); Serial.print(ledIndex + 1);
    Serial.print(F(" (Q")); Serial.print(ledIndex); Serial.println(F(")"));
    ledIndex = (ledIndex + 1) % 7;
  }

  // Кнопки: печатаем при изменении
  uint8_t b = read165();
  if (b != lastButtons) {
    lastButtons = b;
    Serial.print(F("165 = "));
    printBin(b);
    Serial.print(F("  нажаты бит(ы):"));
    bool any = false;
    for (uint8_t i = 0; i < 8; i++) {
      if (b & (1 << i)) { Serial.print(' '); Serial.print(i); any = true; }
    }
    if (!any) Serial.print(F(" нет"));
    Serial.println();
  }
  delay(10);
}
