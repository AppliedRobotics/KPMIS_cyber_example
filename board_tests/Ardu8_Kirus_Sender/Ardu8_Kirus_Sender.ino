/*
  Ardu8 — Arduino читает датчики Kirus и передаёт их в ESP32 по UART.
  Пара к скетчу для ESP32: Ardu8_esp_Kirus_Receiver.

  Подключение датчиков — как в Ardu8_Kirus_All:
    Kirus-Line        X4 (S1) → D3,   X2 (S2) → D4
    Kirus-Ultrasonic  X2 (TRIG) → D5, X4 (ECHO) → D6
    Kirus-uSW                   → D7

  Переключатели:
    - DIP2 = ON  (Arduino TX → ESP32 RX, IO22)
    - DIP1 = OFF (обратный канал не нужен, и прошивка Arduino не мешает)
    - DIP4 = OFF, DIP3 — как угодно, DIP5 = ON

  Плата "Arduino Uno", верхний USB-C.
  Те же строки видны и в мониторе порта Arduino (115200).

  Формат строки (10 раз в секунду):
    K,<S1>,<S2>,<расстояние_см>,<uSW>\n
    S1/S2: 1 = чёрное; расстояние 0 = нет эха; uSW: 1 = нажат.
    Пример: K,0,1,37,0
*/

#include <SPI.h>

const uint8_t PIN_LINE_S1 = 3;
const uint8_t PIN_LINE_S2 = 4;
const uint8_t PIN_TRIG    = 5;
const uint8_t PIN_ECHO    = 6;
const uint8_t PIN_USW     = 7;

// ---------- Светодиоды Ardu8 ----------
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

uint16_t distanceCm() {
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, 25000UL) / 58;
}

void setup() {
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_165_LOAD, OUTPUT);
  pinMode(PIN_165_EN, OUTPUT);
  digitalWrite(PIN_595_LATCH, LOW);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, HIGH);
  SPI.begin();
  ledsUpdate();

  pinMode(PIN_LINE_S1, INPUT);
  pinMode(PIN_LINE_S2, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_USW, INPUT);

  Serial.begin(115200);
}

bool blink = false;

void loop() {
  bool s1      = digitalRead(PIN_LINE_S1) == HIGH;
  bool s2      = digitalRead(PIN_LINE_S2) == HIGH;
  bool pressed = digitalRead(PIN_USW) == LOW;
  uint16_t cm  = distanceCm();

  Serial.print(F("K,"));
  Serial.print(s1);      Serial.print(',');
  Serial.print(s2);      Serial.print(',');
  Serial.print(cm);      Serial.print(',');
  Serial.println(pressed);   // println даёт \r\n — приёмник это учитывает

  led(LED1, s1);
  led(LED2, s2);
  led(LED3, pressed);
  blink = !blink;
  led(RGB_BLUE, blink);      // мигает синим — идёт передача
  ledsUpdate();

  delay(100);
}
