/*
  Робототехнический контроллер «Кибернетический» (Ardu8) —
  модуль «Ультразвуковой дальномер» (Kirus-Ultrasonic V1.0, внутри HC-SR04).

  Модуль:
    TRIG — импульс 10 мкс запускает измерение.
    ECHO — импульс, длительность которого равна времени полёта звука
    туда и обратно: расстояние, см ≈ мкс / 58.
    Диапазон HC-SR04 — примерно 2…400 см.
    На модуле два 3-пиновых разъёма (1 GND, 2 5V, 3 сигнал):
      X2 — TRIG, X4 — ECHO.
    Зелёные светодиоды на модуле горят при НИЗКОМ уровне на своей линии:
      HL2 — ECHO (в покое горит), HL3 — TRIG (горит, когда скетч держит TRIG в LOW).

  Подключение к контроллеру:
    X2 (TRIG) → разъём D5 (XP19)
    X4 (ECHO) → разъём D6 (XP20)
    Нужны оба кабеля. Можно взять любые свободные D3..D7 / A0..A2 —
    поменяйте PIN_TRIG и PIN_ECHO. D2 не используйте, если есть DxlMaster.

  Перед запуском:
    - DIP5 = ON, DIP1 и DIP4 = OFF
    - Плата "Arduino Uno", верхний USB-C, монитор порта 115200
    - Можно открыть «Плоттер по последовательному соединению» — будет график.

  Что делает:
    - раз в 60 мс измеряет расстояние (медиана из 3 замеров отсекает выбросы);
    - выводит его в монитор порта в формате для плоттера;
    - LED1..LED4 — «шкала близости»: чем ближе препятствие, тем больше горит;
    - RGB: красный < 15 см, синий 15…40 см, зелёный > 40 см,
      все погашены — нет эха (дальше ~4 м или модуль не подключён).
*/

#include <SPI.h>

const uint8_t PIN_TRIG = 5;
const uint8_t PIN_ECHO = 6;

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

// ---------- Дальномер ----------
const uint32_t ECHO_TIMEOUT_US = 25000;   // ≈ 4,3 м, дальше — «нет эха»

// Одно измерение, см. 0 — эхо не пришло.
uint16_t measureOnce() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  uint32_t us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  return us / 58;
}

// Медиана из трёх замеров
uint16_t measureCm() {
  uint16_t a = measureOnce(); delay(10);
  uint16_t b = measureOnce(); delay(10);
  uint16_t c = measureOnce();
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

void setup() {
  boardInit();
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.begin(115200);
  Serial.println(F("Модуль «Ультразвуковой дальномер»: расстояние в см"));
}

void loop() {
  uint16_t cm = measureCm();

  Serial.print(F("Distance_cm:"));
  Serial.println(cm);

  bool echo = (cm > 0);
  led(LED1, echo && cm < 100);
  led(LED2, echo && cm < 50);
  led(LED3, echo && cm < 25);
  led(LED4, echo && cm < 10);

  if (!echo)        rgb(false, false, false);
  else if (cm < 15) rgb(true,  false, false);
  else if (cm < 40) rgb(false, false, true);
  else              rgb(false, true,  false);
  ledsUpdate();

  delay(40);
}
