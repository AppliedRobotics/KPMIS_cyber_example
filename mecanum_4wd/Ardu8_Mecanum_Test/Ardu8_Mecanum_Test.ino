/*
  Робототехнический контроллер «Кибернетический» (Ardu8)
  Тест мобильной платформы 4WD с меканум-колёсами: датчики, колёса, движения.

  Монитор порта не используется: Serial занят шиной моторов (DXL).
  Всё видно на светодиодах и RGB верхней группы (Arduino).

  Нужна библиотека DxlMaster2.

  Переключатели:
    - DIP5 = ON  (светодиоды Arduino)
    - DIP4 = ON  (Arduino управляет шиной моторов), DIP1 = OFF, DIP3 = OFF
    - Прошивать при DIP4 = OFF, потом включить DIP4.
  Питание моторов — от XT60/DC, выключатель питания в ON.
  ПЕРЕД ТЕСТОМ КОЛЁС ПОДНИМИТЕ ПЛАТФОРМУ, чтобы колёса не касались стола.

  Подключение:
    Моторы (порты справа, сверху вниз):
      FL переднее левое  — порт 3 (ID 72, M1)
      FR переднее правое — порт 2 (ID 71, M2)
      RL заднее левое    — порт 4 (ID 72, M2)
      RR заднее правое   — порт 1 (ID 71, M1)
    Датчики линии (слева направо поперёк платформы):
      D3 левый внешний, D6 левый внутренний, D4 правый внутренний, D7 правый внешний
    Ультразвуковой дальномер: TRIG — A0, ECHO — A1
    Концевые выключатели: левый — D5, правый — A2
    D2 занят библиотекой DxlMaster2.

  ---------------- РЕЖИМ ДАТЧИКОВ (по умолчанию) ----------------
    LED1..LED4 — датчики линии слева направо (горит = под датчиком чёрное)
    RGB — расстояние: красный < 15 см, жёлтый 15–40 см, зелёный > 40 см,
          не горит — нет эха (больше ~4 м или датчик не отвечает)
    Нажат концевой: левый — синий, правый — фиолетовый, оба — белый
    Нет связи с модулями моторов — все LED1..LED4 мигают

  ---------------- КНОПКИ (верхняя группа, слева направо) ----------------
    1 — тест колёс: по очереди FL, FR, RL, RR «вперёд» по 1,5 с.
        Горит светодиод колеса (LED1=FL, LED2=FR, LED3=RL, LED4=RR), RGB синий.
        Затем пауза 0,5 с (RGB погашен) и результат энкодера на 2 с:
          RGB зелёный — счёт растёт;
          RGB красный — счёт убывает;
          RGB не горит, мигает светодиод колеса — энкодер не изменился;
          мигают все LED1..LED4 и красный — ошибка чтения по шине.
        Колесо, которое крутится НАЗАД, — поставьте ему -1 в WHEEL_SIGN.
    2 — тест движений (после исправления знаков):
        вперёд, назад, влево, вправо, разворот против часовой, по часовой — по 1 с.
        LED1 вперёд, LED2 назад, LED3 влево, LED4 вправо,
        LED1+LED2 разворот против часовой, LED3+LED4 по часовой.
    3 — стоп.
    Любой концевой выключатель во время теста — аварийная остановка.
*/

#include <SPI.h>
#include <DxlMaster2.h>

// ---------- Настройки ----------
const int16_t TEST_POWER = 300;        // мощность для тестов, 0..1023

// Знак «вперёд» для каждого колеса: FL, FR, RL, RR.
// Проверено на железе: правый борт (FR, RR) крутится в обратную сторону.
const int8_t WHEEL_SIGN[4] = {+1, -1, +1, -1};

// Если при движении «влево» платформа едет вправо — поставьте -1.
const int8_t STRAFE_SIGN = +1;

// ---------- Пины датчиков ----------
const uint8_t LINE_PINS[4] = {3, 6, 4, 7};  // слева направо
const uint8_t PIN_TRIG  = A0;
const uint8_t PIN_ECHO  = A1;
const uint8_t PIN_END_L = 5;
const uint8_t PIN_END_R = A2;

// ---------- Светодиоды и кнопки (74HC595 / 74HC165) ----------
const uint8_t PIN_595_LATCH = 8;
const uint8_t PIN_165_LOAD  = 9;
const uint8_t PIN_165_EN    = 10;
const SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE0);

const uint8_t RGB_BLUE = 4, RGB_GREEN = 5, RGB_RED = 6;
uint8_t ledState = 0;  // 1 в бите = светодиод горит

void ledsUpdate() {
  digitalWrite(PIN_165_EN, HIGH);
  SPI.beginTransaction(spiCfg);
  SPI.transfer((uint8_t)~ledState);  // на плате 0 = горит
  SPI.endTransaction();
  digitalWrite(PIN_595_LATCH, HIGH);
  digitalWrite(PIN_595_LATCH, LOW);
}

void setLeds4(uint8_t mask) {  // биты 0..3 = LED1..LED4
  ledState = (ledState & 0xF0) | (mask & 0x0F);
}

void rgb(bool r, bool g, bool b) {
  bitWrite(ledState, RGB_RED, r);
  bitWrite(ledState, RGB_GREEN, g);
  bitWrite(ledState, RGB_BLUE, b);
}

uint8_t readButtons() {  // биты 0..3 = кнопки 1..4, 1 = нажата
  digitalWrite(PIN_165_LOAD, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, LOW);
  SPI.beginTransaction(spiCfg);
  uint8_t v = SPI.transfer(0);
  SPI.endTransaction();
  digitalWrite(PIN_165_EN, HIGH);
  return v & 0x0F;
}

// ---------- Датчики ----------
bool endLeft()  { return digitalRead(PIN_END_L) == LOW; }  // нажат = LOW
bool endRight() { return digitalRead(PIN_END_R) == LOW; }

uint8_t lineMask() {  // бит 0..3 = датчики слева направо, 1 = чёрное
  uint8_t m = 0;
  for (uint8_t i = 0; i < 4; i++)
    if (digitalRead(LINE_PINS[i]) == HIGH) m |= (1 << i);
  return m;
}

int distanceCm() {  // -1 — нет эха
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (us == 0) return -1;
  return us / 58;
}

// ---------- Моторы ----------
const uint8_t REG_MAX_POWER    = 24;
const uint8_t REG_ENCODER_MODE = 27;
const uint8_t REG_M1_POWER = 28, REG_M2_POWER = 30;
const uint8_t REG_M1_ENC   = 32, REG_M2_ENC   = 36;

DynamixelDevice drv71(71);  // правый борт
DynamixelDevice drv72(72);  // левый борт

struct Wheel {
  DynamixelDevice *dev;
  uint8_t powerReg;
  uint8_t encReg;
};

enum { FL = 0, FR = 1, RL = 2, RR = 3 };
Wheel wheels[4] = {
  {&drv72, REG_M1_POWER, REG_M1_ENC},  // FL — порт 3
  {&drv71, REG_M2_POWER, REG_M2_ENC},  // FR — порт 2
  {&drv72, REG_M2_POWER, REG_M2_ENC},  // RL — порт 4
  {&drv71, REG_M1_POWER, REG_M1_ENC},  // RR — порт 1
};

void wheelPower(uint8_t w, int16_t p) {
  p = constrain(p, -1023, 1023) * WHEEL_SIGN[w];
  wheels[w].dev->write(wheels[w].powerReg, p);
}

// false — не удалось прочитать энкодер по шине
// Показания уже приведены к «вперёд» (умножены на WHEEL_SIGN):
// у правого борта мотор и энкодер стоят зеркально.
bool wheelEncoder(uint8_t w, int16_t &e) {
  e = 0;
  bool ok = wheels[w].dev->read(wheels[w].encReg, e) == DYN_STATUS_OK;
  e *= WHEEL_SIGN[w];
  return ok;
}

// Мигать светодиодами mask (LED1..LED4) и RGB заданное время
bool blinkSafe(uint8_t mask, bool r, bool g, bool b, uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    bool on = ((millis() - t0) / 150) % 2 == 0;
    setLeds4(on ? mask : 0);
    rgb(r && on, g && on, b && on);
    ledsUpdate();
    if (!waitSafe(30)) return false;
  }
  return true;
}

void stopAll() {
  for (uint8_t w = 0; w < 4; w++) wheelPower(w, 0);
}

// Меканум, ролики «X» при взгляде сверху.
// vx — вперёд, vy — влево, rot — поворот против часовой.
void drive(int16_t vx, int16_t vy, int16_t rot) {
  vy *= STRAFE_SIGN;
  wheelPower(FL, vx - vy - rot);
  wheelPower(FR, vx + vy + rot);
  wheelPower(RL, vx + vy - rot);
  wheelPower(RR, vx - vy + rot);
}

bool modulesOk() {
  return drv71.ping() == DYN_STATUS_OK && drv72.ping() == DYN_STATUS_OK;
}

// Ждать ms, проверяя аварийную остановку. false — тест прерван.
bool waitSafe(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (endLeft() || endRight() || (readButtons() & 0x04)) {
      stopAll();
      rgb(true, false, false);
      setLeds4(0x0F);
      ledsUpdate();
      delay(1000);
      return false;
    }
    delay(20);
  }
  return true;
}

void waitRelease() {
  while (readButtons()) delay(20);
}

// ---------- Тесты ----------
void testWheels() {
  for (uint8_t w = 0; w < 4; w++) {
    int16_t e0, e1;
    bool readOk = wheelEncoder(w, e0);
    setLeds4(1 << w);
    rgb(false, false, true);                   // синий — колесо крутится
    ledsUpdate();
    wheelPower(w, TEST_POWER);
    bool ok = waitSafe(1500);
    wheelPower(w, 0);
    if (!ok) return;
    setLeds4(1 << w);
    rgb(false, false, false);                  // пауза: RGB погашен
    ledsUpdate();
    if (!waitSafe(500)) return;
    readOk = wheelEncoder(w, e1) && readOk;
    int16_t d = e1 - e0;
    // Результат 2 с, светодиод колеса мигает:
    if (!readOk)     { if (!blinkSafe(0x0F, true, false, false, 2000)) return; } // ошибка чтения: мигают все LED и красный
    else if (d > 5)  { setLeds4(1 << w); rgb(false, true, false); ledsUpdate(); if (!waitSafe(2000)) return; }  // зелёный — растёт
    else if (d < -5) { setLeds4(1 << w); rgb(true, false, false); ledsUpdate(); if (!waitSafe(2000)) return; }  // красный — убывает
    else             { if (!blinkSafe(1 << w, false, false, false, 2000)) return; } // не изменился: RGB не горит, мигает LED колеса
  }
}

void testMoves() {
  struct Move { int16_t vx, vy, rot; uint8_t leds; };
  const int16_t P = TEST_POWER;
  const Move moves[6] = {
    { P,  0,  0, 0b0001},  // вперёд
    {-P,  0,  0, 0b0010},  // назад
    { 0,  P,  0, 0b0100},  // влево
    { 0, -P,  0, 0b1000},  // вправо
    { 0,  0,  P, 0b0011},  // разворот против часовой
    { 0,  0, -P, 0b1100},  // разворот по часовой
  };
  for (uint8_t i = 0; i < 6; i++) {
    setLeds4(moves[i].leds);
    rgb(false, false, true);
    ledsUpdate();
    drive(moves[i].vx, moves[i].vy, moves[i].rot);
    bool ok = waitSafe(1000);
    stopAll();
    if (!ok) return;
    if (!waitSafe(500)) return;
  }
}

// ---------- Основная программа ----------
void setup() {
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_165_LOAD, OUTPUT);
  pinMode(PIN_165_EN, OUTPUT);
  digitalWrite(PIN_595_LATCH, LOW);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, HIGH);
  SPI.begin();
  ledsUpdate();

  for (uint8_t i = 0; i < 4; i++) pinMode(LINE_PINS[i], INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_END_L, INPUT);
  pinMode(PIN_END_R, INPUT);

  DxlMaster.begin(57600);
  delay(300);
  DynamixelDevice *mods[2] = {&drv71, &drv72};
  for (uint8_t i = 0; i < 2; i++) {
    mods[i]->write(REG_ENCODER_MODE, (uint8_t)1);   // энкодер AB
    mods[i]->write(REG_MAX_POWER, (uint16_t)1023);
  }
  stopAll();
}

uint32_t lastPing = 0;
bool linkOk = true;

void loop() {
  // Проверка связи с модулями моторов раз в секунду
  if (millis() - lastPing > 1000) {
    lastPing = millis();
    linkOk = modulesOk();
  }

  uint8_t b = readButtons();
  if (b & 0x01) { waitRelease(); if (linkOk) testWheels(); stopAll(); }
  else if (b & 0x02) { waitRelease(); if (linkOk) testMoves(); stopAll(); }
  else if (b & 0x04) { stopAll(); }

  // Режим датчиков
  if (linkOk) setLeds4(lineMask());
  else        setLeds4((millis() / 250) % 2 ? 0x0F : 0x00);  // мигание: нет связи

  bool l = endLeft(), r = endRight();
  if (l && r)  rgb(true, true, true);
  else if (l)  rgb(false, false, true);
  else if (r)  rgb(true, false, true);
  else {
    int d = distanceCm();
    if (d < 0)       rgb(false, false, false);
    else if (d < 15) rgb(true, false, false);
    else if (d < 40) rgb(true, true, false);
    else             rgb(false, true, false);
  }
  ledsUpdate();
  delay(30);
}
