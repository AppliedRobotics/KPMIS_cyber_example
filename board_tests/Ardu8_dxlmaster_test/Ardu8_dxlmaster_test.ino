/*
  Ardu8 — проверка библиотеки DxlMaster2 на Arduino (ATmega328P).

  Нужна библиотека DxlMaster2 (Скетч → Подключить библиотеку → Добавить .ZIP).
  Монитор порта НЕ используется: Serial занят шиной DXL. Всё видно на светодиодах.

  Переключатели:
    - DIP5 = ON (светодиоды Arduino)
    - DIP4 = ON (Arduino на шине DXL), DIP3 = OFF, DIP1 = OFF
    - Прошивать удобнее при DIP4 = OFF, затем включить DIP4 — скетч сам
      начнёт опрашивать шину.
  Пин D2 занят библиотекой (пин направления), датчики к нему не подключать.

  Индикация (верхняя группа светодиодов):
    LED1 — модуль ID 71 отвечает на ping
    LED2 — модуль ID 72 отвечает на ping
    LED3 — у ID 71 прочитан правильный номер модели (71)
    LED4 — у ID 72 прочитан правильный номер модели (72)
    RGB  — зелёный: всё в порядке, красный: есть ошибки,
           синий: идёт команда моторам

  Кнопки (верхняя группа, слева направо):
    1 — все 4 канала по очереди вперёд (мощность 400, по 1 с)
    2 — все 4 канала по очереди назад
    3 — стоп всем
  Без моторов результат виден по светодиодам у драйверов справа.
*/

#include <SPI.h>
#include <DxlMaster2.h>

// ---------- Светодиоды и кнопки Ardu8 ----------
const uint8_t PIN_595_LATCH = 8;
const uint8_t PIN_165_LOAD  = 9;
const uint8_t PIN_165_EN    = 10;
const SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE0);

// Биты 74HC595 (0 = горит)
const uint8_t LED1 = 0, LED2 = 1, LED3 = 2, LED4 = 3;
const uint8_t RGB_BLUE = 4, RGB_GREEN = 5, RGB_RED = 6;

uint8_t ledState = 0;  // 1 в бите = светодиод включён

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

uint8_t buttons() {  // бит 0..3 = кнопки 1..4, 1 = нажата
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

// ---------- Модули моторов ----------
const uint8_t REG_MODEL    = 0;   // uint16
const uint8_t REG_M1_POWER = 28;  // int16 -1023..1023
const uint8_t REG_M2_POWER = 30;  // int16

DynamixelDevice drv71(71);
DynamixelDevice drv72(72);

bool checkModule(DynamixelDevice &d, uint8_t id, uint8_t ledPing, uint8_t ledModel) {
  bool pingOk = (d.ping() == DYN_STATUS_OK);
  uint16_t model = 0;
  bool modelOk = pingOk && (d.read(REG_MODEL, model) == DYN_STATUS_OK) && (model == id);
  led(ledPing, pingOk);
  led(ledModel, modelOk);
  return pingOk && modelOk;
}

void setPower(DynamixelDevice &d, uint8_t reg, int16_t power) {
  d.write(reg, power);
}

void stopAll() {
  setPower(drv71, REG_M1_POWER, 0);
  setPower(drv71, REG_M2_POWER, 0);
  setPower(drv72, REG_M1_POWER, 0);
  setPower(drv72, REG_M2_POWER, 0);
}

void runSequence(int16_t power) {
  rgb(false, false, true);
  ledsUpdate();
  DynamixelDevice *dev[4] = {&drv71, &drv71, &drv72, &drv72};
  uint8_t reg[4] = {REG_M1_POWER, REG_M2_POWER, REG_M1_POWER, REG_M2_POWER};
  for (uint8_t i = 0; i < 4; i++) {
    setPower(*dev[i], reg[i], power);
    delay(1000);
    setPower(*dev[i], reg[i], 0);
    delay(200);
  }
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

  DxlMaster.begin(57600);
  delay(300);
}

void loop() {
  bool ok71 = checkModule(drv71, 71, LED1, LED3);
  bool ok72 = checkModule(drv72, 72, LED2, LED4);
  bool allOk = ok71 && ok72;
  rgb(!allOk, allOk, false);
  ledsUpdate();

  uint8_t b = buttons();
  if (b & 0x01) runSequence(400);
  else if (b & 0x02) runSequence(-400);
  else if (b & 0x04) stopAll();

  delay(100);
}
