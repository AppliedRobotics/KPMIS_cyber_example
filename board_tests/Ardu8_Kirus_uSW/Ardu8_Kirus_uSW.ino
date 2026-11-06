/*
  Робототехнический контроллер «Кибернетический» (Ardu8) —
  модуль «Концевой выключатель» (Kirus-uSW V1.0).

  Модуль:
    Микропереключатель DM1-01C с подтяжкой 10 кОм к 5 В.
    Не нажат — SIG = HIGH, нажат — SIG = LOW (замыкается на GND).
    Зелёный светодиод HL1 на модуле горит, пока он нажат, красный HL2 — питание.
    На модуле два одинаковых 3-пиновых разъёма (X2, X4): 1 GND, 2 5V, 3 SIG.
    Можно подключить любой из них.

  Подключение к контроллеру:
    3-пиновый кабель от X2 (или X4) модуля → разъём D7 (XP21) контроллера.
    (Подойдёт любой из D3..D7 или A0..A2 — поменяйте PIN_USW.
     D2 не используйте, если в проекте есть DxlMaster.)

  Перед запуском:
    - DIP5 = ON (светодиоды Arduino), DIP1 и DIP4 = OFF
    - Плата в Arduino IDE: "Arduino Uno", USB — верхний USB-C на левом краю
    - Монитор порта: 115200

  Что делает:
    - LED1 горит, пока выключатель нажат;
    - считает нажатия (с защитой от дребезга), счётчик выводится
      в монитор порта и в двоичном виде на LED2..LED4 (0..7);
    - RGB: зелёный — отпущен, красный — нажат;
    - кнопка BUT1 (верхняя группа, левая) сбрасывает счётчик.
*/

#include <SPI.h>

const uint8_t PIN_USW = 7;          // сигнал концевого выключателя

// ---------- Светодиоды и кнопки контроллера ----------
const uint8_t PIN_595_LATCH = 8;
const uint8_t PIN_165_LOAD  = 9;
const uint8_t PIN_165_EN    = 10;
const SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE0);

const uint8_t LED1 = 0, LED2 = 1, LED3 = 2, LED4 = 3;
const uint8_t RGB_BLUE = 4, RGB_GREEN = 5, RGB_RED = 6;

uint8_t ledState = 0;  // 1 в бите = светодиод включён

void ledsUpdate() {
  digitalWrite(PIN_165_EN, HIGH);
  SPI.beginTransaction(spiCfg);
  SPI.transfer((uint8_t)~ledState);   // на плате 0 = горит
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

// ---------- Концевой выключатель с защитой от дребезга ----------
const uint16_t DEBOUNCE_MS = 20;

bool stablePressed = false;   // подтверждённое состояние
bool lastRaw = false;
uint32_t rawChangedMs = 0;

// Возвращает true один раз в момент нажатия
bool uswUpdate() {
  bool raw = (digitalRead(PIN_USW) == LOW);   // LOW = нажат
  if (raw != lastRaw) {
    lastRaw = raw;
    rawChangedMs = millis();
  }
  if (raw != stablePressed && millis() - rawChangedMs >= DEBOUNCE_MS) {
    stablePressed = raw;
    return stablePressed;                     // фронт нажатия
  }
  return false;
}

uint16_t pressCount = 0;

void setup() {
  boardInit();
  pinMode(PIN_USW, INPUT);   // подтяжка уже есть на модуле
  Serial.begin(115200);
  Serial.println(F("Модуль «Концевой выключатель»: нажмите на рычаг"));
}

void loop() {
  if (uswUpdate()) {
    pressCount++;
    Serial.print(F("Нажатие #"));
    Serial.println(pressCount);
  }

  if (buttons() & 0x01) {        // BUT1 — сброс
    if (pressCount) Serial.println(F("Счётчик сброшен"));
    pressCount = 0;
  }

  led(LED1, stablePressed);
  led(LED2, pressCount & 0x01);
  led(LED3, pressCount & 0x02);
  led(LED4, pressCount & 0x04);
  rgb(stablePressed, !stablePressed, false);
  ledsUpdate();

  delay(2);
}
