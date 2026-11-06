/*
  Робототехнический контроллер «Кибернетический»: два привода «в зеркало»

  Привод 1 и привод 2 качаются в противоположные стороны одновременно.
  Обе цели отправляются одним пакетом SYNC WRITE (bus.syncMoveTo), поэтому приводы
  стартуют без задержки. Приводам заранее назначьте ID 1 и 2 (пример Ardu8_ST3215_SetID).

  Плата в Arduino IDE: ESP32 Wrover Module. Кабель USB-C — в НИЖНИЙ разъём (ESP32).
  Нужна библиотека FeetechSTS. DIP3 = ON, DIP4 = OFF, привод в разъём XP2 или XP3.
  Нужно питание XT60/DC, напряжение должно подходить приводу.
  ⚠ Пример ещё не проверен на контроллере.
*/
#include <FeetechSTS.h>

// ---------- Светодиоды, кнопки и пищалка ESP32 ----------
// Светодиоды горят от LOW. Слева направо на плате: LED4, LED3, LED2, LED1.
const int LED1 = 27, LED2 = 14, LED3 = 12, LED4 = 13;
const int RGB_R = 15, RGB_G = 2, RGB_B = 18;
// Кнопки дают HIGH при нажатии. Слева направо на плате: BUT4, BUT3, BUT2, BUT1.
const int BUT1 = 34, BUT2 = 35, BUT3 = 25, BUT4 = 26;
const int BUZZER = 19;

void setRGB(bool r, bool g, bool b) {
  digitalWrite(RGB_R, r ? LOW : HIGH);
  digitalWrite(RGB_G, g ? LOW : HIGH);
  digitalWrite(RGB_B, b ? LOW : HIGH);
}

void boardBegin() {
  int leds[] = {LED1, LED2, LED3, LED4, RGB_R, RGB_G, RGB_B};
  for (int p : leds) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }  // всё погасить
  int buttons[] = {BUT1, BUT2, BUT3, BUT4};
  for (int p : buttons) pinMode(p, INPUT);
  pinMode(BUZZER, OUTPUT);
}

FeetechBus bus;
FeetechServo servo1(bus, 1);
FeetechServo servo2(bus, 2);
const uint8_t ids[2] = {1, 2};

void moveBoth(float deg1, float deg2) {
  int16_t pos[2] = {FeetechServo::degToPos(deg1), FeetechServo::degToPos(deg2)};
  bus.syncMoveTo(ids, pos, 2, 2000, 50);   // одна команда для обоих приводов
}

void setup() {
  boardBegin();
  bus.begin(Serial2, 1000000, 33, 32);
  servo1.setMode(STS_MODE_POSITION);
  servo2.setMode(STS_MODE_POSITION);
  servo1.hold();
  servo2.hold();
}

void loop() {
  moveBoth(135, 225);    // в разные стороны от середины (180°)
  delay(1000);
  moveBoth(225, 135);
  delay(1000);
}
