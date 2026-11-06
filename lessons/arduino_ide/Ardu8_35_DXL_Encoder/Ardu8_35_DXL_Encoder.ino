/*
  Робототехнический контроллер «Кибернетический»: энкодеры и путь

  Печатает показания энкодеров обоих моторов (регистр 32) в монитор порта (115200).
  BUT1 — проехать вперёд N оборотов колеса и остановиться.
  Сначала определите TICKS_PER_REV опытом: поверните колесо рукой на 10 оборотов,
  посмотрите, на сколько изменилось число, и разделите на 10.
  Левый мотор — в разъём 1 (модуль ID 71, M1), правый — в разъём 3 (модуль ID 72, M1).

  Плата в Arduino IDE: ESP32 Wrover Module. Кабель USB-C — в НИЖНИЙ разъём (ESP32).
  DIP3 = ON, DIP4 = OFF. Вкладка dxl.h — функции обмена по шине.
  Для вращения моторов нужно питание XT60/DC и выключатель в положении ON.
*/
#include "dxl.h"

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

const uint8_t LEFT_ID = 71, RIGHT_ID = 72;
const uint8_t REG_POWER = 28, REG_ENCODER = 32;
const int TICKS_PER_REV = 360;    // тиков на оборот колеса — замените своим числом
const int REVOLUTIONS = 3;        // сколько оборотов проехать

int16_t readLeft() {
  int16_t v = 0;
  dxlRead16(LEFT_ID, REG_ENCODER, v);
  return v;
}

void setup() {
  boardBegin();
  Serial.begin(115200);
  dxlBegin();
}

void loop() {
  int16_t left = 0, right = 0;
  dxlRead16(LEFT_ID, REG_ENCODER, left);
  dxlRead16(RIGHT_ID, REG_ENCODER, right);
  Serial.print("left:"); Serial.print(left);
  Serial.print(" right:"); Serial.println(right);

  if (digitalRead(BUT1) == HIGH) {
    int16_t start = readLeft();
    int16_t target = REVOLUTIONS * TICKS_PER_REV;
    dxlWrite16(LEFT_ID, REG_POWER, 400);
    dxlWrite16(RIGHT_ID, REG_POWER, 400);
    setRGB(false, true, false);
    // Разница int16_t правильно считается даже при переполнении счётчика
    while (abs((int16_t)(readLeft() - start)) < target) delay(5);
    dxlWrite16(LEFT_ID, REG_POWER, 0);
    dxlWrite16(RIGHT_ID, REG_POWER, 0);
    setRGB(false, false, false);
  }
  delay(100);
}
