/*
  Робототехнический контроллер «Кибернетический»: режим скорости и PID-регулятор

  Мотор в разъёме 1 вращается с заданной скоростью (регистр 40, тиков за 100 мс).
  Регулятор модуля сам подбирает мощность. В плоттер (115200) выводятся заданная
  и текущая скорость (регистр 34). Кнопки: BUT1 — P = 4, BUT2 — P = 16 (по умолчанию),
  BUT3 — P = 64. Притормозите колесо рукой и посмотрите, как регулятор держит скорость.

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

const uint8_t ID = 71;
const uint8_t REG_GOAL_SPEED = 40, REG_PRESENT_SPEED = 34, REG_P = 47;
const int GOAL = 30;              // заданная скорость, тиков за 100 мс

void setP(uint8_t p) {
  dxlWrite8(ID, REG_P, p);
  dxlWrite16(ID, REG_GOAL_SPEED, GOAL);   // запись скорости включает режим скорости
}

void setup() {
  boardBegin();
  Serial.begin(115200);
  dxlBegin();
  setP(16);
}

void loop() {
  if (digitalRead(BUT1) == HIGH) setP(4);
  if (digitalRead(BUT2) == HIGH) setP(16);
  if (digitalRead(BUT3) == HIGH) setP(64);
  if (digitalRead(BUT4) == HIGH) dxlWrite16(ID, REG_GOAL_SPEED, 0);   // стоп

  int16_t speed = 0;
  dxlRead16(ID, REG_PRESENT_SPEED, speed);
  Serial.print("goal:"); Serial.print(GOAL);
  Serial.print(" speed:"); Serial.println(speed);
  delay(50);
}
