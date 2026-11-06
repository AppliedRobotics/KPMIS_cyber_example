/*
  Робототехнический контроллер «Кибернетический»: мощность мотора

  Кнопки ESP32: BUT1 — вперёд, BUT2 — назад (мощность 400 из 1023), без нажатия — стоп.
  Левый мотор — в разъём 1 (модуль ID 71, M1), правый — в разъём 3 (модуль ID 72, M1).
  Мощность записывается в регистр 28 (M1) числом от -1023 до 1023: знак — направление.

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
const uint8_t REG_POWER = 28;     // мощность M1
int lastPower = 1;                // чтобы первая команда точно отправилась

void setup() {
  boardBegin();
  Serial.begin(115200);
  dxlBegin();
}

void loop() {
  int power = 0;
  if (digitalRead(BUT1) == HIGH) power = 400;
  if (digitalRead(BUT2) == HIGH) power = -400;

  if (power != lastPower) {        // отправляем только при изменении
    dxlWrite16(LEFT_ID, REG_POWER, power);
    dxlWrite16(RIGHT_ID, REG_POWER, power);
    Serial.print("Мощность: ");
    Serial.println(power);
    lastPower = power;
  }
  setRGB(power < 0, power > 0, power == 0);
  delay(50);
}
