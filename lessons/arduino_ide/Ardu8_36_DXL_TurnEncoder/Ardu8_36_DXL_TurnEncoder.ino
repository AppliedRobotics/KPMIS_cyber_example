/*
  Робототехнический контроллер «Кибернетический»: поворот на угол по энкодерам

  BUT1 — разворот на месте на 90°, BUT2 — на 180°.
  Путь колеса при развороте = π × колея × угол / 360. Колёса крутятся в разные стороны.
  Впишите свои размеры робота и TICKS_PER_REV (см. пример Ardu8_35_DXL_Encoder).
  Левый мотор — в разъём 1 (модуль ID 71, M1), правый — в разъём 3 (модуль ID 72, M1).

  Плата в Arduino IDE: ESP32 Wrover Module. Кабель USB-C — в НИЖНИЙ разъём (ESP32).
  DIP3 = ON, DIP4 = OFF. Вкладка dxl.h — функции обмена по шине.
  Для вращения моторов нужно питание XT60/DC и выключатель в положении ON.
  ⚠ Пример ещё не проверен на контроллере.
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
const float TICKS_PER_REV = 360;   // тиков на оборот колеса
const float WHEEL_D = 65;          // диаметр колеса, мм
const float TRACK = 150;           // колея — расстояние между колёсами, мм
const float K = 1.0;               // поправочный коэффициент, подбирается опытом

int16_t readLeft() {
  int16_t v = 0;
  dxlRead16(LEFT_ID, REG_ENCODER, v);
  return v;
}

void turn(float degrees) {
  float path = PI * TRACK * degrees / 360;              // путь колеса, мм
  int ticks = path / (PI * WHEEL_D) * TICKS_PER_REV * K; // то же в тиках
  int16_t start = readLeft();
  while (true) {
    int done = abs((int16_t)(readLeft() - start));
    if (done >= ticks) break;
    int power = done < ticks - 100 ? 400 : 200;          // перед целью — медленнее
    dxlWrite16(LEFT_ID, REG_POWER, power);
    dxlWrite16(RIGHT_ID, REG_POWER, -power);             // правое колесо назад
  }
  dxlWrite16(LEFT_ID, REG_POWER, 0);
  dxlWrite16(RIGHT_ID, REG_POWER, 0);
}

void setup() {
  boardBegin();
  dxlBegin();
}

void loop() {
  if (digitalRead(BUT1) == HIGH) { turn(90);  delay(500); }
  if (digitalRead(BUT2) == HIGH) { turn(180); delay(500); }
}
