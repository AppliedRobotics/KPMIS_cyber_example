/*
  Робототехнический контроллер «Кибернетический»: режим колеса — конвейер

  Привод работает как мотор: непрерывно вращается со скоростью 1500 шагов/с.
  BUT1 — пуск, BUT2 — обратное направление, BUT4 — стоп.
  Режим сохраняется в приводе, поэтому перед работой в режиме позиции его нужно вернуть
  (в других примерах это делает servo.setMode(STS_MODE_POSITION)).

  Плата в Arduino IDE: ESP32 Wrover Module. Кабель USB-C — в НИЖНИЙ разъём (ESP32).
  Нужна библиотека FeetechSTS. DIP3 = ON, DIP4 = OFF, привод в разъём XP2 или XP3.
  Нужно питание XT60/DC, напряжение должно подходить приводу.
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
FeetechServo servo(bus, 1);
int speed = 1500;                 // шагов в секунду, знак — направление

void setup() {
  boardBegin();
  bus.begin(Serial2, 1000000, 33, 32);
  servo.setMode(STS_MODE_WHEEL);  // режим колеса
  servo.torque(true);
  servo.setSpeed(0);
}

void loop() {
  if (digitalRead(BUT1) == HIGH) { servo.setSpeed(speed, 50); setRGB(false, true, false); }
  if (digitalRead(BUT2) == HIGH) { speed = -speed; servo.setSpeed(speed, 50); delay(300); }
  if (digitalRead(BUT4) == HIGH) { servo.setSpeed(0, 50); setRGB(true, false, false); }
  delay(20);
}
