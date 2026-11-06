/*
  Робототехнический контроллер «Кибернетический»: обучение показом

  Вал отпущен — поворачивайте его рукой. BUT1 записывает текущее положение (до 5 штук).
  BUT2 — воспроизвести записанные положения по очереди. BUT4 — стереть запись.
  Число записанных положений показывают светодиоды LED1–LED4 (5 — горит RGB).

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
int points[5];                    // записанные положения, шаги 0..4095
int count = 0;

void showCount() {
  int leds[] = {LED1, LED2, LED3, LED4};
  for (int i = 0; i < 4; i++) digitalWrite(leds[i], i < count ? LOW : HIGH);
  setRGB(false, count == 5, false);
}

void setup() {
  boardBegin();
  Serial.begin(115200);
  bus.begin(Serial2, 1000000, 33, 32);
  servo.setMode(STS_MODE_POSITION);
  servo.torque(false);            // вал свободен — можно крутить рукой
}

void loop() {
  if (digitalRead(BUT1) == HIGH && count < 5) {
    points[count] = servo.readPosition();
    Serial.print("Записано положение "); Serial.println(points[count]);
    count++;
    tone(BUZZER, 1500, 50);
    delay(300);
  }
  if (digitalRead(BUT2) == HIGH && count > 0) {
    servo.torque(true);
    for (int i = 0; i < count; i++) {
      servo.moveTo(points[i], 1500, 30);
      delay(1200);                // ждём, пока вал доедет
    }
    servo.torque(false);
  }
  if (digitalRead(BUT4) == HIGH) { count = 0; delay(300); }
  showCount();
}
