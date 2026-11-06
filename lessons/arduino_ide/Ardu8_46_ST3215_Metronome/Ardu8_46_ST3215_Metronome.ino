/*
  Робототехнический контроллер «Кибернетический»: сервопривод — метроном

  Вал привода (ID 1) качается между 150° и 210°, как маятник метронома.
  BUT1 — быстрее, BUT2 — медленнее. Текущий темп печатается в монитор порта (115200).

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
FeetechServo servo(bus, 1);       // привод с ID 1
int pauseMs = 600;                // время одного качания

void setup() {
  boardBegin();
  Serial.begin(115200);
  bus.begin(Serial2, 1000000, 33, 32);   // шина: 1 Мбит/с, RX = IO33, TX = IO32
  servo.setMode(STS_MODE_POSITION);
  servo.hold();                   // держать текущий угол — вал не дёрнется при старте
}

void swing(float deg) {
  servo.moveToDeg(deg, 3000, 50); // угол, скорость (шагов/с), ускорение
  tone(BUZZER, 2000, 20);         // «тик»
  unsigned long t = millis();
  while (millis() - t < pauseMs) {        // во время паузы опрашиваем кнопки
    if (digitalRead(BUT1) == HIGH && pauseMs > 200)  { pauseMs -= 50; delay(150); Serial.println(pauseMs); }
    if (digitalRead(BUT2) == HIGH && pauseMs < 2000) { pauseMs += 50; delay(150); Serial.println(pauseMs); }
  }
}

void loop() {
  swing(150);
  swing(210);
}
