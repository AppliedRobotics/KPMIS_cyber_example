/*
  Робототехнический контроллер «Кибернетический»: прямая задача кинематики

  Читает углы плеча (привод ID 2) и локтя (ID 3) манипулятора и вычисляет, где находится кисть
  в плоскости: x = L1·cos(θ1) + L2·cos(θ1 + θ2), y = L1·sin(θ1) + L2·sin(θ1 + θ2).
  Углы отсчитываются от горизонтали. Результат — в монитор порта (115200) раз в 0,5 с.
  Приводы отпущены: двигайте манипулятор рукой и проверяйте расчёт линейкой.
  Впишите L2 своего манипулятора. Знаки и нули углов зависят от калибровки — подберите
  ZERO1, ZERO2 и DIR1, DIR2 так, чтобы углы совпадали с транспортиром.

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
FeetechServo shoulder(bus, 2);    // плечо
FeetechServo elbow(bus, 3);       // локоть

const float L1 = 120;             // от оси плеча до оси локтя, мм
const float L2 = 110;             // от оси локтя до кисти, мм — измерьте линейкой
// В «позе модели» плечо вертикально (90° от горизонтали), предплечье горизонтально (−90° от плеча)
const float ZERO1 = 90, ZERO2 = -90;
const int DIR1 = 1, DIR2 = 1;     // направление отсчёта углов

void setup() {
  Serial.begin(115200);
  bus.begin(Serial2, 1000000, 33, 32);
  shoulder.torque(false);
  elbow.torque(false);
}

void loop() {
  // Угол привода: 180° (2048 шагов) — положение «позы модели»
  float t1 = ZERO1 + DIR1 * (shoulder.readDeg() - 180);
  float t2 = ZERO2 + DIR2 * (elbow.readDeg() - 180);

  float a1 = t1 * PI / 180, a2 = (t1 + t2) * PI / 180;   // в радианы
  float x = L1 * cos(a1) + L2 * cos(a2);
  float y = L1 * sin(a1) + L2 * sin(a2);

  Serial.print("θ1 = "); Serial.print(t1, 1);
  Serial.print("°  θ2 = "); Serial.print(t2, 1);
  Serial.print("°   x = "); Serial.print(x, 0);
  Serial.print(" мм  y = "); Serial.print(y, 0); Serial.println(" мм");
  delay(500);
}
