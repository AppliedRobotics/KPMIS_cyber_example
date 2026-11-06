/*
  Робототехнический контроллер «Кибернетический»: обратная связь и захват с ограничением усилия

  Раз в 200 мс в монитор порта (115200) выводятся угол, нагрузка, напряжение, температура и ток.
  BUT1 — «захват»: вал медленно поворачивается шагами по 2°, пока нагрузка не превысит порог,
  и останавливается — предмет держится, мотор не перегревается. BUT4 — отпустить вал.

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
const int LOAD_LIMIT = 300;       // порог нагрузки (0,1 % от максимума)

void grip() {
  servo.torque(true);
  float deg = servo.readDeg();
  setRGB(false, false, true);
  for (int i = 0; i < 60; i++) {         // не больше 120°
    deg += 2;
    servo.moveToDeg(deg, 500, 0);
    delay(60);
    int load = servo.readLoad();
    if (abs(load) > LOAD_LIMIT) {        // упёрлись в предмет
      Serial.println("Предмет зажат");
      setRGB(false, true, false);
      return;
    }
  }
  setRGB(true, false, false);            // ничего не нашли
}

void setup() {
  boardBegin();
  Serial.begin(115200);
  bus.begin(Serial2, 1000000, 33, 32);
  servo.setMode(STS_MODE_POSITION);
  servo.hold();
}

void loop() {
  if (digitalRead(BUT1) == HIGH) grip();
  if (digitalRead(BUT4) == HIGH) { servo.torque(false); setRGB(false, false, false); }

  STSState st;
  if (servo.readState(st)) {
    Serial.print("угол "); Serial.print(FeetechServo::posToDeg(st.position), 1);
    Serial.print("°  нагрузка "); Serial.print(st.load);
    Serial.print("  "); Serial.print(st.voltage, 1);
    Serial.print(" В  "); Serial.print(st.temperature);
    Serial.print(" °C  ток "); Serial.println(st.current);
  } else {
    Serial.println("Привод не отвечает");
  }
  delay(200);
}
