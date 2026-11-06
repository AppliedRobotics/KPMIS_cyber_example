/*
  Ardu8 + Feetech ST3215: режим позиции, управление кнопками ESP32.

  Нижняя группа кнопок (у модуля ESP32), слева направо:
    BUT4 (IO26) — отпустить вал (крутится рукой)
    BUT3 (IO25) — 270°
    BUT2 (IO35) — 180° (середина)
    BUT1 (IO34) — 90°
  Зелёный цвет RGB — привод движется, красный — нет связи.
  В монитор порта раз в 200 мс выводится состояние привода.

  Ardu8: DIP3 = ON, DIP4 = OFF. Привод с ID 1 на скорости 1 Мбит/с (заводские).
*/
#include <FeetechSTS.h>

const uint8_t SERVO_ID = 1;

const int BUT1 = 34, BUT2 = 35, BUT3 = 25, BUT4 = 26;   // нажатие = HIGH
const int RGB_RED = 15, RGB_GREEN = 2, RGB_BLUE = 18;   // LOW = горит

FeetechBus bus;
FeetechServo servo(bus, SERVO_ID);

bool lastBtn[4];
const int BTN_PINS[4] = { BUT1, BUT2, BUT3, BUT4 };

void setRGB(bool r, bool g, bool b) {
  digitalWrite(RGB_RED,   r ? LOW : HIGH);
  digitalWrite(RGB_GREEN, g ? LOW : HIGH);
  digitalWrite(RGB_BLUE,  b ? LOW : HIGH);
}

bool pressed(int i) {             // срабатывает один раз на нажатие
  bool now = digitalRead(BTN_PINS[i]) == HIGH;
  bool edge = now && !lastBtn[i];
  lastBtn[i] = now;
  return edge;
}

void go(float deg) {
  servo.torque(true);
  servo.moveToDeg(deg, 2000, 50);   // 2000 шагов/с, плавный разгон
  Serial.printf("-> %.0f°\n", deg);
}

void setup() {
  Serial.begin(115200);
  for (int p : BTN_PINS) pinMode(p, INPUT);
  pinMode(RGB_RED, OUTPUT); pinMode(RGB_GREEN, OUTPUT); pinMode(RGB_BLUE, OUTPUT);
  setRGB(false, false, false);

  bus.begin(Serial2, 1000000, 33, 32);

  if (!servo.ping()) {
    Serial.println("Привод не отвечает: проверьте питание, DIP3, ID и скорость (пример Scan)");
    setRGB(true, false, false);
  }
  servo.setMode(STS_MODE_POSITION);
  servo.hold();          // держать текущее положение, вал при старте не дёргается
}

void loop() {
  if (pressed(0)) go(90);
  if (pressed(1)) go(180);
  if (pressed(2)) go(270);
  if (pressed(3)) { servo.torque(false); Serial.println("Вал свободен"); }

  static uint32_t t = 0;
  if (millis() - t >= 200) {
    t = millis();
    STSState st;
    if (servo.readState(st)) {
      Serial.printf("угол %6.1f°  скорость %5d  нагрузка %5d  %.1f В  %d °C  ток %d  ошибки 0x%02X\n",
                    FeetechServo::posToDeg(st.position), st.speed, st.load,
                    st.voltage, st.temperature, st.current, st.status);
      setRGB(false, st.moving, false);
    } else {
      setRGB(true, false, false);
    }
  }
}
