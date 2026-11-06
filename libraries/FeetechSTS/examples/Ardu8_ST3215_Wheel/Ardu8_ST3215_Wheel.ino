/*
  Ardu8 + Feetech ST3215: режим колеса (непрерывное вращение с заданной скоростью).

  Нижняя группа кнопок ESP32 (слева направо BUT4, BUT3, BUT2, BUT1):
    BUT1 (IO34) — быстрее по часовой  (+500 шагов/с)
    BUT2 (IO35) — стоп
    BUT3 (IO25) — быстрее против часовой (-500 шагов/с)
  4096 шагов = 1 оборот, максимум у ST3215 около 3000–3400 шагов/с.

  Ardu8: DIP3 = ON, DIP4 = OFF. Привод с ID 1.
*/
#include <FeetechSTS.h>

const uint8_t SERVO_ID = 1;
const int BUT1 = 34, BUT2 = 35, BUT3 = 25;

FeetechBus bus;
FeetechServo servo(bus, SERVO_ID);

int speed = 0;
bool last1, last2, last3;

bool edge(int pin, bool& last) {
  bool now = digitalRead(pin) == HIGH;
  bool e = now && !last;
  last = now;
  return e;
}

void apply() {
  servo.setSpeed(speed, 50);
  Serial.printf("Заданная скорость: %d шагов/с (%.1f об/мин)\n", speed, speed * 60.0 / 4096);
}

void setup() {
  Serial.begin(115200);
  pinMode(BUT1, INPUT); pinMode(BUT2, INPUT); pinMode(BUT3, INPUT);

  bus.begin(Serial2, 1000000, 33, 32);
  if (!servo.ping()) Serial.println("Привод не отвечает");

  servo.setMode(STS_MODE_WHEEL);
  servo.torque(true);
  apply();
}

void loop() {
  if (edge(BUT1, last1)) { speed = min(speed + 500, 3000);  apply(); }
  if (edge(BUT2, last2)) { speed = 0;                       apply(); }
  if (edge(BUT3, last3)) { speed = max(speed - 500, -3000); apply(); }

  static uint32_t t = 0;
  if (millis() - t >= 300) {
    t = millis();
    Serial.printf("  фактическая скорость %d, нагрузка %d\n", servo.readSpeed(), servo.readLoad());
  }
}
