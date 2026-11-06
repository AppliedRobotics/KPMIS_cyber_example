/*
  Ardu8 + Feetech ST3215: смена ID привода.

  ВАЖНО: на шине должен быть только ОДИН сервопривод Feetech.
  Не назначайте ID 71 и 72 — их занимают модули моторов Ardu8.

  1. Загрузите скетч, откройте монитор порта (115200, «Новая строка»).
  2. Скетч найдёт привод (на 1 Мбит/с) и попросит ввести новый ID.
  3. Введите число 1..253 и нажмите Enter.

  Ardu8: DIP3 = ON, DIP4 = OFF.
*/
#include <FeetechSTS.h>

FeetechBus bus;
FeetechServo servo(bus, 1);
bool ready = false;

int readNumber() {                 // читает число из монитора порта до Enter
  static int value = -1;
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '0' && c <= '9') {
      value = (value < 0 ? 0 : value * 10) + (c - '0');
    } else if (c == '\n' || c == '\r') {
      int v = value;
      value = -1;
      if (v >= 0) return v;
    }
  }
  return -1;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  bus.begin(Serial2, 1000000, 33, 32);
  bus.setTimeout(3);

  Serial.println("Ищу привод...");
  for (int id = 0; id < 254; id++) {
    if (id == 71 || id == 72) continue;
    if (bus.ping(id)) {
      servo.attachId(id);
      ready = true;
      break;
    }
  }
  bus.setTimeout(STS_DEFAULT_TIMEOUT_MS);

  if (!ready) {
    Serial.println("Привод не найден. Проверьте питание, DIP3 и скорость (пример Scan).");
    return;
  }
  Serial.printf("Найден привод с ID %d. Введите новый ID (1..253):\n", servo.id());
}

void loop() {
  if (!ready) return;
  int newId = readNumber();
  if (newId < 0) return;

  if (newId < 1 || newId > 253 || newId == 71 || newId == 72) {
    Serial.println("Недопустимый ID, попробуйте ещё раз");
    return;
  }
  Serial.printf("Меняю ID %d -> %d ... ", servo.id(), newId);
  if (servo.changeId(newId)) {
    Serial.println("готово, привод отвечает на новый ID");
  } else {
    Serial.println("ошибка! Запустите пример Scan, чтобы проверить");
  }
  Serial.println("Можно ввести другой ID:");
}
