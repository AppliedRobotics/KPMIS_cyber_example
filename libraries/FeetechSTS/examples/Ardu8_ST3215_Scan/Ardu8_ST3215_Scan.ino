/*
  Ardu8 + Feetech ST3215: поиск сервоприводов на шине.

  Перебирает все скорости Feetech и ID 0..253, печатает найденные приводы.
  Удобно, если неизвестны ID или скорость привода.

  Ardu8: DIP3 = ON, DIP4 = OFF. Шина DXL ESP32: RX = IO33, TX = IO32.
  Плата в Arduino IDE: ESP32 Dev Module (ESP32-WROVER). Монитор порта 115200.
*/
#include <FeetechSTS.h>

FeetechBus bus;

const uint32_t BAUDS[] = { 1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400 };

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("Поиск сервоприводов Feetech...");

  bus.begin(Serial2, 1000000, 33, 32);
  Serial.printf("Эхо на шине: %s\n", bus.echo() ? "есть" : "нет");
  bus.setTimeout(3);

  int found = 0;
  for (uint32_t baud : BAUDS) {
    bus.setBaudRate(baud);
    Serial.printf("Скорость %lu ...\n", (unsigned long)baud);
    for (int id = 0; id < 254; id++) {
      if (!bus.ping(id)) continue;
      found++;
      if (id == 71 || id == 72) {
        Serial.printf("  ID %3d  — модуль моторов Ardu8 (STM8)\n", id);
        continue;
      }
      FeetechServo s(bus, id);
      uint16_t model = 0;
      s.readModel(model);
      Serial.printf("  ID %3d  модель 0x%04X  позиция %d  %.1f В  %d °C\n",
                    id, model, s.readPosition(), s.readVoltage(), s.readTemperature());
    }
  }
  bus.setBaudRate(1000000);
  Serial.printf("Готово, найдено устройств: %d\n", found);
}

void loop() {
}
