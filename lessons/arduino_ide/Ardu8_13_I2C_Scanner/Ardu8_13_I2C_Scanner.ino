/*
  Робототехнический контроллер «Кибернетический»: сканер шины I2C

  Перебирает адреса 1–126 и печатает в мониторе порта (115200) те, на которые ответило устройство.
  На контроллере должен найтись датчик движения ICM-20948 по адресу 0x68.

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF.
*/

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();                    // шина I2C: SDA — A4, SCL — A5
}

void loop() {
  Serial.println("Поиск устройств I2C...");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {   // 0 — устройство ответило
      Serial.print("Найдено устройство: 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  Serial.print("Всего: ");
  Serial.println(found);
  delay(3000);
}
