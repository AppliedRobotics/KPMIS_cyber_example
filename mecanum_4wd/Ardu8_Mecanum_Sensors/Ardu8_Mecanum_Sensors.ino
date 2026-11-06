/*
  Робототехнический контроллер «Кибернетический» (Ardu8)
  Мобильная платформа 4WD с меканум-колёсами — Arduino-часть пульта.

  Arduino читает датчики и 20 раз в секунду отправляет их в ESP32 по UART.
  Моторами управляет ESP32 (скетч Ardu8_esp_Mecanum_Web).

  Переключатели DIP для пульта:
    1 — OFF, 2 — ON (Arduino TX → ESP32), 3 — ON (ESP32 на шине моторов),
    4 — OFF, 5 — ON (светодиоды Arduino)
  Прошивать можно, не меняя переключателей.

  Подключение датчиков:
    Датчики линии слева направо: D3 левый внешний, D6 левый внутренний,
                                 D4 правый внутренний, D7 правый внешний
    Ультразвуковой дальномер: TRIG — A0, ECHO — A1
    Концевые выключатели: левый — D5, правый — A2
    IMU ICM-20948 — на плате контроллера, I2C (A4/A5), адрес 0x68,
      магнитометр AK09916 внутри — 0x0C (через режим bypass)

  Строка в ESP32 (115200 бод, также видна в мониторе порта):
    S,<линия1>,<линия2>,<линия3>,<линия4>,<см>,<концевой Л>,<концевой П>,<кнопки>,
      <imu>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<mx>,<my>,<mz>
    линия: 1 — чёрное, 0 — белое; см: -1 — нет эха;
    концевой: 1 — нажат; кнопки: биты 0..3 = кнопки 1..4 Arduino;
    imu: 0 — не найден, 1 — акселерометр и гироскоп, 2 — плюс магнитометр;
    ax..az — ускорение, мg (1000 = 1 g); gx..gz — угловая скорость, 0,1 °/с;
    mx..mz — магнитное поле, 0,1 мкТл.

  Индикация на Arduino:
    LED1..LED4 — датчики линии (горит = чёрное)
    RGB — расстояние: красный < 15 см, жёлтый 15–40, зелёный > 40;
          концевой: левый — синий, правый — фиолетовый, оба — белый.
  Кнопка 3 Arduino — аварийный стоп (пока нажата, ESP32 не крутит моторы).
*/

#include <SPI.h>
#include <Wire.h>

const uint8_t LINE_PINS[4] = {3, 6, 4, 7};  // слева направо
const uint8_t PIN_TRIG  = A0;
const uint8_t PIN_ECHO  = A1;
const uint8_t PIN_END_L = 5;
const uint8_t PIN_END_R = A2;

const uint8_t PIN_595_LATCH = 8;
const uint8_t PIN_165_LOAD  = 9;
const uint8_t PIN_165_EN    = 10;
const SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE0);

const uint8_t RGB_BLUE = 4, RGB_GREEN = 5, RGB_RED = 6;
uint8_t ledState = 0;

void ledsUpdate() {
  digitalWrite(PIN_165_EN, HIGH);
  SPI.beginTransaction(spiCfg);
  SPI.transfer((uint8_t)~ledState);
  SPI.endTransaction();
  digitalWrite(PIN_595_LATCH, HIGH);
  digitalWrite(PIN_595_LATCH, LOW);
}

void rgb(bool r, bool g, bool b) {
  bitWrite(ledState, RGB_RED, r);
  bitWrite(ledState, RGB_GREEN, g);
  bitWrite(ledState, RGB_BLUE, b);
}

uint8_t readButtons() {
  digitalWrite(PIN_165_LOAD, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, LOW);
  SPI.beginTransaction(spiCfg);
  uint8_t v = SPI.transfer(0);
  SPI.endTransaction();
  digitalWrite(PIN_165_EN, HIGH);
  return v & 0x0F;
}

int distanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (us == 0) return -1;
  return us / 58;
}

// ---------- IMU ICM-20948 (без внешних библиотек) ----------
const uint8_t ICM_WHO_AM_I    = 0x00;  // = 0xEA
const uint8_t ICM_USER_CTRL   = 0x03;
const uint8_t ICM_PWR_MGMT_1  = 0x06;
const uint8_t ICM_PWR_MGMT_2  = 0x07;
const uint8_t ICM_INT_PIN_CFG = 0x0F;
const uint8_t ICM_ACCEL_XOUT_H = 0x2D; // далее гироскоп, всего 12 байт
const uint8_t ICM_BANK_SEL    = 0x7F;
const uint8_t AK_ADDR  = 0x0C;
const uint8_t AK_WIA2  = 0x01;         // = 0x09
const uint8_t AK_ST1   = 0x10;
const uint8_t AK_CNTL2 = 0x31;
const uint8_t AK_CNTL3 = 0x32;

uint8_t imuAddr = 0;    // 0 — IMU не найден
uint8_t imuLevel = 0;   // 0 / 1 акселерометр+гироскоп / 2 + магнитометр
int16_t acc[3], gyr[3], mag[3];

void i2cWrite(uint8_t addr, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(v);
  Wire.endTransmission();
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void imuInit() {
  uint8_t id = 0;
  const uint8_t addrs[2] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2 && !imuAddr; i++) {
    i2cWrite(addrs[i], ICM_BANK_SEL, 0x00);
    if (i2cRead(addrs[i], ICM_WHO_AM_I, &id, 1) && id == 0xEA) imuAddr = addrs[i];
  }
  if (!imuAddr) return;

  i2cWrite(imuAddr, ICM_PWR_MGMT_1, 0x80);  // сброс
  delay(100);
  i2cWrite(imuAddr, ICM_BANK_SEL, 0x00);
  i2cWrite(imuAddr, ICM_PWR_MGMT_1, 0x01);  // проснуться, автовыбор тактирования
  delay(50);
  i2cWrite(imuAddr, ICM_PWR_MGMT_2, 0x00);  // включить акселерометр и гироскоп
  // По умолчанию: акселерометр ±2 g (16384 ед./g), гироскоп ±250 °/с (131 ед./(°/с))
  i2cWrite(imuAddr, ICM_USER_CTRL, 0x00);   // внутренний I2C-мастер выключен
  i2cWrite(imuAddr, ICM_INT_PIN_CFG, 0x02); // bypass: магнитометр виден на шине
  delay(10);
  imuLevel = 1;

  if (i2cRead(AK_ADDR, AK_WIA2, &id, 1) && id == 0x09) {
    i2cWrite(AK_ADDR, AK_CNTL3, 0x01);      // сброс магнитометра
    delay(10);
    i2cWrite(AK_ADDR, AK_CNTL2, 0x08);      // непрерывные измерения, 100 Гц
    imuLevel = 2;
  }
}

void imuRead() {
  if (!imuLevel) return;
  uint8_t b[12];
  if (i2cRead(imuAddr, ICM_ACCEL_XOUT_H, b, 12)) {
    for (uint8_t i = 0; i < 3; i++) {
      int16_t a = (int16_t)(b[i * 2] << 8 | b[i * 2 + 1]);
      int16_t g = (int16_t)(b[6 + i * 2] << 8 | b[7 + i * 2]);
      acc[i] = (long)a * 1000 / 16384;   // мg
      gyr[i] = (long)g * 10 / 131;       // 0,1 °/с
    }
  }
  if (imuLevel == 2) {
    uint8_t m[9];                        // ST1, HXL..HZH, TMPS, ST2
    if (i2cRead(AK_ADDR, AK_ST1, m, 9) && (m[0] & 0x01)) {
      for (uint8_t i = 0; i < 3; i++) {
        int16_t v = (int16_t)(m[2 + i * 2] << 8 | m[1 + i * 2]);
        mag[i] = (long)v * 3 / 2;        // 0,15 мкТл на единицу → 0,1 мкТл
      }
    }
  }
}

void setup() {
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_165_LOAD, OUTPUT);
  pinMode(PIN_165_EN, OUTPUT);
  digitalWrite(PIN_595_LATCH, LOW);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, HIGH);
  SPI.begin();
  ledsUpdate();

  for (uint8_t i = 0; i < 4; i++) pinMode(LINE_PINS[i], INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_END_L, INPUT);
  pinMode(PIN_END_R, INPUT);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  imuInit();
}

void loop() {
  uint32_t t0 = millis();

  uint8_t line[4];
  uint8_t mask = 0;
  for (uint8_t i = 0; i < 4; i++) {
    line[i] = digitalRead(LINE_PINS[i]) == HIGH;  // 1 — чёрное
    if (line[i]) mask |= (1 << i);
  }
  bool endL = digitalRead(PIN_END_L) == LOW;     // нажат = LOW
  bool endR = digitalRead(PIN_END_R) == LOW;
  int dist = distanceCm();
  uint8_t btn = readButtons();
  imuRead();

  // Индикация
  ledState = (ledState & 0xF0) | mask;
  if (endL && endR)  rgb(true, true, true);
  else if (endL)     rgb(false, false, true);
  else if (endR)     rgb(true, false, true);
  else if (dist < 0) rgb(false, false, false);
  else if (dist < 15) rgb(true, false, false);
  else if (dist < 40) rgb(true, true, false);
  else               rgb(false, true, false);
  ledsUpdate();

  // Отправка в ESP32
  Serial.print(F("S,"));
  for (uint8_t i = 0; i < 4; i++) { Serial.print(line[i]); Serial.print(','); }
  Serial.print(dist);  Serial.print(',');
  Serial.print(endL);  Serial.print(',');
  Serial.print(endR);  Serial.print(',');
  Serial.print(btn);   Serial.print(',');
  Serial.print(imuLevel);
  for (uint8_t i = 0; i < 3; i++) { Serial.print(','); Serial.print(acc[i]); }
  for (uint8_t i = 0; i < 3; i++) { Serial.print(','); Serial.print(gyr[i]); }
  for (uint8_t i = 0; i < 3; i++) { Serial.print(','); Serial.print(mag[i]); }
  Serial.println();

  // Период 50 мс
  while (millis() - t0 < 50) {}
}
