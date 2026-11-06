/*
  Робототехнический контроллер «Кибернетический»: магнитометр — компас

  Магнитометр AK09916 находится внутри ICM-20948 и отвечает по адресу 0x0C
  после включения обходного режима (bypass). Плата лежит горизонтально.
  Направление на север (0–360°) выводится в монитор порта (115200),
  светодиод показывает, куда ближе: LED1 — 0°, LED2 — 90°, LED3 — 180°, LED4 — 270°.
  Для точности нужна калибровка: вычислите MAG_X0 и MAG_Y0 как середину между
  минимумом и максимумом при вращении платы и впишите их ниже.
  ⚠ Пример ещё не проверен на контроллере.

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF, DIP5 = ON (светодиоды).
*/

#include <SPI.h>
#include <Wire.h>

// ---------- Светодиоды и кнопки Arduino-части ----------
// Светодиоды подключены через сдвиговый регистр 74HC595, кнопки — через 74HC165.
// Оба регистра работают по шине SPI (D11 — данные, D12 — приём, D13 — такт).
const int LATCH_595 = 8;   // защёлка 74HC595
const int LOAD_165  = 9;   // «фотография» кнопок в 74HC165
const int EN_165    = 10;  // LOW — 74HC165 подключён к линии приёма SPI

// Номера битов: 1 — LED1, 2 — LED2, 4 — LED3, 8 — LED4, 16 — синий, 32 — зелёный, 64 — красный
const byte LED1 = 1, LED2 = 2, LED3 = 4, LED4 = 8;
const byte BLUE = 16, GREEN = 32, RED = 64;

// Зажечь светодиоды. В переменной leds 1 = горит.
// Регистр зажигает светодиод нулём, поэтому байт инвертируем (~).
void showLeds(byte leds) {
  digitalWrite(LATCH_595, LOW);
  SPI.transfer(~leds);
  digitalWrite(LATCH_595, HIGH);   // импульс на защёлке — байт появляется на выходах
}

// Прочитать кнопки: бит 0 — BUT1 … бит 3 — BUT4 (слева направо). 1 = нажата.
byte readButtons() {
  digitalWrite(LOAD_165, LOW);     // «сфотографировать» кнопки
  delayMicroseconds(5);
  digitalWrite(LOAD_165, HIGH);
  digitalWrite(EN_165, LOW);       // подключить регистр к линии приёма
  byte b = SPI.transfer(0);        // принять байт
  digitalWrite(EN_165, HIGH);
  return b & 0x0F;                 // нужны только 4 младших бита
}

void boardBegin() {
  pinMode(LATCH_595, OUTPUT);
  pinMode(LOAD_165, OUTPUT);
  pinMode(EN_165, OUTPUT);
  digitalWrite(LOAD_165, HIGH);
  digitalWrite(EN_165, HIGH);
  SPI.begin();
  showLeds(0);                     // всё погасить
}

// ---------- Датчик движения ICM-20948 (шина I2C, адрес 0x68) ----------
const byte IMU_ADDR = 0x68;

void imuWrite(byte reg, byte value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Прочитать 16-битное число со знаком (старший байт первым)
int imuRead16(byte reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU_ADDR, (byte)2);
  int hi = Wire.read();
  int lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

void imuBegin() {
  Wire.begin();
  imuWrite(0x06, 0x80);   // PWR_MGMT_1: сброс датчика
  delay(100);
  imuWrite(0x06, 0x01);   // PWR_MGMT_1: проснуться, автовыбор тактирования
  imuWrite(0x07, 0x00);   // PWR_MGMT_2: включить акселерометр и гироскоп
  delay(50);
}

const byte MAG_ADDR = 0x0C;
int MAG_X0 = 0, MAG_Y0 = 0;        // смещения после калибровки

void magWrite(byte reg, byte value) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Прочитать X и Y магнитного поля (младший байт первым)
void magRead(int &mx, int &my) {
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x11);                // HXL — начало данных
  Wire.endTransmission(false);
  Wire.requestFrom(MAG_ADDR, (byte)8);   // X, Y, Z, служебный байт, ST2
  byte d[8];
  for (int i = 0; i < 8; i++) d[i] = Wire.read();
  mx = (int16_t)(d[0] | (d[1] << 8));
  my = (int16_t)(d[2] | (d[3] << 8));
}

void setup() {
  boardBegin();
  imuBegin();
  imuWrite(0x03, 0x00);            // USER_CTRL: внутренний I2C-мастер выключен
  imuWrite(0x0F, 0x02);            // INT_PIN_CFG: обходной режим — магнитометр на общей шине
  delay(10);
  magWrite(0x31, 0x08);            // CNTL2: непрерывные измерения 100 раз в секунду
  Serial.begin(115200);
}

void loop() {
  int mx, my;
  magRead(mx, my);
  float heading = atan2(my - MAG_Y0, mx - MAG_X0) * 180 / PI;
  if (heading < 0) heading += 360;

  Serial.print("mx:"); Serial.print(mx);
  Serial.print(" my:"); Serial.print(my);
  Serial.print(" heading:"); Serial.println(heading);

  int sector = ((int)(heading + 45) % 360) / 90;   // 0, 1, 2, 3
  byte leds[] = {LED1, LED2, LED3, LED4};
  showLeds(leds[sector]);
  delay(100);
}
