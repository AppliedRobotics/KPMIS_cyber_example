/*
  Робототехнический контроллер «Кибернетический»: поворот по гироскопу

  BUT1 — развернуться на 90°, BUT2 — на 180°. Угол поворота считается гироскопом,
  поэтому проскальзывание колёс не мешает. Перед поворотом робот стоит неподвижно —
  измеряется смещение нуля. Скорость снижается по мере приближения к цели.
  Левый мотор — в разъём 1 (модуль ID 71, M1), правый — в разъём 3 (модуль ID 72, M1).

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF, DIP5 = ON.
  Прошивать при DIP4 = OFF, затем DIP4 = ON (DIP3 = OFF). Монитор порта не использовать:
  Serial занят шиной моторов. Вывод D2 занят библиотекой DxlMaster2.
  Нужно внешнее питание (XT60/DC) и выключатель ON
  ⚠ Пример ещё не проверен на контроллере..
*/

#include <SPI.h>
#include <Wire.h>
#include <DxlMaster2.h>

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

// ---------- Моторы по шине DXL (библиотека DxlMaster2) ----------
// Левый мотор — в разъём 1 (модуль ID 71), правый — в разъём 3 (модуль ID 72).
DynamixelDevice leftMotor(71);
DynamixelDevice rightMotor(72);
const int LEFT_DIR = 1;    // если левое колесо крутится назад — поставьте -1
const int RIGHT_DIR = 1;   // то же для правого

// Мощность от -1023 до 1023: знак — направление, 0 — стоп
void drive(int left, int right) {
  leftMotor.write(28, (int16_t)(left * LEFT_DIR));     // регистр 28 — мощность M1
  rightMotor.write(28, (int16_t)(right * RIGHT_DIR));
}

float gyroOffset() {               // среднее показание гироскопа в покое
  long sum = 0;
  for (int i = 0; i < 100; i++) { sum += imuRead16(0x37); delay(5); }
  return sum / 100.0;
}

void turn(float target) {
  showLeds(RED);
  float offset = gyroOffset();
  float angle = 0;
  unsigned long last = micros();
  showLeds(GREEN);
  while (abs(angle) < target) {
    unsigned long now = micros();
    angle += (imuRead16(0x37) - offset) / 131.0 * (now - last) / 1000000.0;
    last = now;
    int power = constrain((target - abs(angle)) * 8, 150, 400);   // ближе к цели — медленнее
    drive(power, -power);
  }
  drive(0, 0);
  showLeds(0);
}

void setup() {
  boardBegin();
  imuBegin();
  DxlMaster.begin(57600);
  delay(300);
  drive(0, 0);
}

void loop() {
  byte b = readButtons();
  if (b & 1) turn(90);
  if (b & 2) turn(180);
}
