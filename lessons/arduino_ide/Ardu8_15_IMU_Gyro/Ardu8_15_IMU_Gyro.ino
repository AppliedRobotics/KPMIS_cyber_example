/*
  Робототехнический контроллер «Кибернетический»: гироскоп — угол поворота

  Гироскоп измеряет скорость поворота (градусы в секунду). Складывая скорость × время,
  получаем угол поворота платы вокруг вертикальной оси. Угол выводится в монитор порта (115200).
  Первые 2 секунды плата должна лежать неподвижно — измеряется смещение нуля.
  BUT1 обнуляет угол.
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

float offset = 0;          // смещение нуля гироскопа
float angle = 0;           // накопленный угол, градусы
unsigned long lastTime;

void setup() {
  boardBegin();
  imuBegin();
  Serial.begin(115200);

  showLeds(RED);                         // калибровка: не трогать плату
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += imuRead16(0x37);              // GYRO_ZOUT — поворот вокруг вертикали
    delay(10);
  }
  offset = sum / 200.0;
  showLeds(GREEN);
  lastTime = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;   // время с прошлого измерения, с
  lastTime = now;

  float rate = (imuRead16(0x37) - offset) / 131.0;  // 131 единица = 1 °/с
  angle += rate * dt;                         // интегрирование

  if (readButtons() & 1) angle = 0;           // BUT1 — обнулить

  Serial.print("angle:");
  Serial.println(angle);
  delay(10);
}
