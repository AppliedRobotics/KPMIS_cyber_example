/*
  Робототехнический контроллер «Кибернетический»: езда вдоль стены

  Дальномер установлен сбоку справа и смотрит на стену. Робот едет вперёд,
  а регулятор держит расстояние 15 см: поправка = K × (расстояние − 15),
  одно колесо ускоряется, другое замедляется. BUT1 — старт, BUT2 — стоп.
  Если робот уходит от стены вместо того, чтобы приближаться, поменяйте знак K.
  Левый мотор — в разъём 1 (модуль ID 71, M1), правый — в разъём 3 (модуль ID 72, M1).

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF, DIP5 = ON.
  Прошивать при DIP4 = OFF, затем DIP4 = ON (DIP3 = OFF). Монитор порта не использовать:
  Serial занят шиной моторов. Вывод D2 занят библиотекой DxlMaster2.
  Нужно внешнее питание (XT60/DC) и выключатель ON
  ⚠ Пример ещё не проверен на контроллере..
*/

#include <SPI.h>
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

// ---------- Модуль «Ультразвуковой дальномер» ----------
const int TRIG = 5;   // X2 модуля → D5 (XP19)
const int ECHO = 6;   // X4 модуля → D6 (XP20)

// Измерить расстояние в сантиметрах. 0 — эхо не вернулось.
int readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);              // импульс 10 мкс запускает измерение
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 30000);  // длительность эха, ждём не дольше 30 мс
  return us / 58;                        // см = мкс / 58
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

const int TARGET = 15;      // расстояние до стены, см
const int BASE = 300;       // мощность движения вперёд
const float K = 20;         // коэффициент регулятора
bool running = false;

void setup() {
  boardBegin();
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  DxlMaster.begin(57600);
  delay(300);
  drive(0, 0);
}

void loop() {
  byte b = readButtons();
  if (b & 1) running = true;
  if (b & 2) running = false;

  if (!running) { drive(0, 0); showLeds(BLUE); delay(50); return; }

  int cm = readDistance();
  if (cm == 0 || cm > 60) cm = 60;              // проём — ограничиваем ошибку
  int correction = K * (cm - TARGET);
  correction = constrain(correction, -200, 200);
  drive(BASE + correction, BASE - correction);  // стена справа: далеко — поворачиваем вправо
  showLeds(GREEN);
  delay(50);
}
