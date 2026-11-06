/*
  Робототехнический контроллер «Кибернетический»: робот едет по линии

  «Датчик линии»: S1 → D3, S2 → D4, датчики по бокам чёрной линии.
  Оба на белом — прямо; S1 на чёрном — поворот в сторону S1; S2 — в сторону S2;
  оба на чёрном — перекрёсток, едем прямо. BUT1 — старт, BUT2 — стоп.
  Считаем, что S1 слева. Если робот поворачивает не туда, поменяйте местами S1 и S2.
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

const int S1 = 3, S2 = 4;     // S1 — левый датчик, S2 — правый
const int SPEED = 300;        // мощность прямо
const int TURN = 0;           // мощность внутреннего колеса на повороте
bool running = false;

void setup() {
  boardBegin();
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  DxlMaster.begin(57600);
  delay(300);
  drive(0, 0);
}

void loop() {
  byte b = readButtons();
  if (b & 1) running = true;
  if (b & 2) running = false;

  int left = digitalRead(S1);     // 1 — чёрное
  int right = digitalRead(S2);

  if (!running) { drive(0, 0); showLeds(BLUE); }
  else if (left && !right) { drive(TURN, SPEED); showLeds(LED1 | GREEN); }  // линия слева
  else if (right && !left) { drive(SPEED, TURN); showLeds(LED2 | GREEN); }  // линия справа
  else                     { drive(SPEED, SPEED); showLeds(GREEN); }        // прямо
  delay(10);
}
