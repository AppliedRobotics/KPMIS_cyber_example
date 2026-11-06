/*
  Робототехнический контроллер «Кибернетический»: проект «Парктроник»

  «Ультразвуковой дальномер»: TRIG → D5, ECHO → D6.
  Больше 50 см — зелёный, 20–50 см — жёлтый, меньше 20 см — красный.
  Чем ближе препятствие, тем чаще мигает LED1–LED4: период = расстояние × 10 мс.
  Мигание сделано на millis(), поэтому датчик опрашивается постоянно.
  BUT1 включает и выключает «режим тишины» (без мигания).

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF, DIP5 = ON (светодиоды).
*/

#include <SPI.h>

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

unsigned long lastToggle = 0;
bool blinkOn = false;
bool silent = false;
bool wasPressed = false;

void setup() {
  boardBegin();
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  int cm = readDistance();
  if (cm == 0) cm = 400;               // нет эха — препятствия нет

  bool pressed = readButtons() & 1;    // BUT1 — режим тишины
  if (pressed && !wasPressed) silent = !silent;
  wasPressed = pressed;

  byte color;
  if (cm > 50)      color = GREEN;
  else if (cm > 20) color = RED | GREEN;   // жёлтый
  else              color = RED;

  unsigned long period = cm * 10;      // чем ближе, тем чаще
  if (millis() - lastToggle > period) {
    blinkOn = !blinkOn;
    lastToggle = millis();
  }

  byte leds = color;
  if (cm <= 50 && blinkOn && !silent) leds |= LED1 | LED2 | LED3 | LED4;
  showLeds(leds);
  delay(40);
}
