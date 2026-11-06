/*
  Робототехнический контроллер «Кибернетический»: светофор — конечный автомат

  Светофор на RGB-светодиоде: ЗЕЛЁНЫЙ → МИГАЮЩИЙ ЗЕЛЁНЫЙ → ЖЁЛТЫЙ → КРАСНЫЙ → ЗЕЛЁНЫЙ.
  Кнопка пешехода BUT1 сокращает зелёный. Состояние хранится в переменной state,
  переходы — в switch. Вместо delay() используется millis(), поэтому кнопка работает всегда.

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

// Состояния светофора — просто номера
const int GREEN_ON = 0, GREEN_BLINK = 1, YELLOW_ON = 2, RED_ON = 3;
int state = GREEN_ON;
unsigned long t0 = 0;              // когда вошли в текущее состояние

void setState(int s) {
  state = s;
  t0 = millis();
}

void setup() {
  boardBegin();
  setState(GREEN_ON);
}

void loop() {
  unsigned long t = millis() - t0; // сколько мс прошло в этом состоянии
  bool pedestrian = readButtons() & 1;

  switch (state) {
    case GREEN_ON:
      showLeds(GREEN);
      if (t > 5000 || (pedestrian && t > 1000)) setState(GREEN_BLINK);
      break;
    case GREEN_BLINK:
      showLeds((t / 300) % 2 ? GREEN : 0);  // мигание каждые 300 мс
      if (t > 1800) setState(YELLOW_ON);
      break;
    case YELLOW_ON:
      showLeds(RED | GREEN);               // жёлтый = красный + зелёный
      if (t > 1000) setState(RED_ON);
      break;
    case RED_ON:
      showLeds(RED | LED1);                // LED1 — «пешеходам можно идти»
      if (t > 4000) setState(GREEN_ON);
      break;
  }
}
