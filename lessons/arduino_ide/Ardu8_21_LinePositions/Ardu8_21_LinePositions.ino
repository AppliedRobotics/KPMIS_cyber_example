/*
  Робототехнический контроллер «Кибернетический»: положение относительно линии

  Датчик линии: S1 → D3, S2 → D4. Два канала собираются в одно число state = s1·2 + s2,
  а switch выбирает ситуацию: «линии нет», «линия у S2», «линия у S1», «перекрёсток».
  Перекрёстки считаются по событию «оба канала стали чёрными».

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

const int S1 = 3, S2 = 4;
int lastState = -1;
int crossings = 0;

void setup() {
  boardBegin();
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  Serial.begin(115200);
}

void loop() {
  int state = digitalRead(S1) * 2 + digitalRead(S2);   // 0, 1, 2 или 3

  if (state != lastState) {        // печатаем только при изменении
    switch (state) {
      case 0: Serial.println("Линии нет (или она между датчиками)"); showLeds(BLUE); break;
      case 1: Serial.println("Линия со стороны S2");                 showLeds(LED2 | GREEN); break;
      case 2: Serial.println("Линия со стороны S1");                 showLeds(LED1 | GREEN); break;
      case 3:
        crossings++;               // перекрёсток — только в момент появления
        Serial.print("Перекрёсток № ");
        Serial.println(crossings);
        showLeds(LED1 | LED2 | RED);
        break;
    }
    lastState = state;
  }
  delay(20);
}
