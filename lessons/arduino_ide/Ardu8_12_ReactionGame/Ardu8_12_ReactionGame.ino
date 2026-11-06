/*
  Робототехнический контроллер «Кибернетический»: игра «Реакция» на двоих

  Через случайное время загорается зелёный RGB. Игрок 1 жмёт BUT1, игрок 2 — BUT4.
  Кто первый — тот выиграл раунд: загорается его крайний светодиод (LED1 или LED4).
  Нажатие до сигнала — фальстарт (красный). Время реакции — в мониторе порта (115200).
  Состояния: ОЖИДАНИЕ → СИГНАЛ → РЕЗУЛЬТАТ.

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

void setup() {
  boardBegin();
  Serial.begin(115200);
  randomSeed(analogRead(A0));      // случайные числа каждый раз разные
}

void loop() {
  // ОЖИДАНИЕ: всё погашено, ждём случайное время и ловим фальстарт
  showLeds(0);
  Serial.println("Приготовьтесь...");
  unsigned long waitUntil = millis() + random(1500, 5000);
  while (millis() < waitUntil) {
    byte b = readButtons();
    if (b & 1 || b & 8) {
      showLeds(RED | (b & 1 ? LED1 : LED4));
      Serial.println(b & 1 ? "Фальстарт игрока 1" : "Фальстарт игрока 2");
      delay(2000);
      return;                      // начать раунд заново
    }
  }

  // СИГНАЛ: зелёный, ждём первое нажатие
  showLeds(GREEN);
  unsigned long start = millis();
  byte b = 0;
  while (b == 0) b = readButtons() & (1 | 8);

  // РЕЗУЛЬТАТ
  unsigned long reaction = millis() - start;
  if (b & 1) { showLeds(LED1); Serial.print("Победил игрок 1, "); }
  else       { showLeds(LED4); Serial.print("Победил игрок 2, "); }
  Serial.print(reaction);
  Serial.println(" мс");
  delay(2000);
}
