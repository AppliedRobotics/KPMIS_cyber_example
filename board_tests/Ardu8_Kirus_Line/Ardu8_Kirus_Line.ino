/*
  Робототехнический контроллер «Кибернетический» (Ardu8) —
  модуль «Датчик линии» (Kirus-Line V2.0, 2 канала).

  Модуль:
    Два оптрона TCRT5000 и компаратор LM293. Выходы цифровые:
      белое (свет отражается)      → LOW,  зелёный светодиод канала горит
      чёрная линия (не отражается) → HIGH, светодиод канала погашен
    Порог общий для обоих каналов — подстроечный резистор R7 на модуле.
    Канал S1 — оптрон DD1, светодиод HL2, выход на разъём X4.
    Канал S2 — оптрон DD2, светодиод HL3, выход на разъём X2.
    Каждый разъём стоит со стороны «своего» оптрона.
    Разъёмы 3-пиновые: 1 GND, 2 5V, 3 сигнал.

  Подключение к контроллеру:
    X4 (S1) → разъём D3 (XP17)
    X2 (S2) → разъём D4 (XP18)
    Если нужен только один канал — достаточно одного кабеля.

  Настройка порога:
    Поставьте модуль на белое поле на рабочей высоте (обычно 5…10 мм)
    и крутите R7, пока светодиоды каналов не загорятся. Затем проверьте
    на чёрной линии: светодиоды должны погаснуть. Если на линии
    светодиод горит, поверните R7 немного обратно.

  Перед запуском:
    - DIP5 = ON, DIP1 и DIP4 = OFF
    - Плата "Arduino Uno", верхний USB-C, монитор порта 115200

  Что делает:
    - LED1 повторяет канал S1, LED2 — канал S2 (горит = видит ЧЁРНОЕ);
    - RGB: зелёный — линия между оптронами (оба на белом),
           красный — линия под обоими, синий — линия под одним каналом;
    - в монитор порта пишет состояние при каждом изменении.
*/

#include <SPI.h>

const uint8_t PIN_LINE_S1 = 3;
const uint8_t PIN_LINE_S2 = 4;

// ---------- Светодиоды контроллера ----------
const uint8_t PIN_595_LATCH = 8;
const uint8_t PIN_165_LOAD  = 9;
const uint8_t PIN_165_EN    = 10;
const SPISettings spiCfg(1000000, MSBFIRST, SPI_MODE0);

const uint8_t LED1 = 0, LED2 = 1, LED3 = 2, LED4 = 3;
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

void led(uint8_t bit, bool on) {
  if (on) ledState |= (1 << bit);
  else    ledState &= ~(1 << bit);
}

void rgb(bool r, bool g, bool b) {
  led(RGB_RED, r); led(RGB_GREEN, g); led(RGB_BLUE, b);
}

void boardInit() {
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_165_LOAD, OUTPUT);
  pinMode(PIN_165_EN, OUTPUT);
  digitalWrite(PIN_595_LATCH, LOW);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, HIGH);
  SPI.begin();
  ledsUpdate();
}

// ---------- Модуль «Датчик линии» ----------
// true = под каналом чёрная линия
bool lineBlack(uint8_t pin) {
  return digitalRead(pin) == HIGH;
}

uint8_t lastState = 0xFF;

void setup() {
  boardInit();
  pinMode(PIN_LINE_S1, INPUT);   // подтяжка 10 кОм есть на модуле
  pinMode(PIN_LINE_S2, INPUT);
  Serial.begin(115200);
  Serial.println(F("Модуль «Датчик линии»: S1 S2 (1 = чёрное, 0 = белое)"));
}

void loop() {
  bool s1 = lineBlack(PIN_LINE_S1);
  bool s2 = lineBlack(PIN_LINE_S2);

  uint8_t state = (s1 ? 1 : 0) | (s2 ? 2 : 0);
  if (state != lastState) {
    lastState = state;
    Serial.print(F("S1=")); Serial.print(s1);
    Serial.print(F("  S2=")); Serial.print(s2);
    switch (state) {
      case 0: Serial.println(F("  — оба на белом")); break;
      case 1: Serial.println(F("  — линия под S1")); break;
      case 2: Serial.println(F("  — линия под S2")); break;
      case 3: Serial.println(F("  — линия под обоими (перекрёсток?)")); break;
    }
  }

  led(LED1, s1);
  led(LED2, s2);
  if (s1 && s2)      rgb(true,  false, false);
  else if (s1 || s2) rgb(false, false, true);
  else               rgb(false, true,  false);
  ledsUpdate();

  delay(5);
}
