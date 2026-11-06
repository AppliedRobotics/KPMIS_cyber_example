/*
  Ardu8 — робот едет по линии (Kirus-Line), останавливается перед
  препятствием (Kirus-Ultrasonic) и при ударе бампером (Kirus-uSW).
  Моторы — через модули STM8 на шине DXL, библиотека DxlMaster2.

  Нужна библиотека DxlMaster2 (Скетч → Подключить библиотеку → Добавить .ZIP).

  Подключение датчиков (3-пиновые кабели):
    Kirus-Line        X4 (S1) → D3,   X2 (S2) → D4
    Kirus-Ultrasonic  X2 (TRIG) → D5, X4 (ECHO) → D6
    Kirus-uSW (бампер спереди)  → D7
    D2 НЕ использовать — его занимает DxlMaster.
  Датчик линии ставится спереди, оптроны по обе стороны от линии:
  на прямом участке оба канала видят белое.

  Моторы (разъёмы справа на плате, сверху вниз):
    1 — ID 71 M1 (левый),  2 — ID 71 M2 (правый)  — см. настройки ниже.

  Переключатели:
    - DIP5 = ON (светодиоды), DIP4 = ON (Arduino на шине DXL),
      DIP3 = OFF, DIP1 = OFF.
    - Прошивать при DIP4 = OFF, потом включить DIP4.
    - Монитор порта не используется: Serial занят шиной DXL.

  Кнопки (верхняя группа, слева направо):
    BUT1 — старт, BUT2 — стоп,
    BUT3 — проверка моторов: оба 1 с «вперёд» (настройте знаки ниже),
    BUT4 — проверка поворота: 1 с поворот в сторону S1.

  Индикация:
    LED1/LED2 — S1/S2 видят чёрное, LED3 — бампер, LED4 — препятствие.
    RGB: синий — ждём старта, зелёный — едем, красный — остановка
    (препятствие или удар; после удара нужен повторный старт BUT1).

  Первая настройка:
    1. Поднимите робота, нажмите BUT3. Если какое-то колесо крутится назад —
       поменяйте знак LEFT_SIGN или RIGHT_SIGN.
    2. Нажмите BUT4: робот должен поворачивать в сторону канала S1.
       Если наоборот — поменяйте S1_IS_LEFT.
    3. Настройте порог датчика линии резистором R7 (см. Ardu8_Kirus_Line).
*/

#include <SPI.h>
#include <DxlMaster2.h>

// ---------- Настройки робота ----------
const uint8_t LEFT_ID   = 71,  LEFT_REG  = 28;   // ID 71, M1 (разъём 1)
const uint8_t RIGHT_ID  = 71,  RIGHT_REG = 30;   // ID 71, M2 (разъём 2)
const int8_t  LEFT_SIGN  = 1;                    // 1 или -1
const int8_t  RIGHT_SIGN = -1;                   // моторы обычно зеркальны
const bool    S1_IS_LEFT = true;                 // канал S1 слева?

const int16_t SPEED_FWD   = 350;   // мощность прямо (0..1023)
const int16_t SPEED_OUTER = 380;   // внешнее колесо в повороте
const int16_t SPEED_INNER = -120;  // внутреннее колесо в повороте
const uint16_t STOP_CM    = 15;    // остановка перед препятствием

// ---------- Пины датчиков ----------
const uint8_t PIN_LINE_S1 = 3;
const uint8_t PIN_LINE_S2 = 4;
const uint8_t PIN_TRIG    = 5;
const uint8_t PIN_ECHO    = 6;
const uint8_t PIN_USW     = 7;

// ---------- Светодиоды и кнопки Ardu8 ----------
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

uint8_t buttons() {
  digitalWrite(PIN_165_LOAD, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, LOW);
  SPI.beginTransaction(spiCfg);
  uint8_t v = SPI.transfer(0);
  SPI.endTransaction();
  digitalWrite(PIN_165_EN, HIGH);
  return v & 0x0F;
}

// ---------- Моторы ----------
DynamixelDevice drvLeft(LEFT_ID);
DynamixelDevice drvRight(RIGHT_ID);

int16_t curL = 0x7FFF, curR = 0x7FFF;   // последнее отправленное
uint32_t lastSendMs = 0;

// Мощности в «логике робота»: + = вперёд
void drive(int16_t left, int16_t right) {
  // Отправляем только изменения + повтор раз в 300 мс
  if (left == curL && right == curR && millis() - lastSendMs < 300) return;
  drvLeft.write(LEFT_REG,  (int16_t)(left  * LEFT_SIGN));
  drvRight.write(RIGHT_REG, (int16_t)(right * RIGHT_SIGN));
  curL = left; curR = right;
  lastSendMs = millis();
}

void stopMotors() { drive(0, 0); }

// Поворот в сторону канала S1 (true) или S2 (false)
void turnTo(bool towardS1) {
  bool left = (towardS1 == S1_IS_LEFT);
  if (left) drive(SPEED_INNER, SPEED_OUTER);
  else      drive(SPEED_OUTER, SPEED_INNER);
}

// ---------- Датчики ----------
uint16_t distanceCm() {               // 0 = дальше ~2 м или нет эха
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  return pulseIn(PIN_ECHO, HIGH, 12000UL) / 58;
}

// ---------- Логика ----------
enum State { WAIT, RUN, OBSTACLE, BUMPED };
State state = WAIT;

uint16_t lastCm = 0;
uint32_t lastSonarMs = 0;

void setup() {
  pinMode(PIN_595_LATCH, OUTPUT);
  pinMode(PIN_165_LOAD, OUTPUT);
  pinMode(PIN_165_EN, OUTPUT);
  digitalWrite(PIN_595_LATCH, LOW);
  digitalWrite(PIN_165_LOAD, HIGH);
  digitalWrite(PIN_165_EN, HIGH);
  SPI.begin();

  pinMode(PIN_LINE_S1, INPUT);
  pinMode(PIN_LINE_S2, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_USW, INPUT);

  DxlMaster.begin(57600);
  delay(300);
  stopMotors();
}

void loop() {
  // --- датчики ---
  bool s1     = digitalRead(PIN_LINE_S1) == HIGH;   // чёрное
  bool s2     = digitalRead(PIN_LINE_S2) == HIGH;
  bool bumper = digitalRead(PIN_USW) == LOW;
  if (millis() - lastSonarMs >= 60) {
    lastSonarMs = millis();
    lastCm = distanceCm();
  }
  bool obstacle = lastCm > 0 && lastCm < STOP_CM;

  // --- кнопки ---
  uint8_t b = buttons();
  if (b & 0x01) state = RUN;
  if (b & 0x02) state = WAIT;
  if (state == WAIT && (b & 0x04)) {           // тест: вперёд
    rgb(false, false, true); ledsUpdate();
    drive(SPEED_FWD, SPEED_FWD); delay(1000); stopMotors();
  }
  if (state == WAIT && (b & 0x08)) {           // тест: поворот к S1
    rgb(false, false, true); ledsUpdate();
    turnTo(true); delay(1000); stopMotors();
  }

  // --- переходы ---
  if (state == RUN || state == OBSTACLE) {
    if (bumper) {
      state = BUMPED;
      drive(-SPEED_FWD, -SPEED_FWD);           // немного отъехать назад
      delay(300);
    } else {
      state = obstacle ? OBSTACLE : RUN;
    }
  }

  // --- движение ---
  switch (state) {
    case RUN:
      if (s1 && !s2)      turnTo(true);        // линия ушла под S1
      else if (s2 && !s1) turnTo(false);       // линия ушла под S2
      else                drive(SPEED_FWD, SPEED_FWD);  // на линии / перекрёсток
      rgb(false, true, false);
      break;
    case OBSTACLE:
      stopMotors();
      rgb(true, false, false);
      break;
    case BUMPED:
      stopMotors();
      rgb(true, false, false);
      break;
    case WAIT:
      stopMotors();
      rgb(false, false, true);
      break;
  }

  led(LED1, s1);
  led(LED2, s2);
  led(LED3, bumper);
  led(LED4, obstacle);
  ledsUpdate();
}
