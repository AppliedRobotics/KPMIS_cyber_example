/*
  Ardu8 — датчик Kirus через разъём RJ12 (XS9) одним кабелем.

  У всех датчиков Kirus есть разъём RJ12 X1 (6P6C):
    1 — не подключён, 2 — не подключён, 3 — 5V,
    4 — сигнал 1, 5 — сигнал 2, 6 — GND.
  На Ardu8 разъём RJ12 XS9 «I2C» разведён так:
    1 VCC, 2 DXL, 3 5V, 4 SDA (A4), 5 SCL (A5), 6 GND.
  Значит, через XS9 сигналы датчика приходят на A4 и A5:

    Датчик            контакт 4 → A4      контакт 5 → A5
    Kirus-uSW         —                   SIG (нажат = LOW)
    Kirus-Ultrasonic  TRIG                ECHO
    Kirus-Line        S1                  S2

  ВАЖНО:
    - Подключать ТОЛЬКО в XS9 (I2C). Во втором RJ12 (XS10) контакты 4/5 —
      это линии CAN, датчик там работать не будет.
    - A4/A5 — это шина I2C, на ней же IMU ICM-20948 и подтяжки 2,7 кОм к 5 В.
      Пока датчик в XS9, не используйте Wire/IMU и разъём XP5.
    - В XS9 можно подключить только один датчик.
    - Для нескольких датчиков сразу используйте 3-пиновые кабели
      (примеры Ardu8_Kirus_uSW / _Ultrasonic / _Line / _All).

  Выберите датчик строкой ниже и загрузите скетч.

  Перед запуском:
    - DIP5 = ON, DIP1 и DIP4 = OFF
    - Плата "Arduino Uno", верхний USB-C, монитор порта 115200

  Индикация: LED1/LED2 — сигналы на A4/A5 (для дальномера LED1..LED4 —
  шкала близости), RGB — как в отдельных примерах.
*/

#define SENSOR_USW        1
#define SENSOR_ULTRASONIC 2
#define SENSOR_LINE       3

#define SENSOR SENSOR_LINE      // <-- выберите подключённый датчик

#include <SPI.h>

const uint8_t PIN_RJ_4 = A4;    // контакт 4 RJ12
const uint8_t PIN_RJ_5 = A5;    // контакт 5 RJ12

// ---------- Светодиоды Ardu8 ----------
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

void setup() {
  boardInit();
  Serial.begin(115200);

#if SENSOR == SENSOR_USW
  pinMode(PIN_RJ_5, INPUT);
  Serial.println(F("RJ12: Kirus-uSW (A5)"));
#elif SENSOR == SENSOR_ULTRASONIC
  pinMode(PIN_RJ_4, OUTPUT);           // TRIG
  digitalWrite(PIN_RJ_4, LOW);
  pinMode(PIN_RJ_5, INPUT);            // ECHO
  Serial.println(F("RJ12: Kirus-Ultrasonic (TRIG A4, ECHO A5)"));
#elif SENSOR == SENSOR_LINE
  pinMode(PIN_RJ_4, INPUT);
  pinMode(PIN_RJ_5, INPUT);
  Serial.println(F("RJ12: Kirus-Line (S1 A4, S2 A5), 1 = чёрное"));
#else
  #error "Выберите SENSOR"
#endif
}

void loop() {
#if SENSOR == SENSOR_USW
  bool pressed = (digitalRead(PIN_RJ_5) == LOW);
  Serial.print(F("Pressed:")); Serial.println(pressed);
  led(LED1, pressed);
  rgb(pressed, !pressed, false);
  ledsUpdate();
  delay(50);

#elif SENSOR == SENSOR_ULTRASONIC
  digitalWrite(PIN_RJ_4, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_RJ_4, LOW);
  uint16_t cm = pulseIn(PIN_RJ_5, HIGH, 25000UL) / 58;
  Serial.print(F("Distance_cm:")); Serial.println(cm);
  bool echo = cm > 0;
  led(LED1, echo && cm < 100);
  led(LED2, echo && cm < 50);
  led(LED3, echo && cm < 25);
  led(LED4, echo && cm < 10);
  if (!echo)        rgb(false, false, false);
  else if (cm < 15) rgb(true,  false, false);
  else if (cm < 40) rgb(false, false, true);
  else              rgb(false, true,  false);
  ledsUpdate();
  delay(60);

#elif SENSOR == SENSOR_LINE
  bool s1 = digitalRead(PIN_RJ_4) == HIGH;   // HIGH = чёрное
  bool s2 = digitalRead(PIN_RJ_5) == HIGH;
  Serial.print(F("S1:")); Serial.print(s1);
  Serial.print(F(" S2:")); Serial.println(s2);
  led(LED1, s1);
  led(LED2, s2);
  if (s1 && s2)      rgb(true,  false, false);
  else if (s1 || s2) rgb(false, false, true);
  else               rgb(false, true,  false);
  ledsUpdate();
  delay(50);
#endif
}
