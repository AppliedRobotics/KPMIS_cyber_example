/*
  Робототехнический контроллер «Кибернетический»: модуль «Ультразвуковой дальномер»

  TRIG (разъём X2 модуля) → D5 (XP19), ECHO (X4) → D6 (XP20).
  Расстояние в сантиметрах выводится в монитор порта (115200) 10 раз в секунду.
  Расстояние, см = длительность эха, мкс / 58.

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF.
*/


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

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(115200);
}

void loop() {
  int cm = readDistance();
  if (cm == 0) {
    Serial.println("нет эха");
  } else {
    Serial.print("Расстояние: ");
    Serial.print(cm);
    Serial.println(" см");
  }
  delay(100);
}
