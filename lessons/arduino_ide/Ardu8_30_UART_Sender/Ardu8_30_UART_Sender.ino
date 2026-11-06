/*
  Робототехнический контроллер «Кибернетический»: передача данных датчиков в ESP32

  Arduino-часть читает модули Kirus и 5 раз в секунду отправляет строку
  K,S1,S2,расстояние,концевик  (например «K,0,1,25,0») на скорости 115200.
  DIP2 = ON соединяет Arduino TX с ESP32 RX. Прошивать при DIP2 = OFF, затем включить DIP2.
  Подключение модулей — как в примере Ardu8_24_AllSensors.
  Строку видно и в мониторе порта Arduino.

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

const int S1 = 3, S2 = 4, ENDSTOP = 7;

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(ENDSTOP, INPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.print("K,");                         // буква K — признак нашей строки
  Serial.print(digitalRead(S1));
  Serial.print(",");
  Serial.print(digitalRead(S2));
  Serial.print(",");
  Serial.print(readDistance());
  Serial.print(",");
  Serial.println(digitalRead(ENDSTOP) == LOW ? 1 : 0);   // конец строки — перевод строки
  delay(200);
}
