/*
  Робототехнический контроллер «Кибернетический»: мигание светодиодом HL1

  Светодиод HL1 (вывод D13) мигает раз в секунду.
  Попробуйте поменять паузы и посмотрите, как меняется мигание.

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF.
*/

const int LED_PIN = 13;   // светодиод HL1

void setup() {
  pinMode(LED_PIN, OUTPUT);     // вывод работает как выход
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // включить
  delay(500);                   // пауза 500 мс
  digitalWrite(LED_PIN, LOW);   // выключить
  delay(500);
}
