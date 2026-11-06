/*
  Робототехнический контроллер «Кибернетический»: монитор порта

  Выводит «Привет!», а затем раз в секунду — значение счётчика.
  Откройте монитор порта и выберите скорость 115200.

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF.
*/

int counter = 0;    // переменная-счётчик

void setup() {
  Serial.begin(115200);         // скорость обмена — как в мониторе порта
  Serial.println("Привет!");
}

void loop() {
  Serial.print("Счётчик: ");
  Serial.println(counter);      // println — с переводом строки
  counter = counter + 1;
  delay(1000);
}
