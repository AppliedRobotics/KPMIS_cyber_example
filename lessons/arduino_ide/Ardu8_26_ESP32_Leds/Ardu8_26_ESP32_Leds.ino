/*
  Робототехнический контроллер «Кибернетический»: светодиоды ESP32 — бегущий огонь

  Светодиоды нижней группы загораются по очереди слева направо, затем RGB показывает
  красный, зелёный и синий. У ESP32 светодиоды подключены прямо к выводам и горят от LOW.

  Плата в Arduino IDE: ESP32 Wrover Module. Кабель USB-C — в НИЖНИЙ разъём (ESP32).
*/

const int LEDS[] = {13, 12, 14, 27};   // слева направо: LED4, LED3, LED2, LED1
const int RGB[]  = {15, 2, 18};        // красный, зелёный, синий

void setup() {
  for (int p : LEDS) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }   // HIGH — погашен
  for (int p : RGB)  { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }
}

void loop() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LEDS[i], LOW);    // зажечь
    delay(200);
    digitalWrite(LEDS[i], HIGH);   // погасить
  }
  for (int i = 0; i < 3; i++) {
    digitalWrite(RGB[i], LOW);
    delay(300);
    digitalWrite(RGB[i], HIGH);
  }
}
