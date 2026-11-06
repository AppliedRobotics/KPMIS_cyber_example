/*
  Робототехнический контроллер «Кибернетический»: приём данных от Arduino-части

  ESP32 принимает строки «K,S1,S2,см,концевик» от примера Ardu8_30_UART_Sender,
  печатает их в монитор порта (115200) и показывает: LED1/LED2 — датчик линии,
  RGB — расстояние (красный < 20 см < зелёный), пищалка — нажатие концевого выключателя.
  DIP2 = ON. Приём: Serial2, RX = IO22.

  Плата в Arduino IDE: ESP32 Wrover Module. Кабель USB-C — в НИЖНИЙ разъём (ESP32).
*/

// ---------- Светодиоды, кнопки и пищалка ESP32 ----------
// Светодиоды горят от LOW. Слева направо на плате: LED4, LED3, LED2, LED1.
const int LED1 = 27, LED2 = 14, LED3 = 12, LED4 = 13;
const int RGB_R = 15, RGB_G = 2, RGB_B = 18;
// Кнопки дают HIGH при нажатии. Слева направо на плате: BUT4, BUT3, BUT2, BUT1.
const int BUT1 = 34, BUT2 = 35, BUT3 = 25, BUT4 = 26;
const int BUZZER = 19;

void setRGB(bool r, bool g, bool b) {
  digitalWrite(RGB_R, r ? LOW : HIGH);
  digitalWrite(RGB_G, g ? LOW : HIGH);
  digitalWrite(RGB_B, b ? LOW : HIGH);
}

void boardBegin() {
  int leds[] = {LED1, LED2, LED3, LED4, RGB_R, RGB_G, RGB_B};
  for (int p : leds) { pinMode(p, OUTPUT); digitalWrite(p, HIGH); }  // всё погасить
  int buttons[] = {BUT1, BUT2, BUT3, BUT4};
  for (int p : buttons) pinMode(p, INPUT);
  pinMode(BUZZER, OUTPUT);
}

int s1, s2, cm, endstop;

void setup() {
  boardBegin();
  Serial.begin(115200);                          // монитор порта (USB)
  Serial2.begin(115200, SERIAL_8N1, 22, 23);     // связь с Arduino: RX = IO22, TX = IO23
}

void loop() {
  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n'); // читаем строку до перевода строки
    line.trim();
    // разбор строки по запятым; sscanf вернёт 4, если все числа прочитаны
    if (sscanf(line.c_str(), "K,%d,%d,%d,%d", &s1, &s2, &cm, &endstop) == 4) {
      Serial.println(line);
      digitalWrite(LED1, s1 ? LOW : HIGH);
      digitalWrite(LED2, s2 ? LOW : HIGH);
      setRGB(cm > 0 && cm < 20, cm >= 20, false);
      if (endstop) tone(BUZZER, 1000, 50);
    }
  }
}
