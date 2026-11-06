/*
  Робототехнический контроллер «Кибернетический»: мелодия из массивов

  Нажмите BUT1 (крайняя правая кнопка) — прозвучит мелодия.
  Ноты и длительности хранятся в двух массивах одинаковой длины. 0 — пауза.

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

// Ноты первой октавы, Гц
const int DO = 262, RE = 294, MI = 330, FA = 349, SOL = 392, LA = 440, SI = 494;

int melody[]    = {DO, DO, SOL, SOL, LA, LA, SOL, 0, FA, FA, MI, MI, RE, RE, DO};
int durations[] = {300, 300, 300, 300, 300, 300, 600, 100, 300, 300, 300, 300, 300, 300, 600};
const int NOTES = sizeof(melody) / sizeof(melody[0]);

void playMelody() {
  for (int i = 0; i < NOTES; i++) {
    digitalWrite(LED1, LOW);
    if (melody[i] > 0) tone(BUZZER, melody[i], durations[i]);
    delay(durations[i] + 50);      // ждём окончания ноты и небольшую паузу
    digitalWrite(LED1, HIGH);
  }
}

void setup() {
  boardBegin();
}

void loop() {
  if (digitalRead(BUT1) == HIGH) playMelody();
}
