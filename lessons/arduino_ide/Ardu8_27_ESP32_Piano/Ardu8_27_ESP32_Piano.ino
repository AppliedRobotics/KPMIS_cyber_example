/*
  Робототехнический контроллер «Кибернетический»: пианино на кнопках

  Каждая кнопка нижней группы играет свою ноту, пока её держат, и зажигает светодиод над собой.
  Пищалка пассивная, подключена к IO19: tone(вывод, частота, длительность).

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

// Слева направо: BUT4 — до, BUT3 — ми, BUT2 — соль, BUT1 — до следующей октавы
const int keys[]  = {BUT4, BUT3, BUT2, BUT1};
const int leds[]  = {LED4, LED3, LED2, LED1};
const int notes[] = {523, 659, 784, 1047};   // частоты нот, Гц

int playing = 0;   // какая частота звучит сейчас (0 — тишина)

void setup() {
  boardBegin();
}

void loop() {
  int note = 0;
  for (int i = 0; i < 4; i++) {
    bool pressed = digitalRead(keys[i]) == HIGH;
    digitalWrite(leds[i], pressed ? LOW : HIGH);
    if (pressed && note == 0) note = notes[i];   // если зажато несколько — левая
  }
  if (note != playing) {           // меняем звук только когда нота сменилась
    if (note == 0) noTone(BUZZER);
    else tone(BUZZER, note);       // без длительности — звучит до noTone
    playing = note;
  }
  delay(10);
}
