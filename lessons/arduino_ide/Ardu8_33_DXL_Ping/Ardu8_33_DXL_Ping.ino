/*
  Робототехнический контроллер «Кибернетический»: поиск модулей моторов на шине DXL

  Раз в секунду отправляет пакет PING модулям 71 и 72 и печатает в монитор порта (115200),
  ответили ли они, вместе с байтами отправленного пакета.
  LED1 горит — ответил модуль 71, LED2 — модуль 72.

  Плата в Arduino IDE: ESP32 Wrover Module. Кабель USB-C — в НИЖНИЙ разъём (ESP32).
  DIP3 = ON, DIP4 = OFF. Вкладка dxl.h — функции обмена по шине.
  Для вращения моторов нужно питание XT60/DC и выключатель в положении ON.
*/
#include "dxl.h"

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

void printPacket() {               // показать байты последнего отправленного пакета
  for (int i = 0; i < dxlTxLen; i++) {
    if (dxlTx[i] < 16) Serial.print("0");
    Serial.print(dxlTx[i], HEX);
    Serial.print(" ");
  }
}

void setup() {
  boardBegin();
  Serial.begin(115200);
  dxlBegin();
}

void loop() {
  bool ok71 = dxlPing(71);
  Serial.print("PING 71: "); printPacket(); Serial.println(ok71 ? " → ответил" : " → нет ответа");
  bool ok72 = dxlPing(72);
  Serial.print("PING 72: "); printPacket(); Serial.println(ok72 ? " → ответил" : " → нет ответа");

  digitalWrite(LED1, ok71 ? LOW : HIGH);
  digitalWrite(LED2, ok72 ? LOW : HIGH);
  setRGB(!(ok71 && ok72), ok71 && ok72, false);
  delay(1000);
}
