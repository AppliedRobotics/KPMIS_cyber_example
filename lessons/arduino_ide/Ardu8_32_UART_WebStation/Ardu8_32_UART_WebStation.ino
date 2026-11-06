/*
  Робототехнический контроллер «Кибернетический»: датчики на веб-странице

  ESP32 принимает данные от Ardu8_30_UART_Sender (DIP2 = ON) и показывает их
  на странице http://192.168.4.1. Wi-Fi сеть «Kiber-Station», пароль 12345678.
  Страница обновляется сама раз в секунду.

  Плата в Arduino IDE: ESP32 Wrover Module. Кабель USB-C — в НИЖНИЙ разъём (ESP32).
  ⚠ Пример ещё не проверен на контроллере.
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

#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);
int s1 = 0, s2 = 0, cm = 0, endstop = 0;

void sendPage() {
  String color = cm > 0 && cm < 20 ? "#d33" : (cm < 50 ? "#c80" : "#2a2");
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta http-equiv='refresh' content='1'>"           // обновлять раз в секунду
                "<meta name='viewport' content='width=device-width'></head>"
                "<body style='font-family:sans-serif;font-size:22px'>"
                "<h2>Датчики Kirus</h2>"
                "<p style='color:" + color + "'>Расстояние: " + String(cm) + " см</p>"
                "<p>Линия S1: " + (s1 ? "чёрное" : "белое") + "</p>"
                "<p>Линия S2: " + (s2 ? "чёрное" : "белое") + "</p>"
                "<p>Концевой выключатель: " + (endstop ? "нажат" : "отпущен") + "</p>"
                "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  boardBegin();
  Serial2.begin(115200, SERIAL_8N1, 22, 23);
  WiFi.softAP("Kiber-Station", "12345678");
  server.on("/", sendPage);
  server.begin();
}

void loop() {
  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    sscanf(line.c_str(), "K,%d,%d,%d,%d", &s1, &s2, &cm, &endstop);
    digitalWrite(LED1, s1 ? LOW : HIGH);
    digitalWrite(LED2, s2 ? LOW : HIGH);
  }
  server.handleClient();
}
