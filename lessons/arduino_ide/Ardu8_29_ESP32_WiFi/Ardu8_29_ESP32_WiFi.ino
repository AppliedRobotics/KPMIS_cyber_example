/*
  Робототехнический контроллер «Кибернетический»: управление светодиодами с телефона

  ESP32 создаёт Wi-Fi сеть «Kiber-ESP32» (пароль 12345678) и веб-страницу http://192.168.4.1
  с кнопками для каждого светодиода. Подключитесь к сети с телефона и откройте адрес в браузере.

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

#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);              // веб-сервер на порту 80

const int pins[]  = {LED1, LED2, LED3, LED4, RGB_R, RGB_G, RGB_B};
const char *names[] = {"LED1", "LED2", "LED3", "LED4", "Красный", "Зелёный", "Синий"};
bool state[7];                     // true — светодиод включён

// Сформировать HTML-страницу с кнопками-ссылками
void sendPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width'>"
                "<style>a{display:block;margin:8px;padding:14px;background:#36c;color:#fff;"
                "text-decoration:none;border-radius:8px;font-size:20px}</style></head><body>"
                "<h2>Контроллер «Кибернетический»</h2>";
  for (int i = 0; i < 7; i++) {
    html += "<a href='/toggle?n=" + String(i) + "'>" + names[i] + (state[i] ? ": вкл" : ": выкл") + "</a>";
  }
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Адрес /toggle?n=номер — переключить светодиод и вернуть страницу
void handleToggle() {
  int n = server.arg("n").toInt();
  if (n >= 0 && n < 7) {
    state[n] = !state[n];
    digitalWrite(pins[n], state[n] ? LOW : HIGH);
  }
  sendPage();
}

void setup() {
  boardBegin();
  Serial.begin(115200);
  WiFi.softAP("Kiber-ESP32", "12345678");   // создать свою сеть
  server.on("/", sendPage);                  // главная страница
  server.on("/toggle", handleToggle);
  server.begin();
  Serial.println("Откройте http://192.168.4.1");
}

void loop() {
  server.handleClient();           // обязательно: обработать запросы браузера
}
