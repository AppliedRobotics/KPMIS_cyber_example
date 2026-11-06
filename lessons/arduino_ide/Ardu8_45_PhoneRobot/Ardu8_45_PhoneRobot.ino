/*
  Робототехнический контроллер «Кибернетический»: управление роботом с телефона

  ESP32 создаёт Wi-Fi сеть «Kiber-Robot» (пароль 12345678) и страницу http://192.168.4.1
  с кнопками «вперёд», «назад», «влево», «вправо», «стоп». Скорость задаётся в режиме
  скорости (регистр 40). Если команды не приходят больше 0,5 с, робот останавливается.
  Левый мотор — в разъём 1 (модуль ID 71, M1), правый — в разъём 3 (модуль ID 72, M1).

  Плата в Arduino IDE: ESP32 Wrover Module. Кабель USB-C — в НИЖНИЙ разъём (ESP32).
  DIP3 = ON, DIP4 = OFF. Вкладка dxl.h — функции обмена по шине.
  Для вращения моторов нужно питание XT60/DC и выключатель в положении ON.
  ⚠ Пример ещё не проверен на контроллере.
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

#include <WiFi.h>
#include <WebServer.h>

WebServer server(80);
const uint8_t LEFT_ID = 71, RIGHT_ID = 72;
const uint8_t REG_GOAL_SPEED = 40;
const int SPEED = 30;                 // тиков за 100 мс
unsigned long lastCommand = 0;
bool moving = false;

void setSpeeds(int left, int right) {
  dxlWrite16(LEFT_ID, REG_GOAL_SPEED, left);
  dxlWrite16(RIGHT_ID, REG_GOAL_SPEED, right);
  moving = left != 0 || right != 0;
  setRGB(false, moving, !moving);
}

// Страница: кнопки, пока их держат, раз в 200 мс отправляют команду
void sendPage() {
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,user-scalable=no'>"
    "<style>button{width:30%;height:80px;font-size:24px;margin:4px}</style></head><body>"
    "<h2>Робот</h2>"
    "<div><button data-c='f'>▲</button></div>"
    "<div><button data-c='l'>◀</button><button data-c='s'>■</button><button data-c='r'>▶</button></div>"
    "<div><button data-c='b'>▼</button></div>"
    "<script>let t;document.querySelectorAll('button').forEach(b=>{"
    "const go=()=>{fetch('/cmd?c='+b.dataset.c);t=setInterval(()=>fetch('/cmd?c='+b.dataset.c),200)};"
    "const stop=()=>{clearInterval(t);fetch('/cmd?c=s')};"
    "b.onpointerdown=go;b.onpointerup=stop;b.onpointerleave=stop;});</script></body></html>";
  server.send(200, "text/html", html);
}

void handleCommand() {
  String c = server.arg("c");
  if (c == "f") setSpeeds(SPEED, SPEED);
  else if (c == "b") setSpeeds(-SPEED, -SPEED);
  else if (c == "l") setSpeeds(-SPEED, SPEED);
  else if (c == "r") setSpeeds(SPEED, -SPEED);
  else setSpeeds(0, 0);
  lastCommand = millis();
  server.send(200, "text/plain", "ok");
}

void setup() {
  boardBegin();
  dxlBegin();
  setSpeeds(0, 0);
  WiFi.softAP("Kiber-Robot", "12345678");
  server.on("/", sendPage);
  server.on("/cmd", handleCommand);
  server.begin();
}

void loop() {
  server.handleClient();
  if (moving && millis() - lastCommand > 500) setSpeeds(0, 0);   // защита: нет команд — стоп
}
