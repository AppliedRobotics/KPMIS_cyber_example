/*
  Ardu8 — ESP32 принимает данные датчиков Kirus от Arduino и показывает их:
  на своих светодиодах, пищалкой, в мониторе порта и на веб-странице.
  Пара к скетчу для Arduino: Ardu8_Kirus_Sender.

  Почему так: 3-пиновые разъёмы датчиков на Ardu8 есть только у Arduino
  (5 В). У ESP32 вход 3,3 В, и своих разъёмов для датчиков у неё нет.
  Поэтому датчики читает Arduino, а ESP32 получает готовые значения по UART.

  Переключатели: DIP2 = ON (Arduino TX → ESP32 IO22), DIP1 = OFF, DIP4 = OFF.

  Arduino IDE:
    - Плата "ESP32 Dev Module" (или "ESP32 Wrover Module"),
      USB — нижний USB-C на левом краю
    - Монитор порта: 115200

  Индикация (нижняя группа, LOW = горит):
    LED1_ESP (IO27) — S1 видит чёрное
    LED2_ESP (IO14) — S2 видит чёрное
    LED3_ESP (IO12) — микропереключатель нажат
    LED4_ESP (IO13) — препятствие ближе 20 см
    RGB: красный < 10 см или нажат uSW, зелёный — всё спокойно,
         синий — нет данных от Arduino больше 1 с.
    Пищалка (IO19): чем ближе препятствие (< 30 см), тем чаще писк.

  Wi-Fi: ESP32 создаёт точку доступа "Ardu8-Kirus" (пароль 12345678).
  Подключитесь к ней телефоном и откройте http://192.168.4.1 —
  значения датчиков обновляются 5 раз в секунду.
*/

#include <WiFi.h>
#include <WebServer.h>

// ---------- Пины ESP32 на Ardu8 ----------
const int PIN_UART_RX = 22;   // от Arduino D1 (через DIP2)
const int PIN_UART_TX = 23;   // к Arduino D0 (через DIP1, здесь не нужен)

const int LED_ESP[4] = {27, 14, 12, 13};
const int PIN_RGB_R = 15, PIN_RGB_G = 2, PIN_RGB_B = 18;
const int PIN_BUZZER = 19;

const char *AP_SSID = "Ardu8-Kirus";
const char *AP_PASS = "12345678";

WebServer server(80);

// ---------- Данные датчиков ----------
struct KirusData {
  bool s1 = false, s2 = false, usw = false;
  int  cm = 0;
  uint32_t rxMs = 0;          // время последней строки
} data;

void ledOn(int pin, bool on) { digitalWrite(pin, on ? LOW : HIGH); }

void rgb(bool r, bool g, bool b) {
  ledOn(PIN_RGB_R, r); ledOn(PIN_RGB_G, g); ledOn(PIN_RGB_B, b);
}

// Разбор строки "K,s1,s2,cm,sw"
bool parseLine(const String &line) {
  if (!line.startsWith("K,")) return false;
  int v[4];
  int start = 2;
  for (int i = 0; i < 4; i++) {
    int comma = line.indexOf(',', start);
    String part = (comma < 0) ? line.substring(start) : line.substring(start, comma);
    v[i] = part.toInt();
    if (comma < 0 && i < 3) return false;
    start = comma + 1;
  }
  data.s1 = v[0]; data.s2 = v[1]; data.cm = v[2]; data.usw = v[3];
  data.rxMs = millis();
  return true;
}

String rxBuf;

void readUart() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (parseLine(rxBuf)) {
        Serial.printf("S1=%d S2=%d dist=%d см uSW=%d\n",
                      data.s1, data.s2, data.cm, data.usw);
      }
      rxBuf = "";
    } else if (rxBuf.length() < 40) {
      rxBuf += c;
    }
  }
}

// ---------- Веб-страница ----------
const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="ru"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Ardu8 Kirus</title>
<style>body{font-family:sans-serif;margin:16px;background:#f4f4f4}
.c{background:#fff;border-radius:10px;padding:14px;margin:10px 0;font-size:20px}
b{font-size:28px}.on{color:#c00}.off{color:#080}</style></head><body>
<h2>Датчики Kirus</h2>
<div class="c">Расстояние: <b id="cm">—</b> см</div>
<div class="c">Линия S1: <b id="s1">—</b> &nbsp; S2: <b id="s2">—</b></div>
<div class="c">Микропереключатель: <b id="sw">—</b></div>
<div class="c" id="st"></div>
<script>
function bw(v){return v?'<span class="on">чёрное</span>':'<span class="off">белое</span>'}
async function upd(){try{const r=await fetch('/data');const d=await r.json();
cm.textContent=d.cm>0?d.cm:'нет эха';s1.innerHTML=bw(d.s1);s2.innerHTML=bw(d.s2);
sw.innerHTML=d.sw?'<span class="on">нажат</span>':'<span class="off">отпущен</span>';
st.textContent=d.age<1000?'Связь с Arduino есть':'Нет данных от Arduino';}catch(e){}}
setInterval(upd,200);upd();
</script></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html; charset=utf-8", PAGE); }

void handleData() {
  char json[96];
  snprintf(json, sizeof(json), "{\"s1\":%d,\"s2\":%d,\"cm\":%d,\"sw\":%d,\"age\":%lu}",
           data.s1, data.s2, data.cm, data.usw, (unsigned long)(millis() - data.rxMs));
  server.send(200, "application/json", json);
}

// ---------- Пищалка: частота писка зависит от расстояния ----------
uint32_t lastBeepMs = 0;

void beeper(bool fresh) {
  if (!fresh || data.cm <= 0 || data.cm >= 30) return;
  uint32_t period = map(data.cm, 1, 30, 80, 600);   // ближе — чаще
  if (millis() - lastBeepMs >= period) {
    lastBeepMs = millis();
    tone(PIN_BUZZER, 2000, 40);
  }
}

void setup() {
  for (int i = 0; i < 4; i++) { pinMode(LED_ESP[i], OUTPUT); ledOn(LED_ESP[i], false); }
  pinMode(PIN_RGB_R, OUTPUT); pinMode(PIN_RGB_G, OUTPUT); pinMode(PIN_RGB_B, OUTPUT);
  rgb(false, false, true);

  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.print("Wi-Fi: "); Serial.print(AP_SSID);
  Serial.print(", откройте http://"); Serial.println(WiFi.softAPIP());
}

void loop() {
  readUart();
  server.handleClient();

  bool fresh = (data.rxMs != 0) && (millis() - data.rxMs < 1000);
  bool near20 = fresh && data.cm > 0 && data.cm < 20;
  bool alarm  = fresh && (data.usw || (data.cm > 0 && data.cm < 10));

  ledOn(LED_ESP[0], fresh && data.s1);
  ledOn(LED_ESP[1], fresh && data.s2);
  ledOn(LED_ESP[2], fresh && data.usw);
  ledOn(LED_ESP[3], near20);

  if (!fresh)     rgb(false, false, true);
  else if (alarm) rgb(true,  false, false);
  else            rgb(false, true,  false);

  beeper(fresh);
}
