/*
  Робототехнический контроллер «Кибернетический» (Ardu8)
  Мобильная платформа 4WD с меканум-колёсами — пульт управления по Wi-Fi (ESP32).

  ESP32 раздаёт Wi-Fi и показывает страницу с пультом и состоянием модулей.
  Моторами управляет ESP32 по шине DXL, датчики присылает Arduino
  (скетч Ardu8_Mecanum_Sensors): линия, дальномер, концевые, кнопки
  и IMU ICM-20948 (акселерометр, гироскоп, магнитометр).

  Как пользоваться:
    1. Подключитесь к сети Wi-Fi «Kibernetichesky-4WD», пароль 12345678.
    2. Откройте в браузере http://192.168.4.1
  Если связь со страницей пропала больше чем на 0,4 с — моторы останавливаются.
  Разгон и торможение плавные (~0,3 с от нуля до полной мощности), аварийный стоп — сразу.
  Причина последнего перезапуска ESP32 и число запусков — на странице и в мониторе порта.

  Переключатели DIP:
    1 — OFF, 2 — ON (данные датчиков от Arduino), 3 — ON (ESP32 на шине моторов),
    4 — OFF, 5 — ON
  Питание моторов — от XT60/DC, выключатель в ON.
  Плата в Arduino IDE: «ESP32 Wrover Module», USB — нижний USB-C.

  Индикация ESP32 (нижняя группа):
    RGB синий — ждём подключения, зелёный — пульт подключён,
    красный — нет данных от Arduino или не отвечает модуль моторов.
    LED1_ESP — горит, пока платформа движется.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_system.h>
#include "page.h"

// ---------- Диагностика перезапусков ----------
const char *resetText = "";   // причина последнего запуска
uint32_t bootCount = 0;       // сколько раз ESP32 запускался (хранится во флеш-памяти)

// ---------- Wi-Fi ----------
const char *AP_SSID = "Kibernetichesky-4WD";
const char *AP_PASS = "12345678";  // не короче 8 символов
WebServer server(80);

// ---------- Пины ESP32 ----------
const uint8_t LED1_ESP = 27;
const uint8_t RGB_R = 15, RGB_G = 2, RGB_B = 18;  // LOW = горит
const uint8_t BUZZER = 19;

// ---------- Шина моторов (Dynamixel 1.0, 57600) ----------
const int DXL_RX = 33, DXL_TX = 32;
HardwareSerial &dxl = Serial2;

// ---------- Данные от Arduino (UART, 115200) ----------
const int ARD_RX = 22, ARD_TX = 23;
HardwareSerial &ard = Serial1;

// ---------- Колёса ----------
enum { FL = 0, FR = 1, RL = 2, RR = 3 };
const int8_t WHEEL_SIGN[4] = {+1, -1, +1, -1};  // проверено на платформе
const int8_t STRAFE_SIGN = +1;                   // проверено на платформе

const uint8_t ID_LEFT  = 72;  // M1 — FL (порт 3), M2 — RL (порт 4)
const uint8_t ID_RIGHT = 71;  // M1 — RR (порт 1), M2 — FR (порт 2)

const uint8_t REG_MAX_POWER    = 24;
const uint8_t REG_ENCODER_MODE = 27;
const uint8_t REG_M1_POWER     = 28;  // M1_POWER, M2_POWER подряд
const uint8_t REG_M1_ENCODER   = 32;  // M1_ENC, M1_SPEED, M2_ENC, M2_SPEED подряд

int16_t wheelPower[4];  // логическая мощность «вперёд», -1023..1023 (что сейчас подано)
int16_t wheelTarget[4]; // к какой мощности плавно идём

// Плавный разгон и торможение: от 0 до полной мощности примерно за RAMP_MS.
// Аварийный стоп (кнопка 3 Arduino) останавливает сразу, без плавности.
const uint16_t RAMP_MS = 300;
const uint16_t MOTOR_PERIOD_MS = 50;
const int16_t RAMP_STEP = 1023L * MOTOR_PERIOD_MS / RAMP_MS;  // ≈170 за шаг
int16_t wheelEnc[4];    // энкодер, приведён к «вперёд»
int16_t wheelSpd[4];    // скорость, тики за 100 мс

struct Module {
  uint8_t id;
  bool ok;
  uint8_t fails;
};
Module modLeft  = {ID_LEFT,  false, 0};
Module modRight = {ID_RIGHT, false, 0};

// ---------- Состояние датчиков ----------
struct Sensors {
  int line[4];
  int dist;
  int endL, endR;
  int btn;
  int imu;      // 0 — нет, 1 — акселерометр+гироскоп, 2 — плюс магнитометр
  int acc[3];   // мg
  int gyr[3];   // 0,1 °/с
  int mag[3];   // 0,1 мкТл
  uint32_t t;   // когда пришла последняя строка
} sens = {{0, 0, 0, 0}, -1, 0, 0, 0, 0, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, 0};

bool sensorsFresh() { return sens.t != 0 && millis() - sens.t < 500; }

// ---------- Команда с пульта ----------
int cmdVx = 0, cmdVy = 0, cmdW = 0;  // -100..100 %
bool guardOn = true;
uint32_t lastCmd = 0;
bool blocked = false, estop = false;

// =====================================================================
// Минимальная реализация Dynamixel 1.0
// =====================================================================
uint8_t txBuf[32];
uint8_t txLen = 0;

void dxlSend(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t n) {
  uint8_t i = 0;
  txBuf[i++] = 0xFF;
  txBuf[i++] = 0xFF;
  txBuf[i++] = id;
  txBuf[i++] = n + 2;
  txBuf[i++] = inst;
  uint8_t sum = id + (n + 2) + inst;
  for (uint8_t k = 0; k < n; k++) { txBuf[i++] = params[k]; sum += params[k]; }
  txBuf[i++] = ~sum;
  txLen = i;
  while (dxl.available()) dxl.read();
  dxl.write(txBuf, txLen);
  dxl.flush();
}

int dxlReadByte(uint32_t deadline) {
  while (!dxl.available()) {
    if ((int32_t)(millis() - deadline) > 0) return -1;
  }
  return dxl.read();
}

// Возвращает число байт параметров или -1. Эхо своего пакета пропускается.
int dxlReceive(uint8_t id, uint8_t *err, uint8_t *params, uint8_t maxParams, uint32_t timeoutMs) {
  uint32_t deadline = millis() + timeoutMs;
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    uint8_t pkt[40];
    uint8_t p = 0;
    int prev = -1, c;
    while (true) {
      c = dxlReadByte(deadline);
      if (c < 0) return -1;
      if (prev == 0xFF && c == 0xFF) break;
      prev = c;
    }
    pkt[p++] = 0xFF; pkt[p++] = 0xFF;
    int rid;
    do { rid = dxlReadByte(deadline); } while (rid == 0xFF);
    if (rid < 0) return -1;
    int len = dxlReadByte(deadline);
    if (len < 2 || len > 34) return -1;
    pkt[p++] = rid; pkt[p++] = len;
    for (int k = 0; k < len; k++) {
      c = dxlReadByte(deadline);
      if (c < 0) return -1;
      pkt[p++] = c;
    }
    if (p == txLen && memcmp(pkt, txBuf, txLen) == 0) continue;  // эхо
    uint8_t sum = 0;
    for (uint8_t k = 2; k < p - 1; k++) sum += pkt[k];
    if ((uint8_t)~sum != pkt[p - 1]) return -1;
    if (rid != id) return -1;
    *err = pkt[4];
    int n = len - 2;
    if (n > maxParams) n = maxParams;
    memcpy(params, &pkt[5], n);
    return n;
  }
  return -1;
}

bool dxlWrite(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t size) {
  uint8_t prm[8];
  prm[0] = addr;
  memcpy(&prm[1], data, size);  // прошивка модуля принимает до 7 байт
  dxlSend(id, 0x03, prm, size + 1);
  uint8_t err, dummy[1];
  return dxlReceive(id, &err, dummy, 0, 20) >= 0 && err == 0;
}

bool dxlRead(uint8_t id, uint8_t addr, uint8_t *data, uint8_t size) {
  uint8_t prm[2] = {addr, size};
  dxlSend(id, 0x02, prm, 2);
  uint8_t err;
  return dxlReceive(id, &err, data, size, 20) == size && err == 0;
}

// =====================================================================
// Моторы
// =====================================================================
void initModule(uint8_t id) {
  uint8_t mode = 1;                          // энкодер AB
  dxlWrite(id, REG_ENCODER_MODE, &mode, 1);
  uint8_t mp[2] = {0xFF, 0x03};              // MAX_POWER = 1023
  dxlWrite(id, REG_MAX_POWER, mp, 2);
}

void markModule(Module &m, bool ok) {
  if (ok) {
    if (!m.ok) initModule(m.id);  // модуль появился или перезагрузился
    m.ok = true;
    m.fails = 0;
  } else if (++m.fails >= 3) {
    m.ok = false;
  }
}

bool writePowers(uint8_t id, int16_t m1, int16_t m2) {
  uint8_t d[4] = {(uint8_t)m1, (uint8_t)((uint16_t)m1 >> 8),
                  (uint8_t)m2, (uint8_t)((uint16_t)m2 >> 8)};
  return dxlWrite(id, REG_M1_POWER, d, 4);
}

// vx — вперёд, vy — влево, w — против часовой; -1023..1023
void computePowers(long vx, long vy, long w) {
  vy *= STRAFE_SIGN;
  long p[4] = {vx - vy - w, vx + vy + w, vx + vy - w, vx - vy + w};
  long m = 1023;
  for (uint8_t i = 0; i < 4; i++) if (labs(p[i]) > m) m = labs(p[i]);
  for (uint8_t i = 0; i < 4; i++) wheelTarget[i] = p[i] * 1023 / m;
}

// Шаг плавного изменения мощности к цели
void rampPowers() {
  for (uint8_t i = 0; i < 4; i++) {
    int16_t d = wheelTarget[i] - wheelPower[i];
    if (d > RAMP_STEP) d = RAMP_STEP;
    else if (d < -RAMP_STEP) d = -RAMP_STEP;
    wheelPower[i] += d;
  }
}

void sendMotors() {
  bool a = writePowers(ID_LEFT,  wheelPower[FL] * WHEEL_SIGN[FL], wheelPower[RL] * WHEEL_SIGN[RL]);
  bool b = writePowers(ID_RIGHT, wheelPower[RR] * WHEEL_SIGN[RR], wheelPower[FR] * WHEEL_SIGN[FR]);
  markModule(modLeft, a);
  markModule(modRight, b);
}

void readEncoders() {
  uint8_t d[8];
  if (dxlRead(ID_LEFT, REG_M1_ENCODER, d, 8)) {
    wheelEnc[FL] = (int16_t)(d[0] | d[1] << 8) * WHEEL_SIGN[FL];
    wheelSpd[FL] = (int16_t)(d[2] | d[3] << 8) * WHEEL_SIGN[FL];
    wheelEnc[RL] = (int16_t)(d[4] | d[5] << 8) * WHEEL_SIGN[RL];
    wheelSpd[RL] = (int16_t)(d[6] | d[7] << 8) * WHEEL_SIGN[RL];
  }
  if (dxlRead(ID_RIGHT, REG_M1_ENCODER, d, 8)) {
    wheelEnc[RR] = (int16_t)(d[0] | d[1] << 8) * WHEEL_SIGN[RR];
    wheelSpd[RR] = (int16_t)(d[2] | d[3] << 8) * WHEEL_SIGN[RR];
    wheelEnc[FR] = (int16_t)(d[4] | d[5] << 8) * WHEEL_SIGN[FR];
    wheelSpd[FR] = (int16_t)(d[6] | d[7] << 8) * WHEEL_SIGN[FR];
  }
}

// =====================================================================
// Данные от Arduino
// =====================================================================
char ardLine[160];
uint8_t ardPos = 0;

void readArduino() {
  while (ard.available()) {
    char c = ard.read();
    if (c == '\n') {
      ardLine[ardPos] = 0;
      ardPos = 0;
      int v[18];
      int n = sscanf(ardLine, "S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                     &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8],
                     &v[9], &v[10], &v[11], &v[12], &v[13], &v[14], &v[15], &v[16], &v[17]);
      if (n >= 8) {
        for (uint8_t i = 0; i < 4; i++) sens.line[i] = v[i];
        sens.dist = v[4];
        sens.endL = v[5];
        sens.endR = v[6];
        sens.btn  = v[7];
        if (n == 18) {
          sens.imu = v[8];
          for (uint8_t i = 0; i < 3; i++) {
            sens.acc[i] = v[9 + i];
            sens.gyr[i] = v[12 + i];
            sens.mag[i] = v[15 + i];
          }
        } else {
          sens.imu = 0;  // старая версия скетча Arduino — без IMU
        }
        sens.t = millis();
      }
    } else if (c != '\r' && ardPos < sizeof(ardLine) - 1) {
      ardLine[ardPos++] = c;
    }
  }
}

// =====================================================================
// Веб-сервер
// =====================================================================
void sendState() {
  bool fresh = sensorsFresh();
  char buf[1024];
  snprintf(buf, sizeof(buf),
    "{\"ard\":%d,\"line\":[%d,%d,%d,%d],\"dist\":%d,\"endL\":%d,\"endR\":%d,\"abtn\":%d,"
    "\"imu\":%d,\"acc\":[%d,%d,%d],\"gyr\":[%d,%d,%d],\"mag\":[%d,%d,%d],"
    "\"m72\":%d,\"m71\":%d,"
    "\"pw\":[%d,%d,%d,%d],\"enc\":[%d,%d,%d,%d],\"spd\":[%d,%d,%d,%d],"
    "\"cl\":%d,\"blk\":%d,\"estop\":%d,\"up\":%lu,\"rst\":\"%s\",\"boots\":%lu,\"heap\":%lu}",
    fresh, sens.line[0], sens.line[1], sens.line[2], sens.line[3],
    sens.dist, sens.endL, sens.endR, sens.btn,
    fresh ? sens.imu : 0,
    sens.acc[0], sens.acc[1], sens.acc[2],
    sens.gyr[0], sens.gyr[1], sens.gyr[2],
    sens.mag[0], sens.mag[1], sens.mag[2],
    modLeft.ok, modRight.ok,
    wheelPower[FL] * 100 / 1023, wheelPower[FR] * 100 / 1023,
    wheelPower[RL] * 100 / 1023, wheelPower[RR] * 100 / 1023,
    wheelEnc[FL], wheelEnc[FR], wheelEnc[RL], wheelEnc[RR],
    wheelSpd[FL], wheelSpd[FR], wheelSpd[RL], wheelSpd[RR],
    WiFi.softAPgetStationNum(), blocked, estop, millis() / 1000,
    resetText, (unsigned long)bootCount, (unsigned long)ESP.getFreeHeap());
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void handleCmd() {
  cmdVx = constrain(server.arg("vx").toInt(), -100, 100);
  cmdVy = constrain(server.arg("vy").toInt(), -100, 100);
  cmdW  = constrain(server.arg("w").toInt(),  -100, 100);
  guardOn = server.arg("g") != "0";
  lastCmd = millis();
  sendState();
}

void handleBeep() {
  tone(BUZZER, 1500, 150);
  server.send(200, "text/plain", "ok");
}

// =====================================================================
void rgb(bool r, bool g, bool b) {
  digitalWrite(RGB_R, r ? LOW : HIGH);
  digitalWrite(RGB_G, g ? LOW : HIGH);
  digitalWrite(RGB_B, b ? LOW : HIGH);
}

// Функции стоят после объявлений структур: Arduino IDE сама вставляет
// объявления функций перед первой функцией файла, и типы к этому месту
// должны быть уже известны.
const char *resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "включение питания, кнопка EN или сброс по USB";
    case ESP_RST_EXT:       return "внешний сброс";
    case ESP_RST_SW:        return "программный перезапуск";
    case ESP_RST_PANIC:     return "сбой программы";
    case ESP_RST_INT_WDT:   return "сторожевой таймер (прерывания)";
    case ESP_RST_TASK_WDT:  return "сторожевой таймер (задачи)";
    case ESP_RST_WDT:       return "сторожевой таймер";
    case ESP_RST_DEEPSLEEP: return "выход из сна";
    case ESP_RST_BROWNOUT:  return "просадка питания";
    default:                return "неизвестно";
  }
}

void setup() {
  Serial.begin(115200);

  esp_reset_reason_t rr = esp_reset_reason();
  resetText = resetReasonText(rr);
  Preferences prefs;
  prefs.begin("diag", false);
  bootCount = prefs.getULong("boots", 0) + 1;
  prefs.putULong("boots", bootCount);
  prefs.end();
  Serial.printf("\nЗапуск №%lu, причина: %s (код %d)\n", (unsigned long)bootCount, resetText, (int)rr);
  pinMode(LED1_ESP, OUTPUT); digitalWrite(LED1_ESP, HIGH);
  pinMode(RGB_R, OUTPUT); pinMode(RGB_G, OUTPUT); pinMode(RGB_B, OUTPUT);
  rgb(false, false, true);
  pinMode(BUZZER, OUTPUT);

  dxl.begin(57600, SERIAL_8N1, DXL_RX, DXL_TX);
  ard.begin(115200, SERIAL_8N1, ARD_RX, ARD_TX);
  delay(500);
  sendMotors();  // нули и проверка модулей

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Wi-Fi: ");
  Serial.print(AP_SSID);
  Serial.print("  адрес: http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() { server.send(200, "text/html; charset=utf-8", PAGE_HTML); });
  server.on("/cmd", handleCmd);
  server.on("/state", sendState);
  server.on("/beep", handleBeep);
  server.onNotFound([]() { server.sendHeader("Location", "/"); server.send(302); });
  server.begin();

  tone(BUZZER, 1000, 100);
}

uint32_t tMotors = 0, tEnc = 0, tLeds = 0;

void loop() {
  server.handleClient();
  readArduino();

  uint32_t now = millis();

  // Моторы — каждые 50 мс
  if (now - tMotors >= MOTOR_PERIOD_MS) {
    tMotors = now;
    long vx = 0, vy = 0, w = 0;
    if (now - lastCmd < 400) { vx = cmdVx; vy = cmdVy; w = cmdW; }

    estop = sensorsFresh() && (sens.btn & 0x04);           // кнопка 3 Arduino
    blocked = guardOn && vx > 0 && sensorsFresh() && sens.dist > 0 && sens.dist < 15;
    if (estop) vx = vy = w = 0;
    if (blocked) vx = 0;

    computePowers(vx * 1023 / 100, vy * 1023 / 100, w * 1023 / 100);
    if (estop) {
      for (uint8_t i = 0; i < 4; i++) wheelPower[i] = 0;  // аварийный стоп — сразу
    } else {
      rampPowers();
    }
    sendMotors();
  }

  // Энкодеры — каждые 150 мс
  if (now - tEnc >= 150) {
    tEnc = now;
    readEncoders();
  }

  // Индикация — каждые 100 мс
  if (now - tLeds >= 100) {
    tLeds = now;
    bool moving = wheelPower[0] || wheelPower[1] || wheelPower[2] || wheelPower[3];
    digitalWrite(LED1_ESP, moving ? LOW : HIGH);
    if (!sensorsFresh() || !modLeft.ok || !modRight.ok) rgb(true, false, false);
    else if (now - lastCmd < 1000)                      rgb(false, true, false);
    else                                                rgb(false, false, true);
  }
}
