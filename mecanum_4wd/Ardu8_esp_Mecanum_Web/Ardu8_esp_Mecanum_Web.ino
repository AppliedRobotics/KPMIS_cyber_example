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

  Манипулятор (металл) — сервоприводы Feetech ST3215 (ID 1–7) на той же шине DXL
  (разъёмы XP2/XP3). Моторы работают на 57600, сервоприводы — на 1 Мбит/с:
  ESP32 переключает скорость шины между обменами. Сервоприводы должны быть
  подготовлены в «Конфигураторе манипулятора» (ID 1–7, 1 Мбит/с, середина 2048).
  Тип манипулятора — ARM_PLASTIC ниже (0 — металл, основной; 1 — пластик).

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
const uint32_t MOTOR_BAUD = 57600;     // модули моторов
const uint32_t SERVO_BAUD = 1000000;   // сервоприводы манипулятора
uint32_t busBaud = MOTOR_BAUD;

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
uint8_t txBuf[96];
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
    uint8_t pkt[96];
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
    if (len < 2 || len > 90) return -1;
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

// Переключение скорости шины (моторы 57600 / сервоприводы 1 Мбит/с)
void setBus(uint32_t baud) {
  if (busBaud == baud) return;
  dxl.flush();
  dxl.updateBaudRate(baud);
  busBaud = baud;
  // Пауза тишины на шине: устройства на другой скорости видят наш трафик как мусор
  // и могут «застрять» в разборе ложного пакета — пауза даёт им сбросить разбор.
  delay(2);
  while (dxl.available()) dxl.read();
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
  setBus(MOTOR_BAUD);
  bool a = writePowers(ID_LEFT,  wheelPower[FL] * WHEEL_SIGN[FL], wheelPower[RL] * WHEEL_SIGN[RL]);
  bool b = writePowers(ID_RIGHT, wheelPower[RR] * WHEEL_SIGN[RR], wheelPower[FR] * WHEEL_SIGN[FR]);
  markModule(modLeft, a);
  markModule(modRight, b);
}

void readEncoders() {
  setBus(MOTOR_BAUD);
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
// Манипулятор: сервоприводы Feetech ST3215, протокол как у Dynamixel 1.0
// =====================================================================
#define ARM_PLASTIC 0   // 0 — «Манипулятор (металл)» (основной), 1 — «Манипулятор (пластик)»

struct ArmJoint {
  const char *name;
  uint8_t id;
  int8_t dir;         // -1 — инверсия (у поворота основания вращается корпус привода)
  int16_t minDeg, maxDeg;
};
const uint8_t ARM_N = 7;
#if ARM_PLASTIC
const char *ARM_TYPE = "Манипулятор (пластик)";
const ArmJoint ARM[ARM_N] = {
  {"Поворот основания",  1, -1, -150, 150},
  {"Плечо",              2, +1, -100, 100},
  {"Локоть",             3, +1, -130, 130},
  {"Поворот предплечья", 4, +1, -150, 150},
  {"Наклон кисти",       5, +1, -110, 110},
  {"Поворот схвата",     6, +1, -150, 150},
  {"Схват",              7, +1,    0,  70},   // 0° — раскрыт, 70° — сомкнут
};
#else
const char *ARM_TYPE = "Манипулятор (металл)";
const ArmJoint ARM[ARM_N] = {
  {"Поворот основания", 1, -1, -150, 150},
  {"Плечо",             2, +1, -100, 100},
  {"Локоть",            3, +1, -130, 130},
  {"Наклон кисти",      4, +1, -120, 120},
  {"Поворот кисти",     5, +1, -150, 150},
  {"Наклон схвата",     6, +1, -110, 110},
  {"Схват",             7, +1,    0,  70},    // 0° — раскрыт, 70° — сомкнут
};
#endif
const uint16_t ARM_ZERO = 2048;   // положение привода в позе модели (середина после подготовки)
const uint8_t ST_TORQUE = 40, ST_ACC = 41, ST_PRESENT = 56;
const uint8_t ARM_ACC = 40;       // ускорение (0 — максимальное, рывком)

float armTarget[ARM_N];           // цель, градусы от позы модели
int16_t armPos[ARM_N];            // текущее положение, шаги 0..4095 (-1 — неизвестно)
bool armFound[ARM_N];
uint8_t armFail[ARM_N];
uint8_t armErr[ARM_N];            // байт ошибки из ответа привода (перегрев, перегрузка, питание…)
// Качество связи: доля успешных чтений за последние 50 попыток, %
uint8_t armTries[ARM_N], armOks[ARM_N], armLink[ARM_N];
const uint8_t ARM_TIMEOUT_MS = 8;   // ожидание ответа привода
const uint8_t ARM_LOST_AFTER = 5;   // привод считается пропавшим после стольких неудач подряд
bool armTorque = false;           // удержание включено
bool armDirty = false;            // есть новая цель — отправить
uint16_t armSpeed = 800;          // шагов/с для самого дальнего сустава
uint8_t armCmd = 0;               // 1 — удерживать, 2 — отпустить, 3 — поза модели
uint8_t armProbe = 0;             // какой ненайденный привод опросить в этом цикле

int armDegToSteps(uint8_t i, float deg) {
  deg = constrain(deg, (float)ARM[i].minDeg, (float)ARM[i].maxDeg);
  long s = ARM_ZERO + lroundf(ARM[i].dir * deg * 4096.0f / 360.0f);
  return constrain(s, 0L, 4095L);
}

float armStepsToDeg(uint8_t i, int steps) {
  return (steps - (int)ARM_ZERO) * 360.0f / 4096.0f * ARM[i].dir;
}

bool ftRead(uint8_t id, uint8_t addr, uint8_t *d, uint8_t n, uint8_t *err) {
  uint8_t prm[2] = {addr, n};
  dxlSend(id, 0x02, prm, 2);
  return dxlReceive(id, err, d, n, ARM_TIMEOUT_MS) == n;
}

bool ftWrite1(uint8_t id, uint8_t addr, uint8_t v) {
  uint8_t prm[2] = {addr, v};
  dxlSend(id, 0x03, prm, 2);
  uint8_t err, dummy[1];
  return dxlReceive(id, &err, dummy, 0, 4) >= 0;
}

void armSetTorque(bool on) {
  for (uint8_t i = 0; i < ARM_N; i++)
    if (armFound[i]) ftWrite1(ARM[i].id, ST_TORQUE, on ? 1 : 0);
  armTorque = on;
}

// SYNC WRITE в регистры 41–47 (ускорение, цель, время, скорость) всем суставам сразу.
// Скорости подобраны так, чтобы все суставы пришли к цели одновременно.
void armSendGoals() {
  int goal[ARM_N];
  int dist[ARM_N];
  int maxd = 0;
  for (uint8_t i = 0; i < ARM_N; i++) {
    goal[i] = armDegToSteps(i, armTarget[i]);
    dist[i] = armPos[i] >= 0 ? abs(goal[i] - armPos[i]) : 0;
    if (dist[i] > maxd) maxd = dist[i];
  }
  uint8_t prm[2 + 8 * ARM_N];
  uint8_t n = 0;
  prm[n++] = ST_ACC;
  prm[n++] = 7;
  for (uint8_t i = 0; i < ARM_N; i++) {
    uint16_t spd = maxd > 0 ? (uint32_t)armSpeed * dist[i] / maxd : armSpeed;
    if (spd < 40) spd = 40;
    prm[n++] = ARM[i].id;
    prm[n++] = ARM_ACC;
    prm[n++] = goal[i] & 0xFF;  prm[n++] = goal[i] >> 8;
    prm[n++] = 0;               prm[n++] = 0;          // время — не используется
    prm[n++] = spd & 0xFF;      prm[n++] = spd >> 8;
  }
  dxlSend(0xFE, 0x83, prm, n);
}

void armReadPositions() {
  for (uint8_t i = 0; i < ARM_N; i++) {
    // найденные приводы читаем каждый цикл, ненайденные — по одному за цикл
    if (!armFound[i] && i != armProbe) continue;
    uint8_t d[2], err = 0;
    bool ok = ftRead(ARM[i].id, ST_PRESENT, d, 2, &err);
    if (!ok && armFound[i]) ok = ftRead(ARM[i].id, ST_PRESENT, d, 2, &err);  // одна повторная попытка
    if (armFound[i] || ok) {
      armTries[i]++;
      if (ok) armOks[i]++;
      if (armTries[i] >= 50) { armLink[i] = armOks[i] * 2; armTries[i] = 0; armOks[i] = 0; }
    }
    if (ok) {
      int v = d[0] | (d[1] << 8);
      if (v & 0x8000) v = -(v & 0x7FFF);    // знак в старшем бите
      armPos[i] = constrain(v, 0, 4095);
      armErr[i] = err;
      armFound[i] = true;
      armFail[i] = 0;
    } else if (++armFail[i] >= ARM_LOST_AFTER) {
      armFound[i] = false;
      armPos[i] = -1;
    }
  }
  do { armProbe = (armProbe + 1) % ARM_N; } while (armFound[armProbe] && armFoundCount() < ARM_N);
}

uint8_t armFoundCount() {
  uint8_t c = 0;
  for (uint8_t i = 0; i < ARM_N; i++) c += armFound[i];
  return c;
}

void armCycle() {
  setBus(SERVO_BAUD);
  armReadPositions();
  if (armCmd == 1) {                        // удерживать там, где стоит
    for (uint8_t i = 0; i < ARM_N; i++)
      if (armPos[i] >= 0) armTarget[i] = armStepsToDeg(i, armPos[i]);
    armDirty = true;
  } else if (armCmd == 2) {                 // отпустить
    armSetTorque(false);
    armDirty = false;
  } else if (armCmd == 3) {                 // поза модели, схват раскрыт
    for (uint8_t i = 0; i < ARM_N; i++) armTarget[i] = 0;
    armDirty = true;
  }
  armCmd = 0;
  if (armDirty && armFoundCount() > 0) {
    armSendGoals();
    if (!armTorque) armSetTorque(true);     // движение ползунком включает удержание
    armDirty = false;
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
  char buf[1700];
  snprintf(buf, sizeof(buf),
    "{\"ard\":%d,\"line\":[%d,%d,%d,%d],\"dist\":%d,\"endL\":%d,\"endR\":%d,\"abtn\":%d,"
    "\"imu\":%d,\"acc\":[%d,%d,%d],\"gyr\":[%d,%d,%d],\"mag\":[%d,%d,%d],"
    "\"m72\":%d,\"m71\":%d,"
    "\"pw\":[%d,%d,%d,%d],\"enc\":[%d,%d,%d,%d],\"spd\":[%d,%d,%d,%d],"
    "\"cl\":%d,\"blk\":%d,\"estop\":%d,\"up\":%lu,\"rst\":\"%s\",\"boots\":%lu,\"heap\":%lu,",
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
  // манипулятор
  size_t L = strlen(buf);
  L += snprintf(buf + L, sizeof(buf) - L, "\"arm\":{\"tq\":%d,\"sp\":%u,\"f\":[", armTorque, armSpeed);
  for (uint8_t i = 0; i < ARM_N; i++) L += snprintf(buf + L, sizeof(buf) - L, "%s%d", i ? "," : "", armFound[i]);
  L += snprintf(buf + L, sizeof(buf) - L, "],\"a\":[");
  for (uint8_t i = 0; i < ARM_N; i++) {
    if (armPos[i] >= 0) L += snprintf(buf + L, sizeof(buf) - L, "%s%.1f", i ? "," : "", armStepsToDeg(i, armPos[i]));
    else                L += snprintf(buf + L, sizeof(buf) - L, "%snull", i ? "," : "");
  }
  L += snprintf(buf + L, sizeof(buf) - L, "],\"t\":[");
  for (uint8_t i = 0; i < ARM_N; i++) L += snprintf(buf + L, sizeof(buf) - L, "%s%.1f", i ? "," : "", armTarget[i]);
  L += snprintf(buf + L, sizeof(buf) - L, "],\"e\":[");
  for (uint8_t i = 0; i < ARM_N; i++) L += snprintf(buf + L, sizeof(buf) - L, "%s%d", i ? "," : "", armErr[i]);
  L += snprintf(buf + L, sizeof(buf) - L, "],\"q\":[");
  for (uint8_t i = 0; i < ARM_N; i++) L += snprintf(buf + L, sizeof(buf) - L, "%s%d", i ? "," : "", armLink[i]);
  snprintf(buf + L, sizeof(buf) - L, "]}}");
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

// /armcfg — описание манипулятора для страницы
void handleArmCfg() {
  String j = "{\"type\":\"";
  j += ARM_TYPE;
  j += "\",\"joints\":[";
  for (uint8_t i = 0; i < ARM_N; i++) {
    if (i) j += ",";
    j += "{\"n\":\""; j += ARM[i].name; j += "\",\"min\":"; j += ARM[i].minDeg;
    j += ",\"max\":"; j += ARM[i].maxDeg; j += "}";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

// /arm?t=угол1,…,угол7&sp=скорость  или  /arm?cmd=hold|release|home
void handleArm() {
  if (server.hasArg("sp")) armSpeed = constrain(server.arg("sp").toInt(), 50, 3000);
  if (server.hasArg("t")) {
    String t = server.arg("t");
    int from = 0;
    for (uint8_t i = 0; i < ARM_N; i++) {
      int comma = t.indexOf(',', from);
      String part = comma < 0 ? t.substring(from) : t.substring(from, comma);
      if (part.length()) armTarget[i] = constrain(part.toFloat(), (float)ARM[i].minDeg, (float)ARM[i].maxDeg);
      if (comma < 0) break;
      from = comma + 1;
    }
    armDirty = true;
  }
  String c = server.arg("cmd");
  if (c == "hold") armCmd = 1;
  else if (c == "release") armCmd = 2;
  else if (c == "home") armCmd = 3;
  server.send(200, "text/plain", "ok");
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

  dxl.begin(MOTOR_BAUD, SERIAL_8N1, DXL_RX, DXL_TX);
  for (uint8_t i = 0; i < ARM_N; i++) { armPos[i] = -1; armTarget[i] = 0; armLink[i] = 100; }
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
  server.on("/armcfg", handleArmCfg);
  server.on("/arm", handleArm);
  server.onNotFound([]() { server.sendHeader("Location", "/"); server.send(302); });
  server.begin();

  tone(BUZZER, 1000, 100);
}

uint32_t tMotors = 0, tEnc = 0, tLeds = 0, tArm = 0;

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
    // Мощность отправляем, только когда она изменилась, и раз в 300 мс для проверки связи:
    // чем реже шина переключается между 57600 и 1 Мбит/с, тем надёжнее связь с сервоприводами.
    static int16_t lastSent[4] = {1, 1, 1, 1};
    static uint32_t tKeep = 0;
    if (memcmp(lastSent, wheelPower, sizeof(lastSent)) != 0 || now - tKeep >= 300) {
      sendMotors();
      memcpy(lastSent, wheelPower, sizeof(lastSent));
      tKeep = now;
    }
  }

  // Энкодеры — каждые 150 мс при движении, раз в секунду на стоянке
  bool movingNow = wheelPower[0] || wheelPower[1] || wheelPower[2] || wheelPower[3];
  if (now - tEnc >= (movingNow ? 150UL : 1000UL)) {
    tEnc = now;
    readEncoders();
  }

  // Манипулятор — каждые 60 мс (команды со страницы — без ожидания)
  if (now - tArm >= 60 || ((armCmd || armDirty) && now - tArm >= 20)) {
    tArm = now;
    armCycle();
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
