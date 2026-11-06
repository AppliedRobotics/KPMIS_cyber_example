/*
  Ardu8 — поиск модулей моторов (STM8) на шине DXL и проверка моторов с ESP32.

  Протокол: Dynamixel Protocol 1.0, 57600 бод (по умолчанию в прошивке STM8).
  ESP32 работает с шиной через Serial2: TX = IO32, RX = IO33.

  Перед запуском:
    - DIP3 = ON (ESP32 подключён к шине DXL), DIP4 = OFF (Arduino отключён от шины)
    - Плата в Arduino IDE: "ESP32 Wrover Module", USB — нижний USB-C
    - Монитор порта: 115200
    - Для вращения моторов нужно питание VCC (XT60 или DC) и выключатель в ON.
      Поиск модулей работает и от одного USB.

  Что делает:
    1) Перебирает ID 0..253 и печатает найденные модули (ID, модель, версия).
    2) По нажатию крайней левой нижней кнопки (BUT4_ESP, IO26) по очереди
       крутит каждый мотор каждого модуля вперёд и назад (мощность 400 из 1023)
       и печатает показания энкодеров.
*/

// ---------- Параметры шины ----------
const int DXL_TX = 32;
const int DXL_RX = 33;
const uint32_t DXL_BAUD = 57600;
HardwareSerial &dxl = Serial2;

// ---------- Регистры прошивки STM8 (stm8-common-sensors-2) ----------
const uint8_t REG_MODEL_NUMBER = 0;   // uint16, R
const uint8_t REG_FIRMWARE     = 2;   // uint8,  R
const uint8_t REG_ID           = 3;   // uint8,  R (сменить по шине нельзя)
const uint8_t REG_MAX_POWER    = 24;  // uint16  0..1023
const uint8_t REG_ENCODER_MODE = 27;  // uint8   0 выкл, 1 AB, 2 A, 3 A по фронту
const uint8_t REG_M1_POWER     = 28;  // int16  -1023..1023 (запись включает режим PWM)
const uint8_t REG_M2_POWER     = 30;  // int16
const uint8_t REG_M1_ENCODER   = 32;  // int16, тики
const uint8_t REG_M2_ENCODER   = 36;  // int16, тики

const uint8_t BTN_START = 26;  // крайняя левая кнопка нижней группы
const uint8_t BUZZER    = 19;

// ---------- Минимальная реализация Dynamixel 1.0 ----------
uint8_t txBuf[16];
uint8_t txLen = 0;

// Отправить пакет: FF FF ID LEN INST PARAMS... CHK
void dxlSend(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t n) {
  uint8_t i = 0;
  txBuf[i++] = 0xFF;
  txBuf[i++] = 0xFF;
  txBuf[i++] = id;
  txBuf[i++] = n + 2;
  txBuf[i++] = inst;
  uint8_t sum = id + (n + 2) + inst;
  for (uint8_t k = 0; k < n; k++) {
    txBuf[i++] = params[k];
    sum += params[k];
  }
  txBuf[i++] = ~sum;
  txLen = i;

  while (dxl.available()) dxl.read();  // очистить мусор
  dxl.write(txBuf, txLen);
  dxl.flush();                          // дождаться окончания передачи
}

int dxlReadByte(uint32_t deadline) {
  while (!dxl.available()) {
    if ((int32_t)(millis() - deadline) > 0) return -1;
  }
  return dxl.read();
}

// Принять пакет ответа. Если шина возвращает эхо нашего же пакета — пропускаем его.
// Возвращает число байт параметров (>=0) или -1 при ошибке/таймауте.
int dxlReceive(uint8_t id, uint8_t *err, uint8_t *params, uint8_t maxParams, uint32_t timeoutMs) {
  uint32_t deadline = millis() + timeoutMs;
  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    uint8_t pkt[32];
    uint8_t p = 0;
    // поиск заголовка FF FF
    int prev = -1, c;
    while (true) {
      c = dxlReadByte(deadline);
      if (c < 0) return -1;
      if (prev == 0xFF && c == 0xFF) break;
      prev = c;
    }
    pkt[p++] = 0xFF; pkt[p++] = 0xFF;
    int rid;
    do {
      rid = dxlReadByte(deadline);
    } while (rid == 0xFF);  // лишние FF
    if (rid < 0) return -1;
    int len = dxlReadByte(deadline);
    if (len < 2 || len > 28) return -1;
    pkt[p++] = rid; pkt[p++] = len;
    for (int k = 0; k < len; k++) {
      c = dxlReadByte(deadline);
      if (c < 0) return -1;
      pkt[p++] = c;
    }
    // эхо собственного пакета?
    if (p == txLen && memcmp(pkt, txBuf, txLen) == 0) continue;

    uint8_t sum = 0;
    for (uint8_t k = 2; k < p - 1; k++) sum += pkt[k];
    if ((uint8_t)~sum != pkt[p - 1]) return -1;  // контрольная сумма
    if (rid != id) return -1;

    *err = pkt[4];
    int n = len - 2;
    if (n > maxParams) n = maxParams;
    memcpy(params, &pkt[5], n);
    return n;
  }
  return -1;
}

bool dxlPing(uint8_t id) {
  dxlSend(id, 0x01, nullptr, 0);
  uint8_t err, dummy[1];
  return dxlReceive(id, &err, dummy, 0, 8) >= 0;
}

bool dxlRead(uint8_t id, uint8_t addr, uint8_t *data, uint8_t size) {
  uint8_t prm[2] = {addr, size};
  dxlSend(id, 0x02, prm, 2);
  uint8_t err;
  int n = dxlReceive(id, &err, data, size, 20);
  return n == size && err == 0;
}

bool dxlWrite(uint8_t id, uint8_t addr, const uint8_t *data, uint8_t size) {
  uint8_t prm[8];
  prm[0] = addr;
  memcpy(&prm[1], data, size);  // прошивка STM8 принимает до 7 байт данных за раз
  dxlSend(id, 0x03, prm, size + 1);
  uint8_t err, dummy[1];
  int n = dxlReceive(id, &err, dummy, 0, 20);
  return n >= 0 && err == 0;
}

bool dxlWrite8(uint8_t id, uint8_t addr, uint8_t v) {
  return dxlWrite(id, addr, &v, 1);
}

bool dxlWrite16(uint8_t id, uint8_t addr, int16_t v) {
  uint8_t d[2] = {(uint8_t)(v & 0xFF), (uint8_t)((uint16_t)v >> 8)};
  return dxlWrite(id, addr, d, 2);
}

bool dxlRead16(uint8_t id, uint8_t addr, int16_t &v) {
  uint8_t d[2];
  if (!dxlRead(id, addr, d, 2)) return false;
  v = (int16_t)(d[0] | (d[1] << 8));
  return true;
}

// ---------- Логика теста ----------
uint8_t found[8];
uint8_t foundCount = 0;

void scanBus() {
  Serial.println("Поиск модулей на шине DXL (ID 0..253)...");
  foundCount = 0;
  for (int id = 0; id <= 253; id++) {
    if (dxlPing(id)) {
      uint8_t info[3] = {0, 0, 0};
      dxlRead(id, REG_MODEL_NUMBER, info, 3);
      uint16_t model = info[0] | (info[1] << 8);
      Serial.printf("  найден ID %d: модель 0x%04X, прошивка %u\n", id, model, info[2]);
      if (foundCount < 8) found[foundCount++] = id;
    }
  }
  Serial.printf("Найдено модулей: %u\n", foundCount);
}

void printEncoders(uint8_t id) {
  int16_t e1 = 0, e2 = 0;
  bool ok1 = dxlRead16(id, REG_M1_ENCODER, e1);
  bool ok2 = dxlRead16(id, REG_M2_ENCODER, e2);
  Serial.printf("    ID %u: энкодер M1 = %s%d, M2 = %s%d\n", id,
                ok1 ? "" : "(ошибка) ", e1, ok2 ? "" : "(ошибка) ", e2);
}

void runMotor(uint8_t id, uint8_t powerReg, const char *name, int16_t power, uint32_t ms) {
  Serial.printf("  ID %u %s: мощность %d\n", id, name, power);
  dxlWrite16(id, powerReg, power);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    printEncoders(id);
    delay(250);
  }
  dxlWrite16(id, powerReg, 0);
  delay(300);
}

void motorTest() {
  if (foundCount == 0) {
    Serial.println("Модули не найдены — проверьте DIP3 = ON и DIP4 = OFF.");
    return;
  }
  for (uint8_t i = 0; i < foundCount; i++) {
    uint8_t id = found[i];
    dxlWrite8(id, REG_ENCODER_MODE, 1);   // энкодер в режиме AB (полный квадратурный)
    dxlWrite16(id, REG_MAX_POWER, 1023);

    runMotor(id, REG_M1_POWER, "M1 вперёд", 400, 1500);
    runMotor(id, REG_M1_POWER, "M1 назад", -400, 1500);
    runMotor(id, REG_M2_POWER, "M2 вперёд", 400, 1500);
    runMotor(id, REG_M2_POWER, "M2 назад", -400, 1500);
  }
  Serial.println("Тест моторов завершён.");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nArdu8 ESP32: тест модулей моторов");

  pinMode(BTN_START, INPUT);
  dxl.begin(DXL_BAUD, SERIAL_8N1, DXL_RX, DXL_TX);
  delay(500);  // дать STM8 загрузиться

  scanBus();
  tone(BUZZER, foundCount ? 1500 : 300, 150);
  Serial.println("Нажмите крайнюю левую нижнюю кнопку, чтобы запустить моторы.");
  Serial.println("ВНИМАНИЕ: колёса будут вращаться — поднимите робота.");
}

void loop() {
  if (digitalRead(BTN_START) == HIGH) {
    tone(BUZZER, 1000, 80);
    delay(300);
    motorTest();
    Serial.println("Нажмите кнопку ещё раз для повтора.");
  }
  delay(20);
}
