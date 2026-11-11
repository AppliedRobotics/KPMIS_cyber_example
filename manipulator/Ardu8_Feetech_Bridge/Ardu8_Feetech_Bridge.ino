/*
  Ardu8_Feetech_Bridge — мост «USB ↔ шина DXL» для веб-приложения «Конфигуратор манипулятора».

  Прошивается в ESP32 робототехнического контроллера «Кибернетический» (Ardu8),
  нижний USB-C.
  В Arduino IDE: плата «ESP32 Wrover Module» (или «ESP32 Dev Module»).

  Переключатели: DIP3 = ON, DIP4 = OFF (шиной управляет ESP32).
  Привод — в 3-пиновый разъём XP2 или XP3.

  Поддерживаемые приводы: Feetech ST3215 (STS) и Dynamixel AX-12A.
  Оба работают по протоколу Dynamixel 1.0 (FF FF ID LEN INSTR ... CHECKSUM)
  на полудуплексной шине, поэтому мост передаёт пакеты как есть и не зависит
  от типа привода: регистры каждого типа знает страница. Приводы разных типов
  можно держать на одной шине (по умолчанию оба — 1 Мбит/с), ID не должны совпадать.

  Связь с компьютером: Serial (USB, CH340) на 1 000 000 бит/с.
  Связь с шиной: Serial2, RX = IO33, TX = IO32, скорость задаёт страница.

  Протокол между страницей и мостом (кадры):
    A5 5A CMD LEN DATA[LEN] XOR      (XOR = CMD ^ LEN ^ все байты DATA)
    CMD 1 — версия, ответ "FTBR1"
    CMD 2 — скорость шины, DATA = 4 байта (little-endian), ответ — установленная скорость
    CMD 3 — обмен: DATA = таймаут_мс, пакет протокола 1.0 (Feetech или Dynamixel); ответ — байты с шины без эха
    CMD 4 — поиск (PING): DATA = ID_от, ID_до, таймаут_мс; ответ — список ответивших ID любого типа

  Светодиоды ESP32: LED1 — мост работает, LED2 — мигает при обмене,
  RGB зелёный — привод ответил, красный — ответа нет.
*/
#include <Arduino.h>

const int BUS_RX = 33;
const int BUS_TX = 32;
const uint32_t USB_BAUD = 1000000;

const int LED_READY   = 27;  // LED1_ESP
const int LED_TRAFFIC = 14;  // LED2_ESP
const int RGB_R = 15, RGB_G = 2, RGB_B = 18;  // светодиоды ESP горят при LOW

enum { CMD_VERSION = 1, CMD_BAUD = 2, CMD_TXRX = 3, CMD_SCAN = 4 };

uint32_t busBaud = 1000000;
uint8_t inBuf[256];
uint8_t outBuf[256];
bool trafficLed = false;

void setRgb(bool r, bool g, bool b) {
  digitalWrite(RGB_R, r ? LOW : HIGH);
  digitalWrite(RGB_G, g ? LOW : HIGH);
  digitalWrite(RGB_B, b ? LOW : HIGH);
}

void toggleTraffic() {
  trafficLed = !trafficLed;
  digitalWrite(LED_TRAFFIC, trafficLed ? LOW : HIGH);
}

// Чтение байта из USB с таймаутом (мс). -1 — таймаут.
int readUsb(uint32_t timeoutMs) {
  uint32_t t = millis();
  while (!Serial.available()) {
    if (millis() - t > timeoutMs) return -1;
  }
  return Serial.read();
}

void sendFrame(uint8_t cmd, const uint8_t *data, uint8_t len) {
  uint8_t x = cmd ^ len;
  for (int i = 0; i < len; i++) x ^= data[i];
  Serial.write(0xA5);
  Serial.write(0x5A);
  Serial.write(cmd);
  Serial.write(len);
  if (len) Serial.write(data, len);
  Serial.write(x);
}

// Отправить пакет в шину и собрать ответ. Эхо собственной передачи отбрасывается.
int busTxRx(const uint8_t *pkt, int plen, uint32_t timeoutMs, uint8_t *out, int outMax) {
  while (Serial2.available()) Serial2.read();
  Serial2.write(pkt, plen);
  Serial2.flush();

  int n = 0, echo = 0;
  bool echoPhase = true, got = false;
  // пауза, после которой ответ считается законченным: ~3 байта + 1 мс
  uint32_t gapUs = 30UL * 1000000UL / busBaud + 1000;
  uint32_t toUs = timeoutMs * 1000UL;
  uint32_t start = micros(), last = start;

  for (;;) {
    if (Serial2.available()) {
      uint8_t v = Serial2.read();
      uint32_t now = micros();
      if (echoPhase) {
        if (v == pkt[echo]) {
          echo++;
          if (echo == plen) echoPhase = false;  // эхо целиком — выбрасываем
          continue;
        }
        // это было не эхо — вернуть совпавшие байты в ответ
        for (int i = 0; i < echo && n < outMax; i++) out[n++] = pkt[i];
        echoPhase = false;
      }
      if (n < outMax) out[n++] = v;
      got = true;
      last = now;
    } else {
      uint32_t now = micros();
      if (got) {
        if (now - last > gapUs) break;
      } else if (now - start > toUs) {
        break;
      }
    }
  }
  if (echoPhase) {
    for (int i = 0; i < echo && n < outMax; i++) out[n++] = pkt[i];
  }
  return n;
}

// Есть ли в буфере корректный ответ от данного ID
bool hasReplyFrom(const uint8_t *b, int n, uint8_t id) {
  for (int i = 0; i + 6 <= n; i++) {
    if (b[i] != 0xFF || b[i + 1] != 0xFF || b[i + 2] != id) continue;
    int len = b[i + 3];
    if (len < 2 || i + len + 4 > n) continue;
    uint8_t s = 0;
    for (int k = i + 2; k < i + len + 3; k++) s += b[k];
    if ((uint8_t)~s == b[i + len + 3]) return true;
  }
  return false;
}

void handle(uint8_t cmd, uint8_t len) {
  switch (cmd) {
    case CMD_VERSION: {
      const uint8_t v[] = {'F', 'T', 'B', 'R', '1'};
      sendFrame(CMD_VERSION, v, sizeof(v));
      break;
    }
    case CMD_BAUD: {
      if (len < 4) return;
      uint32_t b = inBuf[0] | (inBuf[1] << 8) | ((uint32_t)inBuf[2] << 16) | ((uint32_t)inBuf[3] << 24);
      if (b >= 9600 && b <= 3000000) {
        busBaud = b;
        Serial2.updateBaudRate(busBaud);
        while (Serial2.available()) Serial2.read();
      }
      uint8_t r[4] = {(uint8_t)busBaud, (uint8_t)(busBaud >> 8), (uint8_t)(busBaud >> 16), (uint8_t)(busBaud >> 24)};
      sendFrame(CMD_BAUD, r, 4);
      break;
    }
    case CMD_TXRX: {
      if (len < 7) return;
      toggleTraffic();
      int n = busTxRx(inBuf + 1, len - 1, inBuf[0], outBuf, 250);
      setRgb(n == 0, n > 0, false);
      sendFrame(CMD_TXRX, outBuf, n);
      break;
    }
    case CMD_SCAN: {
      if (len < 3) return;
      uint8_t from = inBuf[0], to = inBuf[1], tms = inBuf[2];
      uint8_t found[255];
      int cnt = 0;
      uint8_t ping[6] = {0xFF, 0xFF, 0, 2, 1, 0};
      setRgb(false, false, true);
      for (int id = from; id <= to && id <= 253; id++) {
        ping[2] = id;
        ping[5] = ~(uint8_t)(id + 2 + 1);
        if ((id & 7) == 0) toggleTraffic();
        int n = busTxRx(ping, 6, tms, outBuf, 250);
        if (hasReplyFrom(outBuf, n, id) && cnt < 254) found[cnt++] = id;
      }
      setRgb(false, cnt > 0, false);
      sendFrame(CMD_SCAN, found, cnt);
      break;
    }
  }
}

void setup() {
  Serial.setRxBufferSize(1024);
  Serial.begin(USB_BAUD);
  Serial2.setRxBufferSize(1024);
  Serial2.begin(busBaud, SERIAL_8N1, BUS_RX, BUS_TX);

  pinMode(LED_READY, OUTPUT);
  pinMode(LED_TRAFFIC, OUTPUT);
  pinMode(RGB_R, OUTPUT);
  pinMode(RGB_G, OUTPUT);
  pinMode(RGB_B, OUTPUT);
  digitalWrite(LED_TRAFFIC, HIGH);
  setRgb(false, false, false);
  digitalWrite(LED_READY, LOW);  // мост готов
}

void loop() {
  if (!Serial.available()) return;
  if (Serial.read() != 0xA5) return;
  if (readUsb(20) != 0x5A) return;
  int cmd = readUsb(20);
  int len = readUsb(20);
  if (cmd < 0 || len < 0) return;
  uint8_t x = cmd ^ len;
  for (int i = 0; i < len; i++) {
    int v = readUsb(20);
    if (v < 0) return;
    inBuf[i] = v;
    x ^= v;
  }
  int c = readUsb(20);
  if (c < 0 || (uint8_t)c != x) return;
  handle(cmd, len);
}
