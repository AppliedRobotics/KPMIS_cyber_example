// dxl.h — обмен с модулями моторов по шине DXL (протокол Dynamixel 1.0) для ESP32.
// Эту вкладку менять не нужно. Шина: Serial2, RX = IO33, TX = IO32, 57600 бод.
// Условия: DIP3 = ON, DIP4 = OFF. Для вращения моторов нужно питание XT60/DC.
#pragma once
#include <Arduino.h>

static HardwareSerial &dxl = Serial2;
static uint8_t dxlTx[16];
static uint8_t dxlTxLen = 0;

inline void dxlBegin() {
  dxl.begin(57600, SERIAL_8N1, 33, 32);
}

// Отправить пакет FF FF ID LEN INST PARAMS… CHK
inline void dxlSend(uint8_t id, uint8_t inst, const uint8_t *params, uint8_t n) {
  uint8_t i = 0, sum = id + (n + 2) + inst;
  dxlTx[i++] = 0xFF; dxlTx[i++] = 0xFF;
  dxlTx[i++] = id;   dxlTx[i++] = n + 2;  dxlTx[i++] = inst;
  for (uint8_t k = 0; k < n; k++) { dxlTx[i++] = params[k]; sum += params[k]; }
  dxlTx[i++] = ~sum;                     // контрольная сумма
  dxlTxLen = i;
  while (dxl.available()) dxl.read();    // очистить приёмник
  dxl.write(dxlTx, dxlTxLen);
  dxl.flush();
}

inline int dxlByte(uint32_t deadline) {
  while (!dxl.available()) {
    if ((int32_t)(millis() - deadline) > 0) return -1;
  }
  return dxl.read();
}

// Принять ответ модуля. Возвращает число байт данных или -1 при ошибке.
// Шина однопроводная, поэтому сначала может прийти эхо нашего пакета — его пропускаем.
inline int dxlReceive(uint8_t id, uint8_t *data, uint8_t maxData, uint32_t timeoutMs) {
  uint32_t deadline = millis() + timeoutMs;
  for (int attempt = 0; attempt < 2; attempt++) {
    uint8_t pkt[32]; uint8_t p = 0;
    int prev = -1, c;
    while (true) {                       // ищем заголовок FF FF
      c = dxlByte(deadline);
      if (c < 0) return -1;
      if (prev == 0xFF && c == 0xFF) break;
      prev = c;
    }
    pkt[p++] = 0xFF; pkt[p++] = 0xFF;
    int rid;
    do { rid = dxlByte(deadline); } while (rid == 0xFF);
    int len = dxlByte(deadline);
    if (rid < 0 || len < 2 || len > 28) return -1;
    pkt[p++] = rid; pkt[p++] = len;
    for (int k = 0; k < len; k++) {
      c = dxlByte(deadline);
      if (c < 0) return -1;
      pkt[p++] = c;
    }
    if (p == dxlTxLen && memcmp(pkt, dxlTx, p) == 0) continue;   // это эхо
    uint8_t sum = 0;
    for (uint8_t k = 2; k < p - 1; k++) sum += pkt[k];
    if ((uint8_t)~sum != pkt[p - 1] || rid != id) return -1;
    int n = len - 2;
    if (n > maxData) n = maxData;
    memcpy(data, &pkt[5], n);
    return n;
  }
  return -1;
}

// PING: ответил ли модуль
inline bool dxlPing(uint8_t id) {
  uint8_t d[1];
  dxlSend(id, 1, nullptr, 0);
  return dxlReceive(id, d, 0, 10) >= 0;
}

// Прочитать 2-байтовый регистр (число со знаком)
inline bool dxlRead16(uint8_t id, uint8_t reg, int16_t &value) {
  uint8_t prm[2] = {reg, 2}, d[2];
  dxlSend(id, 2, prm, 2);
  if (dxlReceive(id, d, 2, 20) != 2) return false;
  value = (int16_t)(d[0] | (d[1] << 8));   // младший байт первым
  return true;
}

// Записать 2-байтовый регистр
inline bool dxlWrite16(uint8_t id, uint8_t reg, int16_t value) {
  uint8_t prm[3] = {reg, (uint8_t)(value & 0xFF), (uint8_t)((uint16_t)value >> 8)}, d[1];
  dxlSend(id, 3, prm, 3);
  return dxlReceive(id, d, 0, 20) >= 0;
}

// Записать 1-байтовый регистр
inline bool dxlWrite8(uint8_t id, uint8_t reg, uint8_t value) {
  uint8_t prm[2] = {reg, value}, d[1];
  dxlSend(id, 3, prm, 2);
  return dxlReceive(id, d, 0, 20) >= 0;
}
