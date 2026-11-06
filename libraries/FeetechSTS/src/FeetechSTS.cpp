#include "FeetechSTS.h"
#include <string.h>
#include <math.h>

// ---------------- Вспомогательные функции ----------------

// Знак в старшем бите (sign-magnitude), как у Feetech STS
static uint16_t toSM(int16_t v, uint8_t signBit) {
  uint16_t mask = (uint16_t)((1u << signBit) - 1u);
  if (v < 0) return (uint16_t)((uint16_t)(-(int32_t)v) & mask) | (uint16_t)(1u << signBit);
  return (uint16_t)v & mask;
}

static int16_t fromSM(uint16_t v, uint8_t signBit) {
  uint16_t mask = (uint16_t)((1u << signBit) - 1u);
  if (v & (1u << signBit)) return -(int16_t)(v & mask);
  return (int16_t)(v & mask);
}

// 7 байт с адреса 41: ACC, GOAL_POSITION(2), GOAL_TIME(2), GOAL_SPEED(2)
static void packMove(uint8_t* b, int16_t pos, uint16_t speed, uint8_t acc) {
  uint16_t p = toSM(pos, 15);
  if (speed > 0x7FFF) speed = 0x7FFF;
  b[0] = acc;
  b[1] = p & 0xFF;  b[2] = p >> 8;
  b[3] = 0;         b[4] = 0;
  b[5] = speed & 0xFF; b[6] = speed >> 8;
}

// ======================= FeetechBus =======================

FeetechBus::FeetechBus()
  : _port(nullptr), _dirPin(-1), _timeoutUs((uint32_t)STS_DEFAULT_TIMEOUT_MS * 1000UL),
    _ackWrites(true), _echo(false), _baud(0), _err(STS_OK), _status(0),
    _txLen(0), _rxId(0), _rxLen(0) {}

#if defined(ESP32)
void FeetechBus::begin(HardwareSerial& port, uint32_t baud, int8_t rxPin, int8_t txPin, int8_t dirPin) {
  port.setRxBufferSize(512);              // вызывать до begin()
  port.begin(baud, SERIAL_8N1, rxPin, txPin);
  attach(port, baud, dirPin);
}
#endif

void FeetechBus::begin(HardwareSerial& port, uint32_t baud, int8_t dirPin) {
  port.begin(baud);
  attach(port, baud, dirPin);
}

void FeetechBus::attach(HardwareSerial& port, uint32_t baud, int8_t dirPin) {
  _port = &port;
  _baud = baud;
  _dirPin = dirPin;
  if (_dirPin >= 0) {
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_dirPin, LOW);
  }
  delay(5);
  detectEcho();
}

void FeetechBus::setBaudRate(uint32_t baud) {
  if (!_port) return;
  _port->flush();
#if defined(ESP32)
  _port->updateBaudRate(baud);
#else
  _port->begin(baud);
#endif
  _baud = baud;
  delay(2);
  while (_port->available()) _port->read();
}

uint32_t FeetechBus::baudCodeToRate(uint8_t code) {
  switch (code) {
    case STS_BAUD_1M:     return 1000000UL;
    case STS_BAUD_500K:   return 500000UL;
    case STS_BAUD_250K:   return 250000UL;
    case STS_BAUD_128K:   return 128000UL;
    case STS_BAUD_115200: return 115200UL;
    case STS_BAUD_76800:  return 76800UL;
    case STS_BAUD_57600:  return 57600UL;
    case STS_BAUD_38400:  return 38400UL;
    default:              return 0;
  }
}

// ---------------- Передача / приём ----------------

void FeetechBus::sendPacket(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t n) {
  while (_port->available()) _port->read();     // выбросить старый мусор

  uint8_t len = n + 2;
  uint8_t sum = id + len + inst;
  _tx[0] = 0xFF; _tx[1] = 0xFF; _tx[2] = id; _tx[3] = len; _tx[4] = inst;
  for (uint8_t i = 0; i < n; i++) {
    _tx[5 + i] = params[i];
    sum += params[i];
  }
  _tx[5 + n] = (uint8_t)~sum;
  _txLen = (uint16_t)n + 6;

  if (_dirPin >= 0) digitalWrite(_dirPin, HIGH);
  _port->write(_tx, _txLen);
  _port->flush();                               // дождаться конца передачи
  if (_dirPin >= 0) digitalWrite(_dirPin, LOW);
}

int FeetechBus::rxByte(uint32_t start, uint32_t tmo) {
  while (!_port->available()) {
    if ((uint32_t)(micros() - start) >= tmo) return -1;
  }
  return _port->read();
}

// 1 — пакет принят, 0 — тайм-аут, -1 — испорченный пакет
int8_t FeetechBus::readPacket(uint32_t start, uint32_t tmo) {
  int prev = -1, b;
  for (;;) {                                     // ищем заголовок FF FF
    b = rxByte(start, tmo);
    if (b < 0) return 0;
    if (prev == 0xFF && b == 0xFF) break;
    prev = b;
  }
  do {                                           // лишние FF перед ID
    b = rxByte(start, tmo);
    if (b < 0) return 0;
  } while (b == 0xFF);
  _rxId = (uint8_t)b;

  b = rxByte(start, tmo);
  if (b < 0) return 0;
  if (b < 2) return -1;
  _rxLen = (uint8_t)b;

  uint8_t sum = _rxId + _rxLen;
  for (uint16_t i = 0; i < _rxLen; i++) {
    b = rxByte(start, tmo);
    if (b < 0) return 0;
    _rxBuf[i] = (uint8_t)b;
    if (i + 1 < _rxLen) sum += (uint8_t)b;
  }
  if ((uint8_t)~sum != _rxBuf[_rxLen - 1]) return -1;
  return 1;
}

bool FeetechBus::isEcho() const {
  return _txLen >= 6 && _rxId == _tx[2] && _rxLen == _tx[3] &&
         memcmp(_rxBuf, _tx + 4, _rxLen) == 0;
}

bool FeetechBus::receiveStatus(uint8_t id, uint8_t* out, uint8_t n) {
  uint32_t start = micros();
  bool echoPending = _echo;
  bool bad = false;
  for (;;) {
    int8_t r = readPacket(start, _timeoutUs);
    if (r == 0) break;
    if (r < 0) { bad = true; continue; }
    if (echoPending && isEcho()) { echoPending = false; continue; }
    echoPending = false;
    if (_rxId != id) continue;                   // чужой пакет
    if (_rxLen != (uint16_t)n + 2) { bad = true; continue; }
    _status = _rxBuf[0];
    if (out && n) memcpy(out, _rxBuf + 1, n);
    _err = STS_OK;
    return true;
  }
  return fail(bad ? STS_ERR_CHECKSUM : STS_ERR_TIMEOUT);
}

bool FeetechBus::waitAck(uint8_t id) {
  if (id == STS_BROADCAST_ID || !_ackWrites) { _err = STS_OK; return true; }
  return receiveStatus(id, nullptr, 0);
}

bool FeetechBus::detectEcho() {
  if (!_port) return false;
  _echo = false;
  // Широковещательный READ: приводы на него не отвечают, значит, если что-то
  // и придёт — это наше собственное эхо.
  uint8_t p[2] = { 0, 1 };
  sendPacket(STS_BROADCAST_ID, STS_INST_READ, p, 2);
  uint32_t start = micros();
  for (;;) {
    int8_t r = readPacket(start, 3000);
    if (r == 0) break;
    if (r > 0 && isEcho()) { _echo = true; break; }
  }
  while (_port->available()) _port->read();
  return _echo;
}

// ---------------- Команды ----------------

bool FeetechBus::ping(uint8_t id) {
  if (!_port || id == STS_BROADCAST_ID) return fail(STS_ERR_PARAM);
  sendPacket(id, STS_INST_PING, nullptr, 0);
  return receiveStatus(id, nullptr, 0);
}

bool FeetechBus::read(uint8_t id, uint8_t addr, uint8_t* data, uint8_t len) {
  if (!_port || id == STS_BROADCAST_ID || len == 0 || len > STS_MAX_PARAMS) return fail(STS_ERR_PARAM);
  uint8_t p[2] = { addr, len };
  sendPacket(id, STS_INST_READ, p, 2);
  return receiveStatus(id, data, len);
}

bool FeetechBus::writeInst(uint8_t inst, uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len) {
  if (!_port || len == 0 || len >= STS_MAX_PARAMS) return fail(STS_ERR_PARAM);
  _pbuf[0] = addr;
  memcpy(_pbuf + 1, data, len);
  sendPacket(id, inst, _pbuf, len + 1);
  return waitAck(id);
}

bool FeetechBus::write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len) {
  return writeInst(STS_INST_WRITE, id, addr, data, len);
}

bool FeetechBus::regWrite(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len) {
  return writeInst(STS_INST_REG_WRITE, id, addr, data, len);
}

bool FeetechBus::action(uint8_t id) {
  if (!_port) return fail(STS_ERR_PARAM);
  sendPacket(id, STS_INST_ACTION, nullptr, 0);
  return waitAck(id);
}

bool FeetechBus::readByte(uint8_t id, uint8_t addr, uint8_t& value) {
  return read(id, addr, &value, 1);
}

bool FeetechBus::readWord(uint8_t id, uint8_t addr, uint16_t& value) {
  uint8_t d[2];
  if (!read(id, addr, d, 2)) return false;
  value = (uint16_t)d[0] | ((uint16_t)d[1] << 8);
  return true;
}

bool FeetechBus::writeByte(uint8_t id, uint8_t addr, uint8_t value) {
  return write(id, addr, &value, 1);
}

bool FeetechBus::writeWord(uint8_t id, uint8_t addr, uint16_t value) {
  uint8_t d[2] = { (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
  return write(id, addr, d, 2);
}

bool FeetechBus::syncWrite(uint8_t addr, uint8_t len, const uint8_t* ids, const uint8_t* data, uint8_t count) {
  uint16_t total = 2 + (uint16_t)count * (len + 1);
  if (!_port || count == 0 || len == 0 || total > STS_MAX_PARAMS) return fail(STS_ERR_PARAM);
  _pbuf[0] = addr;
  _pbuf[1] = len;
  uint16_t k = 2;
  for (uint8_t i = 0; i < count; i++) {
    _pbuf[k++] = ids[i];
    memcpy(_pbuf + k, data + (uint16_t)i * len, len);
    k += len;
  }
  sendPacket(STS_BROADCAST_ID, STS_INST_SYNC_WRITE, _pbuf, (uint8_t)k);
  _err = STS_OK;
  return true;
}

uint8_t FeetechBus::syncRead(uint8_t addr, uint8_t len, const uint8_t* ids, uint8_t count,
                             uint8_t* out, bool* okFlags) {
  if (okFlags) for (uint8_t i = 0; i < count; i++) okFlags[i] = false;
  if (!_port || count == 0 || len == 0 || (uint16_t)count + 2 > STS_MAX_PARAMS) {
    _err = STS_ERR_PARAM;
    return 0;
  }
  _pbuf[0] = addr;
  _pbuf[1] = len;
  memcpy(_pbuf + 2, ids, count);
  sendPacket(STS_BROADCAST_ID, STS_INST_SYNC_READ, _pbuf, count + 2);

  uint8_t seen[32];
  memset(seen, 0, sizeof(seen));
  uint8_t got = 0;
  bool bad = false;
  uint32_t start = micros();
  uint32_t tmo = _timeoutUs + (uint32_t)count * 1000UL;

  while (got < count) {
    int8_t r = readPacket(start, tmo);
    if (r == 0) break;
    if (r < 0) { bad = true; continue; }
    if (_rxId == STS_BROADCAST_ID) continue;     // эхо запроса
    if (_rxLen != (uint16_t)len + 2) continue;
    for (uint8_t j = 0; j < count; j++) {
      if (ids[j] == _rxId && !(seen[j >> 3] & (1 << (j & 7)))) {
        seen[j >> 3] |= (1 << (j & 7));
        memcpy(out + (uint16_t)j * len, _rxBuf + 1, len);
        if (okFlags) okFlags[j] = true;
        got++;
        break;
      }
    }
  }
  _err = (got == count) ? STS_OK : (bad ? STS_ERR_CHECKSUM : STS_ERR_TIMEOUT);
  return got;
}

bool FeetechBus::syncMoveTo(const uint8_t* ids, const int16_t* positions, uint8_t count,
                            uint16_t speed, uint8_t acc) {
  if (count == 0 || count > 35) return fail(STS_ERR_PARAM);
  uint8_t data[35 * 7];
  for (uint8_t i = 0; i < count; i++) packMove(data + i * 7, positions[i], speed, acc);
  return syncWrite(STS_REG_ACC, 7, ids, data, count);
}

uint8_t FeetechBus::syncReadPositions(const uint8_t* ids, uint8_t count, int16_t* positions) {
  if (count == 0 || count > 124) { _err = STS_ERR_PARAM; return 0; }
  uint8_t raw[124 * 2];
  bool ok[124];
  uint8_t got = syncRead(STS_REG_PRESENT_POSITION, 2, ids, count, raw, ok);
  for (uint8_t i = 0; i < count; i++) {
    positions[i] = ok[i] ? fromSM((uint16_t)raw[2 * i] | ((uint16_t)raw[2 * i + 1] << 8), 15)
                         : (int16_t)STS_READ_ERROR;
  }
  return got;
}

// ======================= FeetechServo =======================

FeetechServo::FeetechServo(FeetechBus& bus, uint8_t id) : _bus(&bus), _id(id) {}

int16_t FeetechServo::degToPos(float deg) {
  long p = lroundf(deg * 4096.0f / 360.0f);
  if (p < 0) p = 0;
  if (p > STS_POSITION_MAX) p = STS_POSITION_MAX;
  return (int16_t)p;
}

float FeetechServo::posToDeg(int pos) {
  return pos * 360.0f / 4096.0f;
}

bool FeetechServo::ping()               { return _bus->ping(_id); }
bool FeetechServo::torque(bool on)      { return _bus->writeByte(_id, STS_REG_TORQUE_ENABLE, on ? 1 : 0); }
bool FeetechServo::setMode(uint8_t m)   { return _bus->writeByte(_id, STS_REG_MODE, m); }
bool FeetechServo::calibrateMiddle()    { return _bus->writeByte(_id, STS_REG_TORQUE_ENABLE, 128); }
bool FeetechServo::unlockEEPROM()       { return _bus->writeByte(_id, STS_REG_LOCK, 0); }
bool FeetechServo::lockEEPROM()         { return _bus->writeByte(_id, STS_REG_LOCK, 1); }

bool FeetechServo::hold() {
  // Цель = текущее положение, иначе при включении момента привод
  // поедет к старой цели, оставшейся в регистре
  int p = readPosition();
  if (p == STS_READ_ERROR) return false;
  if (!moveTo((int16_t)p)) return false;
  return torque(true);
}

bool FeetechServo::moveTo(int16_t pos, uint16_t speed, uint8_t acc) {
  uint8_t b[7];
  packMove(b, pos, speed, acc);
  return _bus->write(_id, STS_REG_ACC, b, 7);
}

bool FeetechServo::moveToDeg(float deg, uint16_t speed, uint8_t acc) {
  return moveTo(degToPos(deg), speed, acc);
}

bool FeetechServo::regMoveTo(int16_t pos, uint16_t speed, uint8_t acc) {
  uint8_t b[7];
  packMove(b, pos, speed, acc);
  return _bus->regWrite(_id, STS_REG_ACC, b, 7);
}

bool FeetechServo::setSpeed(int16_t speed, uint8_t acc) {
  if (!_bus->writeByte(_id, STS_REG_ACC, acc)) return false;
  return _bus->writeWord(_id, STS_REG_GOAL_SPEED, toSM(speed, 15));
}

bool FeetechServo::setPWM(int16_t pwm) {
  if (pwm > 1000) pwm = 1000;
  if (pwm < -1000) pwm = -1000;
  return _bus->writeWord(_id, STS_REG_GOAL_TIME, toSM(pwm, 10));
}

bool FeetechServo::setTorqueLimit(uint16_t limit) {
  if (limit > 1000) limit = 1000;
  return _bus->writeWord(_id, STS_REG_TORQUE_LIMIT, limit);
}

int FeetechServo::readPosition() {
  uint16_t v;
  if (!_bus->readWord(_id, STS_REG_PRESENT_POSITION, v)) return STS_READ_ERROR;
  return fromSM(v, 15);
}

float FeetechServo::readDeg() {
  int p = readPosition();
  return (p == STS_READ_ERROR) ? NAN : posToDeg(p);
}

int FeetechServo::readSpeed() {
  uint16_t v;
  if (!_bus->readWord(_id, STS_REG_PRESENT_SPEED, v)) return STS_READ_ERROR;
  return fromSM(v, 15);
}

int FeetechServo::readLoad() {
  uint16_t v;
  if (!_bus->readWord(_id, STS_REG_PRESENT_LOAD, v)) return STS_READ_ERROR;
  return fromSM(v, 10);
}

float FeetechServo::readVoltage() {
  uint8_t v;
  if (!_bus->readByte(_id, STS_REG_PRESENT_VOLTAGE, v)) return NAN;
  return v / 10.0f;
}

int FeetechServo::readTemperature() {
  uint8_t v;
  if (!_bus->readByte(_id, STS_REG_PRESENT_TEMPERATURE, v)) return STS_READ_ERROR;
  return v;
}

int FeetechServo::readCurrent() {
  uint16_t v;
  if (!_bus->readWord(_id, STS_REG_PRESENT_CURRENT, v)) return STS_READ_ERROR;
  return fromSM(v, 15);
}

int FeetechServo::isMoving() {
  uint8_t v;
  if (!_bus->readByte(_id, STS_REG_MOVING, v)) return -1;
  return v ? 1 : 0;
}

bool FeetechServo::readState(STSState& st) {
  uint8_t d[15];
  if (!_bus->read(_id, STS_REG_PRESENT_POSITION, d, 15)) return false;
  st.position    = fromSM((uint16_t)d[0] | ((uint16_t)d[1] << 8), 15);
  st.speed       = fromSM((uint16_t)d[2] | ((uint16_t)d[3] << 8), 15);
  st.load        = fromSM((uint16_t)d[4] | ((uint16_t)d[5] << 8), 10);
  st.voltage     = d[6] / 10.0f;
  st.temperature = d[7];
  st.status      = d[9];
  st.moving      = d[10] != 0;
  st.current     = fromSM((uint16_t)d[13] | ((uint16_t)d[14] << 8), 15);
  return true;
}

bool FeetechServo::readModel(uint16_t& model) {
  return _bus->readWord(_id, STS_REG_MODEL, model);
}

bool FeetechServo::changeId(uint8_t newId) {
  if (newId >= STS_BROADCAST_ID) return false;
  if (!unlockEEPROM()) return false;
  bool ack = _bus->ackWrites();
  _bus->setAckWrites(false);                 // ответ может прийти уже с новым ID
  _bus->writeByte(_id, STS_REG_ID, newId);
  _bus->setAckWrites(ack);
  delay(20);
  _id = newId;
  lockEEPROM();
  return ping();
}

bool FeetechServo::changeBaud(uint8_t baudCode) {
  uint32_t rate = FeetechBus::baudCodeToRate(baudCode);
  if (rate == 0) return false;
  if (!unlockEEPROM()) return false;
  bool ack = _bus->ackWrites();
  _bus->setAckWrites(false);
  _bus->writeByte(_id, STS_REG_BAUD_RATE, baudCode);
  _bus->setAckWrites(ack);
  delay(20);
  _bus->setBaudRate(rate);
  delay(20);
  lockEEPROM();
  return ping();
}

bool FeetechServo::setAngleLimits(uint16_t minPos, uint16_t maxPos) {
  if (!unlockEEPROM()) return false;
  bool ok = _bus->writeWord(_id, STS_REG_MIN_ANGLE_LIMIT, minPos) &&
            _bus->writeWord(_id, STS_REG_MAX_ANGLE_LIMIT, maxPos);
  lockEEPROM();
  return ok;
}
