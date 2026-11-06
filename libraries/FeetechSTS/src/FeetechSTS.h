/*
  FeetechSTS — библиотека для сервоприводов Feetech серии STS / ST
  (ST3215, STS3215, STS3032 и др.) по однопроводной полудуплексной шине TTL.

  Протокол:  FF FF ID LEN INST PARAMS... CHK,  CHK = ~(ID + LEN + INST + PARAMS)
  Многобайтные значения — little-endian. Знак у позиции/скорости — старший бит
  (sign-magnitude), а не дополнительный код.

  Основной сценарий — ESP32 (плата Ardu8: шина DXL на IO33 (RX) / IO32 (TX), DIP3 = ON).
  Работает и на других платах с HardwareSerial.
*/
#ifndef FEETECH_STS_H
#define FEETECH_STS_H

#include <Arduino.h>

// ---------------- Инструкции протокола ----------------
#define STS_INST_PING        0x01
#define STS_INST_READ        0x02
#define STS_INST_WRITE       0x03
#define STS_INST_REG_WRITE   0x04
#define STS_INST_ACTION      0x05
#define STS_INST_SYNC_READ   0x82
#define STS_INST_SYNC_WRITE  0x83

#define STS_BROADCAST_ID     0xFE

// ---------------- Регистры (таблица памяти STS / ST3215) ----------------
// EEPROM (сохраняются только после разблокировки STS_REG_LOCK = 0)
#define STS_REG_FIRMWARE_MAJOR     0
#define STS_REG_FIRMWARE_MINOR     1
#define STS_REG_MODEL              3   // 2 байта
#define STS_REG_ID                 5
#define STS_REG_BAUD_RATE          6
#define STS_REG_RETURN_DELAY       7
#define STS_REG_RESPONSE_LEVEL     8
#define STS_REG_MIN_ANGLE_LIMIT    9   // 2 байта
#define STS_REG_MAX_ANGLE_LIMIT    11  // 2 байта
#define STS_REG_MAX_TEMPERATURE    13
#define STS_REG_MAX_VOLTAGE        14
#define STS_REG_MIN_VOLTAGE        15
#define STS_REG_MAX_TORQUE         16  // 2 байта
#define STS_REG_P_COEF             21
#define STS_REG_D_COEF             22
#define STS_REG_I_COEF             23
#define STS_REG_OFFSET             31  // 2 байта
#define STS_REG_MODE               33
// RAM
#define STS_REG_TORQUE_ENABLE      40  // 0 выкл, 1 вкл, 128 — текущее положение = 2048
#define STS_REG_ACC                41
#define STS_REG_GOAL_POSITION      42  // 2 байта
#define STS_REG_GOAL_TIME          44  // 2 байта (в режиме ШИМ — скважность)
#define STS_REG_GOAL_SPEED         46  // 2 байта
#define STS_REG_TORQUE_LIMIT       48  // 2 байта, 0..1000
#define STS_REG_LOCK               55  // 0 — EEPROM открыт, 1 — закрыт
#define STS_REG_PRESENT_POSITION   56  // 2 байта
#define STS_REG_PRESENT_SPEED      58  // 2 байта
#define STS_REG_PRESENT_LOAD       60  // 2 байта
#define STS_REG_PRESENT_VOLTAGE    62  // 0,1 В
#define STS_REG_PRESENT_TEMPERATURE 63 // °C
#define STS_REG_STATUS             65
#define STS_REG_MOVING             66
#define STS_REG_PRESENT_CURRENT    69  // 2 байта

// ---------------- Режимы работы (регистр 33) ----------------
#define STS_MODE_POSITION  0   // сервопривод: угол 0..4095
#define STS_MODE_WHEEL     1   // колесо: регулирование скорости
#define STS_MODE_PWM       2   // открытый контур: мощность
#define STS_MODE_STEP      3   // шаговый режим (относительное перемещение)

// ---------------- Коды скорости шины (регистр 6) ----------------
#define STS_BAUD_1M        0
#define STS_BAUD_500K      1
#define STS_BAUD_250K      2
#define STS_BAUD_128K      3
#define STS_BAUD_115200    4
#define STS_BAUD_76800     5
#define STS_BAUD_57600     6
#define STS_BAUD_38400     7

// ---------------- Коды ошибок библиотеки ----------------
#define STS_OK             0
#define STS_ERR_TIMEOUT    1   // сервопривод не ответил
#define STS_ERR_CHECKSUM   2   // испорченный ответ
#define STS_ERR_PARAM      3   // неверные аргументы / шина не запущена

// Биты байта состояния в ответе сервопривода (lastServoStatus())
#define STS_STATUS_VOLTAGE     0x01
#define STS_STATUS_SENSOR      0x02
#define STS_STATUS_TEMPERATURE 0x04
#define STS_STATUS_CURRENT     0x08
#define STS_STATUS_OVERLOAD    0x20

// Значение, которое возвращают функции read...() при ошибке связи
#define STS_READ_ERROR     (-32768)

#define STS_POSITION_MAX   4095   // 4096 шагов на оборот
#define STS_POSITION_MID   2048
#define STS_MAX_PARAMS     250
#define STS_DEFAULT_TIMEOUT_MS 10

// Снимок состояния сервопривода (одно чтение 15 байт с адреса 56)
struct STSState {
  int16_t position;     // 0..4095
  int16_t speed;        // шаги/с, знак — направление
  int16_t load;         // 0,1 % от максимума, знак — направление
  float   voltage;      // В
  uint8_t temperature;  // °C
  uint8_t status;       // биты ошибок
  bool    moving;       // идёт движение
  int16_t current;      // единицы датчика тока (~6,5 мА для ST3215)
};

class FeetechBus {
public:
  FeetechBus();

#if defined(ESP32)
  // ESP32: свои пины UART. Ardu8: bus.begin(Serial2, 1000000, 33, 32);
  void begin(HardwareSerial& port, uint32_t baud, int8_t rxPin, int8_t txPin, int8_t dirPin = -1);
#endif
  // Стандартные пины UART. dirPin — пин направления для внешнего драйвера (-1 — не нужен)
  void begin(HardwareSerial& port, uint32_t baud, int8_t dirPin = -1);

  void     setBaudRate(uint32_t baud);      // сменить скорость шины на лету
  uint32_t baudRate() const { return _baud; }
  void     setTimeout(uint16_t ms) { _timeoutUs = (uint32_t)ms * 1000UL; }
  void     setAckWrites(bool on) { _ackWrites = on; }   // ждать ответ на запись
  bool     ackWrites() const { return _ackWrites; }

  // Эхо: слышит ли RX собственную передачу. Определяется автоматически в begin().
  bool     detectEcho();
  void     setEcho(bool on) { _echo = on; }
  bool     echo() const { return _echo; }

  // ---- Базовые операции ----
  bool ping(uint8_t id);
  bool read(uint8_t id, uint8_t addr, uint8_t* data, uint8_t len);
  bool write(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len);
  bool regWrite(uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len);
  bool action(uint8_t id = STS_BROADCAST_ID);

  bool readByte(uint8_t id, uint8_t addr, uint8_t& value);
  bool readWord(uint8_t id, uint8_t addr, uint16_t& value);
  bool writeByte(uint8_t id, uint8_t addr, uint8_t value);
  bool writeWord(uint8_t id, uint8_t addr, uint16_t value);

  // SYNC WRITE: count приводов, каждому len байт подряд из data (без ответа)
  bool syncWrite(uint8_t addr, uint8_t len, const uint8_t* ids, const uint8_t* data, uint8_t count);
  // SYNC READ: результат в out (count * len байт). okFlags — успех по каждому ID (можно nullptr).
  // Возвращает число ответивших приводов.
  uint8_t syncRead(uint8_t addr, uint8_t len, const uint8_t* ids, uint8_t count,
                   uint8_t* out, bool* okFlags = nullptr);

  // ---- Групповые команды ----
  bool    syncMoveTo(const uint8_t* ids, const int16_t* positions, uint8_t count,
                     uint16_t speed = 0, uint8_t acc = 0);          // до 35 приводов
  uint8_t syncReadPositions(const uint8_t* ids, uint8_t count, int16_t* positions); // до 124

  uint8_t lastError() const { return _err; }
  uint8_t lastServoStatus() const { return _status; }

  static uint32_t baudCodeToRate(uint8_t code);

private:
  void   attach(HardwareSerial& port, uint32_t baud, int8_t dirPin);
  void   sendPacket(uint8_t id, uint8_t inst, const uint8_t* params, uint8_t n);
  int    rxByte(uint32_t start, uint32_t tmo);
  int8_t readPacket(uint32_t start, uint32_t tmo);
  bool   isEcho() const;
  bool   receiveStatus(uint8_t id, uint8_t* out, uint8_t n);
  bool   waitAck(uint8_t id);
  bool   writeInst(uint8_t inst, uint8_t id, uint8_t addr, const uint8_t* data, uint8_t len);
  bool   fail(uint8_t e) { _err = e; return false; }

  HardwareSerial* _port;
  int8_t   _dirPin;
  uint32_t _timeoutUs;
  bool     _ackWrites;
  bool     _echo;
  uint32_t _baud;
  uint8_t  _err;
  uint8_t  _status;

  uint8_t  _tx[256];
  uint16_t _txLen;
  uint8_t  _pbuf[STS_MAX_PARAMS];
  uint8_t  _rxId;
  uint8_t  _rxLen;
  uint8_t  _rxBuf[256];
};

class FeetechServo {
public:
  FeetechServo(FeetechBus& bus, uint8_t id);

  uint8_t id() const { return _id; }
  void    attachId(uint8_t id) { _id = id; }   // только сменить ID в объекте
  FeetechBus& bus() { return *_bus; }

  bool ping();
  bool torque(bool on);
  bool hold();               // включить удержание в текущем положении, без рывка
  bool setMode(uint8_t mode);

  // Режим позиции. pos 0..4095 (2048 — середина), speed шаги/с (0 = максимум),
  // acc ускорение 0..254 (0 = без плавного разгона)
  bool moveTo(int16_t pos, uint16_t speed = 0, uint8_t acc = 0);
  bool moveToDeg(float deg, uint16_t speed = 0, uint8_t acc = 0);   // 0..360°
  bool regMoveTo(int16_t pos, uint16_t speed = 0, uint8_t acc = 0); // выполнится по bus.action()

  // Режим колеса: speed шаги/с, знак — направление
  bool setSpeed(int16_t speed, uint8_t acc = 0);
  // Режим ШИМ (STS_MODE_PWM): -1000..1000
  bool setPWM(int16_t pwm);
  bool setTorqueLimit(uint16_t limit);   // 0..1000 (RAM)

  // Чтение. При ошибке — STS_READ_ERROR (для voltage — NAN)
  int   readPosition();
  float readDeg();
  int   readSpeed();
  int   readLoad();
  float readVoltage();
  int   readTemperature();
  int   readCurrent();
  int   isMoving();          // 1 / 0, при ошибке -1
  bool  readState(STSState& st);
  bool  readModel(uint16_t& model);

  // Настройка (пишется в EEPROM)
  bool unlockEEPROM();
  bool lockEEPROM();
  bool changeId(uint8_t newId);          // подключайте один привод!
  bool changeBaud(uint8_t baudCode);     // меняет и скорость шины
  bool setAngleLimits(uint16_t minPos, uint16_t maxPos);
  bool calibrateMiddle();                // текущее положение станет 2048

  static int16_t degToPos(float deg);
  static float   posToDeg(int pos);

private:
  FeetechBus* _bus;
  uint8_t     _id;
};

#endif
