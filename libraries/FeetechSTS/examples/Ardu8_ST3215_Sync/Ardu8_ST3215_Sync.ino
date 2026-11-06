/*
  Ardu8 + Feetech ST3215: синхронное движение нескольких приводов.

  Одна команда SYNC WRITE задаёт цели всем приводам сразу — они начинают
  движение одновременно. Положения читаются одной командой SYNC READ.
  Приводы с ID 1, 2, 3 (смените ID примером SetID). Ardu8: DIP3 = ON, DIP4 = OFF.
*/
#include <FeetechSTS.h>

const uint8_t IDS[] = { 1, 2, 3 };
const uint8_t N = sizeof(IDS);

FeetechBus bus;

void setup() {
  Serial.begin(115200);
  bus.begin(Serial2, 1000000, 33, 32);

  for (uint8_t i = 0; i < N; i++) {
    FeetechServo s(bus, IDS[i]);
    bool ok = s.ping();
    Serial.printf("ID %d: %s\n", IDS[i], ok ? "на связи" : "НЕ ОТВЕЧАЕТ");
    s.setMode(STS_MODE_POSITION);
    s.hold();             // удержание без рывка при старте
  }
}

void loop() {
  // Волна: каждый следующий привод сдвинут по фазе
  static float phase = 0;
  int16_t goal[N];
  for (uint8_t i = 0; i < N; i++) {
    goal[i] = STS_POSITION_MID + (int16_t)(800 * sin(phase + i * 1.0));
  }
  bus.syncMoveTo(IDS, goal, N, 3000, 0);
  phase += 0.15;

  static uint32_t t = 0;
  if (millis() - t >= 250) {
    t = millis();
    int16_t pos[N];
    uint8_t got = bus.syncReadPositions(IDS, N, pos);
    if (got < N) {
      // Если старая прошивка привода не знает SYNC READ — читаем по одному
      for (uint8_t i = 0; i < N; i++) pos[i] = FeetechServo(bus, IDS[i]).readPosition();
    }
    for (uint8_t i = 0; i < N; i++) Serial.printf("ID%d=%5d  ", IDS[i], pos[i]);
    Serial.println();
  }
  delay(30);
}
