/*
  Ardu8 — тест светодиодов, кнопок и пищалки на стороне ESP32.

  Перед запуском:
    - Плата в Arduino IDE: "ESP32 Wrover Module" (пакет esp32 от Espressif)
    - USB — нижний USB-C на левом краю
    - Монитор порта: 115200

  Что делает:
    - по очереди зажигает LED1_ESP..LED7_ESP, в мониторе пишет номер и GPIO —
      так определяем цвета каналов RGB (LED5..LED7);
    - при нажатии кнопки пишет её номер и GPIO и коротко пищит
      (каждая кнопка — своей нотой).
*/

// Светодиоды: анод на 3.3 В, горят при LOW
const uint8_t LED_PINS[7] = {
  27,  // LED1_ESP
  14,  // LED2_ESP
  12,  // LED3_ESP
  13,  // LED4_ESP
  15,  // LED5_ESP (RGB)
  2,   // LED6_ESP (RGB)
  18   // LED7_ESP (RGB)
};

// Кнопки: подтянуты к GND, нажатие = HIGH
const uint8_t BTN_PINS[4] = {
  34,  // BUT1_ESP (только вход)
  35,  // BUT2_ESP (только вход)
  25,  // BUT3_ESP
  26   // BUT4_ESP
};

const uint8_t BUZZER_PIN = 19;  // пассивная пищалка через транзистор
const uint16_t BTN_NOTES[4] = {523, 659, 784, 1047};  // до, ми, соль, до

bool btnPrev[4] = {false, false, false, false};
uint8_t ledIndex = 0;
uint32_t lastLedMs = 0;

void allLedsOff() {
  for (uint8_t i = 0; i < 7; i++) digitalWrite(LED_PINS[i], HIGH);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Ardu8 ESP32 test: LEDs + buttons + buzzer");

  for (uint8_t i = 0; i < 7; i++) {
    pinMode(LED_PINS[i], OUTPUT);
  }
  allLedsOff();

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(BTN_PINS[i], INPUT);  // подтяжки внешние
  }

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Короткий сигнал при старте
  tone(BUZZER_PIN, 1000, 100);
}

void loop() {
  // Бегущий огонь, раз в 700 мс
  if (millis() - lastLedMs >= 700) {
    lastLedMs = millis();
    allLedsOff();
    digitalWrite(LED_PINS[ledIndex], LOW);
    Serial.printf("LED%u_ESP (GPIO%u)\n", ledIndex + 1, LED_PINS[ledIndex]);
    ledIndex = (ledIndex + 1) % 7;
  }

  // Кнопки: реагируем на нажатие (фронт)
  for (uint8_t i = 0; i < 4; i++) {
    bool pressed = digitalRead(BTN_PINS[i]) == HIGH;
    if (pressed && !btnPrev[i]) {
      Serial.printf("Нажата BUT%u_ESP (GPIO%u)\n", i + 1, BTN_PINS[i]);
      tone(BUZZER_PIN, BTN_NOTES[i], 120);
    }
    btnPrev[i] = pressed;
  }

  delay(10);
}
