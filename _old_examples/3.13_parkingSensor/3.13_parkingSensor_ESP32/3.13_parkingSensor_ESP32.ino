// Парктроник - часть для ESP32 (управление buzzer)

#define BUZZER_PIN 19
#define BUZZER_PWM_CHANNEL 0

uint16_t currentFrequency = 0;
bool buzzerActive = false;

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 22, 23); // Связь с ATmega328
  Serial.begin(115200);
 
  Serial.println("=== ESP32 Buzzer Controller ===");
  
  // Настройка PWM для buzzer
  ledcSetup(BUZZER_PWM_CHANNEL, 1000, 8);  // Канал 0, 1кГц, 8-бит разрешение
  ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CHANNEL);
  
  Serial.println("Жду команды от ATmega328...");
}

void loop() {
  // Читаем команды от ATmega328
  if (Serial2.available()) {
    String command = Serial2.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("BEEP:")) {
      uint16_t frequency = command.substring(5).toInt();
      
      if (frequency != currentFrequency) {
        currentFrequency = frequency;
        updateBuzzer();
        
        Serial.print("Частота: ");
        Serial.print(frequency);
        Serial.println(" Гц");
      }
    }
  }
}

void updateBuzzer() {
  if (currentFrequency > 0) {
    // Включаем buzzer с заданной частотой
    ledcWriteTone(BUZZER_PWM_CHANNEL, currentFrequency);
    ledcWrite(BUZZER_PWM_CHANNEL, 128);  // 50% duty cycle
    buzzerActive = true;
  } else {
    // Выключаем buzzer
    ledcWrite(BUZZER_PWM_CHANNEL, 0);
    buzzerActive = false;
  }
}
