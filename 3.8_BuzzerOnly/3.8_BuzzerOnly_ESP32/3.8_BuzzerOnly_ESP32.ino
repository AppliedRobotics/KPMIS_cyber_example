#define BUZZER_PIN 19

uint16_t currentFrequency = 0;
bool buzzerActive = false;

void setup() {
  Serial2.begin(115200, SERIAL_8N1, 22, 23);
  Serial.begin(115200);
 
  Serial.println("=== ESP32 Buzzer Controller ===");
  
  pinMode(BUZZER_PIN, OUTPUT);
  
  Serial.println("Жду команды от ATmega328...");
}

void loop() {
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
    tone(BUZZER_PIN, currentFrequency);
    buzzerActive = true;
  } else {
    noTone(BUZZER_PIN);
    buzzerActive = false;
  }
}