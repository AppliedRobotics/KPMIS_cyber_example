// Простое управление схватом

#include <SCServo.h>

#define GRIPPER_ID 6
#define FEETECH_BAUD_RATE 115200
#define SERIAL_BAUD_RATE 115200

SMS_STS servoController;

uint16_t openPosition = 2250;   // Открытое положение (большее значение)
uint16_t closePosition = 3000;   // Закрытое положение (меньшее значение)
bool isOpen = true;
unsigned long lastToggle = 0;

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial);
  Serial.println("=== Управление схватом ===");
  
  Serial.begin(FEETECH_BAUD_RATE);
  servoController.pSerial = &Serial;
  delay(100);
  
  int16_t currentPos = servoController.ReadPos(GRIPPER_ID);
  
  Serial.begin(SERIAL_BAUD_RATE);
  if (currentPos >= 0) {
    Serial.println("Схват подключен");
  } else {
    Serial.println("Ошибка подключения схвата");
  }
  
  delay(1000);
}

void loop() {
  if (millis() - lastToggle >= 3000) {
    lastToggle = millis();
    
    Serial.begin(FEETECH_BAUD_RATE);
    if (isOpen) {
      servoController.WritePosEx(GRIPPER_ID, closePosition, 500);
      isOpen = false;
    } else {
      servoController.WritePosEx(GRIPPER_ID, openPosition, 500);
      isOpen = true;
    }
    Serial.begin(SERIAL_BAUD_RATE);
    
    Serial.print(isOpen ? "Открываем" : "Закрываем");
    Serial.print(" схват до позиции ");
    Serial.println(isOpen ? openPosition : closePosition);
  }
  
  // Показываем текущую позицию для диагностики
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck >= 500) {
    lastCheck = millis();
    
    Serial.begin(FEETECH_BAUD_RATE);
    int16_t currentPos = servoController.ReadPos(GRIPPER_ID);
    Serial.begin(SERIAL_BAUD_RATE);
    
    Serial.print("Текущая позиция: ");
    Serial.println(currentPos);
  }
  
  delay(100);
}

