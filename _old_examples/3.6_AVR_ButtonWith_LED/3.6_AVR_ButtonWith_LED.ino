#include <SPI.h>
#include "arduUnoKeyboard.hpp"

// Определение пинов
const uint8_t ledShiftRegisterPin = 8;    // Пин управления сдвиговым регистром для светодиодов
const uint8_t statusLedPin = 13;          // Встроенный светодиод Arduino

// Переменные состояния
uint8_t ledPattern = 0x00;                // Текущий паттерн светодиодов (8 бит)
uint8_t currentMode = 0;                  // Текущий режим работы (0-4)
bool systemActive = true;                 // Флаг активности системы

// Переменные для режимов работы
uint8_t runningLightPosition = 0;         // Позиция бегущего огня
unsigned long lastModeUpdate = 0;         // Время последнего обновления режима
const unsigned long modeUpdateInterval = 200;  // Интервал обновления режимов (мс)

// Переменные для кнопок
uint8_t previousButtonMask = 0;           // Предыдущее состояние кнопок
unsigned long lastButtonCheck = 0;       // Время последней проверки кнопок
const unsigned long buttonCheckInterval = 50;   // Интервал проверки кнопок (мс)

// Массивы для режимов работы
const char* modeNames[] = {
  "Выключено",
  "Индивидуальное управление",
  "Бегущие огни влево",
  "Бегущие огни вправо",
  "Мигание всех"
};

void setup() {
  // Инициализация Serial порта
  Serial.begin(115200);
  while(!Serial);
  Serial.println("=== Запуск светодиода по кнопке ===");
  
  // Инициализация SPI для работы с клавиатурой и светодиодами
  SPI.begin();
  
  // Настройка пина сдвигового регистра для светодиодов
  pinMode(ledShiftRegisterPin, OUTPUT);
  digitalWrite(ledShiftRegisterPin, HIGH);
  
  // Инициальная очистка светодиодов
  updateLedDisplay();

  
  delay(1000);  // Пауза для стабилизации
  Serial.println("Жду нажатий!");
}

void loop() {
  // Проверка кнопок
  if (millis() - lastButtonCheck >= buttonCheckInterval) {
    lastButtonCheck = millis();
    handleButtonInput();
  }
  
  // Обновление режимов работы
  if (millis() - lastModeUpdate >= modeUpdateInterval) {
    lastModeUpdate = millis();
    updateCurrentMode();
  }
  
  // Обновление дисплея светодиодов
  updateLedDisplay();
}

/*
  Функция обработки нажатий кнопок
*/
void handleButtonInput() {
  uint8_t currentButtonMask = keyboard.getKeyboardMask();
  
  // Проверяем изменения в состоянии кнопок
  if (currentButtonMask != previousButtonMask) {
    // Обработка нажатий (переход от 0 к 1)
    uint8_t pressedButtons = currentButtonMask & (~previousButtonMask);
    
    if (pressedButtons) {
        
        
        if (pressedButtons & ArduUnoKeyboard::BUTTON1) {
        Serial.print("Нажата кнопка: ");
        Serial.print("1 ");
        Serial.println(modeNames[currentMode]);
        handleButton1Press();
      }
      if (pressedButtons & ArduUnoKeyboard::BUTTON2) {
        Serial.print("Нажата кнопка: ");
        Serial.print("2 ");
        Serial.println(modeNames[currentMode]);
        handleButton2Press();
      }
      if (pressedButtons & ArduUnoKeyboard::BUTTON3) {
        Serial.print("Нажата кнопка: ");
        Serial.print("3 ");
        Serial.println(modeNames[currentMode]);
        handleButton3Press();
      }
      if (pressedButtons & ArduUnoKeyboard::BUTTON4) {
        Serial.print("Нажата кнопка: ");
        Serial.print("4 ");
        Serial.println(modeNames[currentMode]);
        handleButton4Press();
      }
      
    }
    
    // Обработка отпускания кнопок
    uint8_t releasedButtons = previousButtonMask & (~currentButtonMask);
    if (releasedButtons) {
      Serial.print("Отпущена кнопка: ");
      if (releasedButtons & ArduUnoKeyboard::BUTTON1) Serial.print("1 ");     
      if (releasedButtons & ArduUnoKeyboard::BUTTON2) Serial.print("2 ");
      if (releasedButtons & ArduUnoKeyboard::BUTTON3) Serial.print("3 ");
      if (releasedButtons & ArduUnoKeyboard::BUTTON4) Serial.print("4 ");
      
      Serial.println();
    }
  }
}

//Обработчики нажатий отдельных кнопок
void handleButton1Press() {
  currentMode = 0;  // off
  runningLightPosition = 0;
}

void handleButton2Press() {
  currentMode = 2;  // Бегущие огни влево
  runningLightPosition = 0;
}

void handleButton3Press() {
  currentMode = 3;  // Бегущие огни вправо
  runningLightPosition = 7;
}

void handleButton4Press() {
  currentMode = 4;  // Мигание всех
  runningLightPosition = 0;
}


//Функция обновления текущего режима работы
void updateCurrentMode() {
  static uint8_t individualLedIndex = 0;
  static bool blinkState = false;
  
  switch (currentMode) {
    case 0:  // Выключено
      ledPattern = 0x00;
      systemActive = false;
      break;
      
    case 2:  // Бегущие огни влево
      systemActive = true;
      ledPattern = 1 << runningLightPosition;
      runningLightPosition = (runningLightPosition + 1) % 8;

      break;
      
    case 3:  // Бегущие огни вправо
      systemActive = true;
      ledPattern = 1 << runningLightPosition;
      runningLightPosition = (runningLightPosition == 0) ? 7 : runningLightPosition - 1;

      break;
      
    case 4:  // Мигание всех
      systemActive = true;
      blinkState = !blinkState;
      ledPattern = blinkState ? 0xFF : 0x00;

      break;
  }
}


//Функция обновления дисплея светодиодов
void updateLedDisplay() {
  digitalWrite(ledShiftRegisterPin, LOW);
  SPI.transfer(ledPattern);
  digitalWrite(ledShiftRegisterPin, HIGH);
}
