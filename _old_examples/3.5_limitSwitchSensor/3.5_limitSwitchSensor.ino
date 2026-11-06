const uint8_t contactLeftPin = 8;     // Пин левого концевого датчика
const uint8_t contactRightPin = 5;    // Пин правого концевого датчика
const uint8_t ledPin = 13;            // Встроенный светодиод

// Переменные для хранения состояния датчиков
bool leftSensorState = false;         // Состояние левого датчика
bool rightSensorState = false;        // Состояние правого датчика
bool previousLeftState = false;       // Предыдущее состояние левого датчика
bool previousRightState = false;      // Предыдущее состояние правого датчика

// Переменные для временных интервалов
unsigned long lastReadTime = 0;       // Время последнего опроса датчиков
const unsigned long readInterval = 50;  // Интервал опроса датчиков (мс)

void setup() {
  // Инициализация Serial порта для вывода информации
  Serial.begin(115200);
  while(!Serial);
  Serial.println("=== Опрос концевого датчика ===");
  
  // Настройка пинов концевых датчиков как входы с подтяжкой
  pinMode(contactLeftPin, INPUT_PULLUP);
  pinMode(contactRightPin, INPUT_PULLUP);
  
  // Настройка пина светодиода как выход
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Выключаем светодиод
  
  // Первоначальное чтение состояния датчиков
  leftSensorState = !digitalRead(contactLeftPin);    // Инвертируем из-за INPUT_PULLUP
  rightSensorState = !digitalRead(contactRightPin);  // Инвертируем из-за INPUT_PULLUP
  
  delay(1000);  // Пауза для стабилизации
}

void loop() {
  // Проверяем, пора ли опрашивать датчики
  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();
    
    // Сохраняем предыдущие состояния
    previousLeftState = leftSensorState;
    previousRightState = rightSensorState;
    
    // Читаем текущие состояния датчиков
    // Используем инверсию из-за INPUT_PULLUP (LOW = нажат, HIGH = свободен)
    leftSensorState = !digitalRead(contactLeftPin);
    rightSensorState = !digitalRead(contactRightPin);
    
    // Проверяем изменения состояния и выводим информацию
    bool stateChanged = (leftSensorState != previousLeftState) || (rightSensorState != previousRightState);
    
    if (stateChanged) {
      // Состояние левого датчика
      Serial.print("Левый: ");
      if (leftSensorState) Serial.print("нажат"); 
      else Serial.print("свободен");
      
      Serial.print(" | Правый: ");
      
      // Состояние правого датчика
      if (rightSensorState) Serial.println("нажат");
      else Serial.println("свободен");
      
      Serial.println();
    }
    
    // Управление светодиодом в зависимости от состояния датчиков
    manageLedIndication();
 
  }
}

void manageLedIndication() {
  static unsigned long lastBlinkTime = 0;
  static bool blinkState = false;
  
  if (leftSensorState && rightSensorState) {
    // Оба датчика нажаты - мигаем светодиодом
    if (millis() - lastBlinkTime >= 250) {  // Мигание с частотой 2 Гц
      lastBlinkTime = millis();
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState);
    }
  } else if (leftSensorState || rightSensorState) {
    // Один из датчиков нажат - светодиод горит постоянно
    digitalWrite(ledPin, HIGH);
  } else {
    // Оба датчика свободны - светодиод выключен
    digitalWrite(ledPin, LOW);
  }
}
