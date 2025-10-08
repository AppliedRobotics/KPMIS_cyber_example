const uint8_t ledPin = 13;            // Встроенный светодиод

// Массив пинов для удобства работы
const uint8_t lineSensorPins[4] = {3, 6, 4, 7};
const char* sensorNames[4] = {"Левый крайний", "Левый центр", "Правый центр", "Правый крайний"};

// Переменные для хранения состояния датчиков
bool sensorStates[4] = {false, false, false, false};         // Текущие состояния датчиков
bool previousSensorStates[4] = {false, false, false, false}; // Предыдущие состояния датчиков

// Переменные для временных интервалов
unsigned long lastReadTime = 0;       // Время последнего опроса датчиков
const unsigned long readInterval = 50;   // Интервал опроса датчиков (мс)

void setup() {
  // Инициализация Serial порта для вывода информации
  Serial.begin(115200);
  while(!Serial);
  Serial.println("=== Опрос датчика линии ===");
  
  // Настройка пинов датчиков линии как входы
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(lineSensorPins[i], INPUT);
  }
  
  // Настройка пина светодиода как выход
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  delay(1000);  // Пауза для стабилизации
  Serial.println("Покажите линию!");
}

void loop() {
  // Проверяем, пора ли опрашивать датчики
  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();
    
    // Сохраняем предыдущие состояния
    for (uint8_t i = 0; i < 4; i++) {
      previousSensorStates[i] = sensorStates[i];
    }
    
    // Читаем текущие состояния датчиков
    for (uint8_t i = 0; i < 4; i++) sensorStates[i] = !digitalRead(lineSensorPins[i]);
    
    // Проверяем изменения и выводим информацию
    checkForChanges();
  }
}

/*
  Функция проверки изменений состояния и вывода информации
*/
void checkForChanges() {
  bool stateChanged = false;
  
  // Проверяем изменения в любом из датчиков
  for (uint8_t i = 0; i < 4; i++) {
    if (sensorStates[i] != previousSensorStates[i]) {
      stateChanged = true;
      break;
    }
  }
  
  if (stateChanged) {
    // Выводим текущее состояние
    Serial.print("Состояние датчиков слева направо: ");
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print(sensorStates[i] ? "1" : "0");
      if (i < 3) Serial.print(",");
    }
    
    Serial.println(" ");
  }
}
