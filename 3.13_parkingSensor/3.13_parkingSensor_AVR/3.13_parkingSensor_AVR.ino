// Парктроник - часть для ATmega328 (измерение расстояния)

const uint8_t trigPin = 10;
const uint8_t echoPin = 9;

float distance = 0.0;
uint16_t beepFrequency = 0;

void setup() {
  Serial.begin(115200);  // Связь с ESP32
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  digitalWrite(trigPin, LOW);
  
  delay(1000);
}

void loop() {
  // Измеряем расстояние
  measureDistance();
  
  // Рассчитываем частоту писка
  calculateBeepFrequency();
  
  // Отправляем частоту на ESP32
  Serial.print("BEEP:");
  Serial.println(beepFrequency);
  
  delay(100);
}

void measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);.
  }
}

void calculateBeepFrequency() {
  if (distance > 50.0) {
    beepFrequency = 0;  // Тишина
  } else {
    // Чем ближе, тем выше частота (от 200 до 2000 Гц)
    beepFrequency = map(distance * 10, 50, 500, 2000, 200);
  }
}
