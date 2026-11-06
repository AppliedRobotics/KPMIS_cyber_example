#include "ardu_esp32_buzzer.hpp"
#include "ardu_esp32_keyboard.hpp"


// Создание объекта buzzer с громкостью 10
ardu_esp32_buzzer buzzer(10);

#define LED_1 27
#define LED_2 14
#define LED_3 12
#define LED_4 13

#define LED_R 15
#define LED_G 2
#define LED_B 18

uint8_t buzzer_time_counter = 0;

// Global variables added to fix previous compilation errors
int16_t speed_value = 0; // Assuming it holds a signed integer value for speed
uint8_t moving_flag = 0; // Assuming 0 for stopped, 1 for moving


void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(115200);
  Serial.println("System Initialized. Ready for Button Press.");

  pinMode(LED_1, OUTPUT);
  digitalWrite(LED_1, HIGH);
  pinMode(LED_2, OUTPUT);
  digitalWrite(LED_2, HIGH);
  pinMode(LED_3, OUTPUT);
  digitalWrite(LED_3, HIGH);
  pinMode(LED_4, OUTPUT);
  digitalWrite(LED_4, HIGH);

  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, HIGH);
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, HIGH);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, HIGH);
}



void loop() {
  // обработка нажатий кнопок и управление buzzer
  uint8_t mask_pressed = keyboard.getKeyboard(), mult = 0;

  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_3, HIGH);
  digitalWrite(LED_4, HIGH);
  
  // Проверка отдельных кнопок и выполнение действий
  if (mask_pressed & 1){   // Кнопка 1
    moving_flag = 0;
    digitalWrite(LED_1, LOW);
    // SERIAL DEBUGGING: Print button 1 info
    Serial.println("Button 1 Pressed (GPIO 34) - Set moving_flag = 0"); 
  }
  
  if (mask_pressed & 2){   // Кнопка 2
    speed_value -= 10;
    digitalWrite(LED_2, LOW);
    // SERIAL DEBUGGING: Print button 2 info
    Serial.print("Button 2 Pressed (GPIO 35) - speed_value: ");
    Serial.println(speed_value);
  }
  
  if (mask_pressed & 4){   // Кнопка 3
    speed_value += 10;
    digitalWrite(LED_3, LOW);
    // SERIAL DEBUGGING: Print button 3 info
    Serial.print("Button 3 Pressed (GPIO 25) - speed_value: ");
    Serial.println(speed_value);
  }
  
  if (mask_pressed & 8){   // Кнопка 4
    moving_flag = 1;
    digitalWrite(LED_4, LOW);
    // SERIAL DEBUGGING: Print button 4 info
    Serial.println("Button 4 Pressed (GPIO 26) - Set moving_flag = 1"); 
  }

  // A small delay to debounce the button readings and prevent flooding the serial monitor
  delay(50); 
  
  if (abs(speed_value) <= 100)
  {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
  }
  else if (abs(speed_value) <= 500)
  {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  }
  else
  {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
  }
  // Подсчет количества нажатых кнопок (mult)
  for (uint32_t mask_buf = mask_pressed & 0xFF; mask_buf > 0; mult++)
  {
    mask_buf >>= 1; 
  }
  // Если кнопки нажаты и счетчик времени < 1, воспроизводим звук
  if (mult && buzzer_time_counter < 1)
  {
    buzzer.playFrequency(mult * 880); // Частота = количество кнопок * 880 Гц
    buzzer_time_counter++;
  }
  else if (mult && buzzer_time_counter >= 1)
  {
    buzzer.mute(); // Отключаем звук после одного цикла
  }
  else
  {
    buzzer.mute();
    buzzer_time_counter = 0;
  }

}