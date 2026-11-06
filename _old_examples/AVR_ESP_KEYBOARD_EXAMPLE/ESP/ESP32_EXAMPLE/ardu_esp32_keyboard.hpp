#ifndef __ARDU_ESP32_KEYBOARD_H__
#define __ARDU_ESP32_KEYBOARD_H__

#include <stdint.h>
#include <SPI.h>
#include <esp32-hal-spi.h>
#include <esp32-hal-gpio.h>

// Пины, к которым подключены кнопки
#define ARDU_ESP32_KEYBOARD_BUT1_PIN 34
#define ARDU_ESP32_KEYBOARD_BUT2_PIN 35
#define ARDU_ESP32_KEYBOARD_BUT3_PIN 25
#define ARDU_ESP32_KEYBOARD_BUT4_PIN 26

class ardu_esp32_keyboard
{
public:
  enum button 
  {
    NOT_PRESSED = 0,
    BUT1 = 1,
    BUT2 = 2,
    BUT3 = 3,
    BUT4 = 4
  };

private:
  uint8_t prev_mask;  // Предыдущее состояние кнопок
  uint8_t mask;       // Текущее состояние кнопок
  
  static const uint8_t numb_of_btns = 4; // Количество кнопок
  const int8_t btns_pins[numb_of_btns] = { // Массив пинов кнопок
    ARDU_ESP32_KEYBOARD_BUT1_PIN,
    ARDU_ESP32_KEYBOARD_BUT2_PIN,
    ARDU_ESP32_KEYBOARD_BUT3_PIN,
    ARDU_ESP32_KEYBOARD_BUT4_PIN
  };
  bool btns_state[numb_of_btns]; // Текущее состояние каждой кнопки
  enum button prev_e_btn;        // Последняя нажатая кнопка

public:
  ardu_esp32_keyboard(void);
  uint8_t getKeyboard(void);           // Чтение состояния всех кнопок
  bool getButton(enum button btn);     // Проверка состояния конкретной кнопки
  enum button getLastPressed(void);    // Получение последней нажатой кнопки
  ~ardu_esp32_keyboard();
};

extern ardu_esp32_keyboard keyboard; // Глобальный объект keyboard

#endif /* __ARDU_ESP32_KEYBOARD_H__ */
