#include "esp32-hal-gpio.h"
#include <SPI.h>
#include <esp32-hal-spi.h>
#include <esp32-hal-log.h>
#include "ardu_esp32_keyboard.hpp"

// Создание глобального объекта
ardu_esp32_keyboard keyboard;

// Конструктор: настройка пинов кнопок как входов
ardu_esp32_keyboard::ardu_esp32_keyboard(void)
{
  prev_mask = 0;
  mask = 0;

  for (uint8_t btn = 0; btn < numb_of_btns; btn++)
  {
    pinMode(btns_pins[btn], INPUT);
    btns_state[btn] = false;
  }
  prev_e_btn = NOT_PRESSED;
}

// Чтение состояния всех кнопок и возврат битовой маски
uint8_t ardu_esp32_keyboard::getKeyboard(void)
{
  uint8_t loc_mask = 0;
  for (uint8_t btn = 0; btn < numb_of_btns; btn++)
  {
    uint8_t stat = digitalRead(btns_pins[btn]) ? 1 : 0;
    loc_mask |= (stat << btn); // Каждый бит маски соответствует одной кнопке

    btns_state[btn] = (bool)stat; 
  }
  return loc_mask;
}

// Проверка состояния конкретной кнопки
bool ardu_esp32_keyboard::getButton(enum button e_btn)
{
  prev_mask = mask;
  mask = getKeyboard();
  return btns_state[e_btn-1];
}

// Определение последней нажатой кнопки
ardu_esp32_keyboard::button ardu_esp32_keyboard::getLastPressed(void)
{
  enum button e_btn = prev_e_btn;
  prev_mask = mask;
  mask = getKeyboard();

  // Если состояние кнопок изменилось и есть нажатые кнопки
  if (prev_mask != mask && mask)
  {
    uint8_t change_mask = prev_mask ^ mask; // Биты, которые изменились
    uint8_t new_pressed = change_mask & mask; // Биты, которые стали 1 (нажатие)

    // Поиск нажатой кнопки
    for (uint8_t btn = 0; btn < numb_of_btns; btn++)
    {
      if ((new_pressed >> btn) & 0x01)
      {
        e_btn = (enum button)(btn + 1);
      }
    }
    prev_e_btn = e_btn;
  }

  return e_btn;
}

// Деструктор (пустой)
ardu_esp32_keyboard::~ardu_esp32_keyboard()
{
}
