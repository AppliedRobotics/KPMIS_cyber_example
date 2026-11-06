#include <SPI.h>
#include "ardu_uno_keyboard.hpp"

ardu_uno_keyboard keyboard;  // Создание глобального объекта

// Конструктор: настройка пинов
ardu_uno_keyboard::ardu_uno_keyboard(void)
{
  but_ref_pin = ARDU_UNO_KEYBOARD_BUT_REF_PIN;
  but_ss_pin = ARDU_UNO_KEYBOARD_BUT_SS_PIN;

  prev_mask = 0;
  mask = 0;

  for (uint8_t btn = 0; btn < numb_of_btns; btn++)
  {
    btns_state[btn] = false;
  }
  prev_e_btn = NOT_PRESSED;

  pinMode(but_ref_pin, OUTPUT);
  digitalWrite(but_ref_pin, HIGH);   // Устанавливаем высокий уровень на референсном пине

  // SPI.begin(spi_sck_pin, spi_miso_pin, spi_mosi_pin, but_ss_pin);
  pinMode(but_ss_pin, OUTPUT);
  digitalWrite(but_ss_pin, HIGH);  
}

// Чтение состояния клавиатуры через SPI
uint8_t ardu_uno_keyboard::getKeyboard(void)
{
  uint8_t loc_mask = 0;
  digitalWrite(but_ref_pin, LOW);    // Активируем схему чтения кнопок
  for (volatile uint32_t del_q = 0; del_q < 10000; del_q++); // Небольшая задержка
  digitalWrite(but_ref_pin, HIGH);
  digitalWrite(but_ss_pin, LOW);     // Активируем устройство
  loc_mask = SPI.transfer(0) & 0x0F; // Читаем 4 младших бита
  digitalWrite(but_ss_pin, HIGH);    // Деактивируем устройство

  // Обновляем состояние каждой кнопки
  for (uint8_t btn = 0; btn < numb_of_btns; btn++)
  {
    btns_state[btn] = (bool)((loc_mask >> btn) & 0x01); 
  }
  return loc_mask;
}

// Получить состояние конкретной кнопки
bool ardu_uno_keyboard::getButton(enum button e_btn)
{
  prev_mask = mask;
  mask = getKeyboard();
  return btns_state[e_btn-1];  // Возвращаем состояние кнопки
}

// Получить последнюю нажатую кнопку
ardu_uno_keyboard::button ardu_uno_keyboard::getLastPressed(void)
{
  enum button e_btn = prev_e_btn;
  prev_mask = mask;
  mask = getKeyboard();

  if (prev_mask != mask && mask)
  {
    uint8_t change_mask = prev_mask ^ mask; // Находим изменившиеся биты
    uint8_t new_pressed = change_mask & mask; // Те, которые стали нажаты

    // Определяем, какая кнопка была нажата
    for (uint8_t btn = 0; btn < numb_of_btns; btn++)
    {
      if ((new_pressed >> btn) & 0x01)
      {
        e_btn = (enum button)(btn + 1);  // Преобразуем индекс в enum
      }
    }
    prev_e_btn = e_btn;
  }

  return e_btn;
}
// Деструктор: отключаем пины
ardu_uno_keyboard::~ardu_uno_keyboard()
{
  digitalWrite(but_ref_pin, LOW);
  pinMode(but_ref_pin, INPUT);
  digitalWrite(but_ss_pin, LOW);
  pinMode(but_ref_pin, INPUT);

  // SPI.end();
}
