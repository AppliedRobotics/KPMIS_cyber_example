#ifndef __ARDU_UNO_KEYBOARD_H__
#define __ARDU_UNO_KEYBOARD_H__

#include <stdint.h>
#include <SPI.h>

#define ARDU_UNO_KEYBOARD_BUT_SS_PIN 10 // Пин выбора ведомого (Slave Select) для клавиатуры
#define ARDU_UNO_KEYBOARD_BUT_REF_PIN 9 // Референсный пин для питания/подтяжки кнопок

class ardu_uno_keyboard
{
public:
  enum button 
  {
    NOT_PRESSED = 0,
    BUT1 = 0,  // значения начинаются с 0
    BUT2 = 1,
    BUT3 = 2,
    BUT4 = 4
  };

private:
  int8_t but_ref_pin; 
  int8_t but_ss_pin;
  int8_t spi_sck_pin;
  int8_t spi_miso_pin; 
  int8_t spi_mosi_pin;
  
  uint8_t prev_mask;   // Предыдущее состояние кнопок
  uint8_t mask;        // Текущее состояние кнопок
  
  static const uint8_t numb_of_btns = 4;  // Количество кнопок
  bool btns_state[numb_of_btns];          // Состояние каждой кнопки
  enum button prev_e_btn;                 // Последняя нажатая кнопка

public:
  ardu_uno_keyboard(void);
  uint8_t getKeyboard(void);             // Получить состояние всех кнопок
  bool getButton(enum button btn);       // Получить состояние конкретной кнопки
  enum button getLastPressed(void);      // Получить последнюю нажатую кнопку
  ~ardu_uno_keyboard();
};

extern ardu_uno_keyboard keyboard;       // Глобальный объект клавиатуры

#endif /* __ARDU_UNO_KEYBOARD_H__ */
