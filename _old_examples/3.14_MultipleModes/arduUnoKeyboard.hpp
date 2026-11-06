#ifndef __ARDU_UNO_KEYBOARD_H__
#define __ARDU_UNO_KEYBOARD_H__

#include <stdint.h>
#include <SPI.h>

#define ARDU_UNO_KEYBOARD_BUT_SS_PIN 10
#define ARDU_UNO_KEYBOARD_BUT_REF_PIN 9

class ArduUnoKeyboard
{
public:
  enum Button 
  {
    NOT_PRESSED = 0,
    BUTTON1 = 1,
    BUTTON2 = 2,
    BUTTON3 = 4,
    BUTTON4 = 8
  };

private:
  int8_t buttonRefPin; 
  int8_t buttonSsPin;
  
  uint8_t previousMask;
  uint8_t currentMask;
  
  static const uint8_t numberOfButtons = 4;
  bool buttonStates[numberOfButtons];
  Button previousPressedButton;

public:
  ArduUnoKeyboard(void);
  uint8_t getKeyboardMask(void);
  bool isButtonPressed(Button btn);
  Button getLastPressedButton(void);
  bool isAnyButtonPressed(void);
  uint8_t getPressedButtonsCount(void);
  ~ArduUnoKeyboard();
};

extern ArduUnoKeyboard keyboard;

#endif /* __ARDU_UNO_KEYBOARD_H__ */
