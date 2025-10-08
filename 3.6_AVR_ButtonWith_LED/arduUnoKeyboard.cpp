#include <SPI.h>
#include "arduUnoKeyboard.hpp"

ArduUnoKeyboard keyboard;

ArduUnoKeyboard::ArduUnoKeyboard(void)
{
  buttonRefPin = ARDU_UNO_KEYBOARD_BUT_REF_PIN;
  buttonSsPin = ARDU_UNO_KEYBOARD_BUT_SS_PIN;

  previousMask = 0;
  currentMask = 0;

  for (uint8_t btn = 0; btn < numberOfButtons; btn++)
  {
    buttonStates[btn] = false;
  }
  previousPressedButton = NOT_PRESSED;

  pinMode(buttonRefPin, OUTPUT);
  digitalWrite(buttonRefPin, HIGH);

  pinMode(buttonSsPin, OUTPUT);
  digitalWrite(buttonSsPin, HIGH);
}

uint8_t ArduUnoKeyboard::getKeyboardMask(void)
{
  uint8_t localMask = 0;
  
  // Генерируем импульс на REF пине для синхронизации
  digitalWrite(buttonRefPin, LOW);
  for (volatile uint32_t delayCounter = 0; delayCounter < 10000; delayCounter++);
  digitalWrite(buttonRefPin, HIGH);
  
  // Читаем состояние кнопок через SPI
  digitalWrite(buttonSsPin, LOW);
  localMask = SPI.transfer(0) & 0x0F;  // Маскируем только младшие 4 бита
  digitalWrite(buttonSsPin, HIGH);
  
  // Обновляем состояние каждой кнопки
  for (uint8_t btn = 0; btn < numberOfButtons; btn++)
  {
    buttonStates[btn] = (bool)((localMask >> btn) & 0x01); 
  }
  
  return localMask;
}

bool ArduUnoKeyboard::isButtonPressed(Button btn)
{
  previousMask = currentMask;
  currentMask = getKeyboardMask();
  
  // Проверяем конкретную кнопку
  return (currentMask & btn) != 0;
}

ArduUnoKeyboard::Button ArduUnoKeyboard::getLastPressedButton(void)
{
  Button pressedButton = previousPressedButton;
  previousMask = currentMask;
  currentMask = getKeyboardMask();

  if (previousMask != currentMask && currentMask)
  {
    uint8_t changeMask = previousMask ^ currentMask;
    uint8_t newPressed = changeMask & currentMask;
  
    for (uint8_t btn = 0; btn < numberOfButtons; btn++)
    {
      if ((newPressed >> btn) & 0x01)
      {
        pressedButton = (Button)(1 << btn);
      }
    }
    previousPressedButton = pressedButton;
  }

  return pressedButton;
}

bool ArduUnoKeyboard::isAnyButtonPressed(void)
{
  return getKeyboardMask() != 0;
}

uint8_t ArduUnoKeyboard::getPressedButtonsCount(void)
{
  uint8_t mask = getKeyboardMask();
  uint8_t count = 0;
  
  for (uint8_t i = 0; i < numberOfButtons; i++)
  {
    if (mask & (1 << i))
    {
      count++;
    }
  }
  
  return count;
}

ArduUnoKeyboard::~ArduUnoKeyboard()
{
  digitalWrite(buttonRefPin, LOW);
  pinMode(buttonRefPin, INPUT);
  digitalWrite(buttonSsPin, LOW);
  pinMode(buttonSsPin, INPUT);
}
