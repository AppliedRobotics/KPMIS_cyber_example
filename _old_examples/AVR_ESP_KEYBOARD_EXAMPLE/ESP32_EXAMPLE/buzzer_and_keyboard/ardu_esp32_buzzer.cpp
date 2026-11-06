#include "ardu_esp32_buzzer.hpp"


// Конструктор: инициализация пина и ШИМ
ardu_esp32_buzzer::ardu_esp32_buzzer(uint8_t vol, int8_t pin, uint8_t pwm_ch) : buzzer_pin(pin), pwm_channel(pwm_ch)
{
  volume_level = 0;
  duty_cycle = 0;
  pwm_resolution = ARDU_ESP32_BUZZER_MAX_VOL_LEVEL;   // Разрешение = 10 бит
  is_emitting = false;

  setVolumeLevel(vol); // Установка громкости
  ledcAttachChannel(buzzer_pin, 1000, pwm_resolution, pwm_channel); // Настройка ШИМ
  ledcWrite(buzzer_pin, 0); // Изначально звук выключен
}  

// Установка громкости (vol от 0 до 10)
void ardu_esp32_buzzer::setVolumeLevel(uint8_t vol)
{
  volume_level = vol <= pwm_resolution ? vol : pwm_resolution;
  if (volume_level)
  {
    duty_cycle = 1;
    // Расчет заполнения ШИМ: 2^vol (например, для vol=5 -> duty_cycle=32)
    for (uint8_t pw = 1; pw < volume_level; pw++)
    {
      duty_cycle *= 2;
    } 
  }
  else
  {
    duty_cycle = 0;
  }
  // Если звук активен, обновляем уровень ШИМ
  if (is_emitting)
  {
    ledcWrite(buzzer_pin, duty_cycle);
  }
}

// Воспроизведение частоты
void ardu_esp32_buzzer::playFrequency(uint16_t freq)
{
  ledcChangeFrequency(buzzer_pin, freq, pwm_resolution);
  ledcWrite(buzzer_pin, duty_cycle);
  is_emitting = true;
}

// Отключение звука
void ardu_esp32_buzzer::mute(void)
{
  is_emitting = false;
  ledcWrite(buzzer_pin, 0);
}

// Деструктор: отключаем ШИМ
ardu_esp32_buzzer::~ardu_esp32_buzzer()
{
  is_emitting = false;
  ledcWrite(buzzer_pin, 0);
  ledcDetach(buzzer_pin);
}
