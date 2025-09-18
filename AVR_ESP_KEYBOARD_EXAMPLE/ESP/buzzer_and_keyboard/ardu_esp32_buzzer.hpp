#ifndef __ARDU_ESP32_BUZZER_H__
#define __ARDU_ESP32_BUZZER_H__

#include <stdint.h>
#include <esp32-hal-ledc.h>  // Библиотека для работы с ШИМ на ESP32

#define ARDU_ESP32_BUZZER_PIN 19           // Пин по умолчанию
#define ARDU_ESP32_BUZZER_PWM_CHANNEL LEDC_CHANNEL_0  // Канал ШИМ
#define ARDU_ESP32_BUZZER_MAX_VOL_LEVEL 10  // Макс. уровень громкости (0-10)

class ardu_esp32_buzzer
{
  int8_t buzzer_pin;           // Пин, к которому подключен buzzer
  uint8_t volume_level;        // Текущий уровень громкости (0-10)
  uint32_t duty_cycle;         // Расчетный коэффициент заполнения ШИМ
  uint8_t pwm_resolution;      // Разрешение ШИМ (бит)
  uint8_t pwm_channel;         // Канал ШИМ
  bool is_emitting;            // Флаг: воспроизводится ли звук

public:
  // Конструктор: настройка пина и канала ШИМ
  ardu_esp32_buzzer(uint8_t vol = 0, int8_t pin = ARDU_ESP32_BUZZER_PIN, uint8_t pwm_ch = ARDU_ESP32_BUZZER_PWM_CHANNEL);
  void setVolumeLevel(uint8_t vol);  // Установка громкости
  void playFrequency(uint16_t freq); // Воспроизведение частоты
  void mute(void);                   // Отключение звука
  ~ardu_esp32_buzzer();              // Деструктор
};

#endif /* __ARDU_ESP32_BUZZER_H__ */
