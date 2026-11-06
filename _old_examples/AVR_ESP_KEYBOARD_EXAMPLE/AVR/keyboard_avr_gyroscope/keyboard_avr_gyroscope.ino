#include <SPI.h>
#include "ardu_uno_keyboard.hpp"
#include <Wire.h>
#include <ICM20948_WE.h>

#define ICM20948_ADDR 0x68

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

#define LEDS_SS_HC595 8   // Пин для управления сдвиговым регистром (74HC595)

void setup() {
  SPI.begin();
  pinMode(LEDS_SS_HC595, OUTPUT);
  digitalWrite(LEDS_SS_HC595, HIGH);
  Wire.begin();
  Serial.begin(1000000);
  
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  else{
    Serial.println("ICM20948 is connected");
  }

  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();   // Автоматическая калибровка
  Serial.println("Done!"); 

  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);      // Диапазон измерений: ±2g
  myIMU.setAccDLPF(ICM20948_DLPF_6);             // Настройка цифрового фильтра низких частот
  myIMU.setAccSampleRateDivider(10);             // Делитель частоты дискретизации
}

uint8_t led_byte = 0xFF;

void loop() {
  uint8_t mask_pressed = keyboard.getKeyboard();
  
  // Обработка нажатий кнопок
  if (mask_pressed & 0x01) // Кнопка 1
  {
    led_byte &=~0x01;  // Включить светодиод 1
  }
  else
  {
    led_byte |= 0x01; // Выключить светодиод 1
  }
  // ... аналогично для других кнопок
  if (mask_pressed & 0x02)
  {
    led_byte &=~0x02;

    if (led_byte & 0x10)
    {
      led_byte &=~0x10;
    }
    else
    {
      led_byte |= 0x10;
    }
  }
  else
  {
    led_byte |= 0x02;
  }

  if (mask_pressed & 0x04)
  {
    led_byte &=~0x04;

    if (led_byte & 0x20)
    {
      led_byte &=~0x20;
    }
    else
    {
      led_byte |= 0x20;
    }
  }
  else
  {
    led_byte |= 0x04;
  }

  if (mask_pressed & 0x08)
  {
    led_byte &=~0x08;

    if (led_byte & 0x40)
    {
      led_byte &=~0x40;
    }
    else
    {
      led_byte |= 0x40;
    }
  }
  else
  {
    led_byte |= 0x08;
  }

  digitalWrite(LEDS_SS_HC595, LOW);
  SPI.transfer(led_byte);
  digitalWrite(LEDS_SS_HC595, HIGH);
  
  xyzFloat accRaw;
  xyzFloat corrAccRaw;
  xyzFloat gVal;
  myIMU.readSensor();                          // Чтение данных с датчика
  myIMU.getAccRawValues(&accRaw);              // Получить сырые значения акселерометра
  myIMU.getCorrectedAccRawValues(&corrAccRaw); // Получить скорректированные сырые значения
  myIMU.getGValues(&gVal);                     // Получить значения в g
  float resultantG = myIMU.getResultantG(&gVal); // Результирующее ускорение
  // Вывод значений в Serial
  Serial.println("Raw acceleration values (x,y,z):");
  Serial.print(accRaw.x);
  Serial.print("   ");
  Serial.print(accRaw.y);
  Serial.print("   ");
  Serial.println(accRaw.z);

  Serial.println("Corrected raw acceleration values (x,y,z):");
  Serial.print(corrAccRaw.x);
  Serial.print("   ");
  Serial.print(corrAccRaw.y);
  Serial.print("   ");
  Serial.println(corrAccRaw.z);

  Serial.println("g-values (x,y,z):");
  Serial.print(gVal.x);
  Serial.print("   ");
  Serial.print(gVal.y);
  Serial.print("   ");
  Serial.println(gVal.z);

  Serial.print("Resultant g: ");
  Serial.println(resultantG);
  Serial.println("*************************************");
 
  delay(500);

}
