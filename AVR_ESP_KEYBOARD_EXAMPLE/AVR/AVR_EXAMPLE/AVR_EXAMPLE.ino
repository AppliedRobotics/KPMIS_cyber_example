#include <SPI.h>
#include "ardu_drive.hpp"
#include "ardu_uno_keyboard.hpp"
#include <DxlMaster2.h>

#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68
#define PC_SERIAL Serial

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

#define DRV1_DXL_ID 71
// #define DRV1_DXL_ID 0x07
// #define DRV1_DXL_ID 0x05
#define DRV2_DXL_ID 72
// #define DRV1_DXL_ID 0x08
// #define DRV1_DXL_ID 0x06

#define LEDS_SS_HC595 8   // Пин для управления сдвиговым регистром (74HC595)

DynamixelDevice drive1(DRV1_DXL_ID); 
DynamixelDevice drive2(DRV2_DXL_ID);

uint8_t pid_p = 100, pid_i = 50, pid_d = 10;
int16_t power_rf = 0, speed_rf = 0, power_lf = 0, speed_lf = 0,
        power_rb = 0, speed_rb = 0, power_lb = 0, speed_lb = 0; 

int16_t s_spd_m[4] = {100, 100, 100, 100};

void setup(void)
{
  SPI.begin();
  pinMode(LEDS_SS_HC595, OUTPUT);
  digitalWrite(LEDS_SS_HC595, HIGH);

  drive1.protocolVersion(1);
  drive2.protocolVersion(1);
  DxlMaster.setTimeOut(250);

  DxlMaster.begin(57600);
  // DxlMaster.begin(1000000);

  delay(100);

  while (drive1.ping() != DYN_STATUS_OK)
  {
    // PC_SERIAL.println("Trying to conncect to 1st DXL motor controller ...");

    DxlMaster.begin(1000000);
    // DxlMaster.begin(57600);
    delay(10);

    // PC_SERIAL.println("Setting baudrate ...");
    drive1.write(DXL_BAUD, (uint8_t)34);
    // drive1.write(DXL_BAUD, (uint8_t)1);

    DxlMaster.begin(57600);
    // DxlMaster.begin(1000000);
    delay(10);

    if (drive1.ping() != DYN_STATUS_OK)
    {
      // PC_SERIAL.println("Failed to connect to 1st DXL motor controller ");
      delay(10);
    }
  }

  while (drive2.ping() != DYN_STATUS_OK)
  {
    // PC_SERIAL.println("Trying to conncect to 2nd DXL motor controller ...");

    DxlMaster.begin(1000000);
    // DxlMaster.begin(57600);
    delay(10);

    // PC_SERIAL.println("Setting baudrate ...");
    drive2.write(DXL_BAUD, (uint8_t)34);
    // drive2.write(DXL_BAUD, (uint8_t)1);

    DxlMaster.begin(57600);
    // DxlMaster.begin(1000000);
    delay(10);

    if (drive2.ping() != DYN_STATUS_OK)
    {
      // PC_SERIAL.println("Failed to connect to 2nd DXL motor controller ");
      delay(10);
    }
  }

  // Set up motor drivers (control period, encoder mode, PID, power)
  DxlMaster.begin(57600);
  drive1.write(MAX_POWER_L, (uint16_t)1000);
  drive1.write(CONTROL_PERIOD, (uint8_t)50);
  drive1.write(MOTOR_MODE, (uint8_t)2);
  drive1.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  drive1.write(M1_GOAL_SPEED_L, (int16_t)0);
  drive1.write(M2_GOAL_SPEED_L, (int16_t)0);
  drive1.write(PID_P, (uint8_t)pid_p);
  drive1.write(PID_I, (uint8_t)pid_i);
  drive1.write(PID_D, (uint8_t)pid_d);
  
  drive2.write(MAX_POWER_L, (uint16_t)1000);
  drive2.write(CONTROL_PERIOD, (uint8_t)50);
  drive2.write(MOTOR_MODE, (uint8_t)2);
  drive2.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  drive2.write(M1_GOAL_SPEED_L, (int16_t)0);
  drive2.write(M2_GOAL_SPEED_L, (int16_t)0);
  drive2.write(PID_P, (uint8_t)pid_p);
  drive2.write(PID_I, (uint8_t)pid_i);
  drive2.write(PID_D, (uint8_t)pid_d);

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

void loop(void)
{
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
