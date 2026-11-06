#include <SPI.h>
#include "ardu_esp32_buzzer.hpp"
#include "ardu_esp32_keyboard.hpp"
#include "ardu_drive.hpp"
#include <DxlMaster2.h>

// Board control defines, objects, variables
#define AR_RST_CNTRL 2

// Serial and DXL defines, objects, variables
#define PC_SERIAL Serial

#define DXL_SERIAL Serial1
#define DXL_SERIAL_TX_PIN 32
#define DXL_SERIAL_RX_PIN 33
#define DXL_NO_DIR_PIN (-1)

#define ATM_SERIAL Serial2
#define ATM_SERIAL_TX_PIN 23
#define ATM_SERIAL_RX_PIN 22

#define PB_DXL_ID 65
#define DRV1_DXL_ID 71
// #define DRV1_DXL_ID 0x07
// #define DRV1_DXL_ID 0x05
#define DRV2_DXL_ID 72
// #define DRV1_DXL_ID 0x08
// #define DRV1_DXL_ID 0x06

DynamixelDevice drive1(DRV1_DXL_ID); 
DynamixelDevice drive2(DRV2_DXL_ID);

uint8_t pid_p = 150, pid_i = 80, pid_d = 0;
int16_t power_rf = 0, speed_rf = 0, power_lf = 0, speed_lf = 0,
        power_rb = 0, speed_rb = 0, power_lb = 0, speed_lb = 0; 

// SPI common lines

#define ESP_SCK 25    
#define ESP_MOSI 13 
#define ESP_MISO 14    

// SD card and external Flash defines, objects, variables
#define SD_CS 15

// Keyboard defines, objects, 

#define BUT_SS 26

// Создание объекта buzzer с громкостью 10
ardu_esp32_buzzer buzzer(10);

#define LED_1 27
#define LED_2 14
#define LED_3 12
#define LED_4 13

#define LED_R 15
#define LED_G 2
#define LED_B 18

void setup(void)
{
  pinMode(LED_1, OUTPUT);
  digitalWrite(LED_1, HIGH);
  pinMode(LED_2, OUTPUT);
  digitalWrite(LED_2, HIGH);
  pinMode(LED_3, OUTPUT);
  digitalWrite(LED_3, HIGH);
  pinMode(LED_4, OUTPUT);
  digitalWrite(LED_4, HIGH);

  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, HIGH);
  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, HIGH);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, HIGH);

  // Starting PC Serial
  PC_SERIAL.setTimeout(5);
  PC_SERIAL.begin(115200);
  // PC_SERIAL.begin(1000000);
  PC_SERIAL.println("\r\n");
  PC_SERIAL.println("Starting Ardu V8");

  // char ap_uniq_part[16] = {0};
  // ap_uniq_part[0] = '-';
  // getUniqName(5, ap_uniq_part + 1);
  // strcat(ap_ssid, ap_uniq_part);
  // strcat(ap_ssid, "-AP");

  // Starting DXL bus 
  drive1.protocolVersion(1);
  drive2.protocolVersion(1);
  DxlMaster.setTimeOut(250);
  
  DxlMaster.begin(57600, &DXL_SERIAL, DXL_NO_DIR_PIN, DXL_NO_DIR_PIN,
                                DXL_SERIAL_TX_PIN, DXL_SERIAL_RX_PIN);
  // DxlMaster.begin(1000000, &DXL_SERIAL, DXL_NO_DIR_PIN, DXL_NO_DIR_PIN,
  //                               DXL_SERIAL_TX_PIN, DXL_SERIAL_RX_PIN);
  delay(100);

  // drive1.write(MAX_POWER_L, (uint16_t)1000);
  // drive1.write(CONTROL_PERIOD, (uint8_t)50);
  // drive1.write(MOTOR_MODE, (uint8_t)2);
  // drive1.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  // drive1.write(M1_GOAL_SPEED_L, (int16_t)0);
  // drive1.write(M2_GOAL_SPEED_L, (int16_t)0);
  // drive1.write(PID_P, (uint8_t)pid_p);
  // drive1.write(PID_I, (uint8_t)pid_i);
  // drive1.write(PID_D, (uint8_t)pid_d);
  
  // drive2.write(MAX_POWER_L, (uint16_t)1000);
  // drive2.write(CONTROL_PERIOD, (uint8_t)50);
  // drive2.write(MOTOR_MODE, (uint8_t)2);
  // drive2.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  // drive2.write(M1_GOAL_SPEED_L, (int16_t)0);
  // drive2.write(M2_GOAL_SPEED_L, (int16_t)0);
  // drive2.write(PID_P, (uint8_t)pid_p);
  // drive2.write(PID_I, (uint8_t)pid_i);
  // drive2.write(PID_D, (uint8_t)pid_d);

  PC_SERIAL.println("DXL motor controller connected");

  // digitalWrite(AR_RST_CNTRL, 1);
  delay(2000);
}

int16_t max_speed_module = 500;
int16_t speed_value = 100;
uint8_t moving_flag = 0;
int16_t s_spd_m[4] = {0};
int16_t r_enc_m[4] = {0}, r_spd_m[4] = {0};

uint8_t uno_firmware_upload_mode = 0, upload_started = 0;

uint8_t buzzer_time_counter = 0;

uint16_t j = 0;

void loop(void)
{
  // обработка нажатий кнопок и управление buzzer
  uint8_t mask_pressed = keyboard.getKeyboard(), mult = 0;

  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);
  digitalWrite(LED_3, HIGH);
  digitalWrite(LED_4, HIGH);
  // Проверка отдельных кнопок и выполнение действий
  if (mask_pressed & 1){   // Кнопка 1
    moving_flag = 0;
    digitalWrite(LED_1, LOW);
  }
  
  if (mask_pressed & 2){   // Кнопка 2
    speed_value -= 10;
    digitalWrite(LED_2, LOW);
  }
  
  if (mask_pressed & 4){   // Кнопка 3
    speed_value += 10;
    digitalWrite(LED_3, LOW);
  }
  
  if (mask_pressed & 8){   // Кнопка 4
    moving_flag = 1;
    digitalWrite(LED_4, LOW);
  }

  if (abs(speed_value) <= 100)
  {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
  }
  else if (abs(speed_value) <= 500)
  {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  }
  else
  {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, HIGH);
  }
  // Подсчет количества нажатых кнопок (mult)
  for (uint32_t mask_buf = mask_pressed & 0xFF; mask_buf > 0; mult++)
  {
    mask_buf >>= 1; 
  }
  // Если кнопки нажаты и счетчик времени < 1, воспроизводим звук
  if (mult && buzzer_time_counter < 1)
  {
    buzzer.playFrequency(mult * 880); // Частота = количество кнопок * 880 Гц
    buzzer_time_counter++;
  }
  else if (mult && buzzer_time_counter >= 1)
  {
    buzzer.mute(); // Отключаем звук после одного цикла
  }
  else
  {
    buzzer.mute();
    buzzer_time_counter = 0;
  }

  // speed_value = speed_value > max_speed_module ? max_speed_module : 
  //               speed_value < -max_speed_module ? -max_speed_module : speed_value;
  
  if (moving_flag)
  {
    s_spd_m[0] = speed_value;
    s_spd_m[1] = -speed_value;
    s_spd_m[2] = speed_value;
    s_spd_m[3] = -speed_value;
  }
  else
  {
    s_spd_m[0] = 0;
    s_spd_m[1] = 0;
    s_spd_m[2] = 0;
    s_spd_m[3] = 0;
  }

  drive1.write(M1_GOAL_SPEED_L, s_spd_m[0]);
  drive1.write(M2_GOAL_SPEED_L, s_spd_m[1]);
  drive2.write(M1_GOAL_SPEED_L, s_spd_m[2]);
  drive2.write(M2_GOAL_SPEED_L, s_spd_m[3]);

  drive1.read(M1_ENCODER_L, r_enc_m[0]);
  drive1.read(M2_ENCODER_L, r_enc_m[1]);
  drive2.read(M1_ENCODER_L, r_enc_m[2]);
  drive2.read(M2_ENCODER_L, r_enc_m[3]);
  drive1.read(M1_PRESENT_SPEED_L, r_spd_m[0]);
  drive1.read(M2_PRESENT_SPEED_L, r_spd_m[1]);
  drive2.read(M1_PRESENT_SPEED_L, r_spd_m[2]);
  drive2.read(M2_PRESENT_SPEED_L, r_spd_m[3]);

  // Send current info about driver and sensors to PC
  PC_SERIAL.print("\033[2K");
  PC_SERIAL.println("Speed const: " + String(speed_value));
  
  PC_SERIAL.print("\033[2K");
  PC_SERIAL.println("Curr. sp. M1: " + String(r_spd_m[0]) + "\tCurr. sp. M2: " + String(r_spd_m[1]));
  PC_SERIAL.print("\033[2K");
  PC_SERIAL.println("Curr. sp. M3: " + String(r_spd_m[2]) + "\tCurr. sp. M4: " + String(r_spd_m[3]));
  
  PC_SERIAL.print("\033[2K");
  PC_SERIAL.println("Enc. 1: " + String(r_enc_m[0]) + "\tEnc. 2: " + String(r_enc_m[1]));
  PC_SERIAL.print("\033[2K");
  PC_SERIAL.println("Enc. 3: " + String(r_enc_m[2]) + "\tEnc. 4: " + String(r_enc_m[3]));
  
  PC_SERIAL.print("\033[2K");
  PC_SERIAL.print("KEYBOARD MASK: ");
  PC_SERIAL.print(mask_pressed);
  PC_SERIAL.print("\r\033[5A");

  delay(100);
}
