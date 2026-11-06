#ifndef __KIRUS_DRIVE_H__
#define __KIRUS_DRIVE_H__

#define DXL_BAUD                   (4)  // uint8_t   34 - "57600", 16 - "115200", 1 - "1000000" 
#define MAX_POWER_L                (24) // uint16_t  0..1023       driver duty/1023
#define MOTOR_MODE                 (26) // uint8_t   0..2          External, PWM, Speed  
#define ENCODER_MODE               (27) // uint8_t   0..3          Disabled, AB, A rising and falling, A rising only
#define M1_POWER_L                 (28) // int16_t  -1023..1023    driver duty/1023 with direction
#define M2_POWER_L                 (30) // int16_t  -1023..1023    driver duty/1023 with direction
#define M1_ENCODER_L               (32) // int16_t  -32768..32767  encoder ticks
#define M1_PRESENT_SPEED_L         (34) // int16_t                 encoder ticks / 100 ms
#define M2_ENCODER_L               (36) // int16_t                 encoder ticks
#define M2_PRESENT_SPEED_L         (38) // int16_t                 encoder ticks / 100 ms
#define M1_GOAL_SPEED_L            (40) // int16_t  -1000..1000*   encoder ticks / 100 ms
#define M2_GOAL_SPEED_L            (42) // int16_t  -1000..1000*   encoder ticks / 100 ms
#define CONTROL_PERIOD             (45) // uint8_t   1..100**      ms
#define PID_P                      (47) // uint8_t   0..255        power += PID_P*(Mx_GOAL_SPEED_L - Mx_PRESENT_SPEED_L) / 255
#define PID_I                      (48) // uint8_t   0..255        power += PID_I*integral_error / 255, integral_error = -10230..10230
#define PID_D                      (49) // uint8_t   0..255        power += PID_D*diff_error / 255

// *  Limited by about 10kHz max encoder tick frequency that driver can serve. Then tick skipping and inaccurate control may occure
// ** Due to loop/protocol/encoder processing times the reasonable min is about 5 ms

#define ENCODER_MODE_DISABLE  (0)
#define ENCODER_MODE_AB       (1)
#define ENCODER_MODE_A        (2)
#define ENCODER_MODE_A_RISING (3)

#define DXL_SERIAL_TX_PIN 32
#define DXL_SERIAL_RX_PIN 33
#define DXL_NO_DIR_PIN (-1)

#endif /* __KIRUS_DRIVE_H__ */
