#include <SPI.h>
#include "ardu_drive.hpp"
#include <DxlMaster2.h>
#include <SCServo.h>
#include <math.h>

#define DRV1_DXL_ID 71
#define DRV2_DXL_ID 72
#define DXL_BAUD 57600
#define NANOPI_BAUD 1000000
#define FEETECH_BAUD 115200

constexpr int MAX_TICK = 700;
constexpr float RAD_TO_TICK = 63.66;
constexpr float TICK_TO_RAD = 0.0157;

constexpr uint8_t pid_p = 255;
constexpr uint8_t pid_i = 40;
constexpr uint8_t pid_d = 200;

constexpr uint8_t PIN_TRIG = 10;
constexpr uint8_t PIN_ECHO = 9;
constexpr uint8_t joints = 5;

DynamixelDevice drive1(DRV1_DXL_ID);
DynamixelDevice drive2(DRV2_DXL_ID);
SMS_STS st;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // каждые 100 мс

float w_fl_rad = 0; // left forward
float w_bl_rad = 0; // left back
float w_fr_rad = 0; // right back
float w_br_rad = 0; // right forward

int16_t w_fl = 0;
int16_t w_bl = 0;
int16_t w_fr = 0;
int16_t w_br = 0;

int16_t real_fr = 0;
int16_t real_br = 0;
int16_t real_fl = 0;
int16_t real_bl = 0;

//---------------ПЕРЕМЕННЫЕ ДЛЯ ДАТЧИКОВ ------------------//
constexpr uint8_t contactRight = 5; 
constexpr uint8_t contactLeft = 8;
constexpr uint8_t line1 = 3;
constexpr uint8_t line2 = 6; 
constexpr uint8_t line3 = 4; 
constexpr uint8_t line4 = 7; 

long duration;
float cm;

//---------------ПЕРЕМЕННЫЕ ДЛЯ МАНИПУЛЯТОРА ------------------//

constexpr float pi = 3.14159265;      // Число пи
constexpr float unit = 0.0015;         // Переменная, отвечающая за перевод радиан в условные единицы положения привода двигателя 
float X, Y, Z, pitch;   // Текущие координаты схвата, м 
float coords[9];  

float Q_0[joints];   // Текущие обобщенные координаты сочленений, рад 
float Q_d[joints];  // Вычисленные обобщенные координаты сочленений, рад

constexpr float l1 = 0.14;            // Длины звеньев манипулятора, м
constexpr float l2 = 0.12;
constexpr float l3 = 0.11;
constexpr float l4 = 0.055;
constexpr float l5 = 0.11;
constexpr float l45 = sqrt(l4*l4 + l5*l5);
constexpr float angle45 = atan2(l4, l5);

//constexpr float CONSTRAINTS[joints][2] = {{-3.14, 3.14},     // Двумерный массив, содержащий предельные значения для каждой степени подвижности
//                           {-2.01, 2.36},
//                           {-2.3, 2.36},
//                           {-1.57, 1.57},
//                           {-3.14, 3.14}};

constexpr uint8_t ft_id[joints] = {1, 2, 3, 4, 5};           // Массив ID приводов


void setup(void) {
  DxlMaster.begin(DXL_BAUD);
  drive1.protocolVersion(1);
  drive2.protocolVersion(1);
  DxlMaster.setTimeOut(5);

  while (drive1.ping() != DYN_STATUS_OK) {
    delay(10);
  }

  while (drive2.ping() != DYN_STATUS_OK) {
    delay(10);
  }

  drive1.write(CONTROL_PERIOD, (uint8_t)5);
  drive1.write(M1_PRESENT_SPEED_L, (int16_t)0);
  drive1.write(M2_PRESENT_SPEED_L, (int16_t)0);
  drive1.write(MOTOR_MODE, (uint8_t)2);
  drive1.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  drive1.write(M1_GOAL_SPEED_L, (int16_t)0);
  drive1.write(M2_GOAL_SPEED_L, (int16_t)0);
  drive1.write(PID_P, (uint8_t)pid_p);
  drive1.write(PID_I, (uint8_t)pid_i);
  drive1.write(PID_D, (uint8_t)pid_d);
  drive1.write(MAX_POWER_L, (uint16_t)1000);

  drive2.write(CONTROL_PERIOD, (uint8_t)5);
  drive2.write(M1_PRESENT_SPEED_L, (int16_t)0);
  drive2.write(M2_PRESENT_SPEED_L, (int16_t)0);
  drive2.write(MOTOR_MODE, (uint8_t)2);
  drive2.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB);
  drive2.write(M1_GOAL_SPEED_L, (int16_t)0);
  drive2.write(M2_GOAL_SPEED_L, (int16_t)0);
  drive2.write(PID_P, (uint8_t)pid_p);
  drive2.write(PID_I, (uint8_t)pid_i);
  drive2.write(PID_D, (uint8_t)pid_d);
  drive2.write(MAX_POWER_L, (uint16_t)1000);
  
  // 250 - 300 mA
  // 300 - 350 mA
  // 400 - 450 mA
  // 500 - 580 mA
  // 600 - 650 mA
  // 700 - 800 mA
  // 800 - 950 mA
  // 900 - 1100 mA
  // 1000 - 1150 mA
  Serial.begin(FEETECH_BAUD);
  st.pSerial = &Serial;
  st.WritePosEx(1, 2048, 600);
  st.WritePosEx(2, 1250, 600);
  st.WritePosEx(3, 1120, 600);
  st.WritePosEx(4, 1140, 600);
  st.WritePosEx(5, 2048, 600);
  st.WritePosEx(6, 3000, 600);  
  Serial.begin(NANOPI_BAUD);
  pinMode(PIN_TRIG, OUTPUT);
}


void loop() {    
    if (Serial.available()) {
      //processSerialData();
      coords[0] = Serial.parseFloat();  
      coords[1] = Serial.parseFloat();  
      coords[2] = Serial.parseFloat();  
      coords[3] = Serial.parseFloat();  
      coords[4] = Serial.parseFloat();  
      coords[5] = Serial.parseFloat();  
      coords[6] = Serial.parseFloat();  
      coords[7] = Serial.parseFloat();  

      w_fl = radPerSecToTicks(coords[0]);
      w_bl = radPerSecToTicks(coords[1]);
      w_fr = radPerSecToTicks(coords[2]);
      w_br = radPerSecToTicks(coords[3]);

      Serial.begin(DXL_BAUD);
      drive1.write(M1_GOAL_SPEED_L, w_fr);
      drive1.write(M2_GOAL_SPEED_L, w_br);
      drive2.write(M1_GOAL_SPEED_L, w_fl);
      drive2.write(M2_GOAL_SPEED_L, w_bl);

      if(inverse_problem()) {             // Решаем обратную задачу кинематики для введенных координат,
        Serial.begin(FEETECH_BAUD);
        moving(Q_d);                      // если получено корректное решение передаем его в функцию moving
      }
      Serial.begin(NANOPI_BAUD);
    }
  
  if (millis() - lastSendTime >= sendInterval) { 
    lastSendTime = millis();
    get_dist(); // получаем расстояние с сонара в сантиметрах
    Serial.begin(DXL_BAUD);
    drive1.read(M1_PRESENT_SPEED_L, real_fr);
    drive1.read(M2_PRESENT_SPEED_L, real_br);
    drive2.read(M1_PRESENT_SPEED_L, real_fl);
    drive2.read(M2_PRESENT_SPEED_L, real_bl);
    Serial.begin(FEETECH_BAUD);
    get_pos();
    forward_kinematics(Q_0);
    Serial.begin(NANOPI_BAUD);
    Serial.print(ticksToRadPerSec(real_fl)); Serial.print(",");
    Serial.print(ticksToRadPerSec(real_bl)); Serial.print(",");
    Serial.print(ticksToRadPerSec(real_fr)); Serial.print(",");
    Serial.print(ticksToRadPerSec(real_br)); Serial.print(",");
    Serial.print(X); Serial.print(',');
    Serial.print(Y); Serial.print(',');
    Serial.print(Z); Serial.print(',');
    Serial.print(pitch); Serial.print(',');
    Serial.print(digitalRead(line1)); Serial.print(",");
    Serial.print(digitalRead(line2)); Serial.print(",");
    Serial.print(digitalRead(line3)); Serial.print(",");
    Serial.print(digitalRead(line4)); Serial.print(",");
    Serial.print(digitalRead(contactLeft)); Serial.print(",");
    Serial.print(digitalRead(contactRight)); Serial.print(",");
    Serial.println(cm);
  }
}

inline float ticksToRadPerSec(int16_t ticks) {
  return ticks * TICK_TO_RAD;
}

inline int16_t radPerSecToTicks(float radPerSec) {
  int16_t ticks = (int16_t)(radPerSec * RAD_TO_TICK);
  if (ticks > MAX_TICK) return MAX_TICK;
  if (ticks < -MAX_TICK) return -MAX_TICK;
  return ticks;
}

void get_dist(){
  // Сначала генерируем короткий импульс длительностью 2-5 микросекунд.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  
  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  //  Время задержки акустического сигнала на эхолокаторе.
  duration = pulseIn(PIN_ECHO, HIGH, 6000);

  // Теперь осталось преобразовать время в расстояние
  cm = (duration / 2) / 29.1;
}

//---------------ФУНКЦИИ ДЛЯ МАНИПУЛЯТОРА------------------//
void get_pos() {
  Q_0[0] = (2048 - st.ReadPos(1)) * unit;
  Q_0[1] = (st.ReadPos(2) - 226) * unit;
  Q_0[2] = (st.ReadPos(3)-2144) * unit;
  Q_0[3] = (st.ReadPos(4)-1140) * unit;
  Q_0[4] = (st.ReadPos(5) - 2048) * unit;
}

void forward_kinematics(float *q) {
  const float q1 = q[0];
  const float q2 = q[1];
  const float q3 = q[2];
  const float q4 = q[3];
  const float q5 = q[4];

  // Присваиваем глобальным переменным X, Y, Z, pitch рассчитанные координаты схвата   
  X = (l2 * cos(q2) + l3 * cos(q2+q3) + l45 * cos(q2+q3+q4-angle45)) * cos(q1);
  Y = (l2 * cos(q2) + l3 * cos(q2+q3) + l45 * cos(q2+q3+q4-angle45)) * sin(q1);
  Z = l1 + l2 * sin(q2) + l3 * sin(q3+q2) + l45 * sin(q2 + q3 + q4 - angle45);
  pitch = q2 + q3 + q4;
}

void copy_arr(float* in, float* out, int n) {
  for(int i = 0; i < n; i++) {
    in[i] = out[i];
  }
}

bool inverse_problem() {
  const float x = coords[4];
  const float y = coords[5];
  const float z = coords[6];
  const float pitch = coords[7];

  const float z0 = z - l1;
  const float w = sqrt(x*x + y*y);
  float q1;
  const float q1_pre = atan2(y, x);
  const float q5 = coords[8];

  const float z1 = z0 - l45 * sin(pitch - angle45);
  const float w11 = w - l45 * cos(pitch - angle45);
  const float w21 = w + l45 * cos(pitch - angle45);
  float a, b, c, D, w1, c1, k;

  bool success = calc_D(&a, &b, &c, &D, z1, w11);
  if (success) {
    w1 = w11;
    q1 = q1_pre;
  }
  else {
    success = calc_D(&a, &b, &c, &D, z1, w21);
    w1 = w21;
    q1 = q1_pre + pi;
  }
  
  if (success) {
    const float z21 = (-b + sqrt(D)) / (2*a);
    const float z22 = (-b - sqrt(D)) / (2*a);
    const float w21 = (l2*l2 - l3*l3 + w1*w1 + z1*z1 - 2*z21*z1)/(2*w1);
    const float w22 = (l2*l2 - l3*l3 + w1*w1 + z1*z1 - 2*z22*z1)/(2*w1);

    const float alpha = atan2(z1-z21, w1-w21);
    const float beta = atan2(z1-z22, w1-w22); 
    
    const float q21 = atan2(z21, w21);
    const float q22 = atan2(z22, w22);
    const float q31 =  q21-alpha;
    const float q32 = q22 - beta;
    const float q41 =  alpha -pitch;
    const float q42 =  beta - pitch; 

    // Пересчет вычисленных угловых координат в координаты двигателей
    const float Q1[5] = {q1 , q21, -q31, -q41, q5};
    const float Q2[5] = {q1, q22, -q32, -q42, q5};  
    // вычисляем расстояния до каждой из двух возможных позиций, чтобы найти оптимальную
    const float dist1 = sqrt(pow(Q1[0]-Q_0[0],2)+ pow(Q1[1]-Q_0[1],2) + pow(Q1[2]-Q_0[2],2) + pow(Q1[3]-Q_0[3],2) + pow(Q1[4]-Q_0[4],2));
    const float dist2 = sqrt(pow(Q2[0]-Q_0[0],2)+ pow(Q2[1]-Q_0[1],2) + pow(Q2[2]-Q_0[2],2) + pow(Q2[3]-Q_0[3],2) + pow(Q2[4]-Q_0[4],2));

    // С помощью функции check_constrain проверяем полученные решения исходя из ограничений на положение сервоприводов    
//    if (check_constrain(Q1)) {
//      copy_arr(Q_d, Q1, joints);    // Копируем полученное решение в массив Q_d 
//      return true;
//    }

//    if (check_constrain(Q2)) {
//      copy_arr(Q_d, Q2, joints);
//      return true;
//    }
    if (dist1 < dist2) copy_arr(Q_d, Q1, joints); 
    else copy_arr(Q_d, Q2, joints);
    return true;
  }
  // Возвращаем false, если решения ОЗК для заданной конфигурации схвата не существует (D<0)
  // или ни одной из полученных решений не удовлетворяет ограничениям
  return false;      
}

void moving(float* q) {
  st.WritePosEx(ft_id[0], 2048 - (int)(q[0] / unit), 600);
  st.WritePosEx(ft_id[1], (int)(q[1] / unit) + 226, 600);
  st.WritePosEx(ft_id[2], (int)(q[2] / unit) +2144, 600);
  st.WritePosEx(ft_id[3], (int)(q[3] / unit) + 1140, 600);
  st.WritePosEx(ft_id[4], (int)(q[4] / unit) + 2048, 600);
  
  get_pos();                   // Считываем текущее положение сервоприводов
}

bool calc_D(float *a, float *b, float *c, float *D, float z1, float w1) {
  const float c1 = (l3*l3 - w1*w1 - z1*z1 - l2*l2)/(-2);
  
  *a = z1*z1 + w1*w1;
  *b =  -2*z1*c1;
  *c = c1*c1 - l2*l2*w1*w1;
  *D = *b * (*b) - 4 * (*a) * (*c);

  if (*D >= 0) return true; 
  return false;
}

//bool check_constrain(float *q) {
//  bool result;
//  for(int i = 0; i < joints; i++) {
//    result = q[i] > CONSTRAINTS[i][0] && q[i] < CONSTRAINTS[i][1];
//    if(!result) return false; 
//  }
//  return true;
//}
