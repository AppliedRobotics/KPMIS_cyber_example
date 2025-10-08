#include <DxlMaster2.h>
#include <SCServo.h>
#include <math.h>

// ----------------- НАСТРОЙКИ DYNAMIXEL -----------------
#define DXL_BAUD                   (4)  // Скорость порта Dynamixel (34-"57600", 16-"115200", 1-"1000000") 
#define MAX_POWER_L                (24) // Макс. мощность двигателя (0..1023)
#define MOTOR_MODE                 (26) // Режим двигателя: 0..2 - External, PWM, Speed
#define ENCODER_MODE               (27) // Режим энкодера: 0..3 - Disabled, AB, A rising and falling, A rising only
#define M1_POWER_L                 (28) // Мощность двигателя 1 (-1023..1023)
#define M2_POWER_L                 (30) // Мощность двигателя 2 (-1023..1023)
#define M1_ENCODER_L               (32) // Значение энкодера двигателя 1 (-32768..32767)
#define M1_PRESENT_SPEED_L         (34) // Текущая скорость двигателя 1 (такт энкодера/100мс)
#define M2_ENCODER_L               (36) // Значение энкодера двигателя 2
#define M2_PRESENT_SPEED_L         (38) // Текущая скорость двигателя 2
#define M1_GOAL_SPEED_L            (40) // Целевая скорость двигателя 1 (-1000..1000)
#define M2_GOAL_SPEED_L            (42) // Целевая скорость двигателя 2 (-1000..1000)
#define CONTROL_PERIOD             (45) // Период контроля (мс)
#define PID_P                      (47) // Пропорциональная составляющая PID
#define PID_I                      (48) // Интегральная составляющая PID
#define PID_D                      (49) // Дифференциальная составляющая PID

// Примечания:
// * Ограничено частотой энкодера около 10кГц. При превышении возможны пропуски и ошибки.
// ** Минимальный период контроля ~5 мс из-за времени обработки цикла, протокола и энкодеров.

#define ENCODER_MODE_DISABLE  (0) // Энкодер выключен
#define ENCODER_MODE_AB       (1) // Энкодер AB
#define ENCODER_MODE_A        (2) // Только сигнал A
#define ENCODER_MODE_A_RISING (3) // Только фронт A

#define DXL_SERIAL_TX_PIN 32
#define DXL_SERIAL_RX_PIN 33
#define DXL_NO_DIR_PIN (-1)

#define DRV1_DXL_ID 71  // ID привода 1
#define DRV2_DXL_ID 72  // ID привода 2
#define DXL_BAUD 57600
#define NANOPI_BAUD 1000000
#define FEETECH_BAUD 115200

constexpr int MAX_TICK = 700;          // Максимальное количество тактов энкодера
constexpr float RAD_TO_TICK = 63.66;   // Перевод радиан в такты энкодера
constexpr float TICK_TO_RAD = 0.0157;  // Перевод тактов энкодера в радианы

constexpr uint8_t pid_p = 255;  // PID параметры
constexpr uint8_t pid_i = 40;
constexpr uint8_t pid_d = 200;

constexpr uint8_t joints = 5;    // Количество сочленений манипулятора

// ----------------- ОБЪЕКТЫ -----------------
DynamixelDevice drive1(DRV1_DXL_ID);
DynamixelDevice drive2(DRV2_DXL_ID);
SMS_STS st;

// Время последней отправки данных
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // Интервал отправки данных каждые 100 мс
// ----------------- ПЕРЕМЕННЫЕ ДЛЯ КОЛЕС -----------------
float w_fl_rad = 0; // Левая передняя скорость, рад/с
float w_bl_rad = 0; // Левая задняя
float w_fr_rad = 0; // Правая передняя
float w_br_rad = 0; // Правая задняя

int16_t w_fl = 0;
int16_t w_bl = 0;
int16_t w_fr = 0;
int16_t w_br = 0;

int16_t real_fr = 0; // Реальная скорость энкодера
int16_t real_br = 0;
int16_t real_fl = 0;
int16_t real_bl = 0;

// ----------------- ПЕРЕМЕННЫЕ ДЛЯ ДАТЧИКОВ -----------------
constexpr uint8_t contactRight = 5; // Контакт правый
constexpr uint8_t contactLeft = 8;  // Контакт левый
constexpr uint8_t PIN_TRIG = 10;  // Пины ультразвукового датчика
constexpr uint8_t PIN_ECHO = 9;
constexpr uint8_t line1 = 3;         // Датчики линии
constexpr uint8_t line2 = 6;
constexpr uint8_t line3 = 4;
constexpr uint8_t line4 = 7;

long duration;  // Время импульса ультразвука
float cm;       // Расстояние до препятствия в см

// ----------------- ПЕРЕМЕННЫЕ ДЛЯ МАНИПУЛЯТОРА -----------------
constexpr float pi = 3.14159265;      // Число пи
constexpr float unit = 0.0015;        // Перевод радиан в условные единицы привода
float X, Y, Z, pitch;                  // Текущие координаты схвата, м
float coords[9];                        // Массив координат, получаемых из Serial

float Q_0[joints];   // Текущие обобщенные координаты сочленений, рад
float Q_d[joints];   // Вычисленные координаты сочленений, рад

// Длины звеньев манипулятора
constexpr float l1 = 0.14;
constexpr float l2 = 0.12;
constexpr float l3 = 0.11;
constexpr float l4 = 0.055;
constexpr float l5 = 0.11;
constexpr float l45 = sqrt(l4*l4 + l5*l5); // Длина составного звена
constexpr float angle45 = atan2(l4, l5);   // Угол составного звена

constexpr uint8_t ft_id[joints] = {1, 2, 3, 4, 5};           // Массив ID приводов

void setup(void) {
  DxlMaster.begin(DXL_BAUD);             // Инициализация последовательного порта для Dynamixel
  drive1.protocolVersion(1);             // Установка версии протокола для первого драйва
  drive2.protocolVersion(1);             // Установка версии протокола для второго драйва
  DxlMaster.setTimeOut(5);               // Установка таймаута передачи

  while (drive1.ping() != DYN_STATUS_OK) {  // Ожидание доступности первого привода
    delay(10);
  }

  while (drive2.ping() != DYN_STATUS_OK) {  // Ожидание доступности второго привода
    delay(10);
  }

  // Настройка первого привода
  drive1.write(CONTROL_PERIOD, (uint8_t)5);          // Период обновления управления
  drive1.write(M1_PRESENT_SPEED_L, (int16_t)0);      // Обнуление скорости мотора 1
  drive1.write(M2_PRESENT_SPEED_L, (int16_t)0);      // Обнуление скорости мотора 2
  drive1.write(MOTOR_MODE, (uint8_t)2);             // Установка режима мотора
  drive1.write(ENCODER_MODE, (uint8_t)ENCODER_MODE_AB); // Настройка энкодера
  drive1.write(M1_GOAL_SPEED_L, (int16_t)0);        // Начальная цель скорости мотора 1
  drive1.write(M2_GOAL_SPEED_L, (int16_t)0);        // Начальная цель скорости мотора 2
  drive1.write(PID_P, (uint8_t)pid_p);              // Настройка коэффициента PID P
  drive1.write(PID_I, (uint8_t)pid_i);              // Настройка коэффициента PID I
  drive1.write(PID_D, (uint8_t)pid_d);              // Настройка коэффициента PID D
  drive1.write(MAX_POWER_L, (uint16_t)1000);        // Максимальная мощность

  // Настройка второго привода (аналогично первому)
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
  
  // Примеры потребления тока в разных режимах (для информации)
  // 250 - 300 mA
  // 300 - 350 mA
  // 400 - 450 mA
  // 500 - 580 mA
  // 600 - 650 mA
  // 700 - 800 mA
  // 800 - 950 mA
  // 900 - 1100 mA
  // 1000 - 1150 mA

  Serial.begin(FEETECH_BAUD);  // Инициализация последовательного порта для FEETECH
  st.pSerial = &Serial;        // Присваиваем Serial объекту st
  st.WritePosEx(1, 2000, 600); // Установка начальных позиций сервоприводов
  st.WritePosEx(2, 2000, 600);
  st.WritePosEx(3, 2000, 600);
  st.WritePosEx(4, 2000, 600);
  st.WritePosEx(5, 2000, 600);
  st.WritePosEx(6, 2000, 600);  
  Serial.begin(NANOPI_BAUD);   // Переключение на другой порт
  pinMode(PIN_TRIG, OUTPUT);   // Настройка пина триггера для ультразвукового датчика
}

void loop() {    
    if (Serial.available()) {  // Проверка поступления данных с порта
      // Считываем координаты и скорости колес из Serial
      coords[0] = Serial.parseFloat();  
      coords[1] = Serial.parseFloat();  
      coords[2] = Serial.parseFloat();  
      coords[3] = Serial.parseFloat();  
      coords[4] = Serial.parseFloat();  
      coords[5] = Serial.parseFloat();  
      coords[6] = Serial.parseFloat();  
      coords[7] = Serial.parseFloat();  

      // Конвертируем радианы в тики для колес
      w_fl = radPerSecToTicks(coords[0]);
      w_bl = radPerSecToTicks(coords[1]);
      w_fr = radPerSecToTicks(coords[2]);
      w_br = radPerSecToTicks(coords[3]);

      Serial.begin(DXL_BAUD);  // Переключаемся на Dynamixel порт
      // Передаем скорости колес на приводы
      drive1.write(M1_GOAL_SPEED_L, w_fr);
      drive1.write(M2_GOAL_SPEED_L, w_br);
      drive2.write(M1_GOAL_SPEED_L, w_fl);
      drive2.write(M2_GOAL_SPEED_L, w_bl);

      if(inverse_problem()) {             // Решаем обратную задачу кинематики
        Serial.begin(FEETECH_BAUD);
        moving(Q_d);           // Передаем рассчитанные углы приводам
      }
      Serial.begin(NANOPI_BAUD); // Возврат к основному порту
    }
  
  if (millis() - lastSendTime >= sendInterval) {  // Отправка данных с заданным интервалом
    lastSendTime = millis();
    get_dist(); // Получаем расстояние с ультразвукового датчика
    Serial.begin(DXL_BAUD);
    // Считываем реальные скорости моторов
    drive1.read(M1_PRESENT_SPEED_L, real_fr);
    drive1.read(M2_PRESENT_SPEED_L, real_br);
    drive2.read(M1_PRESENT_SPEED_L, real_fl);
    drive2.read(M2_PRESENT_SPEED_L, real_bl);
    Serial.begin(FEETECH_BAUD);
    get_pos();               // Считываем текущие позиции манипулятора
    forward_kinematics(Q_0); // Вычисляем положение схвата в пространстве
    Serial.begin(NANOPI_BAUD);
    // Отправка данных о скорости, координатах, датчиках
    Serial.print(ticksToRadPerSec(real_fl)); Serial.print(",");
    Serial.print(ticksToRadPerSec(real_bl)); Serial.print(",");
    Serial.print(ticksToRadPerSec(real_fr)); Serial.print(",");
    Serial.print(ticksToRadPerSec(real_br)); Serial.print(",");
    Serial.print(' ');
    Serial.print(X); Serial.print(',');
    Serial.print(Y); Serial.print(',');
    Serial.print(Z); Serial.print(',');
    Serial.print(pitch); Serial.print(',');
    Serial.print(' ');
    Serial.print(digitalRead(line1)); Serial.print(",");
    Serial.print(digitalRead(line2)); Serial.print(",");
    Serial.print(digitalRead(line3)); Serial.print(",");
    Serial.print(digitalRead(line4)); Serial.print(",");
    Serial.print(' ');
    Serial.print(digitalRead(contactLeft)); Serial.print(",");
    Serial.print(digitalRead(contactRight)); Serial.print(",");
    Serial.print(' ');
    Serial.println(cm);
  }
}

inline float ticksToRadPerSec(int16_t ticks) {
  return ticks * TICK_TO_RAD;  // Конвертация тиков мотора в радианы в секунду
}

inline int16_t radPerSecToTicks(float radPerSec) {
  int16_t ticks = (int16_t)(radPerSec * RAD_TO_TICK);  // Конвертация радиан в секунду в тики мотора
  if (ticks > MAX_TICK) return MAX_TICK;               // Ограничение максимального значения
  if (ticks < -MAX_TICK) return -MAX_TICK;            // Ограничение минимального значения
  return ticks;
}

void get_dist(){
  // Генерация короткого импульса для ультразвукового датчика
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  
  // Ожидание передачи сигнала
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Измерение времени возврата сигнала
  duration = pulseIn(PIN_ECHO, HIGH, 6000);

  // Преобразование времени в расстояние (см)
  cm = (duration / 2) / 29.1;
}

//---------------ФУНКЦИИ ДЛЯ МАНИПУЛЯТОРА------------------//
void get_pos() {
  // Чтение текущих позиций всех сервоприводов и преобразование в углы
  Q_0[0] = (2048 - st.ReadPos(1)) * unit;
  Q_0[1] = (st.ReadPos(2) - 226) * unit;
  Q_0[2] = (st.ReadPos(3)-2144) * unit;
  Q_0[3] = (st.ReadPos(4)-1140) * unit;
  Q_0[4] = (st.ReadPos(5) - 2048) * unit;
}

void forward_kinematics(float *q) {
  // Извлекаем углы суставов из массива q
  const float q1 = q[0];
  const float q2 = q[1];
  const float q3 = q[2];
  const float q4 = q[3];
  const float q5 = q[4];

  // Вычисляем положение схвата в пространстве (X, Y, Z) и ориентацию (pitch)
  X = (l2 * cos(q2) + l3 * cos(q2+q3) + l45 * cos(q2+q3+q4-angle45)) * cos(q1);
  Y = (l2 * cos(q2) + l3 * cos(q2+q3) + l45 * cos(q2+q3+q4-angle45)) * sin(q1);
  Z = l1 + l2 * sin(q2) + l3 * sin(q3+q2) + l45 * sin(q2 + q3 + q4 - angle45);
  pitch = q2 + q3 + q4; // Общий угол наклона схвата
}

void copy_arr(float* in, float* out, int n) {
  // Копируем массив out в массив in длиной n
  for(int i = 0; i < n; i++) {
    in[i] = out[i];
  }
}

bool inverse_problem() {
  // Извлекаем координаты и угол наклона из входного массива coords
  const float x = coords[4];
  const float y = coords[5];
  const float z = coords[6];
  const float pitch = coords[7];

  // Корректируем высоту для расчета
  const float z0 = z - l1;
  const float w = sqrt(x*x + y*y); // Расстояние до оси вращения q1
  float q1;
  const float q1_pre = atan2(y, x); // Начальное приближение для угла q1
  const float q5 = coords[8];       // Дополнительный угол q5

  // Смещение для пятого сустава
  const float z1 = z0 - l45 * sin(pitch - angle45);
  const float w11 = w - l45 * cos(pitch - angle45);
  const float w21 = w + l45 * cos(pitch - angle45);
  float a, b, c, D, w1, c1, k;

  // Проверяем дискриминант для первой возможной позиции
  bool success = calc_D(&a, &b, &c, &D, z1, w11);
  if (success) {
    w1 = w11;
    q1 = q1_pre;
  } else {
    // Если первая позиция невозможна, пробуем вторую
    success = calc_D(&a, &b, &c, &D, z1, w21);
    w1 = w21;
    q1 = q1_pre + pi;
  }
  
  if (success) {
    // Вычисляем два возможных решения для сустава q2
    const float z21 = (-b + sqrt(D)) / (2*a);
    const float z22 = (-b - sqrt(D)) / (2*a);
    const float w21 = (l2*l2 - l3*l3 + w1*w1 + z1*z1 - 2*z21*z1)/(2*w1);
    const float w22 = (l2*l2 - l3*l3 + w1*w1 + z1*z1 - 2*z22*z1)/(2*w1);

    // Углы для остальных суставов
    const float alpha = atan2(z1-z21, w1-w21);
    const float beta = atan2(z1-z22, w1-w22); 
    const float q21 = atan2(z21, w21);
    const float q22 = atan2(z22, w22);
    const float q31 =  q21-alpha;
    const float q32 = q22 - beta;
    const float q41 =  alpha - pitch;
    const float q42 =  beta - pitch; 

    // Переводим найденные углы в координаты моторов
    const float Q1[5] = {q1 , q21, -q31, -q41, q5};
    const float Q2[5] = {q1, q22, -q32, -q42, q5};  

    // Выбираем оптимальное решение по минимальной дистанции до текущей позиции
    const float dist1 = sqrt(pow(Q1[0]-Q_0[0],2)+ pow(Q1[1]-Q_0[1],2) + pow(Q1[2]-Q_0[2],2) + pow(Q1[3]-Q_0[3],2) + pow(Q1[4]-Q_0[4],2));
    const float dist2 = sqrt(pow(Q2[0]-Q_0[0],2)+ pow(Q2[1]-Q_0[1],2) + pow(Q2[2]-Q_0[2],2) + pow(Q2[3]-Q_0[3],2) + pow(Q2[4]-Q_0[4],2));

    if (dist1 < dist2) copy_arr(Q_d, Q1, joints); 
    else copy_arr(Q_d, Q2, joints);

    return true; // Обратная задача решена успешно
  }

  // Если решения не существует, возвращаем false
  return false;      
}

void moving(float* q) {
  // Отправляем вычисленные позиции моторов на сервоприводы
  st.WritePosEx(ft_id[0], 2048 - (int)(q[0] / unit), 600);
  st.WritePosEx(ft_id[1], (int)(q[1] / unit) + 226, 600);
  st.WritePosEx(ft_id[2], (int)(q[2] / unit) + 2144, 600);
  st.WritePosEx(ft_id[3], (int)(q[3] / unit) + 1140, 600);
  st.WritePosEx(ft_id[4], (int)(q[4] / unit) + 2048, 600);
  
  get_pos(); // Обновляем текущее положение для последующих расчетов
}

bool calc_D(float *a, float *b, float *c, float *D, float z1, float w1) {
  // Вычисляем коэффициенты квадратного уравнения для OЗК
  const float c1 = (l3*l3 - w1*w1 - z1*z1 - l2*l2)/(-2);
  *a = z1*z1 + w1*w1;
  *b = -2*z1*c1;
  *c = c1*c1 - l2*l2*w1*w1;
  *D = *b * (*b) - 4 * (*a) * (*c);

  return *D >= 0; // true если решения существуют
}