#include <SCServo.h>
#include <math.h>

#define FEETECH_BAUD 115200
#define joints 5
SMS_STS st;

float pi = 3.14159265;      // Число пи

float X, Y, Z, pitch;   // Текущие координаты схвата, см 
//float X_d, Y_d, Z_d, pitch_d, roll_d;  // Заданные координаты и ориентация схвата, см
float unit = 0.0015;         // Переменная, отвечающая за перевод радиан в условные единицы положения привода двигателя 
float coords[5];


float Q_0[joints];   // Текущие обобщенные координаты сочленений, рад 
float Q_d[joints];  // Вычисленные обобщенные координаты сочленений, рад


float l1 = 0.14;            // Длины звеньев манипулятора, м
float l2 = 0.12;
float l3 = 0.11;
float l4 = 0.055;
float l5 = 0.11;
float l45 = sqrt(l4*l4 + l5*l5);
float angle45 = atan2(l4, l5);

float CONSTRAINTS[joints][2] = {{-3.14, 3.14},     // Двумерный массив, содержащий предельные значения для каждой степени подвижности
                           {-2.01, 2.36},
                           {-2.3, 2.36},
                           {-1.57, 1.57},
                           {-3.14, 3.14}};

//float HOME_POSITION[joints] = {0, 0, pi/2, 0, 0};  // Массив, задающий начальное положение манипулятора
uint8_t ft_id[joints] = {1, 2, 3, 4, 5};           // Массив ID приводов

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100; // каждые 100 мс

// Буфер для последовательного порта
const int RX_BUFFER_SIZE = 128;
volatile char rxBuffer[RX_BUFFER_SIZE];
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;


void setup(void) {
  Serial.begin(FEETECH_BAUD);
  Serial.setTimeout(5);
  st.pSerial = &Serial;
  st.WritePosEx(1, 2048, 600);
  st.WritePosEx(2, 1250, 600);
  st.WritePosEx(3, 1120, 600);
  st.WritePosEx(4, 1140, 600);
  st.WritePosEx(5, 2048, 600);
  st.WritePosEx(6, 3000, 600);  
}

void loop(){
  Serial.begin(1000000);
  if (Serial.available() > 0) {  
      processSerialData();
      Serial.begin(FEETECH_BAUD);
      if(inverse_problem()) {             // Решаем обратную задачу кинематики для введенных координат,
        moving(Q_d);                      // если получено корректное решение передаем его в функцию moving
      }
  }
  if (millis() - lastSendTime >= sendInterval) { 
    
    lastSendTime = millis();
    get_pos();
    print_cartesian_pos(Q_0); 
    Serial.begin(FEETECH_BAUD);
  }
}


void processSerialData() {
  while (Serial.available()) {
    char c = Serial.read();
    uint8_t next = (rxHead + 1) % RX_BUFFER_SIZE;
    
    if (next != rxTail) {
      rxBuffer[rxHead] = c;
      rxHead = next;
      
      // Обработка конца строки
      if (c == '\n') {
        char line[64];
        uint8_t idx = 0;
        
        // Извлекаем строку из буфера
        while (rxTail != rxHead && idx < sizeof(line) - 1) {
          line[idx++] = rxBuffer[rxTail];
          rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
        }
        line[idx] = '\0';
        
        // Парсим данные
        parseLine(line);
      }
    }
  }
}

void parseLine(char* line) {
  Serial.println(line);
  char* token = strtok(line, ",");
  for (int i = 0; i < 4 && token != nullptr; i++) {
    coords[i] = atof(token);
    token = strtok(nullptr, ",");
  }
}

bool inverse_problem() {
  float x = coords[0];
  float y = coords[1];
  float z = coords[2];
  float pitch = coords[3];

  float z0 = z - l1;
  float w = sqrt(x*x + y*y);
  
  float q1;
  float q1_pre = atan2(y, x);
  float q5 = coords[4];

  float z1 = z0 - l45 * sin(pitch - angle45);

  float w11 = w - l45 * cos(pitch - angle45);
  float w21 = w + l45 * cos(pitch - angle45);

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
    float z21, z22, w21, w22;
    z21 = (-b + sqrt(D)) / (2*a);
    z22 = (-b - sqrt(D)) / (2*a);

    w21 = (l2*l2 - l3*l3 + w1*w1 + z1*z1 - 2*z21*z1)/(2*w1);
    w22 = (l2*l2 - l3*l3 + w1*w1 + z1*z1 - 2*z22*z1)/(2*w1);

    float q21, q31, q41, q22, q32, q42;
    float alpha, beta;
    
    q21 = atan2(z21, w21);
    q22 = atan2(z22, w22);
    
    alpha = atan2(z1-z21, w1-w21);
    q31 =  q21-alpha;
    beta = atan2(z1-z22, w1-w22); 
    q32 = q22 - beta;
    q41 =  alpha -pitch;
    q42 =  beta - pitch; 

    // Пересчет вычисленных угловых координат в координаты двигателей
    float Q1[5] = {q1 , q21, -q31, -q41, q5};
    float Q2[5] = {q1, q22, -q32, -q42, q5};  
    float dist1 = sqrt(pow(Q1[0]-Q_0[0],2)+ pow(Q1[1]-Q_0[1],2) + pow(Q1[2]-Q_0[2],2) + pow(Q1[3]-Q_0[3],2) + pow(Q1[4]-Q_0[4],2));
    float dist2 = sqrt(pow(Q2[0]-Q_0[0],2)+ pow(Q2[1]-Q_0[1],2) + pow(Q2[2]-Q_0[2],2) + pow(Q2[3]-Q_0[3],2) + pow(Q2[4]-Q_0[4],2));

    
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
  get_pos();  // Считываем текущие положения сервоприводов Q_0

  // Перемещаем сервоприводы в требуемое положение
//  for (int i = 0; i < joints; i++) {  
//    st.WritePosEx(ft_id[i], (int)(q[i] / unit), 600);
//  }
  
  st.WritePosEx(ft_id[0], 2048 - (int)(q[0] / unit), 600);
  st.WritePosEx(ft_id[1], (int)(q[1] / unit) + 226, 600);
  st.WritePosEx(ft_id[2], (int)(q[2] / unit) +2144, 600);
  st.WritePosEx(ft_id[3], (int)(q[3] / unit) + 1140, 600);
  st.WritePosEx(ft_id[4], (int)(q[4] / unit) + 2048, 600);
  
  //check_moving();   // Ожидаем, пока манипулятор окончит перемещение
  
  get_pos();                   // Считываем текущее положение сервоприводов
  print_cartesian_pos(Q_0);    // и выводим в Serial-порт текущие координаты схвата
}

void copy_arr(float* in, float* out, int n) {
  for(int i = 0; i < n; i++) {
    in[i] = out[i];
  }
}
void forward_kinematics(float *q) {
  
  float q1 = q[0];
  float q2 = q[1];
  float q3 = q[2];
  float q4 = q[3];
  float q5 = q[4];

  // Присваиваем глобальным переменным X, Y, Z, pitch рассчитанные координаты схвата
      
  X = (l2 * cos(q2) + l3 * cos(q2+q3) + l45 * cos(q2+q3+q4-angle45)) * cos(q1);
 
  Y = (l2 * cos(q2) + l3 * cos(q2+q3) + l45 * cos(q2+q3+q4-angle45)) * sin(q1);
 
  Z = l1 + l2 * sin(q2) + l3 * sin(q3+q2) + l45 * sin(q2 + q3 + q4 - angle45);

  pitch = q2 + q3 + q4;
}

bool calc_D(float *a, float *b, float *c, float *D, float z1, float w1) {
  // *c1 = (w1*w1 + z1*z1 + l2*l2 - l3*l3) / (2 * w1);
  // *k = -z1 / w1;
  // *a = (*k * (*k) + 1);
  // *b = 2 * (*k) * (*c1);
  // *c = (*c1) * (*c1) - l2*l2;

  const float c1 = (l3*l3 - w1*w1 - z1*z1 - l2*l2)/(-2);
  
  *a = z1*z1 + w1*w1;
  *b =  -2*z1*c1;
  *c = c1*c1 - l2*l2*w1*w1;

  *D = *b * (*b) - 4 * (*a) * (*c);

  if (*D >= 0) { return true; }
  return false;
}

bool check_constrain(float *q) {
  bool result;
  for(int i = 0; i < joints; i++) {
    result = q[i] > CONSTRAINTS[i][0] && q[i] < CONSTRAINTS[i][1];
    if(!result) return false; 
  }
  return true;
}

void print_cartesian_pos(float* q) {
  forward_kinematics(q);
  Serial.begin(1000000);
  Serial.print(X*100);
  Serial.print(',');
  Serial.print(Y*100);
  Serial.print(',');
  Serial.print(Z*100);
  Serial.print(',');
  Serial.print(pitch*180/pi);
  Serial.print(',');
//  coords[0] = X;
//  coords[1] = Y;
//  coords[2] = Z;
//  coords[3] = pitch;
//  inverse_problem();
  
//  Serial.print(2048 - (int)(Q_d[0] / unit));
//  Serial.print(',');
//  Serial.print((int)(Q_d[1] / unit) + 226);
//  Serial.print(',');
//  Serial.print((int)(Q_d[2] / unit) +2144);
//  Serial.print(',');
//  Serial.print((int)(Q_d[3] / unit) + 1140);
//  Serial.print(',');
//  Serial.print((int)(Q_d[4] / unit) + 2048);
//  Serial.print(',');
//    
//  
//  Serial.print(2048 - (int)(Q_0[0] / unit));
//  Serial.print(',');
//  Serial.print((int)(Q_0[1] / unit) + 226);
//  Serial.print(',');
//  Serial.print((int)(Q_0[2] / unit) +2144);
//  Serial.print(',');
//  Serial.print((int)(Q_0[3] / unit) + 1140);
//  Serial.print(',');
//  Serial.println((int)(Q_0[4] / unit) + 2048);
  Serial.print(Q_d[0]);
  Serial.print(',');
  Serial.print(Q_d[1]);
  Serial.print(',');
  Serial.print(Q_d[2]);
  Serial.print(',');
  Serial.print(Q_d[3]);
  Serial.print(',');
  Serial.print(Q_d[4]);
  Serial.print(',');  
  Serial.print(Q_0[0]);
  Serial.print(',');
  Serial.print(Q_0[1]);
  Serial.print(',');
  Serial.print(Q_0[2]);
  Serial.print(',');
  Serial.print(Q_0[3]);
  Serial.print(',');
  Serial.println(Q_0[4]);
}

void get_pos() {
  Serial.begin(FEETECH_BAUD);
  Q_0[0] = (2048 - st.ReadPos(1)) * unit;
  Q_0[1] = (st.ReadPos(2) - 226) * unit;
  Q_0[2] = (st.ReadPos(3)-2144) * unit;
  Q_0[3] = (st.ReadPos(4)-1140) * unit;
  Q_0[4] = (st.ReadPos(5) - 2048) * unit;
}
