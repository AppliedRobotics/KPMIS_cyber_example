/*
  Робототехнический контроллер «Кибернетический»: дальномер с медианным фильтром

  Берётся три измерения и выбирается среднее по величине (медиана) — одиночные выбросы пропадают.
  Измерение 0 означает «нет эха» и в расчёт не попадает.
  В плоттер (115200) выводятся «сырое» значение и отфильтрованное.

  Плата в Arduino IDE: Arduino Uno. Кабель USB-C — в ВЕРХНИЙ разъём (Arduino).
  DIP-переключатель: DIP1 и DIP4 = OFF.
*/


// ---------- Модуль «Ультразвуковой дальномер» ----------
const int TRIG = 5;   // X2 модуля → D5 (XP19)
const int ECHO = 6;   // X4 модуля → D6 (XP20)

// Измерить расстояние в сантиметрах. 0 — эхо не вернулось.
int readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);              // импульс 10 мкс запускает измерение
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long us = pulseIn(ECHO, HIGH, 30000);  // длительность эха, ждём не дольше 30 мс
  return us / 58;                        // см = мкс / 58
}

// Медиана трёх чисел — среднее по величине
int median3(int a, int b, int c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

int lastGood = 0;    // последнее правильное значение

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(115200);
}

void loop() {
  int a = readDistance(); delay(40);   // между измерениями нужна пауза 30–60 мс
  int b = readDistance(); delay(40);
  int c = readDistance();

  int raw = c;
  // нули («нет эха») заменяем последним правильным значением
  if (a == 0) a = lastGood;
  if (b == 0) b = lastGood;
  if (c == 0) c = lastGood;
  int filtered = median3(a, b, c);
  if (filtered > 0) lastGood = filtered;

  Serial.print("raw:"); Serial.print(raw);
  Serial.print(" median:"); Serial.println(filtered);
  delay(40);
}
