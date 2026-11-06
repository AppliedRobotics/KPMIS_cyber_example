constexpr uint8_t PIN_TRIG = 10;  // Пины ультразвукового датчика
constexpr uint8_t PIN_ECHO = 9;
long duration, cm;
 
void setup() {
  Serial.begin(57600);
  pinMode(PIN_TRIG, OUTPUT);
 }

void loop() {
  // Сначала генерируем короткий импульс длительностью 2-5 микросекунд.
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  // Выставляем высокий уровень сигнала. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  //  Время задержки акустического сигнала на эхолокаторе.
  duration = pulseIn(PIN_ECHO, HIGH);
  // Преобразование в расстояние
  cm = (duration / 2) / 29.1;
  Serial.print("Расстояние до объекта: ");
  Serial.print(cm);
  Serial.println(" см.");
  delay(250);
}
