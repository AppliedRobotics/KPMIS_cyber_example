uint8_t contactRight = 5;
uint8_t contactLeft = 8;
uint8_t line1 = 3;
uint8_t line2 = 6;
uint8_t line3 = 4;
uint8_t line4 = 7;

void setup() {
  Serial.begin(57600);
  pinMode(contactLeft, INPUT_PULLUP);
  pinMode(contactRight, INPUT_PULLUP);
}

void loop() {
  Serial.print("Lines: "); // датчики слева направо
  Serial.print(digitalRead(line1)); Serial.print(" ");
  Serial.print(digitalRead(line2)); Serial.print(" ");
  Serial.print(digitalRead(line3)); Serial.print(" ");
  Serial.println(digitalRead(line4));
  Serial.print("Contacts: ");
  Serial.print(digitalRead(contactLeft)); Serial.print(" ");
  Serial.println(digitalRead(contactRight));
  delay(1000);
}
