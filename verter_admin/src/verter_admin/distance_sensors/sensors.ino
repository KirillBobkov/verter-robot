const int trigPin1 = 2;   // Trig для первого датчика
const int echoPin1 = 3;   // Echo для первого датчика
const int trigPin2 = 4;   // Trig для второго датчика
const int echoPin2 = 5;   // Echo для второго датчика
const int trigPin3 = 6;   // Trig для третьего датчика
const int echoPin3 = 7;   // Echo для третьего датчика
const int trigPin4 = 8;   // Trig для четвёртого датчика
const int echoPin4 = 9;   // Echo для четвёртого датчика
const int trigPin5 = 10;  // Trig для пятого датчика
const int echoPin5 = 11;  // Echo для пятого датчика
const int trigPin6 = 12;  // Trig для шестого датчика
const int echoPin6 = 13;  // Echo для шестого датчика
const int trigPin7 = 14;  // Trig для седьмого датчика (A0)
const int echoPin7 = 15;  // Echo для седьмого датчика (A1)

void setup() {
  Serial.begin(9600);  // Serial Monitor
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin5, INPUT);
  pinMode(trigPin6, OUTPUT);
  pinMode(echoPin6, INPUT);
  pinMode(trigPin7, OUTPUT);
  pinMode(echoPin7, INPUT);
}

void loop() {
  // Измерение для первого датчика
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(50);  // Импульс 50 мкс
  digitalWrite(trigPin1, LOW);

  long duration1 = pulseIn(echoPin1, HIGH, 50000);  // Таймаут 50 мс
  float distance1 = (duration1 * 0.034) / 2;

  // Измерение для второго датчика
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(50);  // Импульс 50 мкс
  digitalWrite(trigPin2, LOW);

  long duration2 = pulseIn(echoPin2, HIGH, 50000);  // Таймаут 50 мс
  float distance2 = (duration2 * 0.034) / 2;

  // Измерение для третьего датчика
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(50);  // Импульс 50 мкс
  digitalWrite(trigPin3, LOW);

  long duration3 = pulseIn(echoPin3, HIGH, 50000);  // Таймаут 50 мс
  float distance3 = (duration3 * 0.034) / 2;

  // Измерение для четвёртого датчика
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(50);  // Импульс 50 мкс
  digitalWrite(trigPin4, LOW);

  long duration4 = pulseIn(echoPin4, HIGH, 50000);  // Таймаут 50 мс
  float distance4 = (duration4 * 0.034) / 2;

  // Измерение для пятого датчика
  digitalWrite(trigPin5, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin5, HIGH);
  delayMicroseconds(50);  // Импульс 50 мкс
  digitalWrite(trigPin5, LOW);

  long duration5 = pulseIn(echoPin5, HIGH, 50000);  // Таймаут 50 мс
  float distance5 = (duration5 * 0.034) / 2;

  // Измерение для шестого датчика
  digitalWrite(trigPin6, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin6, HIGH);
  delayMicroseconds(50);  // Импульс 50 мкс
  digitalWrite(trigPin6, LOW);

  long duration6 = pulseIn(echoPin6, HIGH, 50000);  // Таймаут 50 мс
  float distance6 = (duration6 * 0.034) / 2;

  // Измерение для седьмого датчика
  digitalWrite(trigPin7, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin7, HIGH);
  delayMicroseconds(50);  // Импульс 50 мкс
  digitalWrite(trigPin7, LOW);

  long duration7 = pulseIn(echoPin7, HIGH, 50000);  // Таймаут 50 мс
  float distance7 = (duration7 * 0.034) / 2;

  // Вывод результатов в формате DISTANCE:10:23:24:23:23:22:22
  Serial.print("DISTANCE:");
  
  // Датчик 1
  if (duration1 == 0 || distance1 > 450 || distance1 < 2) {
    Serial.print("999");  // Ошибка или вне диапазона
  } else {
    Serial.print((int)distance1);
  }
  Serial.print(":");
  
  // Датчик 2
  if (duration2 == 0 || distance2 > 450 || distance2 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance2);
  }
  Serial.print(":");
  
  // Датчик 3
  if (duration3 == 0 || distance3 > 450 || distance3 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance3);
  }
  Serial.print(":");
  
  // Датчик 4
  if (duration4 == 0 || distance4 > 450 || distance4 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance4);
  }
  Serial.print(":");
  
  // Датчик 5
  if (duration5 == 0 || distance5 > 450 || distance5 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance5);
  }
  Serial.print(":");
  
  // Датчик 6
  if (duration6 == 0 || distance6 > 450 || distance6 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance6);
  }
  Serial.print(":");
  
  // Датчик 7
  if (duration7 == 0 || distance7 > 450 || distance7 < 2) {
    Serial.println("999");
  } else {
    Serial.println((int)distance7);
  }

  delay(300);  // Пауза 300 мс между циклами для минимизации помех
}