/*
  Робозахват — версия с «команда → выполнение».  
  Алгоритм тот же, но вместо большой state-машины в loop() используются
  две последовательные процедуры: homeSequence() и grabSequence().
  loop() лишь ждёт ввода по Serial и вызывает нужную процедуру.
*/

// === Аппаратные пины ===
const int in1       = 5;   // L298N IN1
const int in2       = 6;   // L298N IN2
const int buttonPin = 7;   // Концевик (LOW при нажатии)
const int encoderPin = 2;  // Однопроводной энкодер на INT0

// === Глобальное состояние ===
volatile long handEncoderCount = 0; // Счётчик импульсов энкодера
bool handAtHome = false;            // Отфлагирована ли нулевая точка

// === Настройки алгоритма ===
const long  maxMoveTime         = 5000;  // мс, лимит закрытия
const float speedThreshold      = 1.0;   // имп/с, почти ноль
const float suddenDropThreshold = 40.0;  // имп/с, резкое падение
const long  speedCheckInterval  = 200;   // мс, период опроса скорости
const long  minImpulsesForGrab  = 3000;  // импульсы до разрешения остановки

// === Буфер/флаг больше не нужны — читаем строку одной командой
// (оставленные строки закомментированы, чтобы не мешали компиляции)
// String handInputBuffer;
// bool handCommandComplete;

// === Примитивы управления мотором ===
inline void stopHandMotor() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

inline void startBackwardHand() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

inline void startForwardToHomeHand() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

// === Короткие утилиты ===
void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    delay(1);
  }
}

// === Процедура калибровки ===
void handHome() {
  Serial.println("Начат поиск нулевой точки");

  handAtHome = false;
  handEncoderCount = 0;
  startForwardToHomeHand();

  // Крутимся пока не сработает концевик
  while (digitalRead(buttonPin) != LOW) {
    delay(5);
  }

  waitMs(50); // дебаунс
  stopHandMotor();
  handEncoderCount = 0;
  handAtHome = true;
  Serial.println("Нулевая точка найдена");
}

// === Процедура захвата ===
void handGrab() {
  if (!handAtHome) {
    Serial.println("Ошибка: сначала выполните home");
    return;
  }

  Serial.println("Захват начинает закрываться");
  startBackwardHand();

  unsigned long moveStartTime   = millis();
  unsigned long lastSpeedCheck  = millis();
  float          lastImpPerSec  = 0;
  long           lastEncoderVal = handEncoderCount;

  while (true) {
    unsigned long now = millis();

    // Периодическая проверка скорости
    if (now - lastSpeedCheck >= speedCheckInterval) {
      float dt       = (now - lastSpeedCheck) / 1000.0;
      long  impulses = handEncoderCount - lastEncoderVal;
      float impPerSec = impulses / dt;
      float deltaSpeed = lastImpPerSec - impPerSec;

      lastEncoderVal  = handEncoderCount;
      lastSpeedCheck  = now;
      lastImpPerSec   = impPerSec;

      bool slowEnough   = (impPerSec < speedThreshold);
      bool suddenDrop   = (deltaSpeed > suddenDropThreshold);
      bool passedMinImp = (handEncoderCount > minImpulsesForGrab);

      if ((slowEnough || suddenDrop) && passedMinImp) {
        Serial.println(slowEnough ? "Объект захвачен — скорость упала" :
                                     "Объект захвачен — резкое падение скорости");
        break;
      }
    }

    // Ограничение по времени
    if (now - moveStartTime > maxMoveTime) {
      Serial.println("Достигнут максимальный предел движения");
      break;
    }

    delay(10);
  }

  stopHandMotor();
  Serial.print("Пройдено импульсов: ");
  Serial.println(handEncoderCount);
}

// === Прерывание энкодера ===
void handEncoderISR() {
  // Считаем импульсы либо после калибровки, либо во время захвата (мотор назад)
  if (handAtHome || digitalRead(in2) == HIGH) {
    handEncoderCount++;
  }
}

// === Стандартные Arduino функции ===
void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(encoderPin, INPUT);

  stopHandMotor();

  attachInterrupt(digitalPinToInterrupt(encoderPin), handEncoderISR, CHANGE);

  Serial.begin(9600);
  Serial.println("=== Робозахват готов ===");
  Serial.println("Команды: home, grab");
}

// loop(): опрос буфера + отработка, без блокирующего wait Serial
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.equalsIgnoreCase("home")) {
      handHome();
    } else if (cmd.equalsIgnoreCase("grab")) {
      handGrab();
    } else {
      Serial.println("Неизвестная команда");
    }
  }
} 