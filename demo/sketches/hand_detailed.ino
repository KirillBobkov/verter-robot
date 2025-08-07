/*
  ------ Робозахват с подробными комментариями ------
  Данная версия содержит пометки почти к каждой строке, объясняющие
  назначение переменных, функций и логики работы.
*/

// === Пины ===
const int in1 = 5;        // IN1 драйвера L298N — задаёт направление «вперёд»
const int in2 = 6;        // IN2 драйвера L298N — задаёт направление «назад»
const int buttonPin = 7;  // Концевик (микрик), активен при нажатии (LOW)

// === Энкодер (однопроводной) ===
const int encoderPin = 2;   // Линия энкодера; в Arduino UNO совместима c прерыванием 0
volatile long encoderCount = 0; // Глобальный счётчик импульсов; volatile, т.к. меняется в прерывании

// === Переменные состояния ===
bool atHome = false; // Флаг: нашли ли «нулевой» угол захвата

unsigned long moveStartTime = 0; // Время начала движения «grab» (мс)
unsigned long lastSpeedCheck = 0; // Последняя отметка проверки скорости (мс)

// === Настройки алгор. ===
const long  maxMoveTime          = 5000;  // Максимальное время закрытия (мс)
const float speedThreshold       = 1.0;   // Скорость (имп/с) ниже которой считаем, что мотор остановился
const float suddenDropThreshold  = 40.0;  // Порог резкого падения скорости (имп/с)
const long  speedCheckInterval   = 200;   // Шаг проверки скорости (мс)
const long  minImpulsesForGrab   = 3000;  // Минимум импульсов, прежде чем разрешим остановку

float lastImpPerSec = 0; // Последнее измерение скорости (имп/с)

// === Вспомогательные функции управления мотором ===
void stopMotor() {
  digitalWrite(in1, LOW);  // Обнуляем оба вывода — мотор остановлен (режим тормоза OFF)
  digitalWrite(in2, LOW);
}

void startBackward() {
  digitalWrite(in1, LOW);  // LOW / HIGH даёт вращение «назад»
  digitalWrite(in2, HIGH);
}

void startForwardToHome() {
  digitalWrite(in1, HIGH); // HIGH / LOW даёт вращение «вперёд» к концевику
  digitalWrite(in2, LOW);
}

// === Инициализация ===
void setup() {
  pinMode(in1, OUTPUT);         // Настраиваем пины драйвера как выходы
  pinMode(in2, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Концевик с подтяжкой к VCC (нажатие даёт LOW)
  pinMode(encoderPin, INPUT);   // Энкодер — вход (можно оставить без подтяжек)

  stopMotor(); // На старте мотор выключен

  // Назначаем обработчик прерывания CHANGE на encoderPin
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, CHANGE);

  Serial.begin(9600);           // Включаем UART на 9600 бод
  Serial.println("=== Робозахват готов ===");
  Serial.println("Команды: home, grab");
}

// === Главный цикл ===
void loop() {
  // 1. Читаем команды из Serial -----------------------
  if (Serial.available()) {              // Есть ли символы в буфере UART?
    String cmd = Serial.readStringUntil('\n'); // Считываем до перевода строки
    cmd.trim();                         // Убираем пробелы и CR

    if (cmd.equalsIgnoreCase("home")) {       // Команда калибровки
      stopMotor();                      // На всякий случай остановка
      atHome = false;                   // Сброс флага «в нуле»
      encoderCount = 0;                 // Обнуляем счётчик импульсов
      startForwardToHome();             // Запускаем вращение к концевику
      Serial.println("Начат поиск нулевой точки");

    } else if (cmd.equalsIgnoreCase("grab")) { // Команда захвата
      if (!atHome) {                   // Нельзя закрывать, пока не откалиброваны
        Serial.println("Ошибка: сначала найдите нулевую точку ('home')");
        return;                        // Рано — выходим из loop()
      }
      stopMotor();                     // Сбросим всё перед стартом
      moveStartTime   = millis();      // Запоминаем время начала
      lastSpeedCheck  = millis();      // И время последней проверки скорости
      startBackward();                 // Запускаем вращение «назад»
      Serial.println("Захват начинает закрываться");
    }
  }

  // 2. Поиск нулевой точки --------------------------------
  if (!atHome && digitalRead(buttonPin) == LOW) { // Нажат концевик?
    delay(50);                                    // Дебаунс 50 мс
    if (digitalRead(buttonPin) == LOW) {          // Проверяем ещё раз
      stopMotor();                                // Останавливаемся
      encoderCount = 0;                           // Обнуляем импульсы
      atHome = true;                              // Флаг «в нуле»
      Serial.println("Нулевая точка найдена");
    }
  }

  // 3. Закрытие захвата и контроль скорости ---------------
  // Проверяем, что мотор действительно крутится «назад»
  if (digitalRead(in2) == HIGH && digitalRead(in1) == LOW) {
    unsigned long now = millis(); // Текущее время

    // --- Периодический расчёт скорости ---
    if (now - lastSpeedCheck >= speedCheckInterval) {
      float timeDiff = (now - lastSpeedCheck) / 1000.0; // Интервал в секундах
      static long lastEncoderVal = 0;                   // Локальная стат. переменная
      float impulses   = encoderCount - lastEncoderVal; // Сколько импульсов за интервал
      float impPerSec  = impulses / timeDiff;           // Скорость (имп/с)
      float deltaSpeed = lastImpPerSec - impPerSec;     // Насколько скорость просела

      lastEncoderVal  = encoderCount;  // Запоминаем для след. интервала
      lastSpeedCheck  = now;           // Обновляем время послед. проверки

      // --- Критерии остановки ---
      if (impPerSec < speedThreshold && encoderCount > minImpulsesForGrab) {
        stopMotor();
        Serial.println("Объект захвачен — движение остановлено");
        Serial.print("Пройдено импульсов: ");
        Serial.println(encoderCount);
      } else if (deltaSpeed > suddenDropThreshold && encoderCount > minImpulsesForGrab) {
        stopMotor();
        Serial.println("Объект захвачен — резкое падение скорости");
        Serial.print("Пройдено импульсов: ");
        Serial.println(encoderCount);
      }

      lastImpPerSec = impPerSec; // Сохраняем текущую скорость
    }

    // --- Страховка по времени ---
    if (now - moveStartTime > maxMoveTime) { // Превышено макс. время?
      stopMotor();
      Serial.println("Достигнут максимальный предел движения");
      Serial.print("Пройдено импульсов: ");
      Serial.println(encoderCount);
    }
  }

  delay(10); // Маленькая задержка — снижает нагрузку на CPU
}

// === Прерывание от энкодера ===
void encoderISR() {
  // Считаем импульсы только если:
  //   • мы уже откалиброваны (atHome == true)
  //   • ИЛИ мотор сейчас крутится «назад» (digitalRead(in2) == HIGH)
  if (atHome || digitalRead(in2) == HIGH) {
    encoderCount++; // Инкремент глобального счётчика
  }
} 