#include <LedControl.h>
#include <Servo.h>
#include <CytronMotorDriver.h>

// Подключение: DIN = 12, CLK = 13, CS = 14, 2 матрицы
LedControl lc = LedControl(12, 13, 14, 2);

// Подключение Cytron MDD10A: два канала, режим Sign-Magnitude
CytronMD motor1(PWM_DIR, 4, 5);  // Канал 1: PWM = 4, DIR = 5 (левый мотор)
CytronMD motor2(PWM_DIR, 6, 7);  // Канал 2: PWM = 6, DIR = 7 (правый мотор, инвертирован)

// Серво приводы для поворота и наклона головы
Servo headServo;        // Поворот влево-вправо (yaw)
Servo tiltServo;        // Наклон вверх-вниз (pitch)
const int HEAD_SERVO_PIN = 10;
const int TILT_SERVO_PIN = 11;

// Позиции серво для поворота (yaw)
const int HEAD_CENTER = 90;
const int HEAD_LEFT = 135;
const int HEAD_RIGHT = 45;

// Позиции серво для наклона (pitch)
const int TILT_CENTER = 90;
const int TILT_UP = 75;
const int TILT_DOWN = 120;

// === Состояния ===
String currentMode = "ACTION";  // Текущий режим
unsigned long lastBlink = 0;     // Время последнего моргания
byte* currentEyePattern; // Текущий паттерн глаз

// === Плавное движение серво ===
int currentHeadPos = HEAD_CENTER;    // Текущая позиция поворота
int currentTiltPos = TILT_CENTER;    // Текущая позиция наклона
int targetHeadPos = HEAD_CENTER;     // Целевая позиция поворота  
int targetTiltPos = TILT_CENTER;     // Целевая позиция наклона
unsigned long lastServoUpdate = 0;   // Время последнего обновления серво
const int SERVO_SPEED = 1;           // Скорость движения (градусы за шаг)

// === Параметры шасси ===
const float DEFAULT_SPEED = 0.314;   // Скорость по умолчанию в м/с
const float SPEED_TO_PWM = 312.0;    // Коэффициент преобразования м/с в значения PWM (скорректирован по фактической скорости)
const int MAX_PWM = 255;             // Максимальное значение PWM

// === Переменные для движения шасси ===
int motorSpeed1 = 0;                 // Скорость левого мотора
int motorSpeed2 = 0;                 // Скорость правого мотора

// === Большие круглые глаза (ACTION) ===
byte bigEyeOpen[8] = {
  B00111100,
  B01111110,
  B11111111,
  B11111111,
  B11111111,
  B11111111,
  B01111110,
  B00111100
};

byte bigEyeClosed[8] = {
  B00000000,
  B00000000,
  B00111100,
  B00111100,
  B00111100,
  B00111100,
  B00000000,
  B00000000
};

byte bigEyeUp[8] = {
  B00111100,
  B01111110,
  B11111111,
  B11111111,
  B00000000,
  B00000000,
  B00000000,
  B00000000
};

byte bigEyeDown[8] = {
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B11111111,
  B11111111,
  B01111110,
  B00111100
};

byte bigEyeLeft[8] = {
  B00111000,
  B01111000,
  B11110000,
  B11110000,
  B11110000,
  B11110000,
  B01111000,
  B00111000
};

byte bigEyeRight[8] = {
  B00011100,
  B00011110,
  B00001111,
  B00001111,
  B00001111,
  B00001111,
  B00011110,
  B00011100
};

// === Плавное движение серво ===
void updateServoPositions() {
  unsigned long currentTime = millis();
  
  // Обновляем позиции каждые 15мс для плавности
  if (currentTime - lastServoUpdate > 15) {
    bool moved = false;
    
    // Плавно двигаем поворот головы
    if (currentHeadPos < targetHeadPos) {
      currentHeadPos += SERVO_SPEED;
      if (currentHeadPos > targetHeadPos) currentHeadPos = targetHeadPos;
      moved = true;
    } else if (currentHeadPos > targetHeadPos) {
      currentHeadPos -= SERVO_SPEED;
      if (currentHeadPos < targetHeadPos) currentHeadPos = targetHeadPos;
      moved = true;
    }
    
    // Плавно двигаем наклон головы
    if (currentTiltPos < targetTiltPos) {
      currentTiltPos += SERVO_SPEED;
      if (currentTiltPos > targetTiltPos) currentTiltPos = targetTiltPos;
      moved = true;
    } else if (currentTiltPos > targetTiltPos) {
      currentTiltPos -= SERVO_SPEED;
      if (currentTiltPos < targetTiltPos) currentTiltPos = targetTiltPos;
      moved = true;
    }
    
    // Применяем изменения только если были движения
    if (moved) {
      headServo.write(currentHeadPos);
      tiltServo.write(currentTiltPos);
    }
    
    lastServoUpdate = currentTime;
  }
}

// Установка целевых позиций серво
void setServoTargets(int headTarget, int tiltTarget) {
  targetHeadPos = headTarget;
  targetTiltPos = tiltTarget;
}

// === Применение скоростей к моторам ===
void updateMotorSpeeds() {
  // Мгновенно применяем скорости к моторам
  motor1.setSpeed(motorSpeed1);
  motor2.setSpeed(motorSpeed2);
}

// Извлечение значения скорости из команды (например, CHASSIS:FRONT:0.314)
float extractSpeed(String command) {
  float speed = DEFAULT_SPEED;
  if (command.indexOf(':') != command.lastIndexOf(':')) {
    speed = command.substring(command.lastIndexOf(':') + 1).toFloat();
  }
  return speed;
}

// Преобразование скорости из м/с в значение PWM
int speedToPWM(float speedMS) {
  int pwmValue = (int)(speedMS * SPEED_TO_PWM);
  // Ограничиваем значение максимальным PWM
  if (pwmValue > MAX_PWM) pwmValue = MAX_PWM;
  return pwmValue;
}

// Установка скоростей моторов в м/с
void setMotorTargetsMS(float speedMS1, float speedMS2) {
  motorSpeed1 = speedToPWM(abs(speedMS1)) * (speedMS1 >= 0 ? 1 : -1);
  motorSpeed2 = speedToPWM(abs(speedMS2)) * (speedMS2 >= 0 ? 1 : -1);
  updateMotorSpeeds(); // Сразу применяем изменения
}

// Установка скоростей моторов (для совместимости)
void setMotorTargets(int speed1, int speed2) {
  motorSpeed1 = speed1;
  motorSpeed2 = speed2;
  updateMotorSpeeds(); // Сразу применяем изменения
}

// Остановка движения
void stopChassis() {
  motorSpeed1 = 0;
  motorSpeed2 = 0;
  updateMotorSpeeds(); // Сразу применяем изменения
}

// === Инициализация ===
void setup() {
  Serial.begin(9600);

  // Инициализация серво
  headServo.attach(HEAD_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  
  // Устанавливаем начальные позиции серво
  currentHeadPos = HEAD_CENTER;
  currentTiltPos = TILT_CENTER;
  targetHeadPos = HEAD_CENTER;
  targetTiltPos = TILT_CENTER;
  headServo.write(HEAD_CENTER);
  tiltServo.write(TILT_CENTER);
  
  // Инициализация моторов шасси
  motorSpeed1 = 0;
  motorSpeed2 = 0;
  motor1.setSpeed(0);
  motor2.setSpeed(0);

  for (int i = 0; i < 2; i++) {
    lc.shutdown(i, false);     // Включить матрицу
    lc.setIntensity(i, 15);    // Яркость увеличена до максимума (0–15)
    lc.clearDisplay(i);        // Очистить
  }

  // Начальное состояние
  currentEyePattern = bigEyeOpen;
  showBoth(currentEyePattern);
}

// Отображение одного глаза на указанной матрице
void showFace(int device, byte *face) {
  for (int row = 0; row < 8; row++) {
    lc.setRow(device, row, face[row]);
  }
}

// Отображение на обеих матрицах
void showBoth(byte *face) {
  showFace(0, face);  // Левый
  showFace(1, face);  // Правый
}

// Анимация ACTION - активные большие глаза (только моргание)
void actionAnimation() {
  unsigned long currentTime = millis();

  // Моргание каждые 2 секунды
  if (currentTime - lastBlink > 2000) {
    showBoth(bigEyeClosed);
    delay(100);
    showBoth(currentEyePattern); // Возвращаем текущий паттерн вместо bigEyeOpen
    lastBlink = currentTime;
  }
}


void loop() {
  // Проверка команд из Serial - обрабатываем ВСЕ доступные команды
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Пропускаем пустые команды
    if (command.length() == 0) continue;

    if (command == "ACTION") {
      if (currentMode != "ACTION") {
        currentMode = "ACTION";
        currentEyePattern = bigEyeOpen;
        showBoth(currentEyePattern);
        setServoTargets(HEAD_CENTER, TILT_CENTER);
        lastBlink = millis();
      }
    } else if (command == "HEAD:RIGHT") {
      // Поворот головы вправо
      setServoTargets(HEAD_RIGHT, TILT_CENTER);
      currentEyePattern = bigEyeRight;
      showBoth(currentEyePattern);
    } else if (command == "HEAD:LEFT") {
      // Поворот головы влево
      setServoTargets(HEAD_LEFT, TILT_CENTER);
      currentEyePattern = bigEyeLeft;
      showBoth(currentEyePattern);
    } else if (command == "HEAD:CENTER") {
      // Возврат головы в центр
      setServoTargets(HEAD_CENTER, TILT_CENTER);
      currentEyePattern = bigEyeOpen;
      showBoth(currentEyePattern);
    } else if (command == "HEAD:UP") {
      // Наклон головы вверх
      setServoTargets(HEAD_CENTER, TILT_UP);
      currentEyePattern = bigEyeDown;  // Паттерн bigEyeDown показывает глаза вверх
      showBoth(currentEyePattern);
    } else if (command == "HEAD:DOWN") {
      // Наклон головы вниз
      setServoTargets(HEAD_CENTER, TILT_DOWN);
      currentEyePattern = bigEyeUp;    // Паттерн bigEyeUp показывает глаза вниз
      showBoth(currentEyePattern);
    } else if (command.startsWith("CHASSIS:FRONT")) {
      // Движение вперед
      float speed = extractSpeed(command);
      setMotorTargetsMS(speed, -speed);
    } else if (command.startsWith("CHASSIS:BACK")) {
      // Движение назад
      float speed = extractSpeed(command);
      setMotorTargetsMS(-speed, speed);
    } else if (command.startsWith("CHASSIS:LEFT")) {
      // Поворот влево
      float speed = extractSpeed(command);
      setMotorTargetsMS(0, -speed);
    } else if (command.startsWith("CHASSIS:RIGHT")) {
      // Поворот вправо
      float speed = extractSpeed(command);
      setMotorTargetsMS(speed, 0);
    } else if (command == "CHASSIS:STOP") {
      // Остановка
      stopChassis();
    }
  }

  // Выполнение анимации в зависимости от режима
  if (currentMode == "ACTION") {
    actionAnimation();
  }

  // Обновление плавного движения серво
  updateServoPositions();
  
  // Не нужно вызывать updateMotorSpeeds() в цикле, 
  // так как она вызывается при изменении скорости

  delay(10);  // Уменьшенная задержка для более плавного движения
}