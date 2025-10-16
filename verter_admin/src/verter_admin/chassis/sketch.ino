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
const int TARGET_SPEED = 77;        // 30% от 255 (255 * 0.3 ≈ 77)
const int RAMP_STEPS = 15;           // Количество шагов для плавного разгона/торможения
const int RAMP_DELAY = 30;           // Задержка между шагами (мс)

// === Переменные для плавного движения шасси ===
int currentSpeed1 = 0;               // Текущая скорость левого мотора
int currentSpeed2 = 0;               // Текущая скорость правого мотора
int targetSpeed1 = 0;                // Целевая скорость левого мотора
int targetSpeed2 = 0;                // Целевая скорость правого мотора
unsigned long lastMotorUpdate = 0;   // Время последнего обновления моторов

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

// === Плавное движение шасси ===
void updateMotorSpeeds() {
  unsigned long currentTime = millis();
  
  // Обновляем скорости каждые RAMP_DELAY мс для плавности
  if (currentTime - lastMotorUpdate > RAMP_DELAY) {
    bool changed = false;
    
    // Плавно изменяем скорость левого мотора
    if (currentSpeed1 < targetSpeed1) {
      int step = (targetSpeed1 - currentSpeed1) / RAMP_STEPS + 1;
      currentSpeed1 += step;
      if (currentSpeed1 > targetSpeed1) currentSpeed1 = targetSpeed1;
      changed = true;
    } else if (currentSpeed1 > targetSpeed1) {
      int step = (currentSpeed1 - targetSpeed1) / RAMP_STEPS + 1;
      currentSpeed1 -= step;
      if (currentSpeed1 < targetSpeed1) currentSpeed1 = targetSpeed1;
      changed = true;
    }
    
    // Плавно изменяем скорость правого мотора
    if (currentSpeed2 < targetSpeed2) {
      int step = (targetSpeed2 - currentSpeed2) / RAMP_STEPS + 1;
      currentSpeed2 += step;
      if (currentSpeed2 > targetSpeed2) currentSpeed2 = targetSpeed2;
      changed = true;
    } else if (currentSpeed2 > targetSpeed2) {
      int step = (currentSpeed2 - targetSpeed2) / RAMP_STEPS + 1;
      currentSpeed2 -= step;
      if (currentSpeed2 < targetSpeed2) currentSpeed2 = targetSpeed2;
      changed = true;
    }
    
    // Применяем изменения только если были изменения
    if (changed) {
      motor1.setSpeed(currentSpeed1);
      motor2.setSpeed(currentSpeed2);
    }
    
    lastMotorUpdate = currentTime;
  }
}

// Установка целевых скоростей моторов
void setMotorTargets(int speed1, int speed2) {
  targetSpeed1 = speed1;
  targetSpeed2 = speed2;
}

// Остановка движения
void stopChassis() {
  targetSpeed1 = 0;
  targetSpeed2 = 0;
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
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  currentSpeed1 = 0;
  currentSpeed2 = 0;
  targetSpeed1 = 0;
  targetSpeed2 = 0;

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
      Serial.println("Head turned RIGHT");
    } else if (command == "HEAD:LEFT") {
      // Поворот головы влево
      setServoTargets(HEAD_LEFT, TILT_CENTER);
      currentEyePattern = bigEyeLeft;
      showBoth(currentEyePattern);
      Serial.println("Head turned LEFT");
    } else if (command == "HEAD:CENTER") {
      // Возврат головы в центр
      setServoTargets(HEAD_CENTER, TILT_CENTER);
      currentEyePattern = bigEyeOpen;
      showBoth(currentEyePattern);
      Serial.println("Head centered");
    } else if (command == "HEAD:UP") {
      // Наклон головы вверх
      setServoTargets(HEAD_CENTER, TILT_UP);
      currentEyePattern = bigEyeDown;  // Паттерн bigEyeDown показывает глаза вверх
      showBoth(currentEyePattern);
      Serial.println("Head tilted UP");
    } else if (command == "HEAD:DOWN") {
      // Наклон головы вниз
      setServoTargets(HEAD_CENTER, TILT_DOWN);
      currentEyePattern = bigEyeUp;    // Паттерн bigEyeUp показывает глаза вниз
      showBoth(currentEyePattern);
      Serial.println("Head tilted DOWN");
    } else if (command == "CHASSIS:FRONT") {
      // Движение вперед
      setMotorTargets(TARGET_SPEED, -TARGET_SPEED);
      Serial.println("Chassis moving FRONT");
    } else if (command == "CHASSIS:BACK") {
      // Движение назад
      setMotorTargets(-TARGET_SPEED, TARGET_SPEED);
      Serial.println("Chassis moving BACK");
    } else if (command == "CHASSIS:LEFT") {
      // Поворот влево
      setMotorTargets(0, -TARGET_SPEED);
      Serial.println("Chassis turning LEFT");
    } else if (command == "CHASSIS:RIGHT") {
      // Поворот вправо
      setMotorTargets(TARGET_SPEED, 0);
      Serial.println("Chassis turning RIGHT");
    } else if (command == "CHASSIS:STOP") {
      // Остановка
      stopChassis();
      Serial.println("Chassis STOPPED");
    } else {
      // Неизвестная команда - логируем для отладки
      Serial.println("Unknown command: " + command);
    }
  }

  // Выполнение анимации в зависимости от режима
  if (currentMode == "ACTION") {
    actionAnimation();
  }

  // Обновление плавного движения серво
  updateServoPositions();
  
  // Обновление плавного движения шасси
  updateMotorSpeeds();

  delay(10);  // Уменьшенная задержка для более плавного движения
}