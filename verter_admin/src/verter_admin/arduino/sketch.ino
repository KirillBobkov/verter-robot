#include <LedControl.h>
#include <Servo.h>

// Подключение: DIN = 12, CLK = 13, CS = 14, 2 матрицы
LedControl lc = LedControl(12, 13, 14, 2);

// Серво приводы для поворота и наклона головы
Servo headServo;        // Поворот влево-вправо (yaw)
Servo tiltServo;        // Наклон вверх-вниз (pitch)
const int HEAD_SERVO_PIN = 10;
const int TILT_SERVO_PIN = 11;

// Позиции серво для поворота (yaw)
const int HEAD_CENTER = 90;
const int HEAD_LEFT =135;
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

// === Инициализация ===
void setup() {
  Serial.begin(9600);

  // Инициализация серво
  headServo.attach(HEAD_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  
  // Устанавливаем начальные позиции
  currentHeadPos = HEAD_CENTER;
  currentTiltPos = TILT_CENTER;
  targetHeadPos = HEAD_CENTER;
  targetTiltPos = TILT_CENTER;
  headServo.write(HEAD_CENTER);
  tiltServo.write(TILT_CENTER);

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
  // Проверка команд из Serial
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();

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
    }
  }

  // Выполнение анимации в зависимости от режима
  if (currentMode == "ACTION") {
    actionAnimation();
  }

  // Обновление плавного движения серво
  updateServoPositions();

  delay(10);  // Уменьшенная задержка для более плавного движения
}