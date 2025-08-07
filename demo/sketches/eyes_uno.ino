#include <LedControl.h>
#include <Servo.h>

// === КОНСТАНТЫ ===
// Пины подключения
const int LED_DIN_PIN = 11;
const int LED_CLK_PIN = 13; 
const int LED_CS_PIN = 10;
const int HEAD_SERVO_PIN = 9;
const int TILT_SERVO_PIN = 6;

// Настройки LED матриц
const int LED_MATRIX_COUNT = 2;
const int LED_BRIGHTNESS = 8;
const int MATRIX_SIZE = 8;

// Углы серво
const int HEAD_CENTER = 90, HEAD_LEFT = 45, HEAD_RIGHT = 135;
const int TILT_CENTER = 90, TILT_UP = 75, TILT_DOWN = 120;

// Тайминги
const int ACTION_BLINK_INTERVAL = 2000;
const int WAITING_BLINK_INTERVAL = 3000;
const int SERVO_UPDATE_INTERVAL = 15;
const int MAIN_LOOP_DELAY = 10;
const int BLINK_DURATION_ACTION = 100;
const int BLINK_DURATION_WAITING = 200;
const int SERVO_SPEED = 1;
const int HEAD_MOVE_DELAY = 1000; // Задержка 1 секунда перед движением головы

// === ТИПЫ ДАННЫХ ===
enum Mode {
  MODE_ACTION,
  MODE_WAITING
};

enum EyeDirection {
  EYE_OPEN,
  EYE_UP,
  EYE_RIGHT, 
  EYE_DOWN,
  EYE_LEFT,
  EYE_CLOSED
};

struct ServoState {
  int currentHead;
  int currentTilt;
  int targetHead;
  int targetTilt;
  unsigned long lastUpdate;
};

struct AnimationState {
  unsigned long lastBlink;
  unsigned long actionStartTime;
  bool waitingForHeadMove;
  int pendingHeadTarget;
  int pendingTiltTarget;
  EyeDirection pendingEyeDirection;
};

// === ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ===
LedControl lc(LED_DIN_PIN, LED_CLK_PIN, LED_CS_PIN, LED_MATRIX_COUNT);
Servo headServo, tiltServo;

// === СОСТОЯНИЕ ===
Mode currentMode = MODE_WAITING;
ServoState servo = {HEAD_CENTER, TILT_CENTER, HEAD_CENTER, TILT_CENTER, 0};
AnimationState animation = {0, 0, false, HEAD_CENTER, TILT_CENTER, EYE_OPEN};

// === ПАТТЕРНЫ ГЛАЗ ===
const byte bigEyePatterns[][8] = {
  // EYE_OPEN
  {B00111100, B01111110, B11111111, B11111111, B11111111, B11111111, B01111110, B00111100},
  // EYE_UP  
  {B00000000, B00000000, B00000000, B00000000, B11111111, B11111111, B01111110, B00111100},
  // EYE_RIGHT
  {B00011100, B00011110, B00001111, B00001111, B00001111, B00001111, B00011110, B00011100},
  // EYE_DOWN
  {B00111100, B01111110, B11111111, B11111111, B00000000, B00000000, B00000000, B00000000},
  // EYE_LEFT
  {B00111000, B01111000, B11110000, B11110000, B11110000, B11110000, B01111000, B00111000},
  // EYE_CLOSED
  {B00000000, B00000000, B00111100, B00111100, B00111100, B00111100, B00000000, B00000000}
};

const byte smallEyePatterns[][8] = {
  // EYE_OPEN
  {B00000000, B00011000, B00111100, B00111100, B00111100, B00111100, B00011000, B00000000},
  // EYE_CLOSED
  {B00000000, B00000000, B00000000, B00011000, B00011000, B00000000, B00000000, B00000000}
};

// === ФУНКЦИИ ОТОБРАЖЕНИЯ ===
void showPattern(int device, const byte pattern[8]) {
  for (int row = 0; row < MATRIX_SIZE; row++) {
    lc.setRow(device, row, pattern[row]);
  }
}

void showBothEyes(const byte pattern[8]) {
  showPattern(0, pattern);
  showPattern(1, pattern);
}

void displayEye(EyeDirection direction, bool isBigEye = true) {
  if (isBigEye) {
    showBothEyes(bigEyePatterns[direction]);
  } else {
    showBothEyes(smallEyePatterns[direction == EYE_CLOSED ? 1 : 0]);
  }
}

// === СЕРВО УПРАВЛЕНИЕ ===
int moveServoToward(int current, int target) {
  if (current < target) {
    current += SERVO_SPEED;
    return (current > target) ? target : current;
  } else if (current > target) {
    current -= SERVO_SPEED;
    return (current < target) ? target : current;
  }
  return current;
}

void updateServoPositions() {
  unsigned long currentTime = millis();
  
  if (currentTime - servo.lastUpdate >= SERVO_UPDATE_INTERVAL) {
    int newHead = moveServoToward(servo.currentHead, servo.targetHead);
    int newTilt = moveServoToward(servo.currentTilt, servo.targetTilt);
    
    if (newHead != servo.currentHead || newTilt != servo.currentTilt) {
      servo.currentHead = newHead;
      servo.currentTilt = newTilt;
      headServo.write(servo.currentHead);
      tiltServo.write(servo.currentTilt);
    }
    
    servo.lastUpdate = currentTime;
  }
}

void setServoTargets(int headTarget, int tiltTarget) {
  servo.targetHead = headTarget;
  servo.targetTilt = tiltTarget;
}

// === АТОМАРНЫЕ ДВИЖЕНИЯ ===
void startDirectionalLook(EyeDirection eyeDir, int headPos, int tiltPos) {
  // Переключаемся в ACTION режим - глаза расширяются
  currentMode = MODE_ACTION;
  displayEye(EYE_OPEN, true); // Показываем расширенные глаза
  
  // Запускаем таймер на 1 секунду
  animation.actionStartTime = millis();
  animation.waitingForHeadMove = true;
  animation.pendingHeadTarget = headPos;
  animation.pendingTiltTarget = tiltPos;
  animation.pendingEyeDirection = eyeDir;
}

void lookUp() {
  startDirectionalLook(EYE_UP, HEAD_CENTER, TILT_UP);
}

void lookDown() {
  startDirectionalLook(EYE_DOWN, HEAD_CENTER, TILT_DOWN);
}

void lookLeft() {
  startDirectionalLook(EYE_LEFT, HEAD_LEFT, TILT_DOWN);
}

void lookRight() {
  startDirectionalLook(EYE_RIGHT, HEAD_RIGHT, TILT_DOWN);
}

void lookCenter() {
  // Сразу переключаемся в WAITING режим
  currentMode = MODE_WAITING;
  displayEye(EYE_OPEN, false); // Маленькие глаза
  setServoTargets(HEAD_CENTER, TILT_CENTER);
  
  // Отменяем любое ожидающее движение
  animation.waitingForHeadMove = false;
}

// === АНИМАЦИИ ===
void performBlink(bool isBigEye, int duration) {
  displayEye(EYE_CLOSED, isBigEye);
  delay(duration);
  displayEye(EYE_OPEN, isBigEye);
}

void checkDelayedHeadMovement() {
  if (!animation.waitingForHeadMove) return;
  
  unsigned long currentTime = millis();
  if (currentTime - animation.actionStartTime >= HEAD_MOVE_DELAY) {
    // Время вышло - выполняем движение головы и глаз
    displayEye(animation.pendingEyeDirection, true);
    setServoTargets(animation.pendingHeadTarget, animation.pendingTiltTarget);
    animation.waitingForHeadMove = false;
  }
}

void actionAnimation() {
  unsigned long currentTime = millis();
  
  // Только моргание в ACTION режиме
  if (currentTime - animation.lastBlink >= ACTION_BLINK_INTERVAL) {
    performBlink(true, BLINK_DURATION_ACTION);
    animation.lastBlink = currentTime;
  }
}

void waitingAnimation() {
  unsigned long currentTime = millis();
  
  if (currentTime - animation.lastBlink >= WAITING_BLINK_INTERVAL) {
    performBlink(false, BLINK_DURATION_WAITING);
    animation.lastBlink = currentTime;
  }
}

// === СИСТЕМА ===
void initializeLEDMatrices() {
  for (int i = 0; i < LED_MATRIX_COUNT; i++) {
    lc.shutdown(i, false);
    lc.setIntensity(i, LED_BRIGHTNESS);
    lc.clearDisplay(i);
  }
}

void initializeServos() {
  headServo.attach(HEAD_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);
  headServo.write(HEAD_CENTER);
  tiltServo.write(TILT_CENTER);
}

// Функция switchMode удалена - режимы переключаются автоматически

void processSerialCommands() {
  if (!Serial.available()) return;
  
  String command = Serial.readString();
  command.trim();
  
  // Команды движений
  if (command == "UP") {
    lookUp();
  } else if (command == "DOWN") {
    lookDown();
  } else if (command == "LEFT") {
    lookLeft();
  } else if (command == "RIGHT") {
    lookRight();
  } else if (command == "CENTER") {
    lookCenter();
  }
}

// === ОСНОВНЫЕ ФУНКЦИИ ===
void setup() {
  Serial.begin(9600);
  
  initializeServos();
  initializeLEDMatrices();
  
  // По умолчанию: WAITING режим, маленькие глаза, голова в центре
  currentMode = MODE_WAITING;
  displayEye(EYE_OPEN, false);
  setServoTargets(HEAD_CENTER, TILT_CENTER);
}

void loop() {
  processSerialCommands();
  
  // Проверяем задержанные движения головы
  checkDelayedHeadMovement();
  
  if (currentMode == MODE_ACTION) {
    actionAnimation();
  } else {
    waitingAnimation();
  }
  
  updateServoPositions();
  delay(MAIN_LOOP_DELAY);
}