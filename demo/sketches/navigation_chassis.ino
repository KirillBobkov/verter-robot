/*
 * Arduino скетч для работы с ROS2 через узел arduino_proxy_node
 * Расширенная версия: команды движения и управление глазами
 */

#include <LedControl.h>
#include <Servo.h>

// Пины для управления левым мотором
const int ENA = 9;   // ШИМ пин скорости левого мотора
const int IN1 = 2;   // Направление 1 для левого мотора
const int IN2 = 3;   // Направление 2 для левого мотора

// Пины для управления правым мотором
const int ENB = 6;   // ШИМ пин скорости правого мотора
const int IN3 = 5;   // Направление 1 для правого мотора
const int IN4 = 4;   // Направление 2 для правого мотора

// === ПИНЫ ДЛЯ ГЛАЗ ===
const int LED_DIN_PIN = 51;  // MOSI pin на Arduino Mega
const int LED_CLK_PIN = 52;  // SCK pin на Arduino Mega
const int LED_CS_PIN = 53;   // SS pin на Arduino Mega
const int HEAD_SERVO_PIN = 11;  // Arduino Mega пин
const int TILT_SERVO_PIN = 10;  // Arduino Mega пин

// Скорость последовательного порта
const long BAUD_RATE = 9600;

// Максимальная ШИМ скорость при движении (0-255)
const int MAX_PWM_FORWARD_SPEED = 180;
const int MAX_PWM_ROTATE_SPEED = 200;

// Параметры плавного старта
const int ACCELERATION_STEP = 5;    // Шаг увеличения скорости
const int ACCELERATION_DELAY = 30;  // Задержка между шагами (мс)
const int DECELERATION_STEP = 10;   // Шаг уменьшения скорости при остановке
const int DECELERATION_DELAY = 20;  // Задержка между шагами при остановке (мс)

// === КОНСТАНТЫ ДЛЯ ГЛАЗ ===
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
const int HEAD_MOVE_DELAY = 100; // Задержка 1 секунда перед движением головы

// === ТИПЫ ДАННЫХ ДЛЯ ГЛАЗ ===
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

// Function prototypes
void displayEye(EyeDirection direction, bool isBigEye = true);
void initializeServos();
void initializeLEDMatrices();
void setServoTargets(int headTarget, int tiltTarget);
void stopMotors();
void showBothEyes(const byte pattern[8]);
void showPattern(int device, const byte pattern[8]);
void readSerialCommands();
void processCommand(String command);
void handleMovementCommand(String params);
void handleEyesCommand(String params);
void moveForward(int speed);
void turnLeft(int turnTime = 4000);
void turnRight(int turnTime = 4000);
void lookUp();
void lookDown();
void lookLeft();
void lookRight();
void lookCenter();
void startDirectionalLook(EyeDirection eyeDir, int headPos, int tiltPos);
void actionAnimation();
void waitingAnimation();
void updateServoPositions();
void checkDelayedHeadMovement();
void performBlink(bool isBigEye, int duration);
int moveServoToward(int current, int target);
void performDance();
void updateAnimations();

// Буфер для хранения входящих команд
String inputBuffer = "";
boolean commandComplete = false;

// === ГЛОБАЛЬНЫЕ ОБЪЕКТЫ ДЛЯ ГЛАЗ ===
LedControl lc(LED_DIN_PIN, LED_CLK_PIN, LED_CS_PIN, LED_MATRIX_COUNT);
Servo headServo, tiltServo;

// === СОСТОЯНИЕ ГЛАЗ ===
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

void setup() {
  Serial.begin(BAUD_RATE);
  
  // Инициализация пинов моторов
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

//   stopMotors();
  
  // Инициализация глаз
  initializeServos();
  initializeLEDMatrices();
  
  // По умолчанию: WAITING режим, маленькие глаза, голова в центре
  currentMode = MODE_WAITING;
  displayEye(EYE_OPEN, false);
  setServoTargets(HEAD_CENTER, TILT_CENTER);
  
  Serial.println("STATUS:0:Arduino initialized");
}

void loop() {
  readSerialCommands();

  if (commandComplete) {
    processCommand(inputBuffer);
    inputBuffer = "";
    commandComplete = false;
  }
  
  // Обновляем анимацию глаз и плавное движение серво
  updateAnimations();
}

// === ФУНКЦИИ ОТОБРАЖЕНИЯ ГЛАЗ ===
void showPattern(int device, const byte pattern[8]) {
  for (int row = 0; row < MATRIX_SIZE; row++) {
    lc.setRow(device, row, pattern[row]);
  }
}

void showBothEyes(const byte pattern[8]) {
  showPattern(0, pattern);
  showPattern(1, pattern);
}

void displayEye(EyeDirection direction, bool isBigEye) {
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
      
      Serial.print("HEAD:");
      Serial.println(servo.currentHead);
      Serial.print("TILT:");
      Serial.println(servo.currentTilt);
      
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

// === АТОМАРНЫЕ ДВИЖЕНИЯ ГЛАЗ ===
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

// === АНИМАЦИИ ГЛАЗ ===
void performBlink(bool isBigEye, int duration) {
  displayEye(EYE_CLOSED, isBigEye);
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
  }
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

// === ИНИЦИАЛИЗАЦИЯ ГЛАЗ ===
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

void readSerialCommands() {
  while (Serial.available() > 0 && !commandComplete) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      commandComplete = true;
    } else {
      inputBuffer += inChar;
    }
  }
}

void processCommand(String command) {
  int firstSeparator = command.indexOf(':');
  if (firstSeparator == -1) {
    Serial.println("ERROR:3:Invalid command format");
    return;
  }

  String cmdType = command.substring(0, firstSeparator);
  String params = command.substring(firstSeparator + 1);

  if (cmdType == "CHASSIS") {
    handleChassisCommand(params);
  } else if (cmdType == "EYES") {
    handleEyesCommand(params);
  } else if (cmdType == "GET") {
    if (params == "STATUS") {
      Serial.println("STATUS:0:OK");
    } else {
      Serial.print("ERROR:7:Unknown GET parameter: ");
      Serial.println(params);
    }
  } else {
    Serial.print("ERROR:4:Unknown command type: ");
    Serial.println(cmdType);
  }
}

void handleEyesCommand(String params) {
  if (params == "UP") {
    lookUp();
    Serial.println("ACK:EYES:UP");
  } else if (params == "DOWN") {
    lookDown();
    Serial.println("ACK:EYES:DOWN");
  } else if (params == "LEFT") {
    lookLeft();
    Serial.println("ACK:EYES:LEFT");
  } else if (params == "RIGHT") {
    lookRight();
    Serial.println("ACK:EYES:RIGHT");
  } else if (params == "CENTER") {
    lookCenter();
    Serial.println("ACK:EYES:CENTER");
  } else {
    Serial.print("ERROR:8:Unsupported eyes direction: ");
    Serial.println(params);
  }
}

void handleChassisCommand(String params) {
  int separator = params.indexOf(':');
  String action;
  int speed = MAX_PWM_FORWARD_SPEED;

  if (separator == -1) {
    action = params;
  } else {
    action = params.substring(0, separator);
    speed = params.substring(separator + 1).toInt();
  }

  speed = constrain(speed, 0, MAX_PWM_FORWARD_SPEED);
  if (action == "MOVE_FORWARD") {
    moveForward(speed);
    Serial.print("ACK:CHASSIS:MOVE_FORWARD:");
    Serial.println(speed);
  } 
  else if (action == "MOVE_BACKWARD") {
    moveBackward(speed);
    Serial.print("ACK:CHASSIS:MOVE_BACKWARD:");
    Serial.println(speed);
  }
  else if (action == "STOP") {
    stopMotors();
    Serial.println("ACK:CHASSIS:STOP");
  }
  else if (action == "ROTATE_LEFT") {
    turnLeft();
    Serial.println("ACK:CHASSIS:ROTATE_LEFT");
  }
  else if (action == "ROTATE_RIGHT") {
    turnRight();
    Serial.println("ACK:CHASSIS:ROTATE_RIGHT");
  }
  else if (action == "DANCE") {
    performDance();
    Serial.println("ACK:CHASSIS:DANCE");
  }
  else {
    Serial.print("ERROR:5:Unsupported chassis action: ");
    Serial.println(action);
  }
}

void moveForward(int speed) {
  lookDown();   
  // Устанавливаем направление движения
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Плавное увеличение скорости
  for (int currentSpeed = 0; currentSpeed <= speed; currentSpeed += ACCELERATION_STEP) {
    if (currentSpeed > speed) {
      currentSpeed = speed; // Не превышаем целевую скорость
    }
    
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
    
    if (currentSpeed < speed) {
      busyDelay(ACCELERATION_DELAY);
    }
  }
}

void moveBackward(int speed) {  
  // Устанавливаем направление движения назад (инвертированное)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Плавное увеличение скорости
  for (int currentSpeed = 0; currentSpeed <= speed; currentSpeed += ACCELERATION_STEP) {
    if (currentSpeed > speed) {
      currentSpeed = speed; // Не превышаем целевую скорость
    }
    
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
    
    if (currentSpeed < speed) {
      busyDelay(ACCELERATION_DELAY);
    }
  }
}

void stopMotors() {
  // Получаем текущую скорость (примерно, так как нет обратной связи)
  // Будем предполагать максимальную скорость для плавной остановки
  int currentSpeed = MAX_PWM_FORWARD_SPEED;
  
  // Плавное уменьшение скорости до 0
  for (int speed = currentSpeed; speed > 0; speed -= DECELERATION_STEP) {
    if (speed < 0) {
      speed = 0;
    }
    
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    
    if (speed > 0) {
      busyDelay(DECELERATION_DELAY);
    }
  }
  
  // Полная остановка
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
  lookCenter();
}

void turnLeft(int turnTime) {
  // Запускаем анимацию глаз и задаём новые цели для сервоприводов.
  displayEye(EYE_LEFT, true);
  setServoTargets(HEAD_LEFT, TILT_DOWN);
  
  // Устанавливаем направление поворота
  // Левый мотор назад
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  // Правый мотор вперёд
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Плавное увеличение скорости поворота
  for (int currentSpeed = 0; currentSpeed <= MAX_PWM_ROTATE_SPEED; currentSpeed += ACCELERATION_STEP) {
    if (currentSpeed > MAX_PWM_ROTATE_SPEED) {
      currentSpeed = MAX_PWM_ROTATE_SPEED;
    }
    
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
    
    if (currentSpeed < MAX_PWM_ROTATE_SPEED) {
      busyDelay(ACCELERATION_DELAY);
    }
  }
  
  // Поворот на полной скорости (учитываем только время разгона)
  int accelerationTime = (MAX_PWM_ROTATE_SPEED / ACCELERATION_STEP * ACCELERATION_DELAY);
  int fullSpeedTime = turnTime - accelerationTime;
  if (fullSpeedTime > 0) {
    busyDelay(fullSpeedTime);
  }
  
  stopMotors(); // Уже содержит плавную остановку
}

void turnRight(int turnTime) {
  displayEye(EYE_RIGHT, true);
  setServoTargets(HEAD_RIGHT, TILT_DOWN);
  
  // Устанавливаем направление поворота
  // Левый мотор вперёд
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Правый мотор назад
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  // Плавное увеличение скорости поворота
  for (int currentSpeed = 0; currentSpeed <= MAX_PWM_ROTATE_SPEED; currentSpeed += ACCELERATION_STEP) {
    if (currentSpeed > MAX_PWM_ROTATE_SPEED) {
      currentSpeed = MAX_PWM_ROTATE_SPEED;
    }
    
    analogWrite(ENA, currentSpeed);
    analogWrite(ENB, currentSpeed);
    
    if (currentSpeed < MAX_PWM_ROTATE_SPEED) {
      busyDelay(ACCELERATION_DELAY);
    }
  }
  
  // Поворот на полной скорости (учитываем только время разгона)
  int accelerationTime = (MAX_PWM_ROTATE_SPEED / ACCELERATION_STEP * ACCELERATION_DELAY);
  int fullSpeedTime = turnTime - accelerationTime;
  if (fullSpeedTime > 0) {
    busyDelay(fullSpeedTime);
  }
  
  stopMotors(); // Уже содержит плавную остановку
}

void performDance() {
  busyDelay(5000);
  lookCenter();
  int turnTime = 1000; // 1 секунда в каждую сторону
  
  for (int i = 0; i < 2; i++) {
    turnLeft(turnTime);
    turnRight(turnTime);
  }
  
  stopMotors();
}

// === ВСПОМОГАТЕЛЬНЫЕ ЗАДЕРЖКИ ===
// Задержка, во время которой продолжаем плавно обновлять сервоприводы
void busyDelay(unsigned long durationMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < durationMs) {
    updateAnimations();
  }
}

void updateAnimations() {
  checkDelayedHeadMovement();
  if (currentMode == MODE_ACTION) {
    actionAnimation();
  } else {
    waitingAnimation();
  }
  updateServoPositions();
}
