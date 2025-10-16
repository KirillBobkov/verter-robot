#include <CytronMotorDriver.h>

// Подключение Cytron MDD10A: два канала, режим Sign-Magnitude
CytronMD motor1(PWM_DIR, 4, 5);  // Канал 1: PWM = 4, DIR = 5 (левый мотор)
CytronMD motor2(PWM_DIR, 6, 7);  // Канал 2: PWM = 6, DIR = 7 (правый мотор, инвертирован)

// Параметры
const int TARGET_SPEED = 77;    // 30% от 255 (255 * 0.3 ≈ 77)
const int RAMP_STEPS = 10;      // Количество шагов для плавного разгона/торможения
const int RAMP_DELAY = 50;      // Задержка между шагами (мс), итого разгон ~500 мс
const int MOVE_DURATION = 1000; // Длительность движения (мс)

// Функция для плавного изменения скорости моторов
void rampSpeed(int speed1, int speed2) {
  int currentSpeed1 = 0;
  int currentSpeed2 = 0;
  int step1 = speed1 / RAMP_STEPS;
  int step2 = speed2 / RAMP_STEPS;

  // Разгон
  for (int i = 0; i < RAMP_STEPS; i++) {
    currentSpeed1 += step1;
    currentSpeed2 += step2;
    motor1.setSpeed(currentSpeed1);
    motor2.setSpeed(currentSpeed2);
    delay(RAMP_DELAY);
  }

  // Полная скорость на заданное время (минус время разгона)
  delay(MOVE_DURATION - (RAMP_STEPS * RAMP_DELAY));

  // Торможение
  for (int i = 0; i < RAMP_STEPS; i++) {
    currentSpeed1 -= step1;
    currentSpeed2 -= step2;
    motor1.setSpeed(currentSpeed1);
    motor2.setSpeed(currentSpeed2);
    delay(RAMP_DELAY);
  }

  // Стоп
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

void setup() {
  // Инициализация Serial для отладки
  Serial.begin(9600);
  Serial.println("Cytron MDD10A Test Start");

  // Начальная остановка моторов
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

// Переменные для управления состоянием
String currentCommand = "";
bool isMoving = false;
unsigned long moveStartTime = 0;
unsigned long moveDuration = 0;

void executeCommand(String command) {
  if (command == "CHASSIS:FRONT") {
    Serial.println("Executing: Forward (7 seconds)");
    motor1.setSpeed(TARGET_SPEED);
    motor2.setSpeed(-TARGET_SPEED);
    isMoving = true;
    moveStartTime = millis();
    moveDuration = 7000; // 7 секунд
  }
  else if (command == "CHASSIS:BACK") {
    Serial.println("Executing: Backward (7 seconds)");
    motor1.setSpeed(-TARGET_SPEED);
    motor2.setSpeed(TARGET_SPEED);
    isMoving = true;
    moveStartTime = millis();
    moveDuration = 7000; // 7 секунд
  }
  else if (command == "CHASSIS:LEFT") {
    Serial.println("Executing: Turn Left (3 seconds)");
    motor1.setSpeed(0);
    motor2.setSpeed(-TARGET_SPEED);
    isMoving = true;
    moveStartTime = millis();
    moveDuration = 3000; // 3 секунды
  }
  else if (command == "CHASSIS:RIGHT") {
    Serial.println("Executing: Turn Right (3 seconds)");
    motor1.setSpeed(TARGET_SPEED);
    motor2.setSpeed(0);
    isMoving = true;
    moveStartTime = millis();
    moveDuration = 3000; // 3 секунды
  }
  else if (command == "CHASSIS:STOP") {
    Serial.println("Executing: Stop");
    motor1.setSpeed(0);
    motor2.setSpeed(0);
    isMoving = false;
    moveDuration = 0;
  }
}

void stopMovement() {
  Serial.println("Movement timeout - stopping");
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  isMoving = false;
  moveDuration = 0;
}

void loop() {
  // Проверяем наличие команд в Serial
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Убираем лишние пробелы и символы
    
    if (command.startsWith("CHASSIS:")) {
      currentCommand = command;
      executeCommand(command);
    }
  }
  
  // Проверяем таймаут движения
  if (isMoving && moveDuration > 0) {
    if (millis() - moveStartTime >= moveDuration) {
      stopMovement();
    }
  }
  
  // Небольшая задержка для стабильности
  delay(10);
}