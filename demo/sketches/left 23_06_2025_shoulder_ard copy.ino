#include <Servo.h>
#include <Wire.h>
#include "MPU6050.h"
MPU6050 mpu;

/*
  Скетч для управления манипулятором с упрощенным форматом команд.
  - Локоть (реле): движение вверх/вниз на заданное время.
  - Плечо поворот (серво): поворот на заданный угол.
  - Плечо подъем (серво): подъем/опускание руки.

  Формат команд: ДЕЙСТВИЕ:ЗНАЧЕНИЕ или ДЕЙСТВИЕ (без значения)
*/

// --- КОНСТАНТЫ КОМАНД ---
const String CMD_ELBOW_UP = "ELBOW_UP";
const String CMD_ELBOW_DOWN = "ELBOW_DOWN";
const String CMD_SHOULDER_ROTATE_OUT = "SHOULDER_ROTATE_OUT";
const String CMD_SHOULDER_ROTATE_IN = "SHOULDER_ROTATE_IN";
const String CMD_SHOULDER_UP = "SHOULDER_UP";
const String CMD_SHOULDER_DOWN = "SHOULDER_DOWN";

// --- ПИНЫ ---
const int ELBOW_UP_PIN = 27;
const int ELBOW_DOWN_PIN = 26;
const int SHOULDER_SERVO_PIN = 39;    // Серво для поворота левого плеча
const int SHOULDER_LIFT_PIN = 9;      // Серво для подъема/опускания левого плеча

// --- MPU-6050 КОНСТАНТЫ ---
const int MPU_addr = 0x68; // Адрес MPU-6050 по умолчанию
const int TARGET_ANGLE_X = 0;
const int ANGLE_TOLERANCE = 10; // Допустимое отклонение

// --- ОБЪЕКТЫ ---
Servo shoulderServo;      // Серво поворота плеча
Servo shoulderLiftServo;  // Серво подъема плеча

// --- ПЕРЕМЕННЫЕ ПОВОРОТА ПЛЕЧА ---
const int SHOULDER_STOP = 90;      // Значение для остановки
const int SHOULDER_ROTATE_OUT = 120; // Вращение наружу  
const int SHOULDER_ROTATE_IN = 60;   // Вращение внутрь

// --- ПЕРЕМЕННЫЕ ПОДЪЕМА ПЛЕЧА ---
const int LIFT_STOP = 97;          // Значение для остановки
const int LIFT_UP = 91;            // Вращение для подъема
const int LIFT_DOWN = 108;         // Вращение для опускания
const int LIFT_DURATION = 2000;    // 1 секунда на движение

// --- MPU-6050 ПЕРЕМЕННЫЕ ---
int16_t ax, ay, az;
float angleX;
float initialRoll = 86;

// --- ПЕРЕМЕННЫЕ КОМАНД ---
String inputString = "";
boolean stringComplete = false;

void setup() {
  initializeSystem();
}

void loop() {
  if (stringComplete) {
    processCommand(inputString);
    resetCommandInput();
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// --- ИНИЦИАЛИЗАЦИЯ ---
void initializeSystem() {
 Serial.begin(9600);
  printWelcomeMessage();

  initializeElbow();
  initializeServos();
  initializeMPU6050();
  calibrateShoulder();
}

void initializeElbow() {
  pinMode(ELBOW_UP_PIN, OUTPUT);
  pinMode(ELBOW_DOWN_PIN, OUTPUT);
  digitalWrite(ELBOW_UP_PIN, LOW);
  digitalWrite(ELBOW_DOWN_PIN, LOW);
}

void initializeServos() {
  shoulderServo.attach(SHOULDER_SERVO_PIN);
  shoulderLiftServo.attach(SHOULDER_LIFT_PIN);
  shoulderLiftServo.write(97);
  // Не посылаем команды серво при инициализации
  delay(1000);
}

void initializeMPU6050() {
  Wire.begin();
  mpu.initialize(); 
}

void printWelcomeMessage() {
  Serial.println("=== Система управления манипулятором ===");
  Serial.println("Доступные команды:");
  Serial.println("- ELBOW_UP:время_мс");
  Serial.println("- ELBOW_DOWN:время_мс");
  Serial.println("- SHOULDER_ROTATE_OUT:время_мс");
  Serial.println("- SHOULDER_ROTATE_IN:время_мс");
  Serial.println("- SHOULDER_UP");
  Serial.println("- SHOULDER_DOWN");
  Serial.println("========================================");
}


void calculateAngles() {
  int16_t ax = mpu.getAccelerationX();  // ускорение по оси Х
  // стандартный диапазон: +-2g
  ax = constrain(ax, -16384, 16384);    // ограничиваем +-1g
  angleX = ax / 16384.0;           // переводим в +-1.0
  // и в угол через арксинус
  if (angleX < 0) angleX = 90 - degrees(acos(angleX));
  else angleX = degrees(acos(-angleX)) - 90;
}

void calibrateShoulder() {
  Serial.println("Начинаем калибровку плеча...");
  Serial.print("Целевой угол X=");
  Serial.print(TARGET_ANGLE_X);
  Serial.print(", Допуск=");
  Serial.println(ANGLE_TOLERANCE);

  // Начальное чтение для логов
  calculateAngles();
  
  unsigned long startTime = millis();
  const unsigned long maxDuration = 3000; // 20 секунд
  
  while (millis() - startTime < maxDuration) {
    calculateAngles();
    
    int currentAngleX = round(angleX);

      Serial.println(" -> Стартуем новый круг цикла");
      Serial.println(currentAngleX);

    // Проверяем достижение цели
    if (currentAngleX >= 0 && currentAngleX <= ANGLE_TOLERANCE) {
      liftShoulderStop(); 
      Serial.println(" -> В допуске. СТОП.");
      Serial.println("Целевой угол достигнут!");
      Serial.print("Финальный угол: ");
      Serial.println(currentAngleX);
      break;
    }

    // Определяем и корректируем направление движения
    if (currentAngleX > ANGLE_TOLERANCE) {
      Serial.print(" -> Решение: Двигать ВНИЗ (");

      shoulderLiftServo.write(LIFT_UP);
    } else if (currentAngleX < 0) {
      Serial.print(" -> Решение: Двигать ВВЕРХ (");
      shoulderLiftServo.write(LIFT_DOWN);
    }
    
    delay(10); // Короткая задержка для реакции серво и нового замера
  }
  
  // Если время вышло, останавливаем
  liftShoulderStop();
  Serial.println("ПРЕДУПРЕЖДЕНИЕ: Время калибровки истекло");
  calculateAngles();
  Serial.print("Остановились на угле: ");
  Serial.println(round(angleX));
}

// --- ОБРАБОТКА КОМАНД ---
void processCommand(String command) {
  command.trim();
  logReceivedCommand(command);
  
  String action;
  int value;
  bool hasValue = parseCommand(command, action, value);
  
  executeCommand(action, value, hasValue);
}

void logReceivedCommand(String command) {
  Serial.print("Получена команда: ");
  Serial.println(command);
}

bool parseCommand(String command, String &action, int &value) {
  int colonIndex = command.indexOf(':');
  
  if (colonIndex == -1) {
    action = command;
    return false; // Нет значения
  } else {
    action = command.substring(0, colonIndex);
    String valueStr = command.substring(colonIndex + 1);
    value = valueStr.toInt();
    return true; // Есть значение
  }
}

void executeCommand(String action, int value, bool hasValue) {
  if (action == CMD_ELBOW_UP) {
    handleElbowCommand(action, value, hasValue, true);
  }
  else if (action == CMD_ELBOW_DOWN) {
    handleElbowCommand(action, value, hasValue, false);
  }
  else if (action == CMD_SHOULDER_ROTATE_OUT) {
    handleRotateCommand(action, value, hasValue, false);
  }
  else if (action == CMD_SHOULDER_ROTATE_IN) {
    handleRotateCommand(action, value, hasValue, true);
  }
  else if (action == CMD_SHOULDER_UP) {
    liftShoulderUp();
  }
  else if (action == CMD_SHOULDER_DOWN) {
    liftShoulderDown();
  }
  else {
    printUnknownCommand();
  }
}

void handleElbowCommand(String action, int value, bool hasValue, bool isUp) {
  if (!hasValue) {
    Serial.print("ОШИБКА: Команда ");
    Serial.print(action);
    Serial.println(" требует значение времени");
    return;
  }
  
  if (isUp) {
    moveElbowUp(value);
  } else {
    moveElbowDown(value);
  }
}

void handleRotateCommand(String action, int value, bool hasValue, bool isInward) {
  if (!hasValue) {
    Serial.print("ОШИБКА: Команда ");
    Serial.print(action);
    Serial.println(" требует значение времени");
    return;
  }
  
  rotateShoulder(value, isInward);
}

void printUnknownCommand() {
  Serial.println("ОШИБКА: Неизвестная команда");
  Serial.println("Доступные команды: ELBOW_UP, ELBOW_DOWN, SHOULDER_ROTATE_OUT, SHOULDER_ROTATE_IN, SHOULDER_UP, SHOULDER_DOWN");
}

void resetCommandInput() {
  inputString = "";
  stringComplete = false;
}

// --- ДВИЖЕНИЯ ЛОКТЯ ---
void moveElbowUp(int duration) {
  Serial.print("Движение локтя ВВЕРХ на ");
  Serial.print(duration);
  Serial.println(" мс");
  
  stopElbowMovement();
  digitalWrite(ELBOW_UP_PIN, HIGH);
  delay(duration);
  stopElbowMovement();
  
  Serial.println("Движение локтя ЗАВЕРШЕНО");
}

void moveElbowDown(int duration) {
  Serial.print("Движение локтя ВНИЗ на ");
  Serial.print(duration);
  Serial.println(" мс");
  
  stopElbowMovement();
  digitalWrite(ELBOW_DOWN_PIN, HIGH);
  delay(duration);
  stopElbowMovement();
  
  Serial.println("Движение локтя ЗАВЕРШЕНО");
}

void stopElbowMovement() {
  digitalWrite(ELBOW_UP_PIN, LOW);
  digitalWrite(ELBOW_DOWN_PIN, LOW);
}

// --- ПОВОРОТ ПЛЕЧА ---
void rotateShoulder(int duration, bool inward) {
  Serial.print("Поворот плеча ");
  Serial.print(inward ? "ВНУТРЬ" : "НАРУЖУ");
  Serial.print(" на ");
  Serial.print(duration);
  Serial.println(" мс");
  
  // Начинаем вращение
  if (inward) {
    shoulderServo.write(SHOULDER_ROTATE_IN);
  } else {
    shoulderServo.write(SHOULDER_ROTATE_OUT);
  }
  
  // Вращаем заданное время
  delay(duration);
  
  // Останавливаем вращение
  shoulderServo.write(SHOULDER_STOP);
  
  Serial.println("Поворот плеча ЗАВЕРШЕН");
}

// --- ПОДЪЕМ ПЛЕЧА ---
void liftShoulderUp() {
  Serial.print("Подъем плеча ВВЕРХ на ");
  Serial.print(LIFT_DURATION);
  Serial.println(" мс");
  
  shoulderLiftServo.write(LIFT_UP);
  delay(LIFT_DURATION);
  liftShoulderStop();
  delay(2000);
  Serial.println("Подъем плеча ЗАВЕРШЕН");
  calibrateShoulder();
}

void liftShoulderDown() {
  Serial.print("Опускание плеча ВНИЗ на ");
  Serial.print(LIFT_DURATION);
  Serial.println(" мс");
  
  shoulderLiftServo.write(LIFT_DOWN);
  delay(LIFT_DURATION);
  liftShoulderStop();
  Serial.println("Опускание плеча ЗАВЕРШЕНО");
   delay(2000);
  calibrateShoulder();
} 

void liftShoulderStop() {
  shoulderLiftServo.write(LIFT_STOP);
}