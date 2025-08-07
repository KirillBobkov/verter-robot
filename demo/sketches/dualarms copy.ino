#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

/*
  Скетч для управления двумя манипуляторами (левый и правый).
  - Локти (реле): движение вверх/вниз на заданное время.
  - Плечи поворот (серво): поворот на заданное время.
  - Плечи подъем (серво): подъем/опускание до калибровки по гироскопу.

  Формат команд: РУКА_ДЕЙСТВИЕ:ЗНАЧЕНИЕ или РУКА_ДЕЙСТВИЕ
  Примеры: "LEFT_ELBOW_UP:1000", "RIGHT_SHOULDER_UP"
*/

// --- КОНСТАНТЫ КОМАНД ---
const String CMD_ARM_RIGHT_WAVE = "ARM_RIGHT:WAVE";
const String CMD_ARM_RIGHT_RAISE = "ARM_RIGHT:RAISE";
const String CMD_ARM_LEFT_RAISE = "ARM_LEFT:RAISE";
const String CMD_ARM_BOTH_RAISE = "ARM_BOTH:RAISE";
const String CMD_ARM_BOTH_CALIBRATE = "ARM_BOTH:CALIBRATE";
const String CMD_ARM_BOTH_DANCE = "ARM_BOTH:DANCE";

const String CMD_ARM_LEFT_BRUSH_OPEN  = "ARM_LEFT:BRUSH_OPEN";
const String CMD_ARM_LEFT_BRUSH_CLOSE = "ARM_LEFT:BRUSH_CLOSE";

const String CMD_ARM_RIGHT_BRUSH_OPEN  = "ARM_RIGHT:BRUSH_OPEN";
const String CMD_ARM_RIGHT_BRUSH_CLOSE = "ARM_RIGHT:BRUSH_CLOSE";

// Буфер для команд
const int BUFFER_SIZE = 64;
char inputBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// --- ПИНЫ ПРАВОЙ РУКИ ---
// L298N для правого локтя (ENA подключен к +5V)
const int RIGHT_ELBOW_IN1 = 26;    // Направление 1 правого локтя
const int RIGHT_ELBOW_IN2 = 28;    // Направление 2 правого локтя

const int RIGHT_SHOULDER_SERVO_PIN = 39;
const int RIGHT_SHOULDER_LIFT_PIN = 8;

// --- ПИНЫ ЛЕВОЙ РУКИ ---
// L298N для левого локтя (ENB подключен к +5V)
const int LEFT_ELBOW_IN3 = 27;     // Направление 1 левого локтя
const int LEFT_ELBOW_IN4 = 29;     // Направление 2 левого локтя

const int LEFT_SHOULDER_SERVO_PIN = 40; // Новый пин, чтобы не конфликтовать
const int LEFT_SHOULDER_LIFT_PIN = 9;

// --- MPU-6050 КОНСТАНТЫ ---
// Оба сенсора на одном I2C-басе (SDA:20, SCL:21). У левого AD0 подключен к VCC.
const int TARGET_ANGLE_X = 0;
const int ANGLE_TOLERANCE = 6;
MPU6050 rightMpu(0x68); // Адрес по умолчанию
MPU6050 leftMpu(0x69);  // Адрес при AD0 = HIGH

// --- ОБЪЕКТЫ СЕРВО ---
Servo rightShoulderServo, rightShoulderLiftServo;
Servo leftShoulderServo, leftShoulderLiftServo;

// --- ПАРАМЕТРЫ ПОВОРОТА ПЛЕЧА (ОДИНАКОВЫ ДЛЯ ОБЕИХ РУК) ---

const int SHOULDER_STOP = 90;
const int SHOULDER_ROTATE_OUT_LEFT = 120;
const int SHOULDER_ROTATE_IN_LEFT = 90;
const int SHOULDER_ROTATE_OUT_RIGHT = 60;
const int SHOULDER_ROTATE_IN_RIGHT = 90;

// --- ПАРАМЕТРЫ ПОДЪЕМА ПЛЕЧА ---
// Правая рука
const int RIGHT_LIFT_STOP = 97;
const int RIGHT_LIFT_UP = 108;
const int RIGHT_LIFT_DOWN = 91;
// Левая рука (инвертировано)
const int LEFT_LIFT_STOP = 97;
const int LEFT_LIFT_UP = 91;
const int LEFT_LIFT_DOWN = 108;
const int LIFT_DURATION = 2000;

// --- ПИНЫ КИСТИ (ГРИППЕРА) ---
const int HAND_LEFT_IN1 = 31;    // L298N IN1 для захвата
const int HAND_LEFT_IN2 = 30;    // L298N IN2 для захвата
const int HAND_LEFT_BUTTON_PIN = 22; // Концевик (LOW при нажатии)
const int HAND_LEFT_ENCODER_PIN = 2; // Энкодер (INT1)

// --- ПИНЫ ПРАВОЙ КИСТИ (ГРИППЕРА) ---
const int HAND_RIGHT_IN1 = 33;    // L298N IN1 для захвата
const int HAND_RIGHT_IN2 = 32;    // L298N IN2 для захвата
const int HAND_RIGHT_BUTTON_PIN = 23; // Концевик (LOW при нажатии)
const int HAND_RIGHT_ENCODER_PIN = 3; // Энкодер (INT2)

// --- ГЛОБАЛЬНОЕ СОСТОЯНИЕ КИСТИ ---
volatile long handLeftEncoderCount = 0;
bool handLeftAtHome = false;

// --- ГЛОБАЛЬНОЕ СОСТОЯНИЕ ПРАВОЙ КИСТИ ---
volatile long handRightEncoderCount = 0;
bool handRightAtHome = false;

// --- НАСТРОЙКИ АЛГОРИТМА ЗАХВАТА ---
const long  maxMoveTime         = 4500;  // мс, лимит закрытия
const float speedThreshold      = 1.0;   // имп/с, почти ноль
const float suddenDropThreshold = 20.0;  // имп/с, резкое падение
const long  speedCheckInterval  = 200;   // мс, период опроса скорости
const long  minImpulsesForGrab  = 2000;  // импульсы до разрешения остановки

void setup() {
  initializeSystem();
}

void loop() {
  if (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      inputBuffer[bufferIndex] = '\0';
      processCommand(inputBuffer);
      bufferIndex = 0;
    } else if (bufferIndex < BUFFER_SIZE - 1) {
      inputBuffer[bufferIndex++] = inChar;
    }
  }
}

// --- ИНИЦИАЛИЗАЦИЯ ---
void initializeSystem() {
  Serial.begin(9600);

  initializeElbows();
  initializeServos();
  initializeMPUs();
  initializeHandLeft();
  initializeHandRight();
  
  calibrateRightShoulder();
  calibrateLeftShoulder();
  
  //Serial.println("Калибровка правой руки...");
  //Serial.println("Калибровка левой руки...");
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
  }
  Serial.println("Опускам локти...");
  
  // Опускаем левый локоть (L298N)
  digitalWrite(LEFT_ELBOW_IN3, LOW);
  digitalWrite(LEFT_ELBOW_IN4, HIGH);
  // Опускаем правый локоть (L298N)
  digitalWrite(RIGHT_ELBOW_IN1, LOW);
  digitalWrite(RIGHT_ELBOW_IN2, HIGH);
  
  startTime = millis();
  while (millis() - startTime < 5000) {
  }
  
  // Останавливаем двигатели
  digitalWrite(LEFT_ELBOW_IN3, LOW);
  digitalWrite(LEFT_ELBOW_IN4, LOW);
  digitalWrite(RIGHT_ELBOW_IN1, LOW);
  digitalWrite(RIGHT_ELBOW_IN2, LOW);
  Serial.println("Локти опущены");
// изначальная калибровка
  Serial.println("Калибруем ротацию плечей");
  rightShoulderServo.write(SHOULDER_STOP);
  leftShoulderServo.write(SHOULDER_STOP);

  //Serial.println("========================================");
  Serial.println("Система готова к работе!");

  armBothDance();
}

void initializeElbows() {
  // Настройка пинов L298N (ENA и ENB подключены к +5V)
  pinMode(RIGHT_ELBOW_IN1, OUTPUT);
  pinMode(RIGHT_ELBOW_IN2, OUTPUT);
  pinMode(LEFT_ELBOW_IN3, OUTPUT);
  pinMode(LEFT_ELBOW_IN4, OUTPUT);
  
  // Остановка всех двигателей
  digitalWrite(RIGHT_ELBOW_IN1, LOW);
  digitalWrite(RIGHT_ELBOW_IN2, LOW);
  digitalWrite(LEFT_ELBOW_IN3, LOW);
  digitalWrite(LEFT_ELBOW_IN4, LOW);
}

void initializeServos() {
  rightShoulderServo.attach(RIGHT_SHOULDER_SERVO_PIN);
  rightShoulderLiftServo.attach(RIGHT_SHOULDER_LIFT_PIN);
  rightShoulderLiftServo.write(RIGHT_LIFT_STOP);

  leftShoulderServo.attach(LEFT_SHOULDER_SERVO_PIN);
  leftShoulderLiftServo.attach(LEFT_SHOULDER_LIFT_PIN);
  leftShoulderLiftServo.write(LEFT_LIFT_STOP);
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {
  }
}

void initializeMPUs() {
  Wire.begin();

  // Инициализация правого MPU
  //Serial.println("Инициализация правого MPU (0x68)...");
  rightMpu.initialize();
  if (rightMpu.testConnection()) {
    //Serial.println("Правый MPU6050 найден!");
  } else {
    Serial.println("Не удалось найти правый MPU6050!");
    while (1);
  }

  // Инициализация левого MPU
  //Serial.println("Инициализация левого MPU (0x69)...");
  leftMpu.initialize();
  if (leftMpu.testConnection()) {
    //Serial.println("Левый MPU6050 найден!");
  } else {
    Serial.println("Не удалось найти левый MPU6050! Проверьте, что AD0 подключен к VCC.");
    while (1);
  }
}

// Добавляем инициализацию кисти
void initializeHandLeft() {
  pinMode(HAND_LEFT_IN1, OUTPUT);
  pinMode(HAND_LEFT_IN2, OUTPUT);
  pinMode(HAND_LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(HAND_LEFT_ENCODER_PIN, INPUT);

  // Держим мотор остановленным
  digitalWrite(HAND_LEFT_IN1, LOW);
  digitalWrite(HAND_LEFT_IN2, LOW);


  Serial.println("[HAND] Init: pins configured, motor stopped");

  // Прерывание энкодера
  attachInterrupt(digitalPinToInterrupt(HAND_LEFT_ENCODER_PIN), handLeftEncoderISR, CHANGE);

  Serial.println("[HAND] Init: encoder ISR attached");

  // Автоматически приводим кисть в нулевую точку
  handLeftHome();
}

// --- КАЛИБРОВКА ---
float calculateAngleX(MPU6050 &mpu, bool mirrored) {
    float angleX;
   int16_t ax = mpu.getAccelerationX();  // ускорение по оси Х
  if (mirrored) ax = -ax;
  // стандартный диапазон: +-2g
  ax = constrain(ax, -16384, 16384);    // ограничиваем +-1g
  angleX = ax / 16384.0;           // переводим в +-1.0
  // и в угол через арксинус
  if (angleX < 0) angleX = 90 - degrees(acos(angleX));
  else angleX = degrees(acos(-angleX)) - 90;
  return angleX;
}

void calibrateRightShoulder() {
    //Serial.println("Начинаем калибровку правого плеча...");
    unsigned long startTime = millis();
    const unsigned long maxDuration = 5000;

    while (millis() - startTime < maxDuration) {
        float currentAngleX = calculateAngleX(rightMpu, false);
        //Serial.print("Правое плечо: Текущий угол X = ");
        //Serial.println(currentAngleX);
        
        if (abs(currentAngleX - TARGET_ANGLE_X) <= ANGLE_TOLERANCE) {
            rightShoulderLiftServo.write(RIGHT_LIFT_STOP);
             unsigned long delayInner = millis();
              while (millis() - delayInner < 50) {
              }
            //Serial.println("Целевой угол для правой руки достигнут!");
            return;
        }

        if (currentAngleX > TARGET_ANGLE_X) {
            //Serial.println(" -> Решение: Двигать ВНИЗ");
            rightShoulderLiftServo.write(RIGHT_LIFT_DOWN);
        } else {
            //Serial.println(" -> Решение: Двигать ВВЕРХ");
            rightShoulderLiftServo.write(RIGHT_LIFT_UP);
        }
        unsigned long delayStart = millis();
        while (millis() - delayStart < 50) {
        }
    }
    
    rightShoulderLiftServo.write(RIGHT_LIFT_STOP);
    //Serial.println("ПРЕДУПРЕЖДЕНИЕ: Время калибровки правого плеча истекло.");
    float finalAngle = calculateAngleX(rightMpu, false);
    //Serial.print("Остановка на угле: ");
    //Serial.println(finalAngle);
}

void calibrateLeftShoulder() {
    //Serial.println("Начинаем калибровку левого плеча...");
    unsigned long startTime = millis();
    const unsigned long maxDuration = 5000;

    while (millis() - startTime < maxDuration) {
        float currentAngleX = calculateAngleX(leftMpu, true);
        //Serial.print("Левое плечо: Текущий угол X = ");
        //Serial.println(currentAngleX);
        
        if (abs(currentAngleX - TARGET_ANGLE_X) <= ANGLE_TOLERANCE) {
            leftShoulderLiftServo.write(LEFT_LIFT_STOP);
            unsigned long delayStart = millis();
            while (millis() - delayStart < 50) {
            }
            //Serial.println("Целевой угол для левой руки достигнут!");
            return;
        }

        if (currentAngleX > TARGET_ANGLE_X) {
            //Serial.println(" -> Решение: Двигать ВВЕРХ");
            leftShoulderLiftServo.write(LEFT_LIFT_DOWN);
        } else {
            //Serial.println(" -> Решение: Двигать ВНИЗ");
            leftShoulderLiftServo.write(LEFT_LIFT_UP);
        }
        unsigned long delayStart = millis();
        while (millis() - delayStart < 50) {
        }
    }
    
    leftShoulderLiftServo.write(LEFT_LIFT_STOP);
    //Serial.println("ПРЕДУПРЕЖДЕНИЕ: Время калибровки левого плеча истекло.");
    float finalAngle = calculateAngleX(leftMpu, true);
    //Serial.print("Остановка на угле: ");
    //Serial.println(finalAngle);
}

// --- ОБРАБОТКА КОМАНД ---
void processCommand(const char* command) {
  String cmdStr = String(command);
  cmdStr.trim();
  
  String action;
  int value = 0;
  bool hasValue = parseCommand(cmdStr, action, value);
  executeCommand(action, value, hasValue);
}

bool parseCommand(String command, String &action, int &value) {
  int colonIndex = command.indexOf(':');
  if (colonIndex == -1) {
    action = command;
    return false;
  }
  
  String secondPart = command.substring(colonIndex + 1);
  
  // Если вторая часть - число, то это команда с значением
  if (secondPart.toInt() != 0 || secondPart == "0") {
    action = command.substring(0, colonIndex);
    value = secondPart.toInt();
    return true;
  } else {
    // Иначе это команда типа ARM_RIGHT:WAVE - передаем целиком
    action = command;
    return false;
  }
}

void executeCommand(String action, int value, bool hasValue) {
    if (action == CMD_ARM_RIGHT_WAVE) armRightWave();
    else if (action == CMD_ARM_RIGHT_RAISE) armRightRaise();
    else if (action == CMD_ARM_LEFT_RAISE) armLeftRaise();
    else if (action == CMD_ARM_BOTH_RAISE) armBothRaise();
    else if (action == CMD_ARM_BOTH_CALIBRATE) armBothCalibrate();
    else if (action == CMD_ARM_BOTH_DANCE) armBothDance();
    else if (action == CMD_ARM_LEFT_BRUSH_OPEN)  armLeftBrushOpen();
    else if (action == CMD_ARM_LEFT_BRUSH_CLOSE) armLeftBrushClose();
    else if (action == CMD_ARM_RIGHT_BRUSH_OPEN)  armRightBrushOpen();
    else if (action == CMD_ARM_RIGHT_BRUSH_CLOSE) armRightBrushClose();
    else printUnknownCommand(action);
    
    Serial.println("EYES:CENTER");
    
    if (hasValue) {
        //Serial.print("Команда выполнена со значением: ");
        //Serial.println(value);
    } else {
        //Serial.println("Команда выполнена без значения");
    }
}

void printUnknownCommand(String action) {
  //Serial.print("ОШИБКА: Неизвестная команда");
  //Serial.println(action);
}

// --- ДВИЖЕНИЯ ---
void moveRightElbowUp(int duration) {
    //Serial.print("Движение правого локтя ВВЕРХ на ");
    //Serial.print(duration);
    //Serial.println(" мс");
    
    // Направление: IN1=HIGH, IN2=LOW
    digitalWrite(RIGHT_ELBOW_IN1, HIGH);
    digitalWrite(RIGHT_ELBOW_IN2, LOW);
    
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
    }
    
    // Остановка двигателя
    digitalWrite(RIGHT_ELBOW_IN1, LOW);
    digitalWrite(RIGHT_ELBOW_IN2, LOW);
    //Serial.println("Движение правого локтя ЗАВЕРШЕНО");
}

void moveRightElbowDown(int duration) {
    //Serial.print("Движение правого локтя ВНИЗ на ");
    //Serial.print(duration);
    //Serial.println(" мс");
    
    // Направление: IN1=LOW, IN2=HIGH (обратное)
    digitalWrite(RIGHT_ELBOW_IN1, LOW);
    digitalWrite(RIGHT_ELBOW_IN2, HIGH);
    
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
    }
    
    // Остановка двигателя
    digitalWrite(RIGHT_ELBOW_IN1, LOW);
    digitalWrite(RIGHT_ELBOW_IN2, LOW);
    //Serial.println("Движение правого локтя ЗАВЕРШЕНО");
}

void moveLeftElbowUp(int duration) {
    //Serial.print("Движение левого локтя ВВЕРХ на ");
    //Serial.print(duration);
    //Serial.println(" мс");
    
    // Направление: IN3=HIGH, IN4=LOW
    digitalWrite(LEFT_ELBOW_IN3, HIGH);
    digitalWrite(LEFT_ELBOW_IN4, LOW);
    
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
    }
    
    // Остановка двигателя
    digitalWrite(LEFT_ELBOW_IN3, LOW);
    digitalWrite(LEFT_ELBOW_IN4, LOW);
    //Serial.println("Движение левого локтя ЗАВЕРШЕНО");
}

void moveLeftElbowDown(int duration) {
    //Serial.print("Движение левого локтя ВНИЗ на ");
    //Serial.print(duration);
    //Serial.println(" мс");
    digitalWrite(LEFT_ELBOW_IN3, LOW);
    digitalWrite(LEFT_ELBOW_IN4, HIGH);
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
    }
    digitalWrite(LEFT_ELBOW_IN3, LOW);
    digitalWrite(LEFT_ELBOW_IN4, LOW);
    //Serial.println("Движение левого локтя ЗАВЕРШЕНО");
}

void rotateRightShoulder(int duration, bool inward) {
  //Serial.print("Поворот правого плеча ");
  //Serial.print(inward ? "ВНУТРЬ" : "НАРУЖУ");
  //Serial.print(" на ");
  //Serial.print(duration);
  //Serial.println(" мс");
  
  rightShoulderServo.write(inward ? SHOULDER_ROTATE_IN_RIGHT : SHOULDER_ROTATE_OUT_RIGHT);
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
  }
  rightShoulderServo.write(SHOULDER_STOP);
  
  //Serial.println("Поворот правого плеча ЗАВЕРШЕН");
}

void rotateLeftShoulder(int duration, bool inward) {
  //Serial.print("Поворот левого плеча ");
  //Serial.print(inward ? "ВНУТРЬ" : "НАРУЖУ");
  //Serial.print(" на ");
  //Serial.print(duration);
  //Serial.println(" мс");
  
  leftShoulderServo.write(inward ? SHOULDER_ROTATE_IN_LEFT : SHOULDER_ROTATE_OUT_LEFT);
  unsigned long startTime = millis();
  while (millis() - startTime < duration) {
  }
  leftShoulderServo.write(SHOULDER_STOP);
  
  //Serial.println("Поворот левого плеча ЗАВЕРШЕН");
}

void liftRightShoulderUp() {
  //Serial.println("Подъем правого плеча...");
  rightShoulderLiftServo.write(RIGHT_LIFT_UP);
  unsigned long startTime = millis();
  while (millis() - startTime < LIFT_DURATION) {
  }
  rightShoulderLiftServo.write(RIGHT_LIFT_STOP);
  //Serial.println("Движение завершено. Повторная калибровка...");
}

void liftLeftShoulderUp() {
  //Serial.println("Подъем левого плеча...");
  leftShoulderLiftServo.write(LEFT_LIFT_UP);
  unsigned long startTime = millis();
  while (millis() - startTime < LIFT_DURATION) {
  }
  leftShoulderLiftServo.write(LEFT_LIFT_STOP);
  //Serial.println("Движение завершено. Повторная калибровка...");
}

// --- НОВЫЕ КОМАНДЫ ---
void armRightWave() {
  //Serial.println("Выполнение: Помахать правой рукой");
  calibrateRightShoulder();
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
  }
  moveRightElbowUp(5000);
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  // Поднимаем правую руку
  liftRightShoulderUp();
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  
  // Серия движений вращения для имитации махания
  for (int i = 0; i < 2; i++) {
    rotateRightShoulder(200, true);
    startTime = millis();
    while (millis() - startTime < 10) {
    }
    rotateRightShoulder(200, false);
    startTime = millis();
    while (millis() - startTime < 10) {
    }
  }
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  calibrateRightShoulder();
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  moveRightElbowDown(5000);
  //Serial.println("Команда ARM_RIGHT:WAVE завершена");
}

void armRightRaise() {
  //Serial.println("Выполнение: Поднять правую руку");
  calibrateRightShoulder();
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
  }
  moveRightElbowUp(5000);
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  liftRightShoulderUp();
  startTime = millis();
  while (millis() - startTime < 500) {
  }
  calibrateRightShoulder();
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  moveRightElbowDown(5000);
  //Serial.println("Команда ARM_RIGHT:RAISE завершена");
}

void armLeftRaise() {
  //Serial.println("Выполнение: Поднять левую руку");
  calibrateLeftShoulder();
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
  }
  moveLeftElbowUp(5000);
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  liftLeftShoulderUp();
  startTime = millis();
  while (millis() - startTime < 500) {
  }
  calibrateLeftShoulder();
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  moveLeftElbowDown(5000);
  //Serial.println("Команда ARM_LEFT:RAISE завершена");
}

void armBothRaise() {
  //Serial.println("Выполнение: Поднять обе руки");
  
  // Поднимаем левый локоть
  digitalWrite(LEFT_ELBOW_IN3, HIGH);
  digitalWrite(LEFT_ELBOW_IN4, LOW);

  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
  }
    // Поднимаем правый локоть
  digitalWrite(RIGHT_ELBOW_IN1, HIGH);
  digitalWrite(RIGHT_ELBOW_IN2, LOW);

  startTime = millis();
  while (millis() - startTime < 5000) {
  }
  digitalWrite(LEFT_ELBOW_IN3, LOW);
  digitalWrite(LEFT_ELBOW_IN4, LOW);
  

  startTime = millis();
  while (millis() - startTime < 100) {
  }

  digitalWrite(RIGHT_ELBOW_IN1, LOW);
  digitalWrite(RIGHT_ELBOW_IN2, LOW);
  
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  
  // Поднимаем оба плеча одновременно
  rightShoulderLiftServo.write(RIGHT_LIFT_UP);
  leftShoulderLiftServo.write(LEFT_LIFT_UP);
  startTime = millis();
  while (millis() - startTime < LIFT_DURATION) {
  }
  rightShoulderLiftServo.write(RIGHT_LIFT_STOP);
  leftShoulderLiftServo.write(LEFT_LIFT_STOP);
  
  //Serial.println("Движение завершено. Калибровка обеих рук...");

  startTime = millis();
  while (millis() - startTime < 100) {
  }

  // Калибруем обе руки
  calibrateLeftShoulder();
   
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  calibrateRightShoulder();
 

  startTime = millis();
  while (millis() - startTime < 100) {
  }
  
  // Опускаем левый локоть
  digitalWrite(LEFT_ELBOW_IN3, LOW);
  digitalWrite(LEFT_ELBOW_IN4, HIGH);

  
  startTime = millis();
  while (millis() - startTime < 100) {
  }

  // Опускаем правый локоть
  digitalWrite(RIGHT_ELBOW_IN1, LOW);
  digitalWrite(RIGHT_ELBOW_IN2, HIGH);

  startTime = millis();
  while (millis() - startTime < 5000) {
  }

  digitalWrite(LEFT_ELBOW_IN3, LOW);
  digitalWrite(LEFT_ELBOW_IN4, LOW);
  
  startTime = millis();
  while (millis() - startTime < 100) {
  }
  
  digitalWrite(RIGHT_ELBOW_IN1, LOW);
  digitalWrite(RIGHT_ELBOW_IN2, LOW);
  
  //Serial.println("Команда ARM_BOTH:RAISE завершена");
} 

void armBothCalibrate() {
  // Калибруем обе руки
  calibrateLeftShoulder();
   
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
  }
  calibrateRightShoulder();
}

void armBothDance() {
  //Serial.println("Выполнение: Танец руками");
  
  // Поднимаем оба локтя одновременно
  digitalWrite(LEFT_ELBOW_IN3, HIGH);
  digitalWrite(LEFT_ELBOW_IN4, LOW);
  digitalWrite(RIGHT_ELBOW_IN1, HIGH);
  digitalWrite(RIGHT_ELBOW_IN2, LOW);

  // Держим локти поднятыми 5 секунд
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
  }
  
  // Останавливаем движение локтей
  digitalWrite(LEFT_ELBOW_IN3, LOW);
  digitalWrite(LEFT_ELBOW_IN4, LOW);
  digitalWrite(RIGHT_ELBOW_IN1, LOW);
  digitalWrite(RIGHT_ELBOW_IN2, LOW);
  
  // Небольшая пауза
  startTime = millis();
  while (millis() - startTime < 500) {
  }
  
  // Вращаем плечами попеременно (4 раза)
  for (int i = 0; i < 8; i++) {
    // Вращение внутрь
    leftShoulderServo.write(SHOULDER_ROTATE_IN_LEFT);
    rightShoulderServo.write(SHOULDER_ROTATE_IN_RIGHT);
    startTime = millis();
    while (millis() - startTime < 300) {
    }
    
    // Вращение наружу
    leftShoulderServo.write(SHOULDER_ROTATE_OUT_LEFT);
    rightShoulderServo.write(SHOULDER_ROTATE_OUT_RIGHT);
    startTime = millis();
    while (millis() - startTime < 300) {
    }
  }
  
  // Останавливаем вращение
  leftShoulderServo.write(SHOULDER_STOP);
  rightShoulderServo.write(SHOULDER_STOP);
  
  // Небольшая пауза
  startTime = millis();
  while (millis() - startTime < 500) {
  }
  
  // Опускаем локти
  digitalWrite(LEFT_ELBOW_IN3, LOW);
  digitalWrite(LEFT_ELBOW_IN4, HIGH);
  digitalWrite(RIGHT_ELBOW_IN1, LOW);
  digitalWrite(RIGHT_ELBOW_IN2, HIGH);
  
  startTime = millis();
  while (millis() - startTime < 5000) {
  }
  
  // Останавливаем движение
  digitalWrite(LEFT_ELBOW_IN3, LOW);
  digitalWrite(LEFT_ELBOW_IN4, LOW);
  digitalWrite(RIGHT_ELBOW_IN1, LOW);
  digitalWrite(RIGHT_ELBOW_IN2, LOW);
  
  //Serial.println("Команда ARM_BOTH:DANCE завершена");
}

// === КОД КИСТИ ===
inline void stopHandLeftMotor() {
  digitalWrite(HAND_LEFT_IN1, LOW);
  digitalWrite(HAND_LEFT_IN2, LOW);
}

inline void startBackwardHandLeft() {
  digitalWrite(HAND_LEFT_IN1, LOW);
  digitalWrite(HAND_LEFT_IN2, HIGH);
}

inline void startForwardToHomeHandLeft() {
  digitalWrite(HAND_LEFT_IN1, HIGH);
  digitalWrite(HAND_LEFT_IN2, LOW);
}

// === ФУНКЦИИ УПРАВЛЕНИЯ ПРАВОЙ КИСТЬЮ ===
inline void stopHandRightMotor() {
  digitalWrite(HAND_RIGHT_IN1, LOW);
  digitalWrite(HAND_RIGHT_IN2, LOW);
}

inline void startBackwardHandRight() {
  digitalWrite(HAND_RIGHT_IN1, LOW);
  digitalWrite(HAND_RIGHT_IN2, HIGH);
}

inline void startForwardToHomeHandRight() {
  digitalWrite(HAND_RIGHT_IN1, HIGH);
  digitalWrite(HAND_RIGHT_IN2, LOW);
}

void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    delay(1);
  }
}

void handLeftHome() {
  Serial.println("Начат поиск нулевой точки");

  handLeftAtHome = false;
  handLeftEncoderCount = 0;
  startForwardToHomeHandLeft();

  // Крутимся пока не сработает концевик
  while (digitalRead(HAND_LEFT_BUTTON_PIN) != LOW) {
    delay(5);
  }

  waitMs(50); // дебаунс
  stopHandLeftMotor();
  handLeftEncoderCount = 0;
  handLeftAtHome = true;
  Serial.println("Нулевая точка найдена");
}

void handLeftGrab() {
  if (!handLeftAtHome) {
    Serial.println("Ошибка: сначала выполните ARM_LEFT:BRUSH_OPEN");
    return;
  }

  Serial.println("Захват начинает закрываться");
  startBackwardHandLeft();

  unsigned long moveStartTime   = millis();
  unsigned long lastSpeedCheck  = millis();
  float          lastImpPerSec  = 0;
  long           lastEncoderVal = handLeftEncoderCount;

  while (true) {
    unsigned long now = millis();

    // Периодическая проверка скорости
    if (now - lastSpeedCheck >= speedCheckInterval) {
      float dt       = (now - lastSpeedCheck) / 1000.0;
      long  impulses = handLeftEncoderCount - lastEncoderVal;
      float impPerSec = impulses / dt;
      float deltaSpeed = lastImpPerSec - impPerSec;

      lastEncoderVal  = handLeftEncoderCount;
      lastSpeedCheck  = now;
      lastImpPerSec   = impPerSec;

      bool slowEnough   = (impPerSec < speedThreshold);
      bool suddenDrop   = (deltaSpeed > suddenDropThreshold);
      bool passedMinImp = (handLeftEncoderCount > minImpulsesForGrab);

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

  stopHandLeftMotor();
  Serial.print("Пройдено импульсов: ");
  Serial.println(handLeftEncoderCount);
}

void handLeftEncoderISR() {
  // Считаем импульсы либо после калибровки, либо во время захвата (мотор назад)
  if (handLeftAtHome || digitalRead(HAND_LEFT_IN2) == HIGH) {
    handLeftEncoderCount++;
  }
}

// === NEW BRUSH COMMANDS ===
void armLeftBrushOpen() {
  // Открываем хват (калибровка/возврат к нулю) и опускаем локоть
  handLeftHome();
  moveLeftElbowDown(5000); // опустить локоть ~5 секунд
}

void armLeftBrushClose() {
  // Поднимаем локоть и выполняем захват
  moveLeftElbowUp(5000);   // поднять локоть ~5 секунд
  handLeftGrab();
}

// === ФУНКЦИИ ПРАВОЙ КИСТИ ===
void initializeHandRight() {
  pinMode(HAND_RIGHT_IN1, OUTPUT);
  pinMode(HAND_RIGHT_IN2, OUTPUT);
  pinMode(HAND_RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(HAND_RIGHT_ENCODER_PIN, INPUT);

  // Держим мотор остановленным
  digitalWrite(HAND_RIGHT_IN1, LOW);
  digitalWrite(HAND_RIGHT_IN2, LOW);

  Serial.println("[HAND_RIGHT] Init: pins configured, motor stopped");

  // Прерывание энкодера
  attachInterrupt(digitalPinToInterrupt(HAND_RIGHT_ENCODER_PIN), handRightEncoderISR, CHANGE);

  Serial.println("[HAND_RIGHT] Init: encoder ISR attached");

  // Автоматически приводим кисть в нулевую точку
  handRightHome();
}

void handRightHome() {
  Serial.println("Начат поиск нулевой точки правой кисти");

  handRightAtHome = false;
  handRightEncoderCount = 0;
  startForwardToHomeHandRight();

  // Крутимся пока не сработает концевик
  while (digitalRead(HAND_RIGHT_BUTTON_PIN) != LOW) {
    delay(5);
  }

  waitMs(50); // дебаунс
  stopHandRightMotor();
  handRightEncoderCount = 0;
  handRightAtHome = true;
  Serial.println("Нулевая точка правой кисти найдена");
}

void handRightGrab() {
  if (!handRightAtHome) {
    Serial.println("Ошибка: сначала выполните ARM_RIGHT:BRUSH_OPEN");
    return;
  }

  Serial.println("Захват правой кисти начинает закрываться");
  startBackwardHandRight();

  unsigned long moveStartTime   = millis();
  unsigned long lastSpeedCheck  = millis();
  float          lastImpPerSec  = 0;
  long           lastEncoderVal = handRightEncoderCount;

  while (true) {
    unsigned long now = millis();

    // Периодическая проверка скорости
    if (now - lastSpeedCheck >= speedCheckInterval) {
      float dt       = (now - lastSpeedCheck) / 1000.0;
      long  impulses = handRightEncoderCount - lastEncoderVal;
      float impPerSec = impulses / dt;
      float deltaSpeed = lastImpPerSec - impPerSec;

      lastEncoderVal  = handRightEncoderCount;
      lastSpeedCheck  = now;
      lastImpPerSec   = impPerSec;

      bool slowEnough   = (impPerSec < speedThreshold);
      bool suddenDrop   = (deltaSpeed > suddenDropThreshold);
      bool passedMinImp = (handRightEncoderCount > minImpulsesForGrab);

      if ((slowEnough || suddenDrop) && passedMinImp) {
        Serial.println(slowEnough ? "Объект захвачен правой кистью — скорость упала" :
                                     "Объект захвачен правой кистью — резкое падение скорости");
        break;
      }
    }

    // Ограничение по времени
    if (now - moveStartTime > maxMoveTime) {
      Serial.println("Достигнут максимальный предел движения правой кисти");
      break;
    }

    delay(10);
  }

  stopHandRightMotor();
  Serial.print("Правая кисть - пройдено импульсов: ");
  Serial.println(handRightEncoderCount);
}

void handRightEncoderISR() {
  // Считаем импульсы либо после калибровки, либо во время захвата (мотор назад)
  if (handRightAtHome || digitalRead(HAND_RIGHT_IN2) == HIGH) {
    handRightEncoderCount++;
  }
}

// === КОМАНДЫ ПРАВОЙ КИСТИ ===
void armRightBrushOpen() {
  // Открываем хват (калибровка/возврат к нулю) и опускаем локоть
  handRightHome();
  moveRightElbowDown(5000); // опустить локоть ~5 секунд
}

void armRightBrushClose() {
  // Поднимаем локоть и выполняем захват
  moveRightElbowUp(5000);   // поднять локоть ~5 секунд
  handRightGrab();
}