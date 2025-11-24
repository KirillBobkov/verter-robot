#include <Wire.h>
#include <SoftwareWire.h>
#include <CytronMotorDriver.h>

#define AS5600_ADDRESS 0x36
#define RAW_ANGLE_REG 0x0C

// Пины для программной I2C шины (для правого энкодера)
#define SOFT_SDA_PIN 22
#define SOFT_SCL_PIN 23

// Программная I2C шина для правого энкодера
SoftwareWire softWire(SOFT_SDA_PIN, SOFT_SCL_PIN);

// Параметры шасси
const float WHEEL_CIRCUMFERENCE = 0.63;
const float GEAR_RATIO = 4.0007;

// Моторы
CytronMD motor1(PWM_DIR, 4, 5);  // Правое колесо
CytronMD motor2(PWM_DIR, 6, 7);  // Левое колесо

// Структура для управления каждым колесом
struct WheelControl {
  int16_t prevAngle;
  long totalSteps;
  float desiredDistance;
  bool isMoving;
  int fixedPWM;
  int currentPWM;
  bool useSoftwareWire;  // true для правого (SoftwareWire), false для левого (Wire)
};

WheelControl leftWheel = {0, 0, 0.0, false, 98, 0, false};   // Левое: аппаратный I2C, motor2
WheelControl rightWheel = {0, 0, 0.0, false, 98, 0, true};   // Правое: программный I2C, motor1

// Forward declarations
void processCommand(String command, bool updateMotors = true);
void processWheelCommand(String command, WheelControl* wheel, bool forward, int motorNum, bool updateMotors = true);
void stopWheel(WheelControl* wheel, int motorNum, bool updateMotors = true);
void updateMotorSpeeds();

void setup() {
  Serial.begin(9600);
  
  // Инициализация аппаратного I2C для левого энкодера
  Wire.begin();
  
  // Инициализация программной I2C шины для правого энкодера
  softWire.begin();
  softWire.setClock(100000);
  
  delay(100);
  Serial.println("Скетч: Независимое управление двумя колесами");
  Serial.println("Команды:");
  Serial.println("  CHASSIS:LEFT:ASK:расстояние:pwm  - левое вперед (например: CHASSIS:LEFT:ASK:0.63:20)");
  Serial.println("  CHASSIS:LEFT:DESK:расстояние:pwm - левое назад (например: CHASSIS:LEFT:DESK:0.63:20)");
  Serial.println("  CHASSIS:RIGHT:ASK:расстояние:pwm - правое вперед");
  Serial.println("  CHASSIS:RIGHT:DESK:расстояние:pwm - правое назад");
  Serial.println("  CHASSIS:LEFT:STOP - остановка левого");
  Serial.println("  CHASSIS:RIGHT:STOP - остановка правого");
  Serial.println("  CHASSIS:STOP - остановка обоих");
  Serial.println("  Множественные команды через ; (например: CHASSIS:LEFT:ASK:0.63:20;CHASSIS:RIGHT:ASK:0.63:20)");
  
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

void loop() {
  // Чтение углов и обновление totalSteps для обоих колес
  updateEncoder(&leftWheel);
  updateEncoder(&rightWheel);

  // Обработка команд по Serial
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) continue;

    Serial.print("Получена команда: ");
    Serial.println(command);

    // Разделение множественных команд по ;
    int semicolonPos = command.indexOf(';');
    if (semicolonPos != -1) {
      // Обрабатываем обе команды без обновления моторов
      processCommand(command.substring(0, semicolonPos), false);
      processCommand(command.substring(semicolonPos + 1), false);
      // Обновляем моторы один раз после обработки всех команд
      updateMotorSpeeds();
    } else {
      processCommand(command, true);
    }
  }

  // Проверка пройденного расстояния для левого колеса
  if (leftWheel.isMoving) {
    checkWheelProgress(&leftWheel, 2);  // motor2
  }

  // Проверка пройденного расстояния для правого колеса
  if (rightWheel.isMoving) {
    checkWheelProgress(&rightWheel, 1);  // motor1
  }

  delay(50);
}

// Обновление энкодера для колеса
void updateEncoder(WheelControl* wheel) {
  int16_t angle = readAngle(wheel);
  
  if (wheel->useSoftwareWire && angle == -1) {
    return;  // Ошибка чтения программного I2C
  }
  
  if (!wheel->useSoftwareWire && angle == 0 && wheel->prevAngle != 0) {
    // Для аппаратного I2C проверяем валидность
    return;
  }
  
  int16_t diff = angle - wheel->prevAngle;
  if (diff > 2048) diff -= 4096;
  else if (diff < -2048) diff += 4096;
  
  // Левое колесо: инвертированный знак, Правое: без инверсии
  if (wheel->useSoftwareWire) {
    wheel->totalSteps += diff;  // Правое: без инверсии
  } else {
    wheel->totalSteps -= diff;  // Левое: инвертирован знак
  }
  
  wheel->prevAngle = angle;
}

// Чтение угла энкодера
int16_t readAngle(WheelControl* wheel) {
  if (wheel->useSoftwareWire) {
    // Программный I2C для правого энкодера
    softWire.beginTransmission(AS5600_ADDRESS);
    softWire.write(RAW_ANGLE_REG);
    if (softWire.endTransmission(false) != 0) return -1;
    
    if (softWire.requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2, (boolean)true) == 2) {
      int16_t angle = softWire.read() << 8;
      angle |= softWire.read();
      return angle & 0x0FFF;
    }
    return -1;
  } else {
    // Аппаратный I2C для левого энкодера
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(RAW_ANGLE_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_ADDRESS, 2);
    if (Wire.available() == 2) {
      int16_t angle = Wire.read() << 8;
      angle |= Wire.read();
      angle &= 0x0FFF;
      return angle;
    }
    return 0;
  }
}

// Обработка команды
void processCommand(String command, bool updateMotors = true) {
  if (command.startsWith("CHASSIS:LEFT:")) {
    if (command == "CHASSIS:LEFT:STOP") {
      stopWheel(&leftWheel, 2, updateMotors);
      Serial.println("Остановка левого колеса");
    } else if (command.startsWith("CHASSIS:LEFT:ASK:")) {
      processWheelCommand(command, &leftWheel, true, 2, updateMotors);
    } else if (command.startsWith("CHASSIS:LEFT:DESK:")) {
      processWheelCommand(command, &leftWheel, false, 2, updateMotors);
    } else {
      Serial.println("Неизвестная команда для левого колеса");
    }
  } else if (command.startsWith("CHASSIS:RIGHT:")) {
    if (command == "CHASSIS:RIGHT:STOP") {
      stopWheel(&rightWheel, 1, updateMotors);
      Serial.println("Остановка правого колеса");
    } else if (command.startsWith("CHASSIS:RIGHT:ASK:")) {
      processWheelCommand(command, &rightWheel, true, 1, updateMotors);
    } else if (command.startsWith("CHASSIS:RIGHT:DESK:")) {
      processWheelCommand(command, &rightWheel, false, 1, updateMotors);
    } else {
      Serial.println("Неизвестная команда для правого колеса");
    }
  } else if (command == "CHASSIS:STOP") {
    stopWheel(&leftWheel, 2, updateMotors);
    stopWheel(&rightWheel, 1, updateMotors);
    Serial.println("Остановка обоих колес");
  } else {
    Serial.println("Неизвестная команда");
  }
}

// Обработка команды движения для колеса
void processWheelCommand(String command, WheelControl* wheel, bool forward, int motorNum, bool updateMotors = true) {
  // Формат: CHASSIS:LEFT:ASK:расстояние:pwm или CHASSIS:LEFT:ASK:расстояние
  // Находим последние два двоеточия (после ASK/DESK)
  int firstColon = command.indexOf(':');
  int secondColon = command.indexOf(':', firstColon + 1);
  int thirdColon = command.indexOf(':', secondColon + 1);
  int fourthColon = command.indexOf(':', thirdColon + 1);
  
  if (fourthColon != -1) {
    // Есть PWM в команде: CHASSIS:LEFT:ASK:0.63:20
    String distanceStr = command.substring(thirdColon + 1, fourthColon);
    String pwmStr = command.substring(fourthColon + 1);
    
    wheel->desiredDistance = distanceStr.toFloat();
    int commandPWM = pwmStr.toInt();
    
    if (commandPWM >= 0 && commandPWM <= 255) {
      wheel->fixedPWM = commandPWM;
    }
  } else {
    // Без PWM в команде - используем текущий fixedPWM: CHASSIS:LEFT:ASK:0.63
    String distanceStr = command.substring(thirdColon + 1);
    wheel->desiredDistance = distanceStr.toFloat();
  }
  
  // Устанавливаем направление
  if (forward) {
    if (motorNum == 1) {
      // Правое колесо: вперед = положительный PWM
      wheel->currentPWM = wheel->fixedPWM;
    } else {
      // Левое колесо: вперед = отрицательный PWM
      wheel->currentPWM = -wheel->fixedPWM;
    }
    Serial.print(motorNum == 1 ? "Правое" : "Левое");
    Serial.print(" колесо: движение вперед на ");
  } else {
    if (motorNum == 1) {
      // Правое колесо: назад = отрицательный PWM
      wheel->currentPWM = -wheel->fixedPWM;
    } else {
      // Левое колесо: назад = положительный PWM
      wheel->currentPWM = wheel->fixedPWM;
    }
    Serial.print(motorNum == 1 ? "Правое" : "Левое");
    Serial.print(" колесо: движение назад на ");
  }
  
  Serial.print(wheel->desiredDistance, 3);
  Serial.print(" м со скоростью PWM: ");
  Serial.println(wheel->fixedPWM);
  
  resetWheelEncoder(wheel);
  wheel->isMoving = true;
  if (updateMotors) {
    updateMotorSpeeds();
  }
}

// Проверка прогресса колеса
void checkWheelProgress(WheelControl* wheel, int motorNum) {
  float encoderRevolutions = wheel->totalSteps / 4096.0;
  float wheelRevolutions = encoderRevolutions / GEAR_RATIO;
  float traveledDistance = wheelRevolutions * WHEEL_CIRCUMFERENCE;
  float absTraveledDistance = abs(traveledDistance);  // Абсолютное значение пройденного расстояния
  float remainingDistance = wheel->desiredDistance - absTraveledDistance;
  
  // Плавное торможение - последние 20 см
  if (remainingDistance <= 0.20 && remainingDistance > 0) {
    float brakeFactor = remainingDistance / 0.20; // от 1.0 до 0.0
    brakeFactor = constrain(brakeFactor, 0.0, 1.0);
    
    int newPWM = (int)(wheel->fixedPWM * brakeFactor);
    newPWM = max(newPWM, 40); // Минимум 40 PWM для уверенного движения
    
    wheel->currentPWM = (wheel->currentPWM > 0 ? newPWM : (wheel->currentPWM < 0 ? -newPWM : 0));
    
    updateMotorSpeeds();
  }
  
  // Логирование
  Serial.print(motorNum == 1 ? "Правое" : "Левое");
  Serial.print(": ");
  Serial.print(traveledDistance, 3);
  Serial.print(" м (абс: ");
  Serial.print(absTraveledDistance, 3);
  Serial.print(" м) из ");
  Serial.print(wheel->desiredDistance, 3);
  Serial.print(" м (");
  Serial.print((absTraveledDistance / wheel->desiredDistance) * 100, 1);
  Serial.print("%) | PWM: ");
  Serial.println(wheel->fixedPWM);

  if (absTraveledDistance >= wheel->desiredDistance) {
    stopWheel(wheel, motorNum);
    wheel->isMoving = false;
    Serial.print("✅ ");
    Serial.print(motorNum == 1 ? "Правое" : "Левое");
    Serial.print(" колесо: точная остановка - ");
    Serial.print(absTraveledDistance, 3);
    Serial.println(" м");
  }
}

// Обновление скоростей моторов
void updateMotorSpeeds() {
  motor1.setSpeed(rightWheel.currentPWM);  // Правое колесо
  motor2.setSpeed(leftWheel.currentPWM);   // Левое колесо
}

// Остановка колеса
void stopWheel(WheelControl* wheel, int motorNum, bool updateMotors = true) {
  wheel->currentPWM = 0;
  wheel->isMoving = false;
  if (updateMotors) {
    updateMotorSpeeds();
  }
}

// Сброс энкодера колеса
void resetWheelEncoder(WheelControl* wheel) {
  wheel->totalSteps = 0;
  int16_t angle = readAngle(wheel);
  if (wheel->useSoftwareWire) {
    if (angle != -1) {
      wheel->prevAngle = angle;
    }
  } else {
    wheel->prevAngle = angle;
  }
}

