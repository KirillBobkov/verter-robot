#include <Wire.h>
#include <CytronMotorDriver.h>

#define AS5600_ADDRESS 0x36
#define RAW_ANGLE_REG 0x0C

// Параметры энкодера
int16_t prevAngle = 0;
long totalSteps = 0;

// Параметры шасси
const float WHEEL_CIRCUMFERENCE = 0.63;
const float GEAR_RATIO = 4.0007;
int FIXED_PWM = 98;  // Теперь переменная, а не константа

// Моторы
CytronMD motor1(PWM_DIR, 4, 5);
CytronMD motor2(PWM_DIR, 6, 7);

// Текущие PWM
int currentPWM1 = 0;
int currentPWM2 = 0;

// Желаемое расстояние (м)
float desiredDistance = 0.0;
bool isMoving = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(100);
  Serial.println("Скетч: Управление расстоянием с регулируемой скоростью");
  Serial.println("Команды:");
  Serial.println("  CHASSIS:FRONT:расстояние:pwm - вперед (например: CHASSIS:FRONT:1.0:80)");
  Serial.println("  CHASSIS:BACK:расстояние:pwm  - назад (например: CHASSIS:BACK:2.0:120)");
  Serial.println("  CHASSIS:STOP - остановка");
  Serial.println("  PWM:значение - изменить скорость (например: PWM:150)");
  
  motor1.setSpeed(0);
  motor2.setSpeed(0);
}

void loop() {
  // Чтение угла и обновление totalSteps
  int16_t angle = readAngle();
  int16_t diff = angle - prevAngle;
  if (diff > 2048) diff -= 4096;
  else if (diff < -2048) diff += 4096;
  totalSteps += diff;
  prevAngle = angle;

  // Обработка команд по Serial
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) continue;

    Serial.print("Получена команда: ");
    Serial.println(command);

    // Команда изменения PWM
    if (command.startsWith("PWM:")) {
      int newPWM = command.substring(4).toInt();
      if (newPWM >= 0 && newPWM <= 255) {
        FIXED_PWM = newPWM;
        Serial.print("Установлена скорость PWM: ");
        Serial.println(FIXED_PWM);
      } else {
        Serial.println("Ошибка: PWM должен быть от 0 до 255");
      }
    }
    // Команда движения с PWM
    else if (command.startsWith("CHASSIS:FRONT:")) {
      processMovementCommand(command, true);
    }
    else if (command.startsWith("CHASSIS:BACK:")) {
      processMovementCommand(command, false);
    }
    else if (command == "CHASSIS:STOP") {
      stopChassis();
      isMoving = false;
      Serial.println("Остановка");
    }
    else {
      Serial.println("Неизвестная команда");
    }
  }

  // Проверка пройденного расстояния с плавным торможением
  if (isMoving) {
    float encoderRevolutions = totalSteps / 4096.0;
    float wheelRevolutions = encoderRevolutions / GEAR_RATIO;
    float traveledDistance = wheelRevolutions * WHEEL_CIRCUMFERENCE;
    float remainingDistance = desiredDistance - traveledDistance;
    
    // Плавное торможение - последние 20 см
    if (remainingDistance <= 0.20) {
      // Плавно уменьшаем мощность от 100% до 0% на последних 20 см
      float brakeFactor = remainingDistance / 0.20; // от 1.0 до 0.0
      brakeFactor = constrain(brakeFactor, 0.0, 1.0);
      
      int newPWM = (int)(FIXED_PWM * brakeFactor);
      newPWM = max(newPWM, 20); // Минимум 20 PWM для уверенного движения
      
      currentPWM1 = (currentPWM1 > 0 ? newPWM : (currentPWM1 < 0 ? -newPWM : 0));
      currentPWM2 = (currentPWM2 > 0 ? newPWM : (currentPWM2 < 0 ? -newPWM : 0));
      
      updateMotorSpeeds();
    }
    
    // Логирование
    Serial.print("Прогресс: ");
    Serial.print(traveledDistance, 3);
    Serial.print(" м из ");
    Serial.print(desiredDistance, 3);
    Serial.print(" м (");
    Serial.print((traveledDistance / desiredDistance) * 100, 1);
    Serial.print("%) | PWM: ");
    Serial.println(FIXED_PWM);

    if (traveledDistance >= desiredDistance) {
      stopChassis();
      isMoving = false;
      Serial.print("✅ Точная остановка: ");
      Serial.print(traveledDistance, 3);
      Serial.println(" м");
    }
  }

  delay(50);
}

// Обработка команд движения с PWM
void processMovementCommand(String command, bool forward) {
  // Разбиваем команду на части: CHASSIS:FRONT:расстояние:pwm
  int firstColon = command.indexOf(':');
  int secondColon = command.indexOf(':', firstColon + 1);
  int thirdColon = command.indexOf(':', secondColon + 1);
  
  if (thirdColon != -1) {
    // Есть PWM в команде
    String distanceStr = command.substring(secondColon + 1, thirdColon);
    String pwmStr = command.substring(thirdColon + 1);
    
    desiredDistance = distanceStr.toFloat();
    int commandPWM = pwmStr.toInt();
    
    if (commandPWM >= 0 && commandPWM <= 255) {
      FIXED_PWM = commandPWM;
    }
  } else {
    // Без PWM в команде - используем текущий FIXED_PWM
    String distanceStr = command.substring(secondColon + 1);
    desiredDistance = distanceStr.toFloat();
  }
  
  // Устанавливаем направление
  if (forward) {
    currentPWM1 = FIXED_PWM;
    currentPWM2 = -FIXED_PWM;
    Serial.print("Движение вперед на ");
  } else {
    currentPWM1 = -FIXED_PWM;
    currentPWM2 = FIXED_PWM;
    Serial.print("Движение назад на ");
  }
  
  Serial.print(desiredDistance);
  Serial.print(" м со скоростью PWM: ");
  Serial.println(FIXED_PWM);
  
  resetEncoder();
  isMoving = true;
  updateMotorSpeeds();
}

// Функции (без изменений)
int16_t readAngle() {
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

float extractDistance(String command) {
  float distance = 1.0;
  if (command.indexOf(':') != command.lastIndexOf(':')) {
    distance = command.substring(command.lastIndexOf(':') + 1).toFloat();
  }
  return distance;
}

void updateMotorSpeeds() {
  motor1.setSpeed(currentPWM1);
  motor2.setSpeed(currentPWM2);
}

void stopChassis() {
  currentPWM1 = 0;
  currentPWM2 = 0;
  updateMotorSpeeds();
}

void resetEncoder() {
  totalSteps = 0;
  prevAngle = readAngle();
}