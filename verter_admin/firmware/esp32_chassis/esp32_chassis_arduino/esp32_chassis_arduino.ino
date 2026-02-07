/**
 * ESP32 Chassis Controller
 *
 * Порт Arduino кода на ESP32 с сохранением Serial протокола.
 * Совместим с существующим chassis_node.py.
 *
 * Железо:
 * - Cytron MD10C драйвер (PWM + DIR)
 * - AS5600 магнитные энкодеры через I2C (2 шины)
 *
 * Распиновка (уже припаяно):
 * - Левый мотор: PWM=GPIO18, DIR=GPIO25
 * - Правый мотор: PWM=GPIO19, DIR=GPIO27
 * - Левый энкодер I2C0: SDA=GPIO21, SCL=GPIO22
 * - Правый энкодер I2C1: SDA=GPIO32, SCL=GPIO4
 */

#include <Wire.h>

// ============================================================================
// PIN CONFIGURATION (согласно реальной пайке)
// ============================================================================

// Левый мотор (Cytron PWM+DIR)
#define MOTOR_L_PWM   18
#define MOTOR_L_DIR   25

// Правый мотор (Cytron PWM+DIR)
#define MOTOR_R_PWM   19
#define MOTOR_R_DIR   27

// Левый энкодер AS5600 (I2C0 - Wire)
#define I2C0_SDA      21
#define I2C0_SCL      22

// Правый энкодер AS5600 (I2C1 - Wire1)
#define I2C1_SDA      32
#define I2C1_SCL      4

// PWM конфигурация ESP32
#define PWM_FREQ        20000   // 20 kHz (выше слышимого диапазона)
#define PWM_RESOLUTION  8       // 8 bit (0-255)
#define PWM_CHANNEL_L   0
#define PWM_CHANNEL_R   1

// ============================================================================
// AS5600 ENCODER
// ============================================================================

#define AS5600_ADDRESS  0x36
#define RAW_ANGLE_REG   0x0C

// ============================================================================
// ПАРАМЕТРЫ РОБОТА (из Arduino кода)
// ============================================================================

const float WHEEL_CIRCUMFERENCE = 0.63;   // Длина окружности колеса (м)
const float GEAR_RATIO = 4.0007;          // Передаточное число редуктора
const float WHEEL_BASE = 0.356;           // Расстояние между колёсами (м)

// Velocity mode параметры
const float MAX_VELOCITY = 0.5;           // Максимальная линейная скорость (м/с)
const float MAX_ANGULAR = 2.0;            // Максимальная угловая скорость (рад/с)
const int MAX_PWM = 200;                  // Максимальный PWM
const int MIN_PWM = 25;                   // Минимальный PWM (мёртвая зона)
const unsigned long CMD_TIMEOUT = 500;    // Таймаут команд (мс)

// Конверсия энкодер → метры
const float ENCODER_RESOLUTION = 4096.0;  // AS5600: 12 бит = 4096 шагов/оборот
const float METERS_PER_STEP = WHEEL_CIRCUMFERENCE / (ENCODER_RESOLUTION * GEAR_RATIO);

// PID параметры
const float PID_KP = 80.0;
const float PID_KI = 50.0;
const float PID_KD = 5.0;
const float PID_MAX_INTEGRAL = 100.0;
const unsigned long PID_INTERVAL = 50;    // мс
const int MAX_PWM_CHANGE = 15;            // Плавный разгон

// ============================================================================
// НАСТРОЙКИ ВЫВОДА
// ============================================================================

#define DEBUG_ENABLED false
#define ENCODER_INTERVAL 50               // Интервал отправки энкодеров (мс)

// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================================================

// I2C шины
TwoWire Wire1_custom = TwoWire(1);        // Вторая аппаратная I2C шина

// PID состояние
struct PIDState {
  float desiredVelocity;
  float actualVelocity;
  float error;
  float lastError;
  float integral;
  int outputPWM;
  long lastSteps;
  unsigned long lastTime;
};

PIDState leftPID = {0, 0, 0, 0, 0, 0, 0, 0};
PIDState rightPID = {0, 0, 0, 0, 0, 0, 0, 0};

// Управление колёсами
struct WheelControl {
  int16_t prevAngle;
  long totalSteps;
  float desiredDistance;
  bool isMoving;
  int fixedPWM;
  int currentPWM;
  bool useWire1;          // true для правого (Wire1), false для левого (Wire)
};

// ВНИМАНИЕ: I2C шины перепутаны физически, поэтому:
// Левый энкодер на Wire1 (I2C1), Правый на Wire (I2C0)
WheelControl leftWheel = {0, 0, 0.0, false, 98, 0, true};    // useWire1 = true
WheelControl rightWheel = {0, 0, 0.0, false, 98, 0, false};  // useWire1 = false

// Velocity mode
bool velocityMode = false;
unsigned long lastCmdTime = 0;
int velocityLeftPWM = 0;
int velocityRightPWM = 0;

unsigned long lastEncoderSendTime = 0;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void processCommand(String command, bool updateMotors = true);
void processWheelCommand(String command, WheelControl* wheel, bool forward, bool isRight, bool updateMotors = true);
void stopWheel(WheelControl* wheel, bool isRight, bool updateMotors = true);
void updateMotorSpeeds();
void processVelocityCommand(float linearVel, float angularVel);
void stopVelocityMode();
void updatePID();
void calculateWheelVelocity(PIDState* pid, long currentSteps);
int computePID(PIDState* pid);
void resetPID(PIDState* pid);

// ============================================================================
// MOTOR CONTROL (Cytron PWM+DIR)
// ============================================================================

void setMotorLeft(int pwm) {
  if (pwm > 0) {
    digitalWrite(MOTOR_L_DIR, HIGH);      // Вперёд
    ledcWrite(PWM_CHANNEL_L, min(pwm, MAX_PWM));
  } else if (pwm < 0) {
    digitalWrite(MOTOR_L_DIR, LOW);       // Назад
    ledcWrite(PWM_CHANNEL_L, min(-pwm, MAX_PWM));
  } else {
    ledcWrite(PWM_CHANNEL_L, 0);
  }
}

void setMotorRight(int pwm) {
  // ИНВЕРТИРОВАН - правый мотор крутится в обратную сторону
  if (pwm > 0) {
    digitalWrite(MOTOR_R_DIR, LOW);       // Вперёд (инверт)
    ledcWrite(PWM_CHANNEL_R, min(pwm, MAX_PWM));
  } else if (pwm < 0) {
    digitalWrite(MOTOR_R_DIR, HIGH);      // Назад (инверт)
    ledcWrite(PWM_CHANNEL_R, min(-pwm, MAX_PWM));
  } else {
    ledcWrite(PWM_CHANNEL_R, 0);
  }
}

void stopMotors() {
  ledcWrite(PWM_CHANNEL_L, 0);
  ledcWrite(PWM_CHANNEL_R, 0);
}

void updateMotorSpeeds() {
  setMotorRight(rightWheel.currentPWM);
  setMotorLeft(leftWheel.currentPWM);
}

// ============================================================================
// AS5600 ENCODER READING
// ============================================================================

int16_t readAngle(WheelControl* wheel) {
  TwoWire* wire = wheel->useWire1 ? &Wire1_custom : &Wire;

  wire->beginTransmission(AS5600_ADDRESS);
  wire->write(RAW_ANGLE_REG);
  if (wire->endTransmission(false) != 0) {
    return -1;  // Ошибка связи
  }

  if (wire->requestFrom((uint8_t)AS5600_ADDRESS, (uint8_t)2) == 2) {
    int16_t angle = wire->read() << 8;
    angle |= wire->read();
    return angle & 0x0FFF;  // 12 бит
  }
  return -1;
}

void updateEncoder(WheelControl* wheel) {
  int16_t angle = readAngle(wheel);

  if (angle == -1) {
    return;  // Ошибка чтения
  }

  int16_t diff = angle - wheel->prevAngle;

  // Обработка перехода через 0/4095
  if (diff > 2048) diff -= 4096;
  else if (diff < -2048) diff += 4096;

  // Направление: правый энкодер инвертирован относительно левого
  if (wheel->useWire1) {
    wheel->totalSteps += diff;   // Правое колесо
  } else {
    wheel->totalSteps -= diff;   // Левое колесо
  }

  wheel->prevAngle = angle;
}

void resetWheelEncoder(WheelControl* wheel) {
  wheel->totalSteps = 0;
  int16_t angle = readAngle(wheel);
  if (angle != -1) {
    wheel->prevAngle = angle;
  }
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================

void processCommand(String command, bool updateMotors) {
  if (command.startsWith("CHASSIS:LEFT:")) {
    if (command == "CHASSIS:LEFT:STOP") {
      stopWheel(&leftWheel, false, updateMotors);
    } else if (command.startsWith("CHASSIS:LEFT:ASK:")) {
      processWheelCommand(command, &leftWheel, true, false, updateMotors);
    } else if (command.startsWith("CHASSIS:LEFT:DESK:")) {
      processWheelCommand(command, &leftWheel, false, false, updateMotors);
    }
  } else if (command.startsWith("CHASSIS:RIGHT:")) {
    if (command == "CHASSIS:RIGHT:STOP") {
      stopWheel(&rightWheel, true, updateMotors);
    } else if (command.startsWith("CHASSIS:RIGHT:ASK:")) {
      processWheelCommand(command, &rightWheel, true, true, updateMotors);
    } else if (command.startsWith("CHASSIS:RIGHT:DESK:")) {
      processWheelCommand(command, &rightWheel, false, true, updateMotors);
    }
  } else if (command == "CHASSIS:STOP") {
    stopVelocityMode();
    stopWheel(&leftWheel, false, updateMotors);
    stopWheel(&rightWheel, true, updateMotors);
  } else if (command == "TEST:MOTORS") {
    // Прямой тест моторов без PID - 2 секунды вперёд
    Serial.println("Testing motors directly...");
    digitalWrite(MOTOR_L_DIR, HIGH);
    digitalWrite(MOTOR_R_DIR, LOW);   // Правый инвертирован
    ledcWrite(PWM_CHANNEL_L, 100);
    ledcWrite(PWM_CHANNEL_R, 100);
    delay(2000);
    ledcWrite(PWM_CHANNEL_L, 0);
    ledcWrite(PWM_CHANNEL_R, 0);
    Serial.println("Motors stopped");
  } else if (command == "TEST:LEFT") {
    Serial.println("Testing LEFT motor...");
    digitalWrite(MOTOR_L_DIR, HIGH);
    ledcWrite(PWM_CHANNEL_L, 100);
    delay(2000);
    ledcWrite(PWM_CHANNEL_L, 0);
    Serial.println("LEFT stopped");
  } else if (command == "TEST:RIGHT") {
    Serial.println("Testing RIGHT motor...");
    digitalWrite(MOTOR_R_DIR, LOW);   // Правый инвертирован
    ledcWrite(PWM_CHANNEL_R, 100);
    delay(2000);
    ledcWrite(PWM_CHANNEL_R, 0);
    Serial.println("RIGHT stopped");
  } else if (command.startsWith("CHASSIS:VEL:")) {
    // Формат: CHASSIS:VEL:linear:angular
    int firstColon = command.indexOf(':');
    int secondColon = command.indexOf(':', firstColon + 1);
    int thirdColon = command.indexOf(':', secondColon + 1);

    if (thirdColon != -1) {
      String linearStr = command.substring(secondColon + 1, thirdColon);
      String angularStr = command.substring(thirdColon + 1);
      processVelocityCommand(linearStr.toFloat(), angularStr.toFloat());
    }
  }
}

void processWheelCommand(String command, WheelControl* wheel, bool forward, bool isRight, bool updateMotors) {
  int firstColon = command.indexOf(':');
  int secondColon = command.indexOf(':', firstColon + 1);
  int thirdColon = command.indexOf(':', secondColon + 1);
  int fourthColon = command.indexOf(':', thirdColon + 1);

  if (fourthColon != -1) {
    String distanceStr = command.substring(thirdColon + 1, fourthColon);
    String pwmStr = command.substring(fourthColon + 1);

    wheel->desiredDistance = distanceStr.toFloat();
    int commandPWM = pwmStr.toInt();

    if (commandPWM >= 0 && commandPWM <= 255) {
      wheel->fixedPWM = commandPWM;
    }
  } else {
    String distanceStr = command.substring(thirdColon + 1);
    wheel->desiredDistance = distanceStr.toFloat();
  }

  // Установка направления
  if (forward) {
    wheel->currentPWM = isRight ? wheel->fixedPWM : -wheel->fixedPWM;
  } else {
    wheel->currentPWM = isRight ? -wheel->fixedPWM : wheel->fixedPWM;
  }

  resetWheelEncoder(wheel);
  wheel->isMoving = true;

  if (updateMotors) {
    updateMotorSpeeds();
  }
}

void checkWheelProgress(WheelControl* wheel, bool isRight) {
  float encoderRevolutions = wheel->totalSteps / 4096.0;
  float wheelRevolutions = encoderRevolutions / GEAR_RATIO;
  float traveledDistance = wheelRevolutions * WHEEL_CIRCUMFERENCE;
  float absTraveledDistance = abs(traveledDistance);
  float remainingDistance = wheel->desiredDistance - absTraveledDistance;

  // Плавное торможение
  if (remainingDistance <= 0.20 && remainingDistance > 0) {
    float brakeFactor = remainingDistance / 0.20;
    brakeFactor = constrain(brakeFactor, 0.0, 1.0);

    int newPWM = (int)(wheel->fixedPWM * brakeFactor);
    newPWM = max(newPWM, 40);

    wheel->currentPWM = (wheel->currentPWM > 0 ? newPWM : (wheel->currentPWM < 0 ? -newPWM : 0));
    updateMotorSpeeds();
  }

  if (absTraveledDistance >= wheel->desiredDistance) {
    stopWheel(wheel, isRight);
    wheel->isMoving = false;
  }
}

void stopWheel(WheelControl* wheel, bool isRight, bool updateMotors) {
  wheel->currentPWM = 0;
  wheel->isMoving = false;
  if (updateMotors) {
    updateMotorSpeeds();
  }
}

// ============================================================================
// VELOCITY MODE
// ============================================================================

void processVelocityCommand(float linearVel, float angularVel) {
  lastCmdTime = millis();

  leftWheel.isMoving = false;
  rightWheel.isMoving = false;

  if (abs(linearVel) < 0.01 && abs(angularVel) < 0.01) {
    stopVelocityMode();
    return;
  }

  // Дифференциальная кинематика
  float leftVel = linearVel - (angularVel * WHEEL_BASE / 2.0);
  float rightVel = linearVel + (angularVel * WHEEL_BASE / 2.0);

  // Нормализация если превышает максимум
  float maxWheelVel = max(abs(leftVel), abs(rightVel));
  if (maxWheelVel > MAX_VELOCITY) {
    leftVel = leftVel * MAX_VELOCITY / maxWheelVel;
    rightVel = rightVel * MAX_VELOCITY / maxWheelVel;
  }

  if (!velocityMode) {
    resetPID(&leftPID);
    resetPID(&rightPID);
    leftPID.lastSteps = leftWheel.totalSteps;
    rightPID.lastSteps = rightWheel.totalSteps;
    leftPID.lastTime = millis();
    rightPID.lastTime = millis();
  }

  leftPID.desiredVelocity = -leftVel;
  rightPID.desiredVelocity = rightVel;

  velocityMode = true;
}

void stopVelocityMode() {
  velocityMode = false;
  velocityLeftPWM = 0;
  velocityRightPWM = 0;
  leftPID.desiredVelocity = 0;
  rightPID.desiredVelocity = 0;
  resetPID(&leftPID);
  resetPID(&rightPID);
  stopMotors();
}

// ============================================================================
// PID CONTROLLER
// ============================================================================

void updatePID() {
  unsigned long now = millis();

  if (now - leftPID.lastTime < PID_INTERVAL) {
    return;
  }

  calculateWheelVelocity(&leftPID, leftWheel.totalSteps);
  calculateWheelVelocity(&rightPID, rightWheel.totalSteps);

  int targetLeftPWM = computePID(&leftPID);
  int targetRightPWM = computePID(&rightPID);

  // Плавное изменение PWM
  int leftChange = targetLeftPWM - velocityLeftPWM;
  int rightChange = targetRightPWM - velocityRightPWM;

  leftChange = constrain(leftChange, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
  rightChange = constrain(rightChange, -MAX_PWM_CHANGE, MAX_PWM_CHANGE);

  velocityLeftPWM += leftChange;
  velocityRightPWM += rightChange;

  setMotorLeft(velocityLeftPWM);
  setMotorRight(velocityRightPWM);
}

void calculateWheelVelocity(PIDState* pid, long currentSteps) {
  unsigned long now = millis();
  unsigned long dt = now - pid->lastTime;

  if (dt == 0) return;

  long deltaSteps = currentSteps - pid->lastSteps;
  float deltaMeters = deltaSteps * METERS_PER_STEP;
  float dtSeconds = dt / 1000.0;

  pid->actualVelocity = deltaMeters / dtSeconds;

  pid->lastSteps = currentSteps;
  pid->lastTime = now;
}

int computePID(PIDState* pid) {
  pid->error = pid->desiredVelocity - pid->actualVelocity;

  pid->integral += pid->error;
  pid->integral = constrain(pid->integral, -PID_MAX_INTEGRAL, PID_MAX_INTEGRAL);

  float derivative = pid->error - pid->lastError;
  pid->lastError = pid->error;

  float output = PID_KP * pid->error + PID_KI * pid->integral + PID_KD * derivative;

  pid->outputPWM = (int)output;

  // Мёртвая зона
  if (abs(pid->desiredVelocity) > 0.01 && pid->outputPWM != 0 && abs(pid->outputPWM) < MIN_PWM) {
    pid->outputPWM = (pid->outputPWM > 0) ? MIN_PWM : -MIN_PWM;
  }

  pid->outputPWM = constrain(pid->outputPWM, -MAX_PWM, MAX_PWM);

  return pid->outputPWM;
}

void resetPID(PIDState* pid) {
  pid->error = 0;
  pid->lastError = 0;
  pid->integral = 0;
  pid->outputPWM = 0;
  pid->actualVelocity = 0;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(MOTOR_L_DIR, OUTPUT);
  pinMode(MOTOR_R_DIR, OUTPUT);

  // PWM setup для ESP32
  ledcSetup(PWM_CHANNEL_L, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_PWM, PWM_CHANNEL_L);
  ledcAttachPin(MOTOR_R_PWM, PWM_CHANNEL_R);

  stopMotors();

  // I2C для левого энкодера (Wire - I2C0)
  Wire.begin(I2C0_SDA, I2C0_SCL);
  Wire.setClock(400000);  // 400 kHz Fast Mode

  // I2C для правого энкодера (Wire1 - I2C1)
  Wire1_custom.begin(I2C1_SDA, I2C1_SCL);
  Wire1_custom.setClock(400000);

  delay(100);

  // Инициализация начальных углов энкодеров
  leftWheel.prevAngle = readAngle(&leftWheel);
  rightWheel.prevAngle = readAngle(&rightWheel);

  #if DEBUG_ENABLED
  Serial.println("ESP32 Chassis ready (115200 baud)");
  Serial.println("Left encoder I2C0: SDA=21, SCL=22");
  Serial.println("Right encoder I2C1: SDA=32, SCL=4");
  #endif
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
  unsigned long now = millis();

  // Чтение энкодеров
  updateEncoder(&leftWheel);
  updateEncoder(&rightWheel);

  // Обработка Serial команд
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) continue;

    #if DEBUG_ENABLED
    Serial.print("CMD:");
    Serial.println(command);
    #endif

    // Множественные команды через ;
    int semicolonPos = command.indexOf(';');
    if (semicolonPos != -1) {
      processCommand(command.substring(0, semicolonPos), false);
      processCommand(command.substring(semicolonPos + 1), false);
      updateMotorSpeeds();
    } else {
      processCommand(command, true);
    }
  }

  // Проверка distance mode
  if (leftWheel.isMoving) {
    checkWheelProgress(&leftWheel, false);
  }
  if (rightWheel.isMoving) {
    checkWheelProgress(&rightWheel, true);
  }

  // Velocity mode: PID контроль
  if (velocityMode) {
    updatePID();
  }

  // Velocity mode: таймаут безопасности
  if (velocityMode && (now - lastCmdTime > CMD_TIMEOUT)) {
    #if DEBUG_ENABLED
    Serial.println("TIMEOUT");
    #endif
    stopVelocityMode();
  }

  // Публикация энкодеров (для ROS2 через chassis_node.py)
  if (now - lastEncoderSendTime >= ENCODER_INTERVAL) {
    Serial.print("ENC:");
    Serial.print(leftWheel.totalSteps);
    Serial.print(":");
    Serial.print(rightWheel.totalSteps);
    Serial.print(":");
    Serial.println(now);
    lastEncoderSendTime = now;
  }

  delay(5);
}
