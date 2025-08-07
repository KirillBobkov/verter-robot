/*
 * Arduino скетч для работы с ROS2 через узел arduino_proxy_node
 * Расширенная версия: измерение расстояния и команды движения (FORWARD, STOP, LEFT, RIGHT)
 */

// Пины для управления левым мотором
const int ENA = 9;   // ШИМ пин скорости левого мотора
const int IN1 = 2;   // Направление 1 для левого мотора
const int IN2 = 3;   // Направление 2 для левого мотора

// Пины для управления правым мотором
const int ENB = 6;   // ШИМ пин скорости правого мотора (обновлено, чтобы не конфликтовало с IN2)
const int IN3 = 5;   // Направление 1 для правого мотора
const int IN4 = 4;   // Направление 2 для правого мотора

// Пины для датчика расстояния HC-SR04
const int TRIG_PIN = 29;
const int ECHO_PIN = 28;

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

// Интервал отправки данных с датчика (в миллисекундах)
const long MEASURE_INTERVAL = 100;

// Максимальное время для измерения (23300 мкс = 4 м)
const long MAX_DURATION = 23300;

// Таймер для периодической отправки данных с датчика
unsigned long lastDistanceMeasurement = 0;

// Буфер для хранения входящих команд
String inputBuffer = "";
boolean commandComplete = false;

void setup() {
  Serial.begin(BAUD_RATE);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

 // pinMode(TRIG_PIN, OUTPUT);
//  pinMode(ECHO_PIN, INPUT);

  stopMotors();
  Serial.println("STATUS:0:Arduino initialized");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastDistanceMeasurement >= MEASURE_INTERVAL) {
    lastDistanceMeasurement = currentMillis;
    float distance = measureDistance();
    if (distance >= 0) {
      Serial.print("DIST:");
      Serial.println(distance);
    }
  }

  readSerialCommands();

  if (commandComplete) {
    processCommand(inputBuffer);
    inputBuffer = "";
    commandComplete = false;
  }
}

float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) {
    Serial.println("ERROR:1:Echo not received");
    return -1;
  } else if (duration > MAX_DURATION) {
    Serial.println("ERROR:2:Distance out of range");
    return -2;
  } else {
    float distance = (duration * 0.0343) / 2;
    return distance;
  }
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
  } else if (cmdType == "GET") {
    if (params == "DISTANCE") {
      float distance = measureDistance();
      if (distance >= 0) {
        Serial.print("DIST:");
        Serial.println(distance);
      }
    } else if (params == "STATUS") {
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
  else {
    Serial.print("ERROR:5:Unsupported chassis action: ");
    Serial.println(action);
  }
}

void moveForward(int speed) {
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
      delay(ACCELERATION_DELAY);
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
      delay(ACCELERATION_DELAY);
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
      delay(DECELERATION_DELAY);
    }
  }
  
  // Полная остановка
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void turnLeft() {
  int turnTime = 4000; // Подбери точно для 90° поворота

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
      delay(ACCELERATION_DELAY);
    }
  }
  
  // Поворот на полной скорости (учитываем только время разгона)
  int accelerationTime = (MAX_PWM_ROTATE_SPEED / ACCELERATION_STEP * ACCELERATION_DELAY);
  int fullSpeedTime = turnTime - accelerationTime;
  if (fullSpeedTime > 0) {
    delay(fullSpeedTime);
  }
  
  stopMotors(); // Уже содержит плавную остановку
}

void turnRight() {
  int turnTime = 4000; // Подбери точно для 90° поворота

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
      delay(ACCELERATION_DELAY);
    }
  }
  
  // Поворот на полной скорости (учитываем только время разгона)
  int accelerationTime = (MAX_PWM_ROTATE_SPEED / ACCELERATION_STEP * ACCELERATION_DELAY);
  int fullSpeedTime = turnTime - accelerationTime;
  if (fullSpeedTime > 0) {
    delay(fullSpeedTime);
  }
  
  stopMotors(); // Уже содержит плавную остановку
}