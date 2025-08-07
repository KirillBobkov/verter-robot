#include <Servo.h>
#include <SoftwareWire.h>

// Пины - попробуем разные варианты
const int SERVO_PIN = 8;
const int SDA_PIN = 20;  // Стандартный I2C SDA для многих плат
const int SCL_PIN = 21;  // Стандартный I2C SCL для многих плат
// Альтернативные пины если не работает:
// const int SDA_PIN = 31;
// const int SCL_PIN = 30;

// MPU6050 регистры и адреса
#define MPU6050_ADDR 0x68      // Основной адрес
#define MPU6050_ADDR_ALT 0x69  // Альтернативный адрес (если AD0 подключен к VCC)
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define WHO_AM_I 0x75

// Объекты
Servo shoulderServo;
SoftwareWire myWire(SDA_PIN, SCL_PIN);

// Переменные для управления движением
float currentRealAngle = 0;     // Реальный угол от акселерометра
float baseAngle = 0;            // Базовый угол (калибровка)
float targetAngle = 0;          // Целевой угол
unsigned long stateStartTime = 0;
bool mpuWorking = false;        // Флаг работы MPU6050
uint8_t mpuAddress = MPU6050_ADDR; // Используемый адрес

// Состояния движения
enum MovementState {
  MOVING_UP,
  PAUSE_UP,
  MOVING_DOWN,
  PAUSE_DOWN
};
MovementState currentState = MOVING_UP;

// Константы
const float MOVE_ANGLE = 10.0;          // Угол движения в градусах
const unsigned long PAUSE_TIME = 5000;  // Пауза 5 секунд
const float ACCEL_SENSITIVITY = 16384.0; // Чувствительность акселерометра (LSB/g)

void setup() {
  Serial.begin(115200);
  Serial.println("Инициализация системы управления плечом с акселерометром...");
  
  // Настройка SoftwareWire и MPU6050
  myWire.begin();
  myWire.setClock(100000); // 100kHz
  delay(100);
  
  Serial.println("=== ДИАГНОСТИКА I2C ===");
  Serial.print("SDA пин: ");
  Serial.println(SDA_PIN);
  Serial.print("SCL пин: ");
  Serial.println(SCL_PIN);
  Serial.println("Сканирование I2C шины...");
  
  scanI2C();
  
  Serial.println("=== ИНИЦИАЛИЗАЦИЯ MPU6050 ===");
  initMPU6050();
  
  if (testMPU6050Connection()) {
    Serial.println("MPU6050 подключен");
    mpuWorking = true;
  } else {
    Serial.println("КРИТИЧЕСКАЯ ОШИБКА: MPU6050 не подключен!");
    Serial.println();
    Serial.println("=== ПРОВЕРЬТЕ ПОДКЛЮЧЕНИЕ ===");
    Serial.print("1. MPU6050 VCC -> 3.3V (НЕ 5V!) - проверьте мультиметром: ");
    Serial.print(3.3);
    Serial.println("V");
    Serial.println("2. MPU6050 GND -> GND - проверьте мультиметром: 0V");
    Serial.print("3. MPU6050 SDA -> пин ");
    Serial.println(SDA_PIN);
    Serial.print("4. MPU6050 SCL -> пин ");
    Serial.println(SCL_PIN);
    Serial.println("5. Подтягивающие резисторы 4.7кОм на SDA/SCL к 3.3V");
    Serial.println("6. Убедитесь что MPU6050 исправен");
    Serial.println("7. Проверьте качество пайки/контактов");
    Serial.println();
    Serial.println("ОТСУТСТВУЮТ ВСЕ I2C УСТРОЙСТВА!");
    Serial.println("Проверьте питание и провода SDA/SCL.");
    Serial.println();
    Serial.println("=== ПОПРОБУЙТЕ ИЗМЕНИТЬ ПИНЫ ===");
    Serial.println("В коде поменяйте:");
    Serial.println("const int SDA_PIN = 20;  -> другой пин");
    Serial.println("const int SCL_PIN = 21;  -> другой пин");
    Serial.println("Стандартные I2C пины зависят от платы Arduino!");
    Serial.println();
    
    // Останавливаем систему
    while (true) {
      delay(5000);
      Serial.println("Ошибка MPU6050 - проверьте подключение и пины");
    }
  }
  
  // Инициализация серво
  shoulderServo.attach(SERVO_PIN);
  shoulderServo.write(90); // Начальное положение
  
  delay(2000);
  
  // Калибровка - определяем базовый угол
  calibrateBaseAngle();
  
  // Проверяем успешность калибровки
  if (!mpuWorking) {
    Serial.println("КРИТИЧЕСКАЯ ОШИБКА: Калибровка не удалась!");
    Serial.println("Система остановлена.");
    
    while (true) {
      delay(1000);
      Serial.println("Ошибка калибровки - система остановлена");
    }
  }
  
  // Инициализация переменных
  targetAngle = MOVE_ANGLE;
  stateStartTime = millis();
  
  Serial.println("Система готова! Начинаем цикл движения...");
}

void loop() {
  // Читаем данные акселерометра и вычисляем угол
  if (!updateAngleFromAccel()) {
    Serial.println("КРИТИЧЕСКАЯ ОШИБКА: MPU6050 перестал отвечать!");
    Serial.println("Останавливаем серво и систему...");
    
    // Возвращаем серво в безопасное положение
    shoulderServo.write(90);
    
    // Останавливаем систему
    while (true) {
      delay(1000);
      Serial.println("Потеря связи с MPU6050 - система остановлена");
    }
  }
  
  // Управляем движением согласно состоянию
  handleMovement();
  
  // Выводим данные для отладки
  printDebugInfo();
  
  delay(50);
}

void calibrateBaseAngle() {
  Serial.println("Калибровка базового угла...");
  float angleSum = 0;
  int samples = 100;
  int validSamples = 0;
  
  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    if (readAccelData(&ax, &ay, &az)) {
      // Для кисти при согнутом локте 90° используем ax и az
      // ax показывает наклон кисти при движении плеча
      float angle = atan2(ax, az) * 180.0 / PI;
      angleSum += angle;
      validSamples++;
      
      Serial.print("Калибровка ");
      Serial.print(i + 1);
      Serial.print("/");
      Serial.print(samples);
      Serial.print(": ax=");
      Serial.print(ax);
      Serial.print(", az=");
      Serial.print(az);
      Serial.print(", угол=");
      Serial.println(angle, 1);
    } else {
      Serial.print("Ошибка чтения при калибровке ");
      Serial.println(i + 1);
    }
    
    delay(10);
  }
  
  if (validSamples > 0) {
    baseAngle = angleSum / validSamples;
    Serial.print("Базовый угол (");
    Serial.print(validSamples);
    Serial.print(" замеров): ");
    Serial.print(baseAngle, 1);
    Serial.println("°");
  } else {
    Serial.println("Калибровка не удалась!");
    mpuWorking = false;
  }
}

bool updateAngleFromAccel() {
  int16_t ax, ay, az;
  if (readAccelData(&ax, &ay, &az)) {
    // Для кисти при согнутом локте 90°:
    // ax - показывает наклон вперед/назад (движение плеча)
    // az - вертикальная составляющая
    float rawAngle = atan2(ax, az) * 180.0 / PI;
    
    // Применяем калибровку
    currentRealAngle = rawAngle - baseAngle;
    return true;
  } else {
    Serial.println("Ошибка чтения акселерометра!");
    return false;
  }
}

void handleMovement() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - stateStartTime;
  
  switch (currentState) {
    case MOVING_UP:
      moveToTarget(targetAngle);
      if (abs(currentRealAngle - targetAngle) < 2.0) { // Увеличил допуск
        Serial.println("Достигли верхней позиции, пауза 5 сек");
        currentState = PAUSE_UP;
        stateStartTime = currentTime;
      }
      break;
      
    case PAUSE_UP:
      holdPosition(targetAngle);
      if (elapsedTime >= PAUSE_TIME) {
        targetAngle = -MOVE_ANGLE;
        currentState = MOVING_DOWN;
        stateStartTime = currentTime;
        Serial.println("Начинаем движение вниз");
      }
      break;
      
    case MOVING_DOWN:
      moveToTarget(targetAngle);
      if (abs(currentRealAngle - targetAngle) < 2.0) { // Увеличил допуск
        Serial.println("Достигли нижней позиции, пауза 5 сек");
        currentState = PAUSE_DOWN;
        stateStartTime = currentTime;
      }
      break;
      
    case PAUSE_DOWN:
      holdPosition(targetAngle);
      if (elapsedTime >= PAUSE_TIME) {
        targetAngle = MOVE_ANGLE;
        currentState = MOVING_UP;
        stateStartTime = currentTime;
        Serial.println("Начинаем движение вверх");
      }
      break;
  }
}

void moveToTarget(float target) {
  float error = target - currentRealAngle;
  
  // Простой пропорциональный контроллер
  float kP = 3.0; // Коэффициент пропорциональности
  float servoCommand = 90 + target + (error * kP);
  
  // Ограничиваем команду серво
  servoCommand = constrain(servoCommand, 10, 170);
  
  shoulderServo.write((int)servoCommand);
}

void holdPosition(float target) {
  // Удерживаем позицию с небольшой коррекцией
  float error = target - currentRealAngle;
  float kP = 1.0; // Меньший коэффициент для удержания
  float servoCommand = 90 + target + (error * kP);
  
  servoCommand = constrain(servoCommand, 10, 170);
  shoulderServo.write((int)servoCommand);
}

void printDebugInfo() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) { // Печатаем каждые 500мс
    Serial.print("MPU: ");
    Serial.print(mpuWorking ? "OK" : "FAIL");
    Serial.print(", Реальный угол: ");
    Serial.print(currentRealAngle, 1);
    Serial.print("°, Целевой: ");
    Serial.print(targetAngle, 1);
    Serial.print("°, Состояние: ");
    
    switch (currentState) {
      case MOVING_UP: Serial.println("ДВИЖЕНИЕ ВВЕРХ"); break;
      case PAUSE_UP: Serial.println("ПАУЗА ВВЕРХУ"); break;
      case MOVING_DOWN: Serial.println("ДВИЖЕНИЕ ВНИЗ"); break;
      case PAUSE_DOWN: Serial.println("ПАУЗА ВНИЗУ"); break;
    }
    
    lastPrint = millis();
  }
}

void scanI2C() {
  int devicesFound = 0;
  
  Serial.println("Поиск I2C устройств...");
  for (byte address = 1; address < 127; address++) {
    myWire.beginTransmission(address);
    byte error = myWire.endTransmission();
    
    if (error == 0) {
      Serial.print("Найдено I2C устройство по адресу 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      if (address == 0x68) {
        Serial.print(" (MPU6050 адрес 0x68!)");
        mpuAddress = 0x68;
      } else if (address == 0x69) {
        Serial.print(" (MPU6050 адрес 0x69!)");
        mpuAddress = 0x69;
      }
      Serial.println();
      devicesFound++;
    }
    delay(10);
  }
  
  if (devicesFound == 0) {
    Serial.println("I2C устройства НЕ найдены!");
    Serial.println("Проблема с I2C шиной или подключением.");
    Serial.println("ВОЗМОЖНЫЕ ПРИЧИНЫ:");
    Serial.println("1. Неправильные пины I2C");
    Serial.println("2. Нет питания на MPU6050");
    Serial.println("3. Нет подтягивающих резисторов");
    Serial.println("4. Плохие контакты/провода");
  } else {
    Serial.print("Найдено устройств: ");
    Serial.println(devicesFound);
  }
  Serial.println();
}

void initMPU6050() {
  Serial.println("Инициализация MPU6050...");
  
  // Несколько попыток инициализации для обоих адресов
  uint8_t addresses[] = {0x68, 0x69};
  
  for (int addr = 0; addr < 2; addr++) {
    Serial.print("Проверяем адрес 0x");
    Serial.println(addresses[addr], HEX);
    
    for (int i = 0; i < 3; i++) {
      writeMPU6050Register(addresses[addr], PWR_MGMT_1, 0x00);
      delay(100);
      
      uint8_t whoami = readMPU6050Register(addresses[addr], WHO_AM_I);
      Serial.print("  Попытка ");
      Serial.print(i + 1);
      Serial.print(": WHO_AM_I = 0x");
      Serial.println(whoami, HEX);
      
      if (whoami == 0x68) {
        Serial.println("  MPU6050 найден!");
        mpuAddress = addresses[addr];
        return;
      }
      delay(100);
    }
  }
  
  Serial.println("MPU6050 не отвечает на обоих адресах");
}

bool testMPU6050Connection() {
  // Проверяем оба возможных адреса
  uint8_t addresses[] = {0x68, 0x69};
  
  for (int i = 0; i < 2; i++) {
    myWire.beginTransmission(addresses[i]);
    byte error = myWire.endTransmission();
    Serial.print("I2C тест адрес 0x");
    Serial.print(addresses[i], HEX);
    Serial.print(": ");
    Serial.println(error == 0 ? "OK" : "FAIL");
    
    if (error == 0) {
      mpuAddress = addresses[i];
      return true;
    }
  }
  return false;
}

void writeMPU6050Register(uint8_t deviceAddr, uint8_t reg, uint8_t value) {
  myWire.beginTransmission(deviceAddr);
  myWire.write(reg);
  myWire.write(value);
  myWire.endTransmission();
}

uint8_t readMPU6050Register(uint8_t deviceAddr, uint8_t reg) {
  myWire.beginTransmission(deviceAddr);
  myWire.write(reg);
  myWire.endTransmission(false);
  myWire.requestFrom((uint8_t)deviceAddr, (uint8_t)1);
  
  if (myWire.available()) {
    return myWire.read();
  }
  return 0;
}

bool readAccelData(int16_t* ax, int16_t* ay, int16_t* az) {
  myWire.beginTransmission(mpuAddress);
  myWire.write(ACCEL_XOUT_H);
  byte error = myWire.endTransmission(false);
  
  if (error != 0) {
    Serial.print("Ошибка I2C при записи: ");
    Serial.println(error);
    return false;
  }
  
  int bytesReceived = myWire.requestFrom((uint8_t)mpuAddress, (uint8_t)6);
  
  if (bytesReceived != 6) {
    Serial.print("Получено байт: ");
    Serial.print(bytesReceived);
    Serial.println(" вместо 6");
    return false;
  }
  
  // Читаем 6 байт данных акселерометра
  *ax = (myWire.read() << 8) | myWire.read();
  *ay = (myWire.read() << 8) | myWire.read();
  *az = (myWire.read() << 8) | myWire.read();
  
  return true;
} 