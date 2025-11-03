#include <Wire.h>

// Адрес MPU будет определён в рантайме (0x68 или 0x69)
uint8_t g_mpuAddr = 0x68;
const int trigPin1 = 2;   
const int echoPin1 = 3;   
const int trigPin2 = 4;   
const int echoPin2 = 5;   
const int trigPin3 = 6;   
const int echoPin3 = 7;   
const int trigPin4 = 8;   
const int echoPin4 = 9;   
const int trigPin5 = 10;  
const int echoPin5 = 11;  
const int trigPin6 = 12;  
const int echoPin6 = 13;  
const int trigPin7 = 14;  
const int echoPin7 = 15;  

int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

// ===== Компас (HMC5883L/QMC5883L) =====
static const uint8_t HMC5883L_ADDR = 0x1E;
static const uint8_t QMC5883L_ADDR = 0x0D;
static uint8_t g_compassAddr = HMC5883L_ADDR;
static bool g_isQMC = false;
static bool g_infoPrinted = false;

void compassWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(g_compassAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

bool compassReadRegisters(uint8_t startReg, uint8_t count, uint8_t *buffer) {
  Wire.beginTransmission(g_compassAddr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t readBytes = Wire.requestFrom((int)g_compassAddr, (int)count, (int)true);
  if (readBytes != count) return false;
  for (uint8_t i = 0; i < count; i++) buffer[i] = Wire.read();
  return true;
}

void compassSetup() {
  // Детект HMC по ID 0x0A..0x0C = 'H','4','3'
  g_compassAddr = HMC5883L_ADDR;
  uint8_t id[3] = {0};
  bool hmcOk = false;
  if (compassReadRegisters(0x0A, 3, id)) {
    if (id[0] == 0x48 && id[1] == 0x34 && id[2] == 0x33) hmcOk = true;
  }
  if (!hmcOk) {
    g_compassAddr = QMC5883L_ADDR;
    g_isQMC = true;
  } else {
    g_isQMC = false;
  }

  if (g_isQMC) {
    // QMC5883L
    compassWriteRegister(0x0A, 0x80); // soft reset
    delay(10);
    compassWriteRegister(0x0B, 0x01); // set/reset period
    // OSR=128(00), RNG=2G(01), ODR=50Hz(10), MODE=CONT(01) => 0x19
    compassWriteRegister(0x09, 0x19);
  } else {
    // HMC5883L
    compassWriteRegister(0x00, 0x70); // 8 avg, 15Hz, normal
    compassWriteRegister(0x01, 0x20); // gain 1.3Ga
    compassWriteRegister(0x02, 0x00); // continuous
  }
  delay(10);
}

float readCompassHeadingDeg() {
  uint8_t d[6];
  int16_t x = 0, y = 0, z = 0;

  if (g_isQMC) {
    // QMC: 0x00..0x05 => X_L,X_H,Y_L,Y_H,Z_L,Z_H
    if (!compassReadRegisters(0x00, 6, d)) return -1.0f;
    x = (int16_t)((d[1] << 8) | d[0]);
    y = (int16_t)((d[3] << 8) | d[2]);
    z = (int16_t)((d[5] << 8) | d[4]);
  } else {
    // HMC: 0x03..0x08 => X_H,X_L,Z_H,Z_L,Y_H,Y_L
    if (!compassReadRegisters(0x03, 6, d)) return -1.0f;
    x = (int16_t)((d[0] << 8) | d[1]);
    z = (int16_t)((d[2] << 8) | d[3]);
    y = (int16_t)((d[4] << 8) | d[5]);
  }
  (void)z;

  float heading = atan2((float)y, (float)x) * 180.0f / PI;
  if (heading < 0) heading += 360.0f;
  return heading;
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(g_mpuAddr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(true);
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(g_mpuAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)g_mpuAddr, (int)1, (int)true);
  return Wire.read();
}

void readMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  Wire.beginTransmission(g_mpuAddr);
  Wire.write(0x3B);  // Регистр ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((int)g_mpuAddr, (int)14, (int)true);
  
  *ax = (Wire.read() << 8) | Wire.read();
  *ay = (Wire.read() << 8) | Wire.read();
  *az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // Пропускаем температуру
  *gx = (Wire.read() << 8) | Wire.read();
  *gy = (Wire.read() << 8) | Wire.read();
  *gz = (Wire.read() << 8) | Wire.read();
}

// Чтение WHO_AM_I по заданному адресу; возвращает true при успешном чтении
bool mpuReadWhoAmI(uint8_t addr, uint8_t* outVal) {
  Wire.beginTransmission(addr);
  Wire.write(0x75);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)1, (int)true) != 1) return false;
  *outVal = Wire.read();
  return true;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin1, OUTPUT); pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT); pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT); pinMode(echoPin4, INPUT);
  pinMode(trigPin5, OUTPUT); pinMode(echoPin5, INPUT);
  pinMode(trigPin6, OUTPUT); pinMode(echoPin6, INPUT);
  pinMode(trigPin7, OUTPUT); pinMode(echoPin7, INPUT);

  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C
  delay(100);
  Serial.println("Проверка MPU-6050...");
  // Автоопределение адреса MPU (0x68/0x69)
  uint8_t who68 = 0, who69 = 0; bool ok68 = false, ok69 = false;
  ok68 = mpuReadWhoAmI(0x68, &who68);
  ok69 = mpuReadWhoAmI(0x69, &who69);
  if (ok68 && (who68 == 0x68 || who68 == 0x69)) {
    g_mpuAddr = 0x68;
  } else if (ok69 && (who69 == 0x68 || who69 == 0x69)) {
    g_mpuAddr = 0x69;
  }
  
  // Проверка WHO_AM_I (регистр 0x75)
  uint8_t whoami = readRegister(0x75);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  
  if (whoami != 0x68) {
    Serial.println("ПРЕДУПРЕЖДЕНИЕ: WHO_AM_I != 0x68, но продолжаем инициализацию...");
    if (whoami == 0xFF || whoami == 0x00) {
      Serial.println("I2C(MPU) не отвечает: проверь SDA=22, SCL=23, GND, питание, AD0 адрес.");
    }
  }
  
  // Сброс устройства
  writeRegister(0x6B, 0x80);  // PWR_MGMT_1: Reset
  delay(100);
  
  // Выход из sleep mode
  writeRegister(0x6B, 0x00);  // PWR_MGMT_1: Wake up
  delay(10);
  
  // Настройка диапазонов
  writeRegister(0x1B, 0x00);  // GYRO_CONFIG: ±250°/s
  writeRegister(0x1C, 0x00);  // ACCEL_CONFIG: ±2g
  
  Serial.println("MPU-6050 инициализирован! Калибровка...");
  
  // Калибровка
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  
  for (int i = 0; i < 1000; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    delay(2);
  }
  
  ax_offset = ax_sum / 1000;
  ay_offset = ay_sum / 1000;
  az_offset = az_sum / 1000 - 16384;
  gx_offset = gx_sum / 1000;
  gy_offset = gy_sum / 1000;
  gz_offset = gz_sum / 1000;
  
  Serial.println("Калибровка завершена!");

  // Компас
  compassSetup();
  Serial.print("COMPASS:");
  if (g_isQMC) Serial.println("QMC5883L @0x0D");
  else Serial.println("HMC5883L @0x1E");
  g_infoPrinted = true;
}

void loop() {
  // ========== ДАТЧИКИ РАССТОЯНИЯ ==========
  
  // Датчик 1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin1, LOW);
  long duration1 = pulseIn(echoPin1, HIGH, 50000);
  float distance1 = (duration1 * 0.034) / 2;

  // Датчик 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin2, LOW);
  long duration2 = pulseIn(echoPin2, HIGH, 50000);
  float distance2 = (duration2 * 0.034) / 2;

  // Датчик 3
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin3, LOW);
  long duration3 = pulseIn(echoPin3, HIGH, 50000);
  float distance3 = (duration3 * 0.034) / 2;

  // Датчик 4
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin4, LOW);
  long duration4 = pulseIn(echoPin4, HIGH, 50000);
  float distance4 = (duration4 * 0.034) / 2;

  // Датчик 5
  digitalWrite(trigPin5, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin5, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin5, LOW);
  long duration5 = pulseIn(echoPin5, HIGH, 50000);
  float distance5 = (duration5 * 0.034) / 2;

  // Датчик 6
  digitalWrite(trigPin6, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin6, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin6, LOW);
  long duration6 = pulseIn(echoPin6, HIGH, 50000);
  float distance6 = (duration6 * 0.034) / 2;

  // Датчик 7
  digitalWrite(trigPin7, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin7, HIGH);
  delayMicroseconds(50);
  digitalWrite(trigPin7, LOW);
  long duration7 = pulseIn(echoPin7, HIGH, 50000);
  float distance7 = (duration7 * 0.034) / 2;

  // ========== АКСЕЛЕРОМЕТР MPU-6050 ==========
  
  int16_t ax, ay, az, gx, gy, gz;
  readMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Применяем калибровку
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;

  // ========== ВЫВОД В ФОРМАТЕ DISTANCE:..._ACCELEROMETR:..._ ==========
  
  Serial.print("DISTANCE:");
  
  // Датчик 1
  if (duration1 == 0 || distance1 > 450 || distance1 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance1);
  }
  Serial.print(":");
  
  // Датчик 2
  if (duration2 == 0 || distance2 > 450 || distance2 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance2);
  }
  Serial.print(":");
  
  // Датчик 3
  if (duration3 == 0 || distance3 > 450 || distance3 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance3);
  }
  Serial.print(":");
  
  // Датчик 4
  if (duration4 == 0 || distance4 > 450 || distance4 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance4);
  }
  Serial.print(":");
  
  // Датчик 5
  if (duration5 == 0 || distance5 > 450 || distance5 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance5);
  }
  Serial.print(":");
  
  // Датчик 6
  if (duration6 == 0 || distance6 > 450 || distance6 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance6);
  }
  Serial.print(":");
  
  // Датчик 7
  if (duration7 == 0 || distance7 > 450 || distance7 < 2) {
    Serial.print("999");
  } else {
    Serial.print((int)distance7);
  }
  
  // Акселерометр
  Serial.print("_ACCELEROMETR:");
  Serial.print(ax); Serial.print(":");
  Serial.print(ay); Serial.print(":");
  Serial.print(az); Serial.print(":");
  Serial.print(gx); Serial.print(":");
  Serial.print(gy); Serial.print(":");
  Serial.println(gz);

  // Компас
  float heading = readCompassHeadingDeg();
  Serial.print("_COMPASS:");
  if (heading < 0) Serial.println("999");
  else Serial.println((int)heading);

  delay(300);  // Пауза 300 мс между циклами
}