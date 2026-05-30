/*
 * Проверка фикса endTransmission(true) vs endTransmission(false)
 * Прошить на chassis ESP32 (где энкодеры)
 * Serial Monitor @ 115200
 */

#include <Wire.h>

#define I2C0_SDA      21
#define I2C0_SCL      22
#define I2C1_SDA      32
#define I2C1_SCL      4
#define AS5600_ADDR   0x36
#define RAW_ANGLE_REG 0x0C

TwoWire Wire1_custom = TwoWire(1);

unsigned long testRead(TwoWire* wire, int16_t* angle, bool useStop) {
  unsigned long t0 = millis();
  wire->beginTransmission(AS5600_ADDR);
  wire->write(RAW_ANGLE_REG);
  if (wire->endTransmission(useStop) != 0) { *angle = -1; return millis() - t0; }
  if (wire->requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) == 2) {
    *angle = (wire->read() << 8) | wire->read();
    *angle &= 0x0FFF;
  } else { *angle = -1; }
  return millis() - t0;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(I2C0_SDA, I2C0_SCL);
  Wire.setClock(400000);
  Wire1_custom.begin(I2C1_SDA, I2C1_SCL);
  Wire1_custom.setClock(400000);

  Serial.println("\n=== endTransmission(false) vs (true) ===\n");

  int16_t a;
  Serial.println("--- Wire (I2C0, SDA=21 SCL=22) с REPEATED START (false) ---");
  for (int i = 0; i < 3; i++) {
    unsigned long t = testRead(&Wire, &a, false);
    Serial.printf("  %4dms  angle=%d\n", (int)t, a);
  }

  Serial.println("\n--- Wire (I2C0, SDA=21 SCL=22) с STOP (true) ---");
  Wire.begin(I2C0_SDA, I2C0_SCL);  // reinit после зависания
  Wire.setClock(400000);
  delay(50);
  for (int i = 0; i < 3; i++) {
    unsigned long t = testRead(&Wire, &a, true);
    Serial.printf("  %4dms  angle=%d\n", (int)t, a);
  }

  Serial.println("\n--- Wire1 (I2C1, SDA=32 SCL=4) с REPEATED START (false) ---");
  for (int i = 0; i < 3; i++) {
    unsigned long t = testRead(&Wire1_custom, &a, false);
    Serial.printf("  %4dms  angle=%d\n", (int)t, a);
  }

  Serial.println("\n--- Wire1 (I2C1, SDA=32 SCL=4) с STOP (true) ---");
  for (int i = 0; i < 3; i++) {
    unsigned long t = testRead(&Wire1_custom, &a, true);
    Serial.printf("  %4dms  angle=%d\n", (int)t, a);
  }

  Serial.println("\n=== ГОТОВО ===");
  Serial.println("Если Wire+STOP показывает 1ms — фикс работает.");
}

void loop() {}
