#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "decode.h"
#include <iarduino_Position_BMX055.h>
iarduino_Position_BMX055 A(BMA), G(BMG), M(BMM);
// Read-only sensor-register diagnostic. No ROS, motors, ultrasound, BLE or calibration writes.
uint32_t errors = 0, samples = 0, last = 0, lastStatus = 0;
uint8_t acc = 0, gyro = 0;
bool streaming = false;
char command[24];
size_t used = 0;
bool readRegs(uint8_t addr, uint8_t reg, uint8_t *dst, size_t count) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t rc = Wire.endTransmission(false);
    if (rc) {
        ++errors;
        Serial.printf("ERROR addr=%02X reg=%02X phase=pointer code=%u\n", addr, reg, rc);
        return false;
    }
    size_t got = Wire.requestFrom(addr, count, true);
    if (got != count) {
        while (Wire.available())
            Wire.read();
        ++errors;
        Serial.printf("ERROR addr=%02X reg=%02X phase=read got=%u expected=%u\n", addr, reg, unsigned(got),
                      unsigned(count));
        return false;
    }
    for (size_t i = 0; i < count; i++)
        dst[i] = Wire.read();
    return true;
}
uint8_t detect(const char *name, uint8_t a, uint8_t b, uint8_t expected) {
    uint8_t found = 0;
    for (uint8_t addr : {a, b}) {
        uint8_t id = 0;
        if (readRegs(addr, 0, &id, 1)) {
            Serial.printf("ID %s addr=%02X value=%02X expected=%02X\n", name, addr, id, expected);
            if (id == expected) {
                if (found) {
                    Serial.printf("ERROR ambiguous %s\n", name);
                    return 0;
                }
                found = addr;
            }
        }
    }
    return found;
}
void dump(uint8_t addr, const char *name) {
    if (!addr)
        return;
    const uint8_t regs[] = {0x00, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x31, 0x32, 0x33, 0x36, 0x37, 0x38, 0x39, 0x3A};
    for (uint8_t reg : regs) {
        uint8_t v;
        if (readRegs(addr, reg, &v, 1))
            Serial.printf("REG %s %02X=%02X\n", name, reg, v);
    }
    if (addr == acc) {
        uint8_t v;
        if (readRegs(addr, 0x36, &v, 1))
            Serial.printf("CAL accel_ready=%d (not proof of successful prior calibration)\n", accelReady(v));
    }
    if (addr == gyro) {
        uint8_t v;
        if (readRegs(addr, 0x32, &v, 1))
            Serial.printf("CAL gyro_busy=%d (idle is not proof of calibration)\n", gyroBusy(v));
    }
}
void sample() {
    ++samples;
    for (uint8_t addr : {acc, gyro}) {
        if (!addr)
            continue;
        uint8_t raw[6], range;
        if (!readRegs(addr, 0x0F, &range, 1) || !readRegs(addr, 2, raw, 6))
            continue;
        const bool isAcc = addr == acc;
        float fs = isAcc ? accelScale(range) : gyroScale(range);
        int v[3];
        float si[3];
        for (int i = 0; i < 3; i++) {
            v[i] = isAcc ? signed12(raw[2 * i], raw[2 * i + 1]) : signed16(raw[2 * i], raw[2 * i + 1]);
            si[i] = v[i] * fs * (isAcc ? 9.81f / 2048.0f : PI / 180.0f / 32768.0f);
        }
        Serial.printf("SAMPLE ms=%lu sensor=%s range=%02X raw=%d,%d,%d", (unsigned long)millis(),
                      isAcc ? "accel" : "gyro", range, v[0], v[1], v[2]);
        if (fs)
            Serial.printf(" si=%.7f,%.7f,%.7f norm=%.7f", si[0], si[1], si[2],
                          sqrtf(si[0] * si[0] + si[1] * si[1] + si[2] * si[2]));
        else
            Serial.print(" scale=UNKNOWN");
        if (isAcc)
            Serial.printf(" fresh_xyz=%d%d%d", raw[0] & 1, raw[2] & 1, raw[4] & 1);
        Serial.println();
    }
}
void status() {
    Serial.printf("STATUS acc=%02X gyro=%02X samples=%lu errors=%lu stream=%d\n", acc, gyro, (unsigned long)samples,
                  (unsigned long)errors, streaming);
}

bool put(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    int rc = Wire.endTransmission();
    Serial.printf("WRITE addr=%02X reg=%02X value=%02X rc=%d\n", addr, reg, value, rc);
    if (rc)
        ++errors;
    return rc == 0;
}
bool expect(uint8_t addr, uint8_t reg, uint8_t value, uint8_t mask = 0xFF) {
    uint8_t v = 0;
    bool ok = readRegs(addr, reg, &v, 1) && (v & mask) == value;
    Serial.printf("CHECK addr=%02X reg=%02X expected=%02X actual=%02X ok=%d\n", addr, reg, value, v, ok);
    return ok;
}
bool waitBit(uint8_t addr, uint8_t reg, uint8_t mask, bool set, uint32_t timeout) {
    uint32_t start = millis();
    do {
        uint8_t v;
        if (!readRegs(addr, reg, &v, 1))
            return false;
        if (bool(v & mask) == set) {
            Serial.printf("CAL_DONE addr=%02X reg=%02X ms=%lu value=%02X\n", addr, reg,
                          (unsigned long)(millis() - start), v);
            return true;
        }
        delay(10);
    } while (millis() - start < timeout);
    Serial.println("ERROR calibration timeout");
    return false;
}
void window(const char *phase, int count, int interval) {
    Serial.printf("PHASE %s\n", phase);
    for (int i = 0; i < count; i++) {
        sample();
        delay(interval);
    }
}
void staged(uint32_t hz) {
    streaming = false;
    if (acc != 0x19 || gyro != 0x69) {
        Serial.println("RUN_ABORT IDs");
        return;
    }
    Serial.printf("RUN_BEGIN hz=%lu\n", (unsigned long)hz);
    Wire.setClock(hz);
    bool a = A.begin();
    Serial.printf("BEGIN A=%d\n", a);
    dump(acc, "BMA");
    bool g = G.begin();
    Serial.printf("BEGIN G=%d\n", g);
    dump(gyro, "BMG");
    bool m = M.begin();
    Serial.printf("BEGIN M=%d\n", m);
    dump(acc, "BMA");
    dump(gyro, "BMG");
    if (!a || !g || !m) {
        Serial.println("RUN_ABORT begin");
        return;
    }
    A.setScale(BMA_4G);
    G.setScale(BMG_500DPS);
    M.setScale(BMM_REGULAR);
    A.setBandwidths(BMA_125Hz);
    G.setBandwidths(BMG_23Hz);
    M.setBandwidths(BMM_10Hz);
    bool ok = expect(acc, 0x0F, 5, 15);
    ok = expect(gyro, 0x0F, 2, 7) && ok;
    ok = expect(acc, 0x10, 0x0C, 31) && ok;
    ok = expect(gyro, 0x10, 4, 7) && ok;
    Serial.printf("CLOCK actual=%lu\n", (unsigned long)Wire.getClock());
    if (!ok) {
        Serial.println("RUN_ABORT readback");
        return;
    }
    delay(10000);
    window("before_cal", 50, 100);
    // Same targets as existing firmware, but every transfer and completion checked; no NVM writes.
    if (!put(acc, 0x36, 0x80) || !put(acc, 0x37, 0x20)) {
        Serial.println("RUN_ABORT acc_write");
        return;
    }
    for (uint8_t trigger : {0x20, 0x40, 0x60}) {
        if (!put(acc, 0x36, trigger) || !waitBit(acc, 0x36, 0x10, true, 1500)) {
            Serial.println("RUN_ABORT acc_cal");
            return;
        }
    }
    if (!put(gyro, 0x32, 0xFF) || !waitBit(gyro, 0x32, 8, false, 5000)) {
        Serial.println("RUN_ABORT gyro_cal");
        return;
    }
    dump(acc, "BMA");
    dump(gyro, "BMG");
    window("after_hardware_cal", 50, 100);
    Serial.println("PHASE software_bias");
    double sum[3] = {};
    int valid = 0, failed = 0;
    for (int i = 0; i < 2048; i++) {
        if (G.read(BMG_RAD_S)) {
            sum[0] += G.axisX;
            sum[1] += G.axisY;
            sum[2] += G.axisZ;
            ++valid;
        } else
            ++failed;
        delay(10);
    }
    if (valid < 1900) {
        Serial.printf("RUN_ABORT bias valid=%d failed=%d\n", valid, failed);
        return;
    }
    for (auto &v : sum)
        v /= valid;
    Serial.printf("BIAS valid=%d failed=%d xyz=%.9f,%.9f,%.9f\n", valid, failed, sum[0], sum[1], sum[2]);
    Serial.println("PHASE corrected");
    for (int i = 0; i < 500; i++) {
        bool ar = A.read(BMA_M_S), gr = G.read(BMG_RAD_S);
        if (ar && gr)
            Serial.printf("CORRECTED a=%.7f,%.7f,%.7f g=%.9f,%.9f,%.9f\n", A.axisX, A.axisY, A.axisZ, G.axisX - sum[0],
                          G.axisY - sum[1], G.axisZ - sum[2]);
        else
            Serial.printf("READ_FAILED a=%d g=%d\n", ar, gr);
        delay(20);
    }
    dump(acc, "BMA");
    dump(gyro, "BMG");
    Serial.printf("RUN_END hz=%lu errors=%lu\n", (unsigned long)hz, (unsigned long)errors);
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    Wire.setClock(100000);
    Wire.setTimeOut(50);
    Serial.println("BOOT imu_probe SDA=21 SCL=22 I2C=100000 staged_init=1 no_ROS=1");
    acc = detect("BMA", 0x18, 0x19, 0xFA);
    gyro = detect("BMG", 0x68, 0x69, 0x0F);
    dump(acc, "BMA");
    dump(gyro, "BMG");
    status();
    Serial.println(
        "Commands: STATUS DUMP SAMPLE START STOP. RUN100 RUN400 run staged initialization and volatile calibration; no NVM writes.");
}
void loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r')
            continue;
        if (c == '\n') {
            command[used] = 0;
            if (!strcmp(command, "RUN100"))
                staged(100000);
            else if (!strcmp(command, "RUN400"))
                staged(400000);
            else if (!strcmp(command, "STATUS"))
                status();
            else if (!strcmp(command, "DUMP")) {
                dump(acc, "BMA");
                dump(gyro, "BMG");
            } else if (!strcmp(command, "SAMPLE"))
                sample();
            else if (!strcmp(command, "START")) {
                streaming = true;
                status();
            } else if (!strcmp(command, "STOP")) {
                streaming = false;
                status();
            } else if (used)
                Serial.println("ERROR unknown command");
            used = 0;
        } else if (used < sizeof(command) - 1)
            command[used++] = c;
        else {
            used = 0;
            Serial.println("ERROR command overflow");
        }
    }
    if (streaming && millis() - last >= 100) {
        last = millis();
        sample();
    }
    if (millis() - lastStatus >= 5000) {
        lastStatus = millis();
        status();
    }
    delay(1);
}
