#include "imu_task.h"

#include <Wire.h>
#include <iarduino_Position_BMX055.h>
#include "imu_checked.h"
#include <math.h>
ImuHealth imuHealth;
struct CheckedBus {
    bool read(uint8_t a, uint8_t r, uint8_t &v) {
        Wire.beginTransmission(a);
        Wire.write(r);
        if (Wire.endTransmission(false) != 0) {
            ++imuHealth.io_errors;
            return false;
        }
        if (Wire.requestFrom(a, (size_t)1, true) != 1) {
            while (Wire.available())
                Wire.read();
            ++imuHealth.io_errors;
            return false;
        }
        v = Wire.read();
        return true;
    }
    bool write(uint8_t a, uint8_t r, uint8_t v) {
        Wire.beginTransmission(a);
        Wire.write(r);
        Wire.write(v);
        bool ok = Wire.endTransmission() == 0;
        if (!ok)
            ++imuHealth.io_errors;
        return ok;
    }
    uint32_t milliseconds() { return millis(); }
    void sleep(uint32_t ms) { delay(ms); }
} bus;
bool finiteAxes(iarduino_Position_BMX055 &s) {
    return isfinite(s.axisX) && isfinite(s.axisY) && isfinite(s.axisZ);
}
void imuFault(uint32_t reason) {
    imuHealth.fault.store(reason);
    imuHealth.stage.store(5);
    vTaskDelete(nullptr);
}

// Датчики BMX055
iarduino_Position_BMX055 sensorA(BMA); // Акселерометр
iarduino_Position_BMX055 sensorG(BMG); // Гироскоп
iarduino_Position_BMX055 sensorM(BMM); // Магнитометр (для логирования)

// Bias гироскопа
float raw_bias_x[GYRO_CALIBRATION_SAMPLES]{};
float raw_bias_y[GYRO_CALIBRATION_SAMPLES]{};
float raw_bias_z[GYRO_CALIBRATION_SAMPLES]{};
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
uint bias_idx = 0;

void imuTask(void *context) {
    // I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(50);

    // Инициализация IMU
    bool accReady = sensorA.begin();
    bool gyroReady = sensorG.begin();
    bool magReady = sensorM.begin();
    if (!accReady || !gyroReady) {
        imuFault(1);
        return;
    }
    if (!magReady)
        ++imuHealth.mag_errors;
    sensorA.setScale(BMA_4G);
    sensorG.setScale(BMG_500DPS);
    if (magReady)
        sensorM.setScale(BMM_REGULAR); // 0.6 µT noise, default oversampling
    sensorA.setBandwidths(BMA_125Hz);
    //sensorG.setBandwidths(BMG_116Hz);
    sensorG.setBandwidths(BMG_23Hz);
    if (magReady)
        sensorM.setBandwidths(BMM_10Hz); // 10 Hz ODR — достаточно для логирования

    // Аппаратная калибровка BMX055
    if (!imu_checked::config(bus)) {
        imuFault(2);
        return;
    }
    imuHealth.stage.store(1);
    delay(10000);
    imuHealth.stage.store(2);
    if (!imu_checked::calibrate(bus)) {
        imuFault(3);
        return;
    }
    if (!imu_checked::config(bus)) {
        imuFault(2);
        return;
    }
    // Магнитометр: НЕ калибруем автоматически — хотим видеть сырые данные
    // Калибровка делается вручную после оценки помех

    // Программная калибровка гироскопа
    imuHealth.stage.store(3);
    uint32_t biasStart = millis();
    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES;) {
        if (millis() - biasStart > 40000) {
            imuFault(4);
            return;
        }
        if (!sensorG.read(BMG_RAD_S) || !finiteAxes(sensorG)) {
            ++imuHealth.read_failures;
            delay(GYRO_CALIBRATION_DELAY_MS);
            continue;
        }
        float x = sensorG.axisX / (float)GYRO_CALIBRATION_SAMPLES;
        float y = sensorG.axisY / (float)GYRO_CALIBRATION_SAMPLES;
        float z = sensorG.axisZ / (float)GYRO_CALIBRATION_SAMPLES;
        raw_bias_x[i] = x;
        raw_bias_y[i] = y;
        raw_bias_z[i] = z;
        gyroBiasX += x;
        gyroBiasY += y;
        gyroBiasZ += z;
        ++i;
        imuHealth.bias_samples.store(i);
        delay(GYRO_CALIBRATION_DELAY_MS);
    }

    imuHealth.stage.store(4);
    uint32_t lastConfig = millis();

    unsigned long last_imu_tr = millis();
    unsigned long last_mag_tr = millis();

    RosMessage message = {};

    while (true) {
        // Читаем IMU + магнитометр (I2C)
        unsigned long now = millis();

        if (now - lastConfig >= 5000) {
            lastConfig = now;
            if (!imu_checked::config(bus)) {
                imuFault(5);
                return;
            }
        }
        if (now - last_imu_tr >= IMU_PUBLISH_MS) {
            last_imu_tr = now;

            bool ar = sensorA.read(BMA_M_S), gr = sensorG.read(BMG_RAD_S);
            if (!ar || !gr || !finiteAxes(sensorA) || !finiteAxes(sensorG)) {
                ++imuHealth.read_failures;
                delay(1);
                continue;
            }
            imuHealth.last_good_ms.store(millis());

            message.type = SensorType::IMU_SENSOR_TYPE;

            message.data.imu.time_ns = rmw_uros_epoch_nanos();

            float gyroX = sensorG.axisX - gyroBiasX;
            float gyroY = sensorG.axisY - gyroBiasY;
            float gyroZ = sensorG.axisZ - gyroBiasZ;

            //gyroFilteredX += GYRO_FILTER_ALPHA * (gyroX - gyroFilteredX);
            //gyroFilteredY += GYRO_FILTER_ALPHA * (gyroY - gyroFilteredY);
            //gyroFilteredZ += GYRO_FILTER_ALPHA * (gyroZ - gyroFilteredZ);

            message.data.imu.vel_x = gyroX;
            message.data.imu.vel_y = gyroY;
            message.data.imu.vel_z = gyroZ;

            message.data.imu.acc_x = sensorA.axisX;
            message.data.imu.acc_y = sensorA.axisY;
            message.data.imu.acc_z = sensorA.axisZ;

            if (state.agent_is_connected.load()) {
                if (xQueueSend(rosQueue, &message, 0) != pdTRUE)
                    ++imuHealth.queue_drops;

                // Legacy bias policy retained: cmd_vel is NOT proof of physical standstill.
                if (!state.is_motion.load()) {
                    float old_x = raw_bias_x[bias_idx];
                    float old_y = raw_bias_y[bias_idx];
                    float old_z = raw_bias_z[bias_idx];

                    float x = sensorG.axisX / (float)GYRO_CALIBRATION_SAMPLES;
                    float y = sensorG.axisY / (float)GYRO_CALIBRATION_SAMPLES;
                    float z = sensorG.axisZ / (float)GYRO_CALIBRATION_SAMPLES;

                    raw_bias_x[bias_idx] = x;
                    raw_bias_y[bias_idx] = y;
                    raw_bias_z[bias_idx] = z;

                    gyroBiasX = gyroBiasX + x - old_x;
                    gyroBiasY = gyroBiasY + y - old_y;
                    gyroBiasZ = gyroBiasZ + z - old_z;

                    bias_idx++;
                    bias_idx %= GYRO_CALIBRATION_SAMPLES;
                }
            }
        }

        if (magReady && now - last_mag_tr >= MAG_PUBLISH_MS) {
            last_mag_tr = now;

            if (!sensorM.read(BMM_MCT) || !finiteAxes(sensorM)) {
                ++imuHealth.mag_errors;
                delay(1);
                continue;
            }

            message.type = SensorType::MAG_SENSOR_TYPE;

            message.data.mag.raw[0] = sensorM.axisX;
            message.data.mag.raw[1] = sensorM.axisY;
            message.data.mag.raw[2] = sensorM.axisZ;
            message.data.mag.raw[3] = atan2(sensorM.axisY, sensorM.axisX) * 180.0f / M_PI;

            if (state.agent_is_connected.load()) {
                if (xQueueSend(rosQueue, &message, 0) != pdTRUE)
                    ++imuHealth.queue_drops;
            }
        }

        delay(1);
    }
}