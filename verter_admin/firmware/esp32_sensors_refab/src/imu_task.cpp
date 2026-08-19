#include "imu_task.h"

#include <Wire.h>
#include <iarduino_Position_BMX055.h>


// Датчики BMX055
iarduino_Position_BMX055 sensorA(BMA);  // Акселерометр
iarduino_Position_BMX055 sensorG(BMG);  // Гироскоп
iarduino_Position_BMX055 sensorM(BMM);  // Магнитометр (для логирования)

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

    // Инициализация IMU
    sensorA.begin();
    sensorG.begin();
    sensorM.begin();
    sensorA.setScale(BMA_4G);
    sensorG.setScale(BMG_500DPS);
    sensorM.setScale(BMM_REGULAR);      // 0.6 µT noise, default oversampling
    sensorA.setBandwidths(BMA_125Hz);
    //sensorG.setBandwidths(BMG_116Hz);
    sensorG.setBandwidths(BMG_23Hz);
    sensorM.setBandwidths(BMM_10Hz);    // 10 Hz ODR — достаточно для логирования

    // Аппаратная калибровка BMX055
    delay(10000);
    sensorA.setFastOffset();
    sensorG.setFastOffset();
    // Магнитометр: НЕ калибруем автоматически — хотим видеть сырые данные
    // Калибровка делается вручную после оценки помех

    // Программная калибровка гироскопа
    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
        sensorG.read(BMG_RAD_S);
        float x = sensorG.axisX / (float)GYRO_CALIBRATION_SAMPLES;
        float y = sensorG.axisY / (float)GYRO_CALIBRATION_SAMPLES;
        float z = sensorG.axisZ / (float)GYRO_CALIBRATION_SAMPLES;
        raw_bias_x[i] = x;
        raw_bias_y[i] = y;
        raw_bias_z[i] = z;
        gyroBiasX += x;
        gyroBiasY += y;
        gyroBiasZ += z;
        delay(GYRO_CALIBRATION_DELAY_MS);
    }

    float gyroFilteredX = 0.0f;
    float gyroFilteredY = 0.0f;
    float gyroFilteredZ = 0.0f;

    unsigned long last_imu_tr = millis();
    unsigned long last_mag_tr = millis();

    RosMessage message = {};

    while (true) {
        // Читаем IMU + магнитометр (I2C)
        unsigned long now = millis();

        if (now - last_imu_tr >= IMU_PUBLISH_MS) {
            last_imu_tr += IMU_PUBLISH_MS;

            sensorA.read(BMA_M_S);
            sensorG.read(BMG_RAD_S);

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
                xQueueSend(rosQueue, &message, portMAX_DELAY);

                //Выполняем калибровку если привод без движения
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

        
        if (now - last_mag_tr >= MAG_PUBLISH_MS) {
            last_mag_tr += MAG_PUBLISH_MS;

            sensorM.read(BMM_MCT);

            message.type = SensorType::MAG_SENSOR_TYPE;

            message.data.mag.raw[0] = sensorM.axisX;
            message.data.mag.raw[1] = sensorM.axisY;
            message.data.mag.raw[2] = sensorM.axisZ;
            message.data.mag.raw[3] = atan2(sensorM.axisY, sensorM.axisX) * 180.0f / M_PI;

            if (state.agent_is_connected.load()) {
                xQueueSend(rosQueue, &message, portMAX_DELAY);
            }
        }

        delay(1);
    }
}