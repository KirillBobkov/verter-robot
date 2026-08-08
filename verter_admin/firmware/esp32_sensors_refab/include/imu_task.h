#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "defines.h"

#define I2C_SDA 21
#define I2C_SCL 22

#define GYRO_CALIBRATION_SAMPLES  2000
#define GYRO_CALIBRATION_DELAY_MS 10
#define GYRO_FILTER_ALPHA         0.15f

// Частоты публикации
#define IMU_PUBLISH_MS 20   // 50 Hz
#define MAG_PUBLISH_MS 100  // 10 Hz (для логирования достаточно)


void imuTask(void *context);


#endif // IMU_TASK_H