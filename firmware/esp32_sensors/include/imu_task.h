#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "defines.h"

#define I2C_SDA 21
#define I2C_SCL 22

#define GYRO_CALIBRATION_SAMPLES 2048
#define GYRO_CALIBRATION_DELAY_MS 10
#define GYRO_FILTER_ALPHA 0.15f

// Частоты публикации
#define IMU_PUBLISH_MS 20  // 50 Hz
#define MAG_PUBLISH_MS 100 // 10 Hz (для логирования достаточно)

void imuTask(void *context);

// stage: 0 init,1 warmup,2 hardware calibration,3 bias,4 ready,5 fault.
// fault: 1 begin,2 readback,3 calibration,4 bias timeout,5 runtime readback.
struct ImuHealth {
    std::atomic<uint32_t> stage{0}, fault{0}, io_errors{0}, read_failures{0}, queue_drops{0}, last_good_ms{0},
        bias_samples{0}, mag_errors{0};
};
extern ImuHealth imuHealth;

#endif // IMU_TASK_H
