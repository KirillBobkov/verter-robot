#ifndef DEFINES_H
#define DEFINES_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <atomic>

#define LED_INFO_PIN 2

#define MAX_US_SENSORS 7
#define US_TIMEOUT_US  50000
#define US_DEFAULT_STATE 10.0


extern QueueHandle_t rosQueue;


struct State {
    std::atomic<bool> agent_is_connected{false};
};

enum SensorType: uint8_t {
    US_SENSOR_TYPE = 0,
    IMU_SENSOR_TYPE = 1,
    MAG_SENSOR_TYPE = 2
};

union SensorData {
    struct {
        uint8_t idx;
        float value;
    } us;
    struct {
        int64_t time_ns;
        float vel_x;
        float vel_y;
        float vel_z;
        float acc_x;
        float acc_y;
        float acc_z;
    } imu;
    struct {
        float raw[4];
    } mag;
};

struct RosMessage {
    SensorType type;
    SensorData data;
};

extern State state;
extern std_msgs__msg__Float32MultiArray us_raw;
extern std_msgs__msg__Float32MultiArray mag_raw;
extern sensor_msgs__msg__Imu imu_msg;


void ultrasonic_states_init();
void imu_states_init();
void mag_states_init();

#endif // DEFINES_H