#include "defines.h"


QueueHandle_t rosQueue = xQueueCreate(30, sizeof(RosMessage));

State state = {};


void ultrasonic_states_init() {
    us_raw.data.capacity = MAX_US_SENSORS;
    us_raw.data.size = MAX_US_SENSORS;
    us_raw.data.data = (float*)malloc(MAX_US_SENSORS * sizeof(float));
    for (int i = 0; i < MAX_US_SENSORS; i++) {
        us_raw.data.data[i] = US_DEFAULT_STATE;
    }
}

void imu_states_init() {
    imu_msg.header.frame_id.data = (char*)malloc(20);
    imu_msg.header.frame_id.capacity = 20;
    snprintf(imu_msg.header.frame_id.data, 20, "imu_link");
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    memset(imu_msg.orientation_covariance, 0, sizeof(imu_msg.orientation_covariance));
    imu_msg.orientation_covariance[0] = -1.0;

    memset(imu_msg.angular_velocity_covariance, 0, sizeof(imu_msg.angular_velocity_covariance));
    imu_msg.angular_velocity_covariance[0] = 0.001;
    imu_msg.angular_velocity_covariance[4] = 0.001;
    imu_msg.angular_velocity_covariance[8] = 0.00005;

    memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
}

void mag_states_init() {
    mag_raw.data.capacity = 4;
    mag_raw.data.size = 4;
    mag_raw.data.data = (float*)malloc(4 * sizeof(float));
    for (int i = 0; i < 4; i++) {
        mag_raw.data.data[i] = 0.0f;
    }
}