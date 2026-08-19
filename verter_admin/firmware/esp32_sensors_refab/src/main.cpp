#include <Arduino.h>
#include "defines.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include "us_task.h"
#include "imu_task.h"

#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) break;}

#define SERIAL_BUFFER_SIZE 4096


rcl_publisher_t ultrasonic_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rclc_executor_t executor;
rcl_subscription_t cmd_vel_sub;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__Float32MultiArray us_raw;
std_msgs__msg__Float32MultiArray mag_raw;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
RosMessage message;


bool create_entities() {
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_sensors", "", &support));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    RCCHECK(rclc_publisher_init_best_effort(
        &ultrasonic_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/ultrasonics"));
    
    RCCHECK(rclc_publisher_init_best_effort(
        &mag_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/imu/mag_raw"
    ));
    
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"));
    
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(
            geometry_msgs,
            msg,
            Twist
        ),
        "/cmd_vel"
    ));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_sub,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA
    ));
    
    return true;
}


void destroy_entities() {
    rcl_ret_t rc;
    rc = rclc_executor_fini(&executor);
    rc = rcl_subscription_fini(&cmd_vel_sub, &node);
    rc = rcl_publisher_fini(&ultrasonic_publisher, &node);
    rc = rcl_publisher_fini(&imu_publisher, &node);
    rc = rcl_publisher_fini(&mag_publisher, &node);
    rc = rcl_node_fini(&node);
    rc = rclc_support_fini(&support);
}


void setup() {
    pinMode(LED_INFO_PIN, OUTPUT);
    digitalWrite(LED_INFO_PIN, LOW);

    Serial.begin(921600);
    set_microros_serial_transports(Serial);
    delay(2000);

    allocator = rcl_get_default_allocator();
    
    ultrasonic_states_init();
    imu_states_init();
    mag_states_init();

    xTaskCreate(usTask, "Ultrasonics", 4096, NULL, 1, NULL);
    xTaskCreate(imuTask, "IMU", 8192, NULL, 1, NULL);
    delay(2000);
}


void loop() {
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        digitalWrite(LED_INFO_PIN, !digitalRead(LED_INFO_PIN));
        delay(500);
    }

    digitalWrite(LED_INFO_PIN, HIGH);

    if (create_entities()) {
        while (xQueueReceive(rosQueue, &message, 0) == pdTRUE);
        state.agent_is_connected.store(true);
        rmw_uros_sync_session(1000);
        unsigned long last_time_sync = millis();

        while (true) {
            RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));
            rcl_ret_t rc = RMW_RET_OK;
            if (xQueueReceive(rosQueue, &message, portMAX_DELAY) == pdTRUE) {
                switch (message.type) {
                    case SensorType::US_SENSOR_TYPE: {
                        us_raw.data.data[message.data.us.idx] = message.data.us.value;
                        rc = rcl_publish(&ultrasonic_publisher, &us_raw, NULL);
                        break;
                    }
                    case SensorType::IMU_SENSOR_TYPE: {
                        imu_msg.header.stamp.sec = (int32_t)(message.data.imu.time_ns / 1000000000LL);
                        imu_msg.header.stamp.nanosec = (uint32_t)(message.data.imu.time_ns % 1000000000LL);

                        imu_msg.angular_velocity.x = message.data.imu.vel_x;
                        imu_msg.angular_velocity.y = message.data.imu.vel_y;
                        imu_msg.angular_velocity.z = message.data.imu.vel_z;

                        imu_msg.linear_acceleration.x = message.data.imu.acc_x;
                        imu_msg.linear_acceleration.y = message.data.imu.acc_y;
                        imu_msg.linear_acceleration.z = message.data.imu.acc_z;
                        rc = rcl_publish(&imu_publisher, &imu_msg, NULL);
                        break;
                    }
                    case SensorType::MAG_SENSOR_TYPE: {
                        for (uint8_t i=0; i<4; i++) {
                            mag_raw.data.data[i] = message.data.mag.raw[i];
                        }
                        rc = rcl_publish(&mag_publisher, &mag_raw, NULL);
                        break;
                    }
                    default:
                        break;
                }

                RCSOFTCHECK(rc);
            }

            RCSOFTCHECK(rclc_executor_spin_some(&executor, 0));

            unsigned long now = millis();
            if (now - last_time_sync > 60000) {
                RCSOFTCHECK(rmw_uros_sync_session(100));
                last_time_sync = now;
            }

            delay(1);
        }
    }

    state.agent_is_connected.store(false);
    destroy_entities();
}