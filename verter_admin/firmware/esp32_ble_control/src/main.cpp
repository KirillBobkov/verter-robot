#include <Arduino.h>
#include "defines.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>
#include "bl_task.h"

#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) return false; }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) break;}

#define SERIAL_BUFFER_SIZE 4096

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t cmd_vel_pub;
geometry_msgs__msg__Twist cmd_msg;


bool create_entities() {
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_ctrl", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &cmd_vel_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));
    
    return true;
}


void destroy_entities() {
    rcl_ret_t rc;
    rc = rcl_publisher_fini(&cmd_vel_pub, &node);
    rc = rcl_node_fini(&node);
    rc = rclc_support_fini(&support);
}


void setup() {
    pinMode(LED_INFO_PIN, OUTPUT);
    digitalWrite(LED_INFO_PIN, LOW);

    Serial.begin(921600);
    set_microros_serial_transports(Serial);
    delay(1000);

    allocator = rcl_get_default_allocator();

    geometry_msgs__msg__Twist__init(&cmd_msg);

    xTaskCreate(blTask, "BL task", 4096, NULL, 1, NULL);
    delay(1000);
}


void loop() {
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        digitalWrite(LED_INFO_PIN, !digitalRead(LED_INFO_PIN));
        delay(500);
    }

    digitalWrite(LED_INFO_PIN, HIGH);

    if (create_entities()) {
        rmw_uros_sync_session(1000);
        unsigned long last_time_sync = millis();

        while (true) {
            rcl_ret_t rc = RMW_RET_OK;

            xSemaphoreTake(xControlFrameMutex, portMAX_DELAY);
            float _linear_x = ctrl.linear_x;
            float _angular_z = ctrl.angular_z;
            bool is_updated = ctrl.is_updated;
            ctrl.is_updated = false;
            xSemaphoreGive(xControlFrameMutex);

            bool linear_x_action = abs(_linear_x) > 0.0001;
            bool angular_y_action = abs(_angular_z) > 0.0001;

            if (linear_x_action || angular_y_action || is_updated) {
                cmd_msg.linear.x = _linear_x;
                cmd_msg.angular.z = _angular_z;
                rc = rcl_publish(&cmd_vel_pub, &cmd_msg, NULL);
            }

            unsigned long now = millis();
            if (now - last_time_sync > 60000) {
                RCSOFTCHECK(rmw_uros_sync_session(100));
                last_time_sync = now;
            }

            delay(100);
        }
    }

    destroy_entities();
}