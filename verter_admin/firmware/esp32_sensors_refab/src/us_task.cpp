#include "us_task.h"


// Пины: {trig, echo}
const int sensor_pins[MAX_US_SENSORS][2] = {
    {16, 34},  // Sensor 1: right
    {17, 35},  // Sensor 2: front_right_outer
    {18, 32},  // Sensor 3: front_right_inner
    {19, 33},  // Sensor 4: front_center
    {23, 25},  // Sensor 5: front_left_inner
    {26, 27},  // Sensor 6: front_left_outer
    {14, 12},  // Sensor 7: left
};


void usTask(void *context) {
    for (uint8_t i=0; i<MAX_US_SENSORS; i++) {
        pinMode(sensor_pins[i][0], OUTPUT);
        pinMode(sensor_pins[i][1], INPUT);
    }

    RosMessage message = {};
    message.type = SensorType::US_SENSOR_TYPE;

    while (true) {
        for (uint8_t i=0; i<MAX_US_SENSORS; i++) {
            message.data.us.idx = i;
            message.data.us.value = read_sensor(sensor_pins[i][0], sensor_pins[i][1]);
            //message.data.us.value = 400.0;

            if (state.agent_is_connected.load()) {
                xQueueSend(rosQueue, &message, portMAX_DELAY);
            }
        }
        delay(1);
    }
}


float read_sensor(int trig, int echo) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    long duration = pulseIn(echo, HIGH, US_TIMEOUT_US);
    if (duration == 0) {
        return US_DEFAULT_STATE;
    }
    // Скорость звука 343 м/с → 0.000343 м/мкс, туда-обратно /2
    return (duration * 0.000343f) / 2.0f;
}