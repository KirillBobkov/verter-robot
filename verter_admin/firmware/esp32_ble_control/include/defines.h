#ifndef DEFINES_H
#define DEFINES_H

#include <Arduino.h>

#define BLE_POINT "verter-point"

#define LED_INFO_PIN 2

struct ControlFrame {
    float linear_x;
    float angular_z;
    bool is_updated;
};


extern volatile ControlFrame ctrl;
extern xSemaphoreHandle xControlFrameMutex;


#endif // DEFINES_H