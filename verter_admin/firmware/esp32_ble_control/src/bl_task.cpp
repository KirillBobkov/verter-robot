#include "bl_task.h"
#include <Arduino.h>
#include "BluetoothSerial.h"


BluetoothSerial SerialBT;

void blTask(void *context) {
    String point = BLE_POINT;
    SerialBT.begin(point);
    Serial.println("Bluetooth ready as " + point);

    while (true) {
        String cmd = SerialBT.readStringUntil('\n');
        cmd.trim();

        if (cmd.length() == 1) {
            xSemaphoreTake(xControlFrameMutex, portMAX_DELAY);
            float _linear_x = ctrl.linear_x;
            float _angular_z = ctrl.angular_z;
            xSemaphoreGive(xControlFrameMutex);

            uint8_t value = cmd.toInt();

            switch (value) {
                case 1:
                    _angular_z += 0.05;
                    break;
                case 2:
                    _linear_x += 0.01;
                    break;
                case 3:
                    _linear_x -= 0.01;
                    break;
                case 4:
                    _angular_z -= 0.05;
                    break;
                default:
                    _linear_x = 0.0;
                    _angular_z = 0.0;
                    break;
            }

            xSemaphoreTake(xControlFrameMutex, portMAX_DELAY);
            ctrl.linear_x = _linear_x;
            ctrl.angular_z = _angular_z;
            ctrl.is_updated = true;
            xSemaphoreGive(xControlFrameMutex);

            String responce = "";

            if (_linear_x > 0.0001) {
                responce += "Едет вперед ";
            } else if (_linear_x < -0.0001) {
                responce += "Едет назад ";
            } else {
                responce += "Стоит на месте ";
            }

            if (_angular_z > 0.0001) {
                responce += "и поворачивает влево";
            } else if (_angular_z < -0.0001) {
                responce += "и поворачивает вправо";
            }

            SerialBT.println("X: " + String(_linear_x) + " | Z:" + String(_angular_z));
            SerialBT.println(responce);
        }

        delay(10);
    }
}