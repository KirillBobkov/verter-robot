# ESP32 / micro-ROS

## 1. ESP32 не определяется как USB-устройство

**Решение:**
- `sudo dmesg | tail -20` — логи ядра
- Попробовать другой USB-кабель (некоторые только для зарядки)
- Попробовать другой USB-порт

---

## 2. Прошивка не загружается («Failed to connect to ESP32»)

**Решение:** Зажать кнопку BOOT + EN на ESP32, отпустить BOOT — микроконтроллер войдёт в режим прошивки. Альтернативно — уменьшить скорость загрузки до 115200.

---

## 3. micro-ROS agent не видит ESP32

**Решение:**
- Проверить что ESP32 отправляет данные: `screen /dev/esp32_chassis 921600`
- Проверить baud rate (921600 для обоих ESP32 — chassis и sensors/IMU)

---

## 4. Нода появляется и исчезает (таймаут heartbeat)

**Причина:** ESP32 не отправляет регулярный heartbeat.

**Решение:** В коде ESP32 обеспечить регулярный вызов `rclc_executor_spin_some` с коротким таймаутом и минимальной задержкой между итерациями цикла. Пример из `esp32_chassis_modbus.ino`:
```cpp
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
vTaskDelay(pdMS_TO_TICKS(1));  // 1 мс — не больше!
```

В `esp32_sensors_refab` (сенсоры/IMU) executor не используется — данные из FreeRTOS-очереди публикуются в `loop()` с `delay(1)`, что также держит heartbeat.

---

## 5. Моторы не крутятся (ZLAC8015D)

В текущей прошивке `esp32_chassis_modbus` ESP32 не управляет моторами напрямую через PWM — он отправляет целевой RPM драйверу ZLAC8015D по Modbus RTU (UART2 @ 115200, TTL-RS485). PWM/L298N больше не используется.

**Диагностика:**

1. **Проверить состояние chassis:** `ros2 topic echo /chassis/state` — ожидается `3` (ACTIVE). Если `4` (SAFE_STOP) — нет свежего `/cmd_vel` (watchdog `CMD_TIMEOUT_MS = 500` мс). Если `5` (FAULT) — проверить `ros2 topic echo /chassis/fault` (код ошибки ZLAC)
2. **Проверить Modbus-связь:** `ros2 topic echo /chassis/modbus_fails` — `0` нормально, растёт быстро (~50/с) если ZLAC не отвечает. Проверить подключение TTL-RS485: TX2=GPIO16, RX2=GPIO17, GND/3V3
3. **Проверить, что ZLAC включён:** `ros2 topic echo /chassis/cmd_watchdog` — `false` до первого `/cmd_vel`. Питание 12В на ZLAC, общий GND с ESP32
