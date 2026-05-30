# Verter Robot — Hardware Specification

## ESP32 Chassis Controller

### Распиновка

| Устройство | Пин | Примечание |
|-----------|-----|-----------|
| Левый мотор PWM | GPIO 19 | Cytron MD10C |
| Левый мотор DIR | GPIO 27 | LOW = вперёд |
| Правый мотор PWM | GPIO 18 | Cytron MD10C |
| Правый мотор DIR | GPIO 25 | HIGH = вперёд |
| Левый энкодер SDA | GPIO 21 | AS5600, I2C0 |
| Левый энкодер SCL | GPIO 22 | |
| Правый энкодер SDA | GPIO 32 | AS5600, I2C1 |
| Правый энкодер SCL | GPIO 4 | |
| LED | GPIO 2 | Статус подключения |

### Направления (калибровка полярности)

| Колесо | Мотор DIR (вперёд) | Энкодер знак (вперёд) |
|--------|-------------------|----------------------|
| Левый | LOW | -1 |
| Правый | HIGH | +1 |

### Моторы

- Драйвер: Cytron MD10C (PWM + DIR)
- PWM частота: 20 kHz
- PWM разрешение: 8 бит (0-255)
- MAX_PWM: 200
- MIN_PWM: 25 (мёртвая зона)
- MAX_VELOCITY: 0.5 м/с

### Энкодеры

- Тип: AS5600 магнитный абсолютный энкодер
- I2C адрес: 0x36
- Регистр угла: 0x0C (RAW_ANGLE)
- Разрешение: 4096 шагов/оборот (12 бит)
- Левый: I2C0 
- Правый: I2C1

### Известные проблемы

- Правый энкодер: I2C шина периодически зависает на ~1с. Причина: баг arduino-esp32 v2.x — Wire.setTimeOut() не работает для значений < 1000мс (issue #5934).

## ESP32 IMU Controller

### Распиновка

| Устройство | Пин |
|-----------|-----|
| IMU SDA | GPIO 21 |
| IMU SCL | GPIO 22 |

### IMU

- Модуль: Trema IMU 9 DOF V2.0 (Bosch BMX055)
- Акселерометр (BMA): сырые данные, м/с²
- Гироскоп (BMG): сырые данные, рад/с, bias вычитается при старте
- Магнитометр (BMM): отключён (помехи от моторов)
- Частота публикации: 50 Hz
- Физическое положение: центр робота, 0.29м над base_link

## Параметры робота (откалиброваны)

| Параметр | Значение | Примечание |
|----------|---------|-----------|
| WHEEL_CIRCUMFERENCE | 0.576 м | Калибровка |
| GEAR_RATIO | 4.0007 | Калибровка |
| WHEEL_BASE | 0.386 м | Калибровочная нода (новая рама) |
| ENCODER_RESOLUTION | 4096 шагов/оборот | AS5600 12-bit |
| METERS_PER_STEP | ~3.52e-5 м | circumference / (resolution * gear_ratio) |

### PID

| Параметр | Значение |
|----------|---------|
| Kp | 80 |
| Ki | 50 |
| Kd | 5 |
| MAX_INTEGRAL | 100 |
| PID_INTERVAL | 50 мс |
| MAX_PWM_CHANGE | 15 за итерацию (ramp limiter) |

## Коммуникация

### Serial (micro-ROS)

| Параметр | Значение |
|----------|---------|
| Baud rate | 921600 |
| RX buffer | 4096 байт (увеличен с 256) |
| Transport read timeout | max 10мс (ограничен от 1000мс DDS) |
| Transport write | неблокирующий (drop при полном TX буфере) |

### ROS2 топики

| Топик | Тип | Частота | QoS | Направление |
|-------|-----|---------|-----|------------|
| /wheel_encoders | Int64MultiArray | 20 Hz | BEST_EFFORT | ESP32 → ROS |
| /cmd_vel | Twist | по мере поступления | RELIABLE | ROS → ESP32 |
| /imu/data | Imu | 50 Hz | default | ESP32 IMU → ROS |
| /odom | Odometry | 50 Hz (timer) | default | odometry_node |
| /scan_raw | LaserScan | ~6 Hz | default | rplidar_node |
| /scan | LaserScan | ~6 Hz | default | laser_filter |

### Jetson USB устройства

| Устройство | Udev symlink | Baud |
|-----------|-------------|------|
| ESP32 Chassis | /dev/esp32_chassis | 921600 |
| ESP32 IMU | /dev/esp32_imu | 921600 |
| RPLidar A1M8 | /dev/rplidar | 115200 |


## URDF фреймы

| Фрейм | Положение от base_footprint |
|-------|---------------------------|
| base_footprint | 0, 0, 0 (на полу) |
| base_link | 0, 0, 0.1 |
| lidar_link | 0.12, 0, 0.70 (повёрнут 180° по Z) |
| imu_link | 0, 0, 0.29 |
| wheel_left | 0, +0.178, 0.05 |
| wheel_right | 0, -0.178, 0.05 |
