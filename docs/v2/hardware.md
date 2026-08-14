---
description: Полное описание аппаратной части робота Verter — компоненты, ESP32-прошивки, моторные параметры, ультразвуковые датчики, URDF-фреймы
---

# Аппаратная часть

## Обзор комплектующих

| Подсистема | Компонент | Интерфейс | Назначение |
|---|---|---|---|
| Бортовой компьютер | NVIDIA Jetson Orin Nano Super 8 GB | USB 3.2, CSI, NVMe | ROS2 Humble, Nav2, SLAM, AI-ассистент |
| Лидар | RPLiDAR A1M8 | USB-serial (115200) | 2D-сканирование, 360°, 12 м |
| Шасси-контроллер | ESP32 (WROOM-32 DevKit) | USB-serial (921600) | Управление моторами (ZLAC8015D), энкодеры |
| IMU-контроллер | ESP32 + Trema IMU 9 DOF V2 (Bosch BMX055) | USB-serial (921600) | Акселерометр, гироскоп, магнитометр |
| Ультразвук | 7 x HC-SR04 (5 передних + 2 бока) | GPIO (через ESP32) | Обнаружение препятствий |
| Аудио входы | ReSpeaker USB Mic Array | USB 2.0 | Распознавание речи, DOA |
| Аудио выход | Динамик / наушники | 3.5 мм jack | Синтез речи (Silero) |
| Питание робота | LiFePO4 аккумулятор | DC-DC конвертеры | 24 V для моторов (ZLAC8015D), 5 V для сенсоров, 19 V для Jetson |

### Jetson Orin Nano

- **ОС**: JetPack 6.x / Ubuntu 22.04
- **Накопитель**: NVMe SSD 256+ ГБ (microSD только для первичной прошивки)
- **Питание**: стабильный DC-DC, запас не менее 50 W
- **Охлаждение**: активное (радиатор + вентилятор)
- **USB-разводка**: RPLiDAR, ReSpeaker, два ESP32 лучше подключать через powered USB hub

---

## ESP32 + micro-ROS: настройка с нуля

### Выбор платы

| Плата | Цена | Примечание |
|---|---|---|
| ESP32-WROOM-32 DevKit | ~$5 | Базовый вариант, достаточно GPIO |
| ESP32-S3-DevKitC | ~$10 | USB-OTG, больше RAM |
| ESP32-WROVER | ~$8 | 8 MB PSRAM |

Для Verter достаточно **ESP32-WROOM-32 DevKit V1** (двухрядный 30-pin, Type-C CH340C).

### Распиновка ESP32 DevKit 30-pin

```
            ┌──────────────────────────────┐
            │  [BOOT]   USB-C    [RST]     │
            │                              │
     CLK  ──┤○                            ○├── V5
     SD0  ──┤○  ████████████████████████  ○├── CMD
     SD1  ──┤○  █                      █  ○├── SD3
     G15  ──┤○  █    ESP32-WROOM-32    █  ○├── SD2
      G2  ──┤○  █                      █  ○├── G13
      G0  ──┤○  █                      █  ○├── GND
      G4  ──┤○  ████████████████████████  ○├── G12
     G16  ──┤○                            ○├── G14
     G17  ──┤○                            ○├── G27
      G5  ──┤○                            ○├── G26
     G18  ──┤○                            ○├── G25
     G19  ──┤○                            ○├── G23
     GND  ──┤○                            ○├── G32
     G21  ──┤○                            ○├── G35
     RXD  ──┤○                            ○├── G34
     TXD  ──┤○                            ○├── SM
     G22  ──┤○                            ○├── SP
     G23  ──┤○                            ○├── EN
     GND  ──┤○                            ○├── 3V3
            └──────────────────────────────┘
```

**Ограничения по пинам:**

| Пины | Статус |
|---|---|
| SD0-SD3, CLK, CMD | **НЕ использовать** — подключены к Flash |
| G0, G2 | Не использовать — режим загрузки |
| RXD, TXD | Заняты USB-serial (micro-ROS) |
| G25, G26, G27 | ADC2 — не работают при включённом WiFi |
| G12, G14, G15 | Осторожно — влияют на boot |

### Установка инструментов

**Arduino IDE** (простой вариант):

```bash
sudo apt install arduino
```

В Arduino IDE:
1. File -> Preferences -> Additional Board Manager URLs:
   `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Tools -> Board -> Boards Manager -> установить "ESP32 by Espressif Systems"
3. Sketch -> Include Library -> Add .ZIP Library -> скачать с https://github.com/micro-ROS/micro_ros_arduino/releases

**PlatformIO** (рекомендуется):

```bash
pip install platformio
```

Проект шасси: `verter_admin/firmware/esp32_chassis/esp32_chassis_modbus/`

```bash
cd verter_admin/firmware/esp32_chassis/esp32_chassis_modbus
pio run                      # компиляция
pio run --target upload      # прошивка
pio device monitor --baud 921600
```

### Прошивка ESP32

1. Подключи ESP32 к Jetson через USB (Type-A -> Type-C, не Type-C -> Type-C).
2. Проверь, что порт появился:

```bash
ls /dev/ttyUSB*
```

3. Прошей:

```bash
pio run --target upload --upload-port /dev/ttyUSB0
```

4. Если прошивка не загружается ("Failed to connect to ESP32"): зажать **BOOT** + **EN**, отпустить BOOT, повторить загрузку. Альтернативно — снизить Upload Speed до 115200.

### Установка micro-ROS Agent на Jetson

```bash
sudo apt install ros-humble-micro-ros-agent
```

Или из исходников:

```bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws/src
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
cd .. && colcon build && source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

### Запуск агентов

Два ESP32 требуют два отдельных экземпляра agent:

```bash
# Шасси (921600 baud)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32_chassis -b 921600

# IMU (921600 baud)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32_imu -b 921600
```

Для отладки добавь `-v6` для verbose-логов.

### udev-правила для ESP32

Правила уже описаны в [udev_rules.md](udev_rules.md). Кратко:

```bash
sudo nano /etc/udev/rules.d/99-esp32.rules
```

```
# ESP32 CH340C
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp32_chassis", MODE="0666"

# ESP32 CP2102
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="esp32_imu", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

После этого устройства доступны как `/dev/esp32_chassis` и `/dev/esp32_imu`.

### Проверка работы

```bash
# Агент запущен
ros2 node list               # должен появиться /esp32_chassis
ros2 topic echo /wheel_encoders   # крути колёса руками
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once   # робот поедет
```

Проблемы и решения см. [problems_solved/esp32_microros.md](problems_solved/esp32_microros.md).

---

## ESP32 Chassis Controller

### Распиновка (шасси)

| Устройство | GPIO | Примечание |
|---|---|---|
| Modbus TX2 | GPIO 16 | TTL-RS485 → ZLAC8015D |
| Modbus RX2 | GPIO 17 | TTL-RS485 → ZLAC8015D |
| LED статус | GPIO 2 | Индикатор micro-ROS-подключения |

Шасси управляется драйвером ZLAC8015D по Modbus RTU (UART2, 115200 бод).
ESP32 — L1-посредник между Jetson (micro-ROS) и ZLAC8015D; всей замкнутой
моторной петлёй владеет ZLAC, ESP32 лишь задаёт целевые RPM и читает
обратную связь (позиция, RPM, fault-коды).

### Схема подключения

```
Jetson ── USB CDC (921600) ── ESP32 ── UART2/TTL-RS485 (115200) ── ZLAC8015D ── моторы
         (micro-ROS)                                                  (Modbus RTU)
```

### Направления движения

Знаки энкодеров (firmware `LEFT_SIGN` / `RIGHT_SIGN`):

| Колесо | Знак (вперёд) |
|---|---|
| Левый | +1 |
| Правый | -1 |

Знаки применяются к позиции и RPM колеса так, чтобы при движении вперёд
оба значения возрастали.

### Моторный драйвер

| Параметр | Значение |
|---|---|
| Модель | ZLAC8015D (Modbus RTU, ведомый адрес 0x01) |
| Modbus baud | 115200 (UART2, TTL-RS485) |
| Режим | Velocity (регистр 0x200D = 3) |
| Comm-watchdog (ESP32) | 0x2007 = 200 мс |
| Comm-watchdog (Python) | 0x2000 = 1000 мс |
| MAX_MOTOR_RPM (ESP32) | 200 |
| MAX_MOTOR_RPM (Python `chassis_zlac_node`) | 50 |
| MAX_LINEAR_VEL | 0.5 м/с |
| MAX_ANGULAR_VEL | 1.0 рад/с |

### Энкодеры ZLAC8015D

| Параметр | Значение |
|---|---|
| Тип | Встроенные энкодеры ZLAC8015D (чтение по Modbus) |
| Позиция | int32 на канал (регистры 0x20A7–0x20AA), накапливается через int32-обёртки |
| RPM | int16 на канал (регистры 0x20AB, 0x20AC) |
| Разрешение | 4096 шагов/оборот (используется одометрией) |

### ESP32 IMU Controller

| Устройство | GPIO |
|---|---|
| IMU SDA | GPIO 21 |
| IMU SCL | GPIO 22 |

| Параметр | Значение |
|---|---|
| Модуль | Trema IMU 9 DOF V2.0 (Bosch BMX055) |
| Акселерометр | сырые данные, м/с^2 |
| Гироскоп | сырые данные, рад/с, bias вычитается при старте |
| Магнитометр | публикуется на `/imu/mag_raw` (Float32MultiArray), 10 Гц |
| Частота публикации IMU | 50 Гц (`IMU_PUBLISH_MS = 20`) |
| Физическое положение | центр робота, 0.405 м от пола (0.305 м над base_link) |

---

## Калиброванные параметры привода

| Параметр | Значение | Примечание |
|---|---|---|
| WHEEL_CIRCUMFERENCE | 0.6158 м | π × 0.196 (rolling-loaded диаметр) |
| GEAR_RATIO | 1.0 | Direct-drive (без редуктора) |
| WHEEL_BASE | 0.3642 м | Откалибровано 2026-06-02 (chassis_calibrate.py) |
| ENCODER_RESOLUTION | 4096 | Шагов на оборот колеса |
| METERS_PER_STEP | ~1.503e-4 м | circumference / (resolution × gear_ratio) |
| MAX_LINEAR_VEL | 0.5 м/с | Ограничение в firmware |
| MAX_ANGULAR_VEL | 1.0 рад/с | Ограничение в firmware |

Замкнутый контур скорости реализован внутри ZLAC8015D; отдельный программный
PID в ESP32/Python отсутствует.

### Коммуникация ESP32 <-> Jetson

| Параметр | Значение |
|---|---|
| Baud rate (micro-ROS) | 921600 |
| Modbus baud (ESP32 ↔ ZLAC) | 115200 |
| Comm-watchdog (ESP32) | 200 мс |
| Comm-watchdog (Python) | 1000 мс |
| CMD_TIMEOUT (ESP32 cmd_vel) | 500 мс |

---

## Ультразвуковые датчики HC-SR04

7 датчиков подключены к ESP32 (esp32_sensors_refab). Порядок в массиве
соответствует пинам прошивки (см. `us_task.cpp`):

| Индекс | Датчик | Пины (trig, echo) | Угол (rpy Z, по base_link) | Назначение |
|---|---|---|---|---|
| 0 | right | G16, G34 | -76° (1.3265 рад) | Боковое препятствие |
| 1 | front_right_outer | G17, G35 | -13.2° (0.2304 рад) | Дуга препятствий |
| 2 | front_right_inner | G18, G32 | -5.7° (0.0995 рад) | Дуга препятствий |
| 3 | front_center | G19, G33 | 0° | Фронтальное обнаружение |
| 4 | front_left_inner | G23, G25 | +5.7° (0.0995 рад) | Дуга препятствий |
| 5 | front_left_outer | G26, G27 | +13.2° (0.2304 рад) | Дуга препятствий |
| 6 | left | G14, G12 | +76° (1.3265 рад) | Боковое препятствие |

ESP32 публикует `std_msgs/Float32MultiArray` на `/ultrasonics`. Узел
`range_converter_node` подписывается на `/ultrasonic/distances` и публикует
7 отдельных `sensor_msgs/Range` на `/verter/distance_sensors/<имя>`.
Параллельный узел `distance_sensors_node` (Arduino Mega, `BAUD_RATE = 9600`)
также публикует Range на `/verter/distance_sensors/<имя>` и IMU на
`/verter/imu/data`.

---

## URDF: положение фреймов

Все координаты относительно `base_footprint` (уровень пола).

| Фрейм | X, Y, Z (м) | Примечание |
|---|---|---|
| base_footprint | 0, 0, 0 | Ноль на полу |
| base_link | 0, 0, 0.10 | Центр шасси (0.10 м от base_footprint) |
| lidar_link | 0, 0, 0.86 | 0.76 м над base_link, повёрнут 180° по Z |
| imu_link | 0, 0, 0.405 | 0.305 м над base_link, центр робота |
| wheel_left | 0, +0.178, 0.10 | 0 м над base_link (на высоте центра колеса) |
| wheel_right | 0, -0.178, 0.10 | 0 м над base_link (на высоте центра колеса) |

---

## ROS2-топики (ESP32-компоненты)

| Топик | Тип | Частота | QoS | Направление |
|---|---|---|---|---|
| `/wheel_encoders` | Int64MultiArray | 50 Гц | BEST\_EFFORT | ESP32 -> ROS |
| `/cmd_vel` | Twist | по мере поступления | RELIABLE | ROS -> ESP32 |
| `/chassis/state` | UInt8 | 2 Гц | RELIABLE | ESP32 -> ROS |
| `/chassis/fault` | UInt32 | 2 Гц | RELIABLE | ESP32 -> ROS |
| `/chassis/cmd_watchdog` | Bool | 2 Гц | RELIABLE | ESP32 -> ROS |
| `/chassis/wheel_rpm` | Int16MultiArray | 20 Гц | BEST\_EFFORT | ESP32 -> ROS |
| `/chassis/modbus_fails` | UInt32 | 2 Гц | RELIABLE | ESP32 -> ROS |
| `/imu/data` | Imu | 50 Гц | BEST\_EFFORT | ESP32 IMU -> ROS |
| `/imu/mag_raw` | Float32MultiArray | 10 Гц | BEST\_EFFORT | ESP32 IMU -> ROS |
| `/ultrasonics` | Float32MultiArray | по опросу | BEST\_EFFORT | ESP32 -> ROS |
| `/odom_raw` | Odometry | 50 Гц | default | chassis\_zlac\_node |
| `/odom` | Odometry | 50 Гц | default | odometry\_node |
| `/scan_raw` | LaserScan | ~10 Гц | default | rplidar_node (Express/Sensitivity) |
| `/scan` | LaserScan | ~10 Гц | default | laser_filter (после AngularBounds ±0.7 рад) |

---

## Питание

| Линия | Потребитель | Источник |
|---|---|---|
| 24 V | Моторы (через ZLAC8015D) | LiFePO4 аккумулятор |
| 12 V | ESP32 (питание линии, E-stop в этой же линии) | DC-DC с аккумулятора |
| 5 V | ESP32, RPLiDAR, HC-SR04, USB-hub | DC-DC с линии 12 V |
| 19 V | Jetson Orin Nano | Отдельный DC-DC / адаптер |

**Правило**: питание Jetson должно быть отделено от моторной линии отдельным DC-DC с фильтрацией. Аппаратный E-stop размыкает линию 12 V — при этом ESP32 холодно перезагружается. Общий ground между ESP32 и TTL-RS485/ZLAC обязателен.
