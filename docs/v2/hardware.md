---
description: Полное описание аппаратной части робота Verter — компоненты, ESP32-прошивки, моторные параметры, ультразвуковые датчики, URDF-фреймы
---

# Аппаратная часть

## Обзор комплектующих

| Подсистема | Компонент | Интерфейс | Назначение |
|---|---|---|---|
| Бортовой компьютер | NVIDIA Jetson Orin Nano Super 8 GB | USB 3.2, CSI, NVMe | ROS2 Humble, Nav2, SLAM, AI-ассистент |
| Лидар | RPLiDAR A1M8 | USB-serial (115200) | 2D-сканирование, 180, 12 м |
| Шасси-контроллер | ESP32 (WROOM-32 DevKit) | USB-serial (921600) | Управление моторами, энкодеры AS5600 |
| IMU-контроллер | ESP32 + Trema IMU 9 DOF V2 (Bosch BMX055) | USB-serial (921600) | Акселерометр, гироскоп |
| Ультразвук | 5 x HC-SR04 (передняя дуга) + 2 x HC-SR04 (бока) | GPIO (через Arduino/ESP32) | Обнаружение препятствий |
| Аудио входы | ReSpeaker USB Mic Array | USB 2.0 | Распознавание речи, DOA |
| Аудио выход | Динамик / наушники | 3.5 мм jack | Синтез речи (Piper / Silero) |
| Питание робота | Li-Ion/LiPo аккумулятор | DC-DC конвертеры | 12 V для моторов, 5 V для сенсоров, 19 V для Jetson |

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

Проект шасси: `verter_admin/firmware/esp32_chassis/`

```bash
cd verter_admin/firmware/esp32_chassis
pio run                      # компиляция
pio run --target upload      # прошивка
pio device monitor --baud 115200
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
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32_chassis --baud 921600

# IMU (921600 baud)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32_imu --baud 921600
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
| Левый мотор PWM | GPIO 19 | Cytron MD10C |
| Левый мотор DIR | GPIO 27 | LOW = вперёд |
| Правый мотор PWM | GPIO 18 | Cytron MD10C |
| Правый мотор DIR | GPIO 25 | HIGH = вперёд |
| Левый энкодер SDA | GPIO 21 | AS5600, I2C0 |
| Левый энкодер SCL | GPIO 22 | AS5600, I2C0 |
| Правый энкодер SDA | GPIO 32 | AS5600, I2C1 |
| Правый энкодер SCL | GPIO 4 | AS5600, I2C1 |
| LED статус | GPIO 2 | Индикатор micro-ROS-подключения |

### Схема подключения моторов

```
                        Cytron MD10C
                    ┌──────────────────────┐
                    │  +12V  GND           │
                    │   │     │            │
ESP32               │   ○     ○            │
┌──────┐            │                      │
│ G18  ├────────────┼──► ENA (PWM Left)    │
│ G27  ├────────────┼──► DIR-A (Left)      │
│ G19  ├────────────┼──► ENB (PWM Right)   │
│ G25  ├────────────┼──► DIR-B (Right)     │
│ GND  ├────────────┼──► GND (общий)       │
└──────┘            │                      │
                    │  OUT1  OUT2  OUT3 OUT4│
                    │   │    │    │    │   │
                    └───┼────┼────┼────┼───┘
                        │    │    │    │
                    ┌───┴────┴┐ ┌─┴────┴───┐
                    │ Motor L  │ │ Motor R  │
                    └──────────┘ └──────────┘
```

### Направления движения

| Колесо | DIR (вперёд) | Знак энкодера (вперёд) |
|---|---|---|
| Левый | LOW | -1 |
| Правый | HIGH | +1 |

### Моторный драйвер

| Параметр | Значение |
|---|---|
| Модель | Cytron MD10C (PWM + DIR) |
| PWM-частота | 20 кГц |
| PWM-разрешение | 8 бит (0--255) |
| MAX_PWM | 200 |
| MIN_PWM (мёртвая зона) | 25 |
| MAX_VELOCITY | 0.5 м/с |

### Энкодеры AS5600

| Параметр | Значение |
|---|---|
| Тип | AS5600 магнитный абсолютный |
| Разрешение | 4096 шагов/оборот (12 бит) |
| I2C-адрес | 0x36 |
| Регистр угла | 0x0C (RAW_ANGLE) |
| Левый энкодер | I2C0 (SDA=21, SCL=22) |
| Правый энкодер | I2C1 (SDA=32, SCL=4) |

Проблема: правый энкодер (I2C1) периодически зависает на ~1 с. Причина — баг arduino-esp32 v2.x: `Wire.setTimeout()` не работает для значений < 1000 мс.

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
| Магнитометр | отключён (помехи от моторов) |
| Частота публикации | 50 Гц |
| Физическое положение | центр робота, 0.29 м над base_link |

---

## Калиброванные параметры привода

| Параметр | Значение | Примечание |
|---|---|---|
| WHEEL_CIRCUMFERENCE | 0.576 м | Определяется прокаткой колеса на один оборот |
| GEAR_RATIO | 4.0007 | Передаточное число редуктора |
| WHEEL_BASE | 0.386 м | Расстояние между осями колёс (новая рама) |
| ENCODER_RESOLUTION | 4096 | Шагов на оборот колеса (AS5600, 12 бит) |
| METERS_PER_STEP | ~3.52e-5 м | circumference / (resolution x gear_ratio) |

### PID-регулятор скорости

| Параметр | Значение |
|---|---|
| Kp | 80 |
| Ki | 50 |
| Kd | 5 |
| MAX_INTEGRAL | 100 |
| PID_INTERVAL | 50 мс |
| MAX_PWM_CHANGE | 15 за итерацию (ramp limiter) |

### Коммуникация ESP32 <-> Jetson

| Параметр | Значение |
|---|---|
| Baud rate | 921600 |
| RX buffer | 4096 байт |
| Transport read timeout | 10 мс |
| Transport write | неблокирующий (drop при полном TX) |

---

## Ультразвуковые датчики HC-SR04

| Датчик | Позиция | Угол (по base_link) | Назначение |
|---|---|---|---|
| Front 1 | левый передний угол | +45 градусов | Дуга препятствий |
| Front 2 | левый центр | +22.5 градусов | Дуга препятствий |
| Front 3 | центр спереди | 0 градусов | Фронтальное обнаружение |
| Front 4 | правый центр | -22.5 градусов | Дуга препятствий |
| Front 5 | правый передний угол | -45 градусов | Дуга препятствий |
| Left | левый бок | +90 градусов | Боковое препятствие |
| Right | правый бок | -90 градусов | Боковое препятствие |

Все датчики публикуют на `/ultrasonic/measurements` (sensor\_msgs/msg/Range).

---

## URDF: положение фреймов

Все координаты относительно `base_footprint` (уровень пола).

| Фрейм | X, Y, Z (м) | Примечание |
|---|---|---|
| base_footprint | 0, 0, 0 | Ноль на полу |
| base_link | 0, 0, 0.10 | Центр шасси |
| lidar_link | 0.12, 0, 0.70 | Повёрнут 180 deg по Z |
| imu_link | 0, 0, 0.29 | Центр робота |
| wheel_left | 0, +0.178, 0.05 | Левое колесо |
| wheel_right | 0, -0.178, 0.05 | Правое колесо |

---

## ROS2-топики (ESP32-компоненты)

| Топик | Тип | Частота | QoS | Направление |
|---|---|---|---|---|
| `/wheel_encoders` | Int64MultiArray | 20 Гц | BEST\_EFFORT | ESP32 -> ROS |
| `/cmd_vel` | Twist | по мере поступления | RELIABLE | ROS -> ESP32 |
| `/imu/data` | Imu | 50 Гц | default | ESP32 IMU -> ROS |
| `/odom` | Odometry | 50 Гц | default | odometry\_node |
| `/scan_raw` | LaserScan | ~6 Гц | default | rplidar\_node |
| `/scan` | LaserScan | ~6 Гц | default | laser\_filter |

---

## Питание

| Линия | Потребитель | Источник |
|---|---|---|
| 12 V | Моторы (через Cytron MD10C) | Li-Ion/LiPo аккумулятор |
| 5 V | ESP32, RPLiDAR, HC-SR04, USB-hub | DC-DC с линии 12 V |
| 19 V | Jetson Orin Nano | Отдельный DC-DC / адаптер |

**Правило**: питание Jetson должно быть отделено от моторной линии отдельным DC-DC с фильтрацией. Общий ground между ESP32 и моторным драйвером обязателен.
