# ESP32 + micro-ROS: Полная инструкция

Переход с Arduino Mega на ESP32 с micro-ROS для управления шасси робота.

## Содержание

1. [Необходимое оборудование](#1-neobkhodimoe-oborudovanie)
2. [Установка инструментов разработки](#2-ustanovka-instrumentov-razrabotki)
3. [Прошивка ESP32](#3-proshivka-esp32)
4. [Настройка micro-ROS Agent на Jetson](#4-nastroika-micro-ros-agent-na-jetson)
5. [Подключение моторов и энкодеров](#5-podkliuchenie-motorov-i-enkoderov)
6. [Тестирование](#6-testirovanie)
7. [Troubleshooting](#7-troubleshooting)

---

## 1. Необходимое оборудование

### ESP32 плата

**Рекомендуемые варианты:**

| Плата | Цена | Плюсы | Минусы |
|-------|------|-------|--------|
| ESP32-WROOM-32 DevKit | ~$5 | Дешёвая, много GPIO | Базовая |
| ESP32-S3-DevKitC | ~$10 | USB-OTG, больше RAM | Дороже |
| ESP32-WROVER | ~$8 | 8MB PSRAM | Больше плата |

Для начала подойдёт любая **ESP32-WROOM-32 DevKit V1** с AliExpress/Amazon.

### Распиновка ESP32 DevKit 30-pin (Type-C CH340C)

Это плата которую ты купил: ESP32 TYPE-C CH340C

```
              ┌──────────────────────────────┐
              │  [BOOT]   USB-C    [RST]     │
              │                              │
       CLK  ──┤○                            ○├── V5 (5V)
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

⚠️  SD0, SD1, SD2, SD3, CLK, CMD - подключены к Flash, НЕ ИСПОЛЬЗОВАТЬ!
⚠️  G0 - влияет на загрузку, лучше не использовать
```

### Драйвер моторов

Используй тот же **L298N** что и с Arduino, или лучше **TB6612FNG** (эффективнее).

---

## 2. Установка инструментов разработки

### Вариант A: Arduino IDE (проще для начала)

#### 2.1 Установка Arduino IDE

```bash
# Ubuntu/Debian
sudo apt install arduino

# Или скачай с https://www.arduino.cc/en/software
```

#### 2.2 Добавление ESP32 в Arduino IDE

1. Открой Arduino IDE
2. **File → Preferences**
3. В "Additional Board Manager URLs" добавь:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. **Tools → Board → Boards Manager**
5. Найди "esp32" и установи **"ESP32 by Espressif Systems"**

#### 2.3 Установка micro-ROS библиотеки

1. Скачай последний релиз:
   https://github.com/micro-ROS/micro_ros_arduino/releases

2. **Sketch → Include Library → Add .ZIP Library**

3. Выбери скачанный `micro_ros_arduino-X.X.X.zip`

#### 2.4 Настройка платы

**Tools → Board → ESP32 Arduino → ESP32 Dev Module**

Настройки:
- Upload Speed: **921600**
- CPU Frequency: **240 MHz**
- Flash Frequency: **80 MHz**
- Flash Mode: **QIO**
- Flash Size: **4MB**
- Partition Scheme: **Default 4MB with spiffs**
- PSRAM: **Disabled** (если не WROVER)

---

### Вариант B: PlatformIO (рекомендую)

PlatformIO удобнее для серьёзной разработки.

#### 2.1 Установка

```bash
# Установка PlatformIO Core (CLI)
pip install platformio

# Или VS Code + PlatformIO Extension
```

#### 2.2 Структура проекта уже создана

См. `/verter_admin/firmware/esp32_chassis/`

```bash
cd verter_admin/firmware/esp32_chassis
pio run  # Компиляция
pio run --target upload  # Прошивка
```

---

## 3. Прошивка ESP32

### Драйвер CH340C

Твоя плата использует чип CH340C для USB-Serial. На Linux обычно работает из коробки, но на Mac/Windows может понадобиться драйвер:

- **Linux**: Обычно не нужен (встроен в ядро)
- **Mac**: https://github.com/adrianmihalko/ch340g-ch34g-ch34x-mac-os-x-driver
- **Windows**: https://www.wch.cn/download/CH341SER_EXE.html

### Подключение ESP32 к компьютеру

**ВАЖНО**: Используй кабель **Type-A → Type-C**, не Type-C → Type-C! С Type-C кабелем плата может не получать питание.

1. Подключи ESP32 через USB кабель (Type-A → Type-C!)
2. Проверь что устройство определилось:

```bash
# Linux
ls /dev/ttyUSB*
# или
ls /dev/ttyACM*

# Обычно появится /dev/ttyUSB0
```

3. Дай права на порт:
```bash
sudo usermod -a -G dialout $USER
# Перелогинься после этого
```

### Режим прошивки (если не прошивается автоматически)

Некоторые платы требуют ручного входа в режим загрузки:

1. Зажми кнопку **BOOT** (GPIO0)
2. Нажми и отпусти **EN** (Reset)
3. Отпусти **BOOT**
4. Теперь загружай прошивку

### Прошивка через Arduino IDE

1. **Tools → Port** → выбери `/dev/ttyUSB0`
2. **Sketch → Upload** (или Ctrl+U)
3. Дождись "Done uploading"

### Прошивка через PlatformIO

```bash
cd verter_admin/firmware/esp32_chassis

# Прошивка
pio run --target upload --upload-port /dev/ttyUSB0

# Или с мониторингом Serial после прошивки
pio run --target upload && pio device monitor
```

### Мониторинг Serial (для отладки)

```bash
# Arduino IDE: Tools → Serial Monitor (115200 baud)

# PlatformIO
pio device monitor --baud 115200

# Или через screen
screen /dev/ttyUSB0 115200
# Выход: Ctrl+A, K, Y
```

---

## 4. Настройка micro-ROS Agent на Jetson

micro-ROS Agent - это мост между ESP32 и ROS2 на Jetson.

### 4.1 Установка

```bash
# Для ROS2 Humble
sudo apt update
sudo apt install ros-humble-micro-ros-agent
```

Или сборка из исходников (если пакет недоступен):

```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
cd ..
colcon build
source install/setup.bash

# Создание агента
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

### 4.2 Запуск агента (USB Serial)

```bash
# Подключи ESP32 к Jetson через USB
# Найди порт
ls /dev/ttyUSB*

# Запуск агента
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baud 115200

# С verbose логами (для отладки)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 --baud 115200 -v6
```

### 4.3 Добавление в launch файл

```python
# В mapping.launch.py добавить:
from launch.actions import ExecuteProcess

micro_ros_agent = ExecuteProcess(
    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
         'serial', '--dev', '/dev/ttyUSB0', '--baud', '115200'],
    output='screen'
)
```

### 4.4 udev правило для ESP32

```bash
# Создай правило для симлинка
sudo nano /etc/udev/rules.d/99-esp32.rules
```

Содержимое:
```
# ESP32 с чипом CH340C (твоя плата Type-C)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp32_chassis", MODE="0666"

# ESP32 с чипом CP2102 (другие платы)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="esp32_chassis", MODE="0666"
```

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Теперь ESP32 будет доступен как `/dev/esp32_chassis`.

---

## 5. Подключение моторов и энкодеров

### Схема подключения к ESP32 30-pin (Type-C CH340C)

```
ESP32                    L298N                    Motors
─────                    ─────                    ──────
G25 ───────────────────► IN1  ┐
G26 ───────────────────► IN2  ├──► Motor Left
G18 (PWM) ─────────────► ENA  ┘

G27 ───────────────────► IN3  ┐
G14 ───────────────────► IN4  ├──► Motor Right
G19 (PWM) ─────────────► ENB  ┘

GND ───────────────────► GND

(12V от внешнего питания для L298N)


Encoders                 ESP32
────────                 ─────
Encoder Left A ────────► G34 (input only)
Encoder Left B ────────► G35 (input only)
Encoder Right A ───────► G32
Encoder Right B ───────► G4

Encoder VCC ───────────► 3.3V (или 5V через делитель если энкодеры 5V)
Encoder GND ───────────► GND
```

### Визуальная схема подключения

```
                        L298N Motor Driver
                    ┌─────────────────────────┐
                    │  +12V  GND  +5V         │
                    │   │     │    │          │
                    │   ○     ○    ○          │
ESP32               │                         │
┌──────┐            │  ENA IN1 IN2 IN3 IN4 ENB│
│ G18  ├────────────┼───○                     │
│ G25  ├────────────┼───────○                 │
│ G26  ├────────────┼───────────○             │
│ G27  ├────────────┼───────────────○         │
│ G14  ├────────────┼───────────────────○     │
│ G19  ├────────────┼─────────────────────○   │
│ GND  ├────────────┼───○ (общий GND)         │
└──────┘            │                         │
                    │  OUT1 OUT2   OUT3 OUT4  │
                    │   │    │      │    │    │
                    └───┼────┼──────┼────┼────┘
                        │    │      │    │
                    ┌───┴────┴┐  ┌──┴────┴───┐
                    │ Motor L │  │  Motor R  │
                    └─────────┘  └───────────┘
```

### Важно про GPIO на 30-pin плате

| GPIO | Статус | Примечание |
|------|--------|------------|
| G4, G5, G16, G17 | Свободны | Можно использовать |
| G18, G19, G21, G22, G23 | Свободны | Хорошо для PWM |
| G25, G26, G27 | Свободны | ADC2 (не работают при WiFi) |
| G32 | Свободен | ADC1, работает с прерываниями |
| G34, G35 | Input only | Идеально для энкодеров |
| G12, G13 | Осторожно | Влияют на boot |
| G14, G15 | Осторожно | G15 влияет на boot |
| G0, G2 | НЕ использовать | Режим загрузки |
| RXD, TXD | Занят USB | Serial для micro-ROS |
| SD0-SD3, CLK, CMD | НЕТ! | Flash - сломаешь плату |
| SM, SP, EN | Специальные | Не трогать |

---

## 6. Тестирование

### Шаг 1: Проверка связи

```bash
# На Jetson, запусти агент
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6

# В другом терминале проверь ноды
ros2 node list
# Должен появиться /esp32_chassis

# Проверь топики
ros2 topic list
# Должны быть:
# /cmd_vel
# /wheel_encoders
```

### Шаг 2: Тест энкодеров

```bash
# Покрути колёса руками и смотри данные
ros2 topic echo /wheel_encoders
```

### Шаг 3: Тест моторов

```bash
# Отправь команду движения
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Робот должен поехать вперёд

# Стоп
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Шаг 4: Тест телеопа

```bash
ros2 run verter_admin teleop_keyboard
# W - вперёд, S - назад, A/D - повороты
```

---

## 7. Troubleshooting

### ESP32 не определяется как USB устройство

```bash
# Проверь dmesg
sudo dmesg | tail -20

# Если нет устройства - попробуй другой кабель (некоторые только для зарядки)
# Или другой USB порт
```

### Прошивка не загружается

```
A fatal error occurred: Failed to connect to ESP32
```

1. Зажми BOOT, нажми EN, отпусти BOOT - попробуй снова
2. Уменьши скорость загрузки: Upload Speed → 115200

### Agent не видит ESP32

```bash
# Проверь что ESP32 отправляет данные
screen /dev/ttyUSB0 115200
# Должны быть какие-то сообщения

# Проверь baud rate - должен совпадать (115200)
```

### Нода появляется и исчезает

Проблема с таймаутом. В коде ESP32 нужен регулярный `rclc_executor_spin_some()`.

```cpp
// В loop() должно быть:
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
delay(10);  // Не больше!
```

### Моторы не крутятся

1. Проверь питание L298N (12V отдельно)
2. Проверь GND - должен быть общий между ESP32 и L298N
3. Проверь PWM пины - `ledcWrite()` работает только на определённых пинах

---

## Что дальше

После успешного тестирования:

1. **Удали старый `chassis_node.py`** - он больше не нужен
2. **Обнови launch файл** - добавь micro_ros_agent вместо chassis_node
3. **Объедини с sensors** - можно добавить ультразвуковые датчики на тот же ESP32
4. **WiFi режим** - когда USB надоест, переключись на WiFi (см. код в firmware)

---

## Ссылки

- [micro-ROS Arduino](https://github.com/micro-ROS/micro_ros_arduino)
- [ESP32 Arduino Core](https://docs.espressif.com/projects/arduino-esp32/)
- [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent)
- [ESP32 Pinout Reference](https://randomnerdtutorials.com/esp32-pinout-reference-gpios/)
