# Сборка и установка

## 1) Установка ROS2 пакетов

```bash
sudo apt update
sudo apt install ros-humble-twist-mux \
                 ros-humble-laser-filters \
                 ros-humble-slam-toolbox \
                 ros-humble-rplidar-ros \
                 ros-humble-nav2-map-server
```

## 2) Настройка udev правил для устройств

Для стабильной работы Arduino и лидара нужно настроить симлинки через udev.

### 2.1) Узнать информацию об устройствах

```bash
# Посмотреть подключённые устройства
ls /dev/ttyUSB* /dev/ttyACM*

# Узнать vendor/product ID устройства
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct"

# Узнать физический путь USB (для привязки по порту)
udevadm info -q path /dev/ttyUSB0
```

### 2.2) Создать файл правил

```bash
sudo nano /etc/udev/rules.d/99-robot-devices.rules
```

Пример содержимого (адаптируйте под свои устройства):

```
# RPLidar - по vendor ID (работает с любого порта)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"

# Arduino Chassis - по физическому порту USB
SUBSYSTEM=="tty", KERNELS=="1-2.3:1.0", SYMLINK+="arduino_chassis", MODE="0666"

# Arduino Sensors - по физическому порту USB
SUBSYSTEM=="tty", KERNELS=="1-2.4:1.0", SYMLINK+="arduino_sensors", MODE="0666"
```

**Как определить KERNELS:**
```bash
udevadm info -q path /dev/ttyUSB0
# Вывод: /devices/.../1-2.3/1-2.3:1.0/ttyUSB0/...
# Используйте "1-2.3:1.0" как значение KERNELS
```

### 2.3) Применить правила

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2.4) Проверить

```bash
ls -la /dev/rplidar /dev/arduino_chassis /dev/arduino_sensors
```

### 2.5) Добавить пользователя в группу dialout

```bash
sudo usermod -aG dialout $USER
newgrp dialout  # или перелогиниться
```

## 3) Собрать пакет

Из директории workspace (где лежит `verter_admin/`):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -sf ~/verter-robot/verter_admin .

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select verter_admin --symlink-install
source install/setup.bash
```

## 4) Настроить автозагрузку окружения

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 5) Python зависимости

`verter_admin/setup.py` содержит `install_requires`, поэтому при установке/сборке в ROS окружении зависимости должны подтягиваться согласно твоему флоу.

Если ты ставишь Python-зависимости вручную (venv/не-ROS окружение) — сверяйся с `verter_admin/setup.py` и ставь только нужное под твой набор нод (STT/TTS/AI).
