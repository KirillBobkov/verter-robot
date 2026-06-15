# Настройка Ubuntu + ROS2

Установка ROS2 Humble на чистую Ubuntu 22.04.

---

## 1. Настроить локаль

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

## 2. Подключить репозиторий ROS2

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

## 3. Установить ROS2 Humble

```bash
sudo apt update && sudo apt upgrade
```

!!! warning
    Перед установкой ROS2 обязательно выполните `apt upgrade` — без этого на свежей Ubuntu 22.04 могут удалиться системные пакеты (systemd, udev). Подробнее: [ros2/ros2#1272](https://github.com/ros2/ros2/issues/1272).

```bash
sudo apt install ros-humble-desktop
```

`ros-humble-desktop` включает:

| Компонент | Назначение |
|---|---|
| ROS2 core (rclcpp, rclpy) | Базовый фреймворк |
| RViz2 | Визуализация (карты, TF, лазер) |
| Nav2 | Навигация и планирование |
| robot_state_publisher | Публикация TF из URDF |
| joint_state_publisher | Состояния сочленений |
| демо-ноды | Проверка что ROS2 работает |

## 4. Установить пакеты проекта

Пакеты, специфичные для Verter Robot:

```bash
sudo apt install ros-humble-twist-mux \
                 ros-humble-laser-filters \
                 ros-humble-slam-toolbox \
                 ros-humble-rplidar-ros \
                 ros-humble-nav2-map-server \
                 ros-humble-robot-localization \
                 ros-humble-rosbridge-suite \
                 ros-humble-explore-lite
```

| Пакет | Зачем |
|---|---|
| `twist-mux` | Арбитраж команд скорости (safety-gated chain) |
| `laser-filters` | Фильтрация данных лидара |
| `slam-toolbox` | Одновременная локализация и картографирование |
| `rplidar-ros` | Драйвер RPLiDAR A1M8 |
| `nav2-map-server` | Загрузка/сохранение карт |
| `robot-localization` | EKF-слияние одометрии и IMU |
| `rosbridge-suite` | WebSocket-мост для веб-интерфейса |
| `explore-lite` | Автономное исследование помещений |

## 5. Установить инструменты разработки

```bash
sudo apt install ros-dev-tools
```

`ros-dev-tools` включает: colcon, rosdep, ros2bag, и другие утилиты для сборки и отладки ROS2-пакетов.

## 6. Проверить установку ROS2

```bash
# В одном терминале
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# В другом терминале
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Должны увидеть обмен сообщениями между talker и listener — ROS2 работает.
