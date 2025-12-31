# `distance_sensors_node` + конвертеры в LaserScan

## Задача

Собирает данные низкорасположенных ультразвуковых датчиков (HC‑SR04) и IMU с Arduino Mega, публикует их в ROS2.

Дальше отдельные ноды конвертят 7 `Range`-топиков в единый `LaserScan`, чтобы Nav2 costmap видел низкие препятствия (ниже лидара).

## `distance_sensors_node`

### Публикует

- `/verter/distance_sensors/*` (sensor_msgs/Range) — 7 датчиков:
  - `front_center`, `front_left_inner`, `front_left_outer`, `front_right_inner`, `front_right_outer`, `left`, `right`
- `/verter/imu/data` (sensor_msgs/Imu)
- `verter_commands` (String) — если нужно отправлять команды на Arduino (зависит от логики внутри ноды)

### Простыми словами

- Подключается по serial к Arduino Mega
- Читает строки/пакеты сенсоров
- Превращает это в ROS2 сообщения Range/Imu с корректными `frame_id`

## `range_to_laserscan`

Конвертит 7 `sensor_msgs/Range` (реальный робот) → один `sensor_msgs/LaserScan` в `/ultrasonic/ranges`.

## `laserscan_merger`

То же самое, но для Gazebo, где ультразвуки уже публикуются как `LaserScan`.


