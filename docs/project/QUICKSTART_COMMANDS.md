# Команды запуска Verter

## 1. Построить карту (SLAM)

```bash
# Запустить маппинг (hardware + SLAM Toolbox)
ros2 launch verter_admin mapping.launch.py

# В другом терминале — управлять роботом вручную
ros2 run verter_admin teleop_keyboard

# Объезжаем помещение, пока карта не будет полной
```

Или автономное исследование (робот едет сам):

```bash
ros2 launch verter_admin autonomous_mapping_real.launch.py
```

## 2. Сохранить карту

```bash
# Создать директорию (один раз)
mkdir -p ~/maps

# Шаг 1: сохранить SLAM-граф (для возможности дозаполнения)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/maps/my_map'}}"

# Шаг 2: экспортировать как .pgm/.yaml для Nav2
# Вариант А — через скрипт (рекомендуется)
~/verter-robot/verter_admin/scripts/save_map.sh ~/maps/my_map

# Вариант Б — напрямую
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

Результат в `~/maps/`:
```
my_map.posegraph   # SLAM-граф (для продолжения маппинга)
my_map.data        # доп. данные SLAM
my_map.pgm         # изображение карты
my_map.yaml        # метаданные (resolution, origin)
```

## 3. Навигация по готовой карте

```bash
# Запустить навигацию (hardware + AMCL + Nav2 + safety)
ros2 launch verter_admin navigation.launch.py map:=~/maps/my_map.yaml

# ВАЖНО: после запуска в RViz задать начальную позу робота
# кнопкой "2D Pose Estimate", иначе AMCL не локализуется
```

## 4. Управление waypoints через веб-интерфейс

```bash
# Установить rosbridge (один раз на Jetson, если нет)
sudo apt install ros-humble-rosbridge-suite

# Собрать пакеты (после изменений кода)
cd ~/verter-robot/verter_admin
colcon build --packages-select verter_admin_msgs verter_admin
source install/setup.bash

# Запустить навигацию (если ещё не запущена)
ros2 launch verter_admin navigation.launch.py map:=~/maps/my_map.yaml

# В другом терминале — запустить веб-интерфейс с waypoint-менеджером
ros2 launch verter_admin waypoint_ui.launch.py

# Открыть в браузере
# http://<jetson-ip>:8080
```

Что запускает `waypoint_ui.launch.py`:
- `waypoint_manager` (LifecycleNode) — хранит waypoints, управляет Nav2
- `web_server_node` — HTTP-сервер с UI на порту 8080
- `rosbridge_websocket` — WebSocket-мост на порту 9090

Веб-интерфейс умеет:
- **Save Waypoint** — сохранить точку по имени + координатам (x, y, theta в метрах/радианах)
- **Navigate To** — отправить робота к точке по имени
- **Delete** — удалить точку
- **Refresh List** — показать все сохранённые точки
- **Start / Stop Patrol** — запустить/остановить патрулирование по всем точкам

> Координаты задаются вручную в поле формы (x, y, theta в системе карты).
> Текущую позицию робота можно посмотреть через `ros2 topic echo /amcl_pose --once`.

## 5. Waypoints из терминала

```bash
# Сохранить точку
ros2 service call /save_waypoint verter_admin_msgs/srv/SaveWaypoint \
  "{name: 'kitchen', x: 3.5, y: 1.2, theta: 1.57}"

# Навигация к точке по имени
ros2 service call /navigate_to_waypoint verter_admin_msgs/srv/NavigateToWaypoint \
  "{name: 'kitchen'}"

# Удалить точку
ros2 service call /delete_waypoint verter_admin_msgs/srv/DeleteWaypoint \
  "{name: 'kitchen'}"

# Список всех точек
ros2 service call /list_waypoints verter_admin_msgs/srv/ListWaypoints

# Запустить патрулирование
ros2 service call /start_patrol std_srvs/srv/Trigger

# Остановить патрулирование
ros2 service call /stop_patrol std_srvs/srv/Trigger
```

> Сервисы доступны только когда `waypoint_manager` в состоянии ACTIVE.
> Если вернулось `success: false, message: "Node not active"` — нода не активирована.

## 6. Продолжить маппинг существующей карты

```bash
# Запустить в режиме маппинга
ros2 launch verter_admin mapping.launch.py

# Загрузить существующую карту
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: {data: '$HOME/maps/my_map'}}"

# Доехать до новых областей, карта расширится

# Сохранить обновлённую карту
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/maps/my_map_v2'}}"
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map_v2
```

## Диагностика

```bash
# Проверить что лидар публикует
ros2 topic hz /scan

# Проверить одометрию
ros2 topic hz /odometry/filtered

# Проверить локализацию (AMCL)
ros2 topic echo /amcl_pose --once

# Проверить TF дерево
ros2 run tf2_tools view_frames

# Проверить список waypoint-маркеров (RViz)
ros2 topic echo /waypoints/markers --once

# Проверить статус waypoint_manager (lifecycle)
ros2 lifecycle get /waypoint_manager

# Активировать waypoint_manager вручную (если не активирован)
ros2 lifecycle set /waypoint_manager configure
ros2 lifecycle set /waypoint_manager activate

# Посмотреть карту
ros2 topic echo /map --once | head -5

# Проверить rosbridge
ros2 pkg list | grep rosbridge
```
