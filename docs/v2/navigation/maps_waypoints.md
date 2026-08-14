# Карты и точки (Waypoints)

Руководство по сохранению, загрузке карт SLAM и управлению именованными точками на карте.

---

## Сохранение карты

### Способ 1: Скрипт (рекомендуется)

```bash
cd ~/verter-robot/verter_admin
./scripts/save_map.sh ~/maps/my_map
```

Скрипт вызывает `nav2_map_server map_saver_cli` и сохраняет изображение карты (`.pgm`) и метаданные (`.yaml`) в указанную директорию. Полный граф поз SLAM (`.posegraph`) этим скриптом не сохраняется — для этого используйте сервис `/slam_toolbox/serialize_map` (см. ниже).

### Способ 2: ROS2 сервис

```bash
mkdir -p ~/maps

# Сохранить граф поз SLAM Toolbox (.posegraph)
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: {data: '$HOME/maps/my_map'}}"

# Сохранить как PGM/YAML для Nav2
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Способ 3: RViz

1. Откройте RViz.
2. `Panels -> Add New Panel -> SlamToolboxPlugin`.
3. Нажмите `Save Map`, введите путь к файлу.

---

## Структура сохранённой карты

После сохранения карты `my_map` возможны следующие файлы:

```
~/maps/
+-- my_map.posegraph   # Полный граф SLAM (через /slam_toolbox/serialize_map)
+-- my_map.data        # Вспомогательные данные графа (вместе с .posegraph)
+-- my_map.pgm         # Изображение (чёрно-белое, через map_saver / /slam_toolbox/save_map)
+-- my_map.yaml        # Метаданные (разрешение, origin, вместе с .pgm)
```

Скрипт `save_map.sh` и `map_saver_cli` создают только `.pgm` + `.yaml`. Граф поз (`.posegraph` + `.data`) сохраняется отдельно сервисом `/slam_toolbox/serialize_map`.

### Форматы файлов

| Формат           | Назначение                               |
|-----------------|------------------------------------------|
| `.posegraph`    | Для SLAM Toolbox -- продолжить картирование с этой карты |
| `.pgm` + `.yaml`| Для Nav2 map_server -- статическая карта локализации |
| `.data`         | Внутренние данные SLAM Toolbox            |

**`.posegraph`** -- предпочтительный формат для сохранения промежуточного результата. Содержит полный граф поз с ограничениями; позволяет позже загрузить карту и продолжить исследование.

**`.pgm/.yaml`** -- стандартный ROS-формат (OccupancyGrid). Используется Nav2 для AMCL-локализации. Rastr representation карты.

### Метаданные карты (`.yaml`)

Типичное содержимое:

```yaml
image: my_map.pgm
mode: trinary
resolution: 0.03
origin: [-5.23, -3.87, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

---

## Загрузка карты

### Способ 1: Запуск навигации с картой

```bash
ros2 launch verter_admin nav2_navigation.launch.py map:=~/maps/my_map.yaml
```

Основной способ. Map server загружает карту и публикует на `/map`.

### Способ 2: Загрузка posegraph в SLAM Toolbox

```bash
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: {data: '$HOME/maps/my_map'}}"
```

Загружает сохранённый граф поз -- используется для продолжения картирования существующей карты.

### Способ 3: Автозагрузка в конфиге

Редактировать `config/slam/slam_toolbox_params.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    mode: localization
    map_file_name: /home/user/maps/my_map
    map_start_at_dock: true
```

---

## Режимы SLAM Toolbox

### Mapping (картирование)

```yaml
mode: mapping
```

Создание новой карты или расширение существующей. Строит карту с нуля, оптимизирует граф поз в реальном времени. Публикует TF `map -> odom`. Карта доступна на `/map`.

### Localization (локализация)

```yaml
mode: localization
map_file_name: /home/user/maps/my_map
map_start_at_dock: true
```

Навигация по готовой карте. Использует готовую карту, определяет позицию робота. Карта не изменяется. Работает быстрее и стабильнее.

**На практике:** переключение осуществляется выбором launch-файла:
- `mapping.launch.py` -- режим mapping (SLAM Toolbox).
- `nav2_navigation.launch.py` -- режим localization (AMCL + map_server).

---

## Сервисы SLAM Toolbox

```bash
ros2 service list | grep slam_toolbox
```

| Сервис                                         | Описание                    |
|------------------------------------------------|-----------------------------|
| `/slam_toolbox/save_map`                       | Сохранить карту             |
| `/slam_toolbox/deserialize_map`                | Загрузить карту             |
| `/slam_toolbox/serialize_map`                  | Сериализовать текущую карту |
| `/slam_toolbox/toggle_interactive_mode`        | Интерактивный режим         |
| `/slam_toolbox/clear_queue`                    | Очистить очередь            |

---

## Именованные точки (Waypoints)

Waypoint -- это сохранённая позиция на карте с именем. Примеры: «Ресепшен», «Кабинет 101», «Зарядная станция».

### CRUD через ROS2-сервисы

Точки управляются сервисами ноды `waypoint_manager` (сервисы определены в пакете `verter_admin_msgs`):

| Сервис | Тип | Описание |
|--------|-----|----------|
| `/save_waypoint` | `verter_admin_msgs/srv/SaveWaypoint` | Сохранить точку (name, x, y, theta) |
| `/navigate_to_waypoint` | `verter_admin_msgs/srv/NavigateToWaypoint` | Отправить робота к точке |
| `/list_waypoints` | `verter_admin_msgs/srv/ListWaypoints` | Показать все сохранённые точки |
| `/delete_waypoint` | `verter_admin_msgs/srv/DeleteWaypoint` | Удалить точку |
| `/start_patrol` | `std_srvs/srv/Trigger` | Запустить патрульный обход |
| `/stop_patrol` | `std_srvs/srv/Trigger` | Остановить патрулирование |

### Примеры

```bash
# Сохранить текущую позицию робота (x, y, theta берутся из /amcl_pose)
ros2 service call /save_waypoint verter_admin_msgs/srv/SaveWaypoint \
  "{name: 'Ресепшен', x: 0.5, y: 1.2, theta: 0.0}"

# Посмотреть все точки
ros2 service call /list_waypoints verter_admin_msgs/srv/ListWaypoints

# Отправить робота
ros2 service call /navigate_to_waypoint verter_admin_msgs/srv/NavigateToWaypoint \
  "{name: 'Ресепшен'}"

# Удалить точку
ros2 service call /delete_waypoint verter_admin_msgs/srv/DeleteWaypoint \
  "{name: 'Старый офис'}"
```

### Нода waypoint_manager

`waypoint_manager` реализована как обычная нода (`rclpy.node.Node`, не LifecycleNode) с ROS2-сервисным интерфейсом. Навигация к точке выполняется через Nav2 action `navigate_to_pose`. Нода запускается launch-файлом `waypoint_ui.launch.py`.

```bash
# Проверить, что нода запущена
ros2 node list | grep waypoint_manager

# Параметр waypoints_file (по умолчанию ~/verter-robot/verter_admin/waypoints.yaml)
# задаёт файл хранения точек.
```

### YAML-формат файла точек

Файл `waypoints.yaml` (по умолчанию `~/verter-robot/verter_admin/waypoints.yaml`). Текущий формат — отображение имени в поля `x`, `y`, `theta`:

```yaml
"Кабинет директора":
  x: 5.2       # метры от origin карты
  y: 3.1       # метры от origin карты
  theta: 1.57  # радианы (0=восток, 1.57=север, 3.14=запад)

"Ресепшен":
  x: 0.5
  y: 1.2
  theta: 0.0
```

Поддерживается и обратная совместимость при загрузке со старым форматом (список `waypoints:` с полями `name`, `x`, `y`, `yaw`), но новые точки сохраняются в текущем формате.

| Поле    | Тип    | Описание                                |
|---------|--------|-----------------------------------------|
| (ключ)  | string | Уникальное имя точки                    |
| `x`     | float  | Координата X в метрах (система `map`)   |
| `y`     | float  | Координата Y в метрах (система `map`)   |
| `theta` | float  | Ориентация (yaw) в радианах             |

> Примечание: поле `description` моделью данных не поддерживается — сервис `SaveWaypoint` принимает только `name`, `x`, `y`, `theta`.

### Патрульные маршруты

Патрульный маршрут -- последовательный обход всех сохранённых точек (в порядке добавления). Запускается и останавливается сервисами `/start_patrol` / `/stop_patrol`:

```bash
# Запустить патрулирование
ros2 service call /start_patrol std_srvs/srv/Trigger "{}"

# Остановить патрулирование
ros2 service call /stop_patrol std_srvs/srv/Trigger "{}"
```

Точки обходятся в порядке их добавления в хранилище. Политика патрулирования (`patrol_warn_threshold`/`patrol_fault_threshold`) отслеживает повторные неудачи достижения точек и переводит маршрут в состояние `DEGRADED`/`FAULT`.

Для разового обхода нескольких точек можно последовательно вызывать `/navigate_to_waypoint`:

```bash
ros2 service call /navigate_to_waypoint verter_admin_msgs/srv/NavigateToWaypoint "{name: 'Ресепшен'}"
# Дождаться прибытия...
ros2 service call /navigate_to_waypoint verter_admin_msgs/srv/NavigateToWaypoint "{name: 'Кабинет 1'}"
```

### Web-интерфейс

Помимо командной строки, waypoints управляются через веб-интерфейс:

```bash
# Сначала запустите навигацию
ros2 launch verter_admin nav2_navigation.launch.py map:=~/maps/my_map.yaml

# В другом терминале
ros2 launch verter_admin waypoint_ui.launch.py
```

Откройте `http://<ip-jetson>:8080` в браузере. Web-интерфейс позволяет добавлять точки кликом на карте, запускать патрулирование и отслеживать состояние в реальном времени.

---

## Полезные команды

### Просмотр сохранённых карт

```bash
ls -1 ~/maps/*.posegraph | xargs -n 1 basename | sed 's/.posegraph//'
```

### Открыть изображение карты

```bash
eog ~/maps/my_map.pgm
```

### Метаданные карты

```bash
cat ~/maps/my_map.yaml
```

### Конвертация в PNG

```bash
convert ~/maps/my_map.pgm ~/maps/my_map.png
```

### Резервное копирование

```bash
# Копия карт
cp -r ~/maps ~/maps_backup_$(date +%Y%m%d)

# Копия waypoints
cp ~/verter-robot/verter_admin/waypoints.yaml waypoints_backup.yaml

# Перенести на другой робот
scp ~/verter-robot/verter_admin/waypoints.yaml robot2:~/verter-robot/verter_admin/
```

---

## Рекомендации

1. **Ориентация при сохранении точки важна.** Разверните робота в нужную сторону перед сохранением.
2. **Используйте понятные имена.** «Кабинет 101», «Зарядная станция» -- хорошо; «Точка 1» -- плохо.
3. **Тестируйте новые точки сразу.** После создания вызовите `/navigate_to_waypoint` и убедитесь, что робот приезжает правильно.
4. **Создайте базовые точки:** стартовая позиция, зарядная станция, безопасное место парковки.
5. **Сохраняйте карту перед выключением.** SLAM Toolbox не сохраняет автоматически.
6. **Разные карты для разных этажей.** Именуйте `building_A_floor_1`, `building_A_floor_2`.

---

## Troubleshooting

### Карта не сохраняется

```bash
mkdir -p ~/maps
ros2 node list | grep slam_toolbox
```

### Робот не локализуется

1. Установите начальную позицию в RViz (2D Pose Estimate).
2. AMCL стартует с фиксированной позой (0, 0, 0) -- без указания реальной позиции локализация не сработает.
3. Проверьте: `ros2 topic hz /scan` и `ros2 topic hz /map`.

### «Не удалось получить позицию робота» (сохранение точки)

```bash
ros2 topic hz /map
ros2 run tf2_tools view_frames
# Убедитесь, что цепочка map -> odom -> base_footprint существует
```

### «Точка не найдена»

Проверьте точное имя с учётом регистра:

```bash
ros2 service call /list_waypoints verter_admin_msgs/srv/ListWaypoints
ros2 service call /navigate_to_waypoint verter_admin_msgs/srv/NavigateToWaypoint "{name: 'Кабинет директора'}"   # правильно
ros2 service call /navigate_to_waypoint verter_admin_msgs/srv/NavigateToWaypoint "{name: 'Кабинет Директора'}"   # неправильно
```

### «Робот не может добраться до точки»

1. Точка в недоступном месте (за стеной) -- пересохраните в доступной позиции.
2. Препятствия на пути -- обновите карту.
3. Проверьте маршрут в RViz (отобразите `/plan`).

### «Робот приехал, но смотрит не туда»

1. Отвезите робота вручную: `ros2 run verter_admin teleop_keyboard`.
2. Разверните точно в нужную сторону.
3. Пересохраните, вызвав сервис `/save_waypoint` с текущими координатами из `/amcl_pose`.
