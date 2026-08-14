# Запуск проекта

В проекте Verter Robot существует восемь launch-файлов. Каждый из них запускает определённый набор ROS2-узлов для конкретной задачи. Понимание назначения каждого файла помогает быстро подобрать нужный режим работы.

## Карта launch-файлов

| Файл | Что делает | Уровень сложности |
|---|---|---|
| `chassis_bringup.launch.py` | Минимальный запуск ESP32-шасси для стендовой отладки | Базовый |
| `mapping.launch.py` | SLAM-картографирование с ручным управлением (teleop) | Основной |
| `nav2_navigation.launch.py` | Навигация по готовой карте (локализация + планирование) | Основной |
| `autonomous_mapping_real.launch.py` | Полностью автономное картографирование (SLAM + Nav2 + Explore Lite) | Продвинутый |
| `main.launch.py` | Голосовой интерфейс (STT + AI-ассистент + TTS) + шасси/LiDAR/EKF | Дополнительный |
| `waypoint_ui.launch.py` | Web-UI для управления путевыми точками | Дополнительный |
| `beta.launch.py` | Запуск только `chassis_zlac_node` (стендовая отладка ZLAC) | Базовый |
| `ssh_keyboard.launch.py` | Заготовка запуска `chassis_zlac_node` (debug) | Базовый |

---

## 1. chassis_bringup.launch.py — Отладка шасси

**Для чего нужен:** Быстрый старт только связи с ESP32-контроллером шасси. Все остальные компоненты (LiDAR, SLAM, Nav2, IMU) не запускаются.

**Когда использовать:**
- Проверка, что ESP32 корректно прошивится и отвечает
- Отладка команд `/cmd_vel` и состояний `/chassis/state`
- Верификация работы micro-ROS после обновления прошивки

**Запуск:**
```bash
ros2 launch verter_admin chassis_bringup.launch.py
```

**Параметры:**
- `esp32_port` — путь к serial-порту ESP32 (по умолчанию `/dev/esp32_chassis`)
- `micro_ros_agent_extra_args` — дополнительные аргументы для micro-ROS agent (например, `-v6` для подробного логирования)

**Что происходит под капотом:** Запускается один `micro_ros_agent`, соединённый с ESP32 через serial-порт. Встроенный bash-цикл автоматически перезапускает агента при сбое (например, при холодном перезапуске ESP32 после E-stop).

---

## 2. mapping.launch.py — Ручное SLAM-картографирование

**Для чего нужен:** Построение карты помещения в реальном времени. Оператор управляет роботом клавиатурой или джойстиком, а SLAM Toolbox строит карту на основе данных LiDAR.

**Когда использовать:**
- Первое картографирование нового помещения
- Обновление карты после изменения планировки
- Когда нужна точная карта и оператор может аккуратно проехать все коридоры

**Запуск:**
```bash
ros2 launch verter_admin mapping.launch.py
```

**Параметры:**
- `lidar_port` — путь к LiDAR (по умолчанию `/dev/rplidar`)
- `esp32_port` — ESP32 шасси (по умолчанию `/dev/esp32_chassis`) — **в данный момент закомментирован**
- `imu_esp32_port` — ESP32 IMU (по умолчанию `/dev/esp32_imu`) — **в данный момент закомментирован**
- `micro_ros_agent_extra_args` — дополнительные аргументы micro-ROS agent — **в данный момент закомментирован**

**Что запускается:**
1. **micro-ROS agent** — связь с ESP32 (шасси + IMU) — *закомментирован*
2. **twist_mux** — мультиплексор команд скорости, арбитрирует источники по приоритетам — *закомментирован*
3. **odometry_node** — вычисляет одометрию по данным энкодеров колёс — *закомментирован*
4. **ekf_node** — фильтр Калмана, объединяет одометрию и IMU в `/odometry/filtered`
5. **robot_state_publisher** — публикует TF-дерево на основе URDF-модели
6. **rplidar_node** — драйвер LiDAR (задержка 3 с для инициализации USB-устройств, режим `Sensitivity`)
7. **laser_filter** — фильтрация сырых сканов LiDAR
8. **slam_toolbox** — строит карту, публикует на `/map` — *закомментирован*

> **Примечание:** в текущем состоянии файла `mapping.launch.py` большая часть узлов (micro-ROS agents, `twist_mux`, `odometry_node`, `slam_toolbox`) закомментирована — активно запускаются только `ekf_node`, `robot_state_publisher`, `rplidar_node` и `laser_filter`. Полный стек картографирования используется через `autonomous_mapping_real.launch.py`.

**После построения карты** сохраните её:
```bash
mkdir -p ~/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/maps/my_map'}}"
```

---

## 3. nav2_navigation.launch.py — Навигация по готовой карте

**Для чего нужен:** Запуск полноценного стека навигации Nav2 на основе заранее построенной карты. Робот локализует себя на карте и может самостоятельно перемещаться к заданным точкам.

**Когда использовать:**
- Автономная навигация по готовой карте
- Перемещение между путевыми точками
- Интеграция с голосовым интерфейсом или web-UI

**Запуск:**
```bash
ros2 launch verter_admin nav2_navigation.launch.py map:=~/maps/my_map.yaml
```

**Параметры:**
- `map` — путь к YAML-файлу карты (**обязательный** при standalone-запуске)
- `params_file` — путь к Nav2-параметрам (по умолчанию `config/nav2/nav2_navigation_params.yaml`)
- `use_sim_time` — использовать время симуляции (по умолчанию `false`)
- `autostart` — автоматический старт узлов Nav2 (по умолчанию `true`)
- `log_level` — уровень логирования (по умолчанию `info`)

**Что запускается:**
1. **map_server** — загружает и хранит карту
2. **amcl** — локализация робота на карте (Монте-Карло)
3. **controller_server** — локальный планировщик траектории (RegulatedPurePursuitController)
4. **planner_server** — глобальный планировщик пути (NavFn)
5. **smoother_server** — сглаживание траекторий
6. **behavior_server** — поведенческие примитивы (разворот, ожидание)
7. **bt_navigator** — BehaviorTree-навигатор
8. **waypoint_follower** — последовательный объезд точек
9. **lifecycle_manager** — управление жизненным циклом всех узлов

> **Примечание:** `velocity_smoother` присутствует в файле и конфиге закомментированным и в активный стек не входит.

**Важно:** После запуска в RViz укажите начальную позу робота с помощью «2D Pose Estimate» — AMCL стартует с фиксированной позой (0, 0, 0) и без корректной инициализации робот не будет знать своё положение.

**Цепочка команд скорости:**
```
bt_navigator/waypoint_follower → controller_server
→ /nav2/cmd_vel → twist_mux → /cmd_vel → ESP32
```

(`velocity_smoother` убран из цепочки — узел закомментирован в launch-файле и конфиге.)

Этот файл также может **включаться** в другие launch-файлы (например, `autonomous_mapping_real.launch.py`) через `IncludeLaunchDescription`.

---

## 4. autonomous_mapping_real.launch.py — Автономное картографирование

**Для чего нужен:** Полностью автономное исследование и построение карты. Робот сам планирует маршруты, объезжает препятствия и строит карту без участия оператора.

**Когда использовать:**
- Картографирование большого помещения без оператора
- Ситуации, когда ручное управление затруднено (длинный маршрут, утомительное путешествие)
- Демонстрация возможностей автономной навигации

**Запуск:**
```bash
ros2 launch verter_admin autonomous_mapping_real.launch.py
```

**Параметры:**
- `lidar_port` — LiDAR (по умолчанию `/dev/rplidar`)
- `esp32_port` — ESP32 шасси (по умолчанию `/dev/esp32_chassis`)
- `imu_esp32_port` — ESP32 IMU (по умолчанию `/dev/esp32_imu`)
- `micro_ros_agent_extra_args` — аргументы micro-ROS agent
- `stop_distance` — расстояние экстренной остановки (по умолчанию `0.15` м)
- `resume_distance` — расстояние для возобновления движения (по умолчанию `0.20` м)

**Что запускается:**
Собственный полный стек картографирования (не через `mapping.launch.py`, а отдельным набором узлов) плюс дополнительные компоненты:
- **micro-ROS agents** — связь с ESP32 (шасси + IMU)
- **twist_mux**, **odometry_node**, **ekf_node**, **robot_state_publisher** — базовый стек
- **rplidar_node** + **laser_filter** — LiDAR
- **slam_toolbox** — построение карты
- **range_converter_node** — конвертация данных ультразвуковых датчиков (`/ultrasonic/distances` → 7× Range в `/verter/distance_sensors/*`)
- **proximity_safety_node** — аварийная остановка при обнаружении близко расположенного препятствия (публикует `/safety/cmd_vel`)
- **Nav2 стек** — включается через `IncludeLaunchDescription` `nav2_navigation.launch.py` с задержкой 18 секунд (ждёт инициализации SLAM и LiDAR); используется params-файл `config/nav2/nav2_mapping_real_params.yaml`
- **explore_lite** — модуль автономного исследования, запускается через 60 секунд

**Порядок запуска компонентов:**
1. `0 с` — micro-ROS, twist_mux, одометрия, EKF, LiDAR, SLAM
2. `18 с` — Nav2-стек (локализация, планирование)
3. `60 с` — Explore Lite (автономное исследование)

Задержки критически важны — SLAM должен успеть инициализироваться и начать строить карту, прежде чем Nav2 попытается локализироваться.

---

## 5. main.launch.py — Голосовой интерфейс

**Для чего нужен:** Запуск AI-компонентов — распознавание речи, обработка команд, AI-ассистент, синтез речи.

**Когда использовать:**
- Голосовое управление роботом
- Интерактивное взаимодействие с AI-ассистентом
- Демонстрация голосовых команд

**Запуск:**
```bash
ros2 launch verter_admin main.launch.py
```

**Что запускается:**
1. **micro-ROS agents** — два агента: сенсорный ESP32 (`esp32_imu`, по умолчанию `/dev/esp32_imu`) и управляющий ESP32 (`esp32_ctrl`, по умолчанию `/dev/esp32_ctrl`, BLE + DOA)
2. **speech_to_text_node** — распознавание речи (Sherpa-ONNX CTC)
3. **recognition_node** — обработка распознанной речи (NLU, определение намерений)
4. **ai_assistant_node** — AI-ассистент (YandexGPT)
5. **silero_tts_node** — синтез речи (Silero TTS). Узел Piper `text_to_speech_node` и `vosk_tts_node` присутствуют в файле закомментированными.
6. **sound_player_node** — воспроизведение звуковых эффектов (PulseAudio)
7. **chassis_zlac_node** — управление моторами через ZLAC8015D по Modbus RTU (конфиг `chassis/params.yaml`)
8. **doa_node** — определение направления на источник звука
9. **rplidar_node** + **laser_filter** — LiDAR (задержка 3 с)
10. **robot_state_publisher** — публикация TF-дерева (URDF)
11. **ekf_node** — фильтр Калмана (`/odometry/filtered`)

**Важно:** Этот launch-файл **не запускает** навигационный стек. Для голосового управления движением запустите `main.launch.py` параллельно с `mapping.launch.py` или `nav2_navigation.launch.py` в отдельном терминале.

**Требуемые переменные окружения** (для AI-ассистента):
```bash
export YANDEX_CLOUD_FOLDER=<ваш_folder>
export YANDEX_CLOUD_API_KEY=<ваш_key>
export YANDEX_CLOUD_MODEL=<модель>
```

---

## 6. waypoint_ui.launch.py — Web-интерфейс путевых точек

**Для чего нужен:** Запуск веб-интерфейса для визуального управления путевыми точками через браузер. Добавление точек на карте, запуск патрулирования, отслеживание состояния.

**Когда использовать:**
- Визуальное управление маршрутами робота
- Добавление и редактирование путевых точек без командной строки
- Мониторинг состояния патрулирования в реальном времени

**Запуск:**
```bash
ros2 launch verter_admin waypoint_ui.launch.py
```

Откройте в браузере: `http://<ip-jetson>:8080`

**Параметры:**
- `waypoints_file` — путь к YAML-файлу для хранения точек (по умолчанию `~/verter-robot/verter_admin/waypoints.yaml`)
- `rosbridge_port` — порт WebSocket-моста (по умолчанию `9090`)
- `web_port` — порт HTTP-сервера (по умолчанию `8080`)
- `nav2_action_server` — имя action-сервера Nav2 (по умолчанию `navigate_to_pose`)

**Что запускается:**
1. **waypoint_manager_node** — обычный `Node` (не LifecycleNode) для CRUD-операций с путевыми точками
2. **web_server_node** — HTTP-сервер статического веб-интерфейса
3. **rosbridge_websocket** — WebSocket-мост между браузером и ROS2

**Важно:** Nav2 должен быть запущен **до** этого файла. Типичная последовательность:
```bash
# Терминал 1 — навигация
ros2 launch verter_admin nav2_navigation.launch.py map:=~/maps/my_map.yaml

# Терминал 2 — web-UI
ros2 launch verter_admin waypoint_ui.launch.py
```

---

## Комбинации запусков

| Сценарий | Launch-файлы | Порядок |
|---|---|---|
| Построение карты вручную | `mapping.launch.py` | Один файл |
| Автономная навигация | `nav2_navigation.launch.py` | Один файл |
| Автономное картографирование | `autonomous_mapping_real.launch.py` | Один файл |
| Голос + навигация | `nav2_navigation.launch.py` + `main.launch.py` | Сначала Nav2, затем голос |
| Waypoint UI | `nav2_navigation.launch.py` + `waypoint_ui.launch.py` | Сначала Nav2, затем UI |
| Полный режим | `nav2_navigation.launch.py` + `main.launch.py` + `waypoint_ui.launch.py` | Nav2 → остальное |

## Безопасность

Все команды движения проходят через **twist_mux** — мультиплексор, который арбитрирует источники команд (`/cmd_vel`) по приоритетам (см. `contracts/motion.py`):
1. **`/safety/cmd_vel`** — экстренная/близостная остановка (приоритет 200, наивысший)
2. **`/teleop_keyboard/cmd_vel`** — teleop с клавиатуры (приоритет 100)
3. **`/teleop_joy/cmd_vel`** — teleop с джойстика (приоритет 90)
4. **`/nav2/cmd_vel`** — автономная навигация Nav2 (приоритет 10)

В режиме автономного картографирования дополнительно работает **proximity_safety_node**, который блокирует движение при обнаружении препятствия ближе `stop_distance` (по умолчанию в launch-файле `0.15` м; в самой ноде по умолчанию `0.20` м), возобновление — после `resume_distance` (`0.20` м в launch, `0.25` м в ноде).



## Запуск отдельных нод

Пример:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run verter_admin speech_to_text_node
ros2 run verter_admin recognition_node
ros2 run verter_admin silero_tts_node
ros2 run verter_admin sound_player_node
```

