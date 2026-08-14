# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **Источник истины — код.** `docs/v2/` местами устарел; при расхождениях верить коду. Ссылки на docs ниже помечены ⚠️, где нужна осторожность.

## Project Overview

**Verter Robot** — автономный мобильный робот для навигации в помещениях (медицинские учреждения, торговые центры, офисы). ROS2 Humble на NVIDIA Jetson Orin Nano Super 8GB.

**Стек**: ROS2 Humble, Nav2, SLAM Toolbox, micro-ROS (ESP32), React + TypeScript web UI, Sherpa-ONNX STT, YandexGPT AI, Silero/Piper/Vosk TTS.

**Железо**: Jetson Orin Nano Super 8GB; RPLiDAR A1M8; 7× HC-SR04 ультразвук; MPU-6050/BMX055 IMU; ReSpeaker USB mic array с DOA; ZLAC8015D dual servo driver (Modbus RTU); LiFePO4 24V.

## Architecture (SDHR)

Проект следует **Safety-Driven Hexagonal Robotics** из `.claude/rules/robotics_architecture_rulebook.md`:

- **L0 Safety/Actuation** (MCU): E-stop, command timeout — работает без mission-стека.
- **L1 Control/Localization**: детерминированное управление и оценка состояния.
- **L2 Planning/Behavior**: заменяем без изменения L0/L1 контрактов.
- **L3 Mission/HMI/Cloud**: не в прямом actuator-пути.

**Hexagonal boundaries** (per package): `domain` (инварианты, state machine, политики — zero ROS imports) → `application` (use-cases) → `adapters` (ROS topics/services/actions, драйверы) → `infrastructure` (launch, config). Зависимости: domain→(none), application→domain, adapters→domain/application, infrastructure→adapters. Запрещено domain/application импортировать adapters/infrastructure. Буквально соблюдают `waypoints` и `control`; голосовые/web-модули — плоская структура.

## Контракты командного пути

Канонический источник имён — `src/verter_admin/contracts/motion.py`. Править имена/приоритеты здесь, не в нодах.

**`TopicContract`**:
- `SAFETY_CMD_VEL = '/safety/cmd_vel'`
- `TELEOP_CMD_VEL = '/teleop_keyboard/cmd_vel'`
- `TELEOP_JOY_CMD_VEL = '/teleop_joy/cmd_vel'`
- `TELEOP_LOCK = '/teleop_keyboard/lock'`
- `NAV2_CMD_VEL = '/nav2/cmd_vel'`
- `FINAL_CMD_VEL = '/cmd_vel'`
- `SCAN = '/scan'`
- `ULTRASONIC_SCAN = '/ultrasonic/ranges'`

**`ArbitrationPolicy`** (приоритеты twist_mux): `SAFETY=200`, `TELEOP=100`, `TELEOP_JOY=90`, `NAV2=10`, `TELEOP_LOCK=100`.

**`MotionTimeouts`**: `TWIST_MUX_SOURCE_SEC=0.5`, `TWIST_MUX_LOCK_SEC=0.0`, `SENSOR_FRESHNESS_SEC=0.8`, `CHASSIS_WATCHDOG_MS=500`.

## Цепочка безопасности / арбитраж

```
teleop/joy/nav2 ──► twist_mux (priorities) ──► /cmd_vel ──► chassis
                         ▲
   /safety/cmd_vel (priority 200, highest)
   proximity_safety_node блокирует линейное движение у препятствий
```

`twist_mux` конфигурируется через `build_twist_mux_parameters()` из `control/infrastructure/`.

`proximity_safety_node` (`control/adapters/ros/proximity_safety_node.py`): подписан на `/scan` + `/ultrasonic/ranges` + teleop + nav2, публикует `/safety/cmd_vel`, цикл управления 20Hz, гистерезис stop/resume. Доменная модель: `SafetyState` (READY/DEGRADED/FAULT), `EvaluateSafety`, `clamp_linear_motion` при FAULT — блокирует линейное движение, сохраняет поворот на месте для объезда.

**Нюанс**: `/ultrasonic/ranges` (LaserScan) **не публикуется ни одной нодой** — `range_converter_node`/`distance_sensors_node` пишут `Range` в `/verter/distance_sensors/*`. До устранения safety работает только по лидару.

## Навигация / одометрия

**Одометрия** (`control/`): `odometry_node.py` подписан на `/wheel_encoders` (Int64MultiArray, BEST_EFFORT) → `odometry_policy.py` трапециевидная интеграция энкодеров → pub `/odom` 50Hz, TF `odom→base_footprint`. Константы (`odometry_policy.py`): `wheel_base=0.3642`, `wheel_circumference=π×0.196=0.6158`, `encoder_resolution=4096`, `meters_per_step≈1.503e-4`, `max_linear_velocity=0.5`, `max_angular_velocity=1.0`. `publish_tf=False` в launch — TF владеет EKF.

**EKF** (`config/robot_localization/ekf.yaml`, 50Hz, 2D): `odom0: /odom_raw` (velocity: vx, vy, vyaw = true; position = false), `odom0_differential: false`, `world_frame: odom`, `base_link_frame: base_footprint`, `publish_tf: true`.

**Нюанс (противоречие в конфиге)**: заголовок `ekf.yaml` говорит "IMU DISABLED 2026-06-04", но `imu0: /imu/data` (стр.84) **раскомментирован**. Фактическое состояние проверять в файле.

**SLAM Toolbox** (`config/slam/slam_toolbox_params.yaml`): mode=mapping, resolution=0.03, max_laser_range=8.0.

**Nav2** (`config/nav2/nav2_navigation_params.yaml`): AMCL (min/max_particles 500/2000, laser_max_range 5.5, update_min_d/a=0.05/0.05, DifferentialMotionModel); RegulatedPurePursuitController (desired_linear_vel=0.04, controller_frequency=10.0, xy_goal_tolerance=0.10, yaw_goal_tolerance=0.10); local_costmap 4×4м update 10Hz, global 1Hz; robot_radius=0.25, inflation_radius=0.40, cost_scaling_factor=3.0. **Ultrasonic layer в Nav2 отсутствует.**

**laser_filter** (`config/laser_filters/`): AngularBoundsFilterInPlace ±0.7 рад ≈ передняя дуга ~80° (**не 180°**).

**URDF** (`urdf/verter_robot_minimal.urdf`): frames `map→odom→base_footprint→base_link`; base_link z=0.1; lidar_link z=0.86м от пола; imu_link z=0.405м; 7 ultrasonic frames; wheel_radius=0.1.

**Нюанс**: в `nav2_navigation.launch.py` velocity_smoother и ремап `cmd_vel→/nav2/cmd_vel` **закомментированы** → controller_server пишет прямо в `/cmd_vel`, минуя twist_mux. При standalone-запуске Nav2 цепочка безопасности разорвана; twist_mux управляет `/cmd_vel` только когда запущен отдельно (через `autonomous_mapping_real.launch.py`).

Ссылки: `docs/v2/navigation/architecture.md` ⚠️ (архитектурный обзор, параметры сверять с конфигами), `docs/v2/navigation/maps_waypoints.md` ⚠️ (waypoint shell-скрипты не реализованы). **Не использовать** `docs/v2/odometry.md` (устарел: описывает cmd_vel-одометрию).

## Шасси

Две реализации ZLAC8015D (Modbus RTU):

**(а) Python `chassis_zlac_node`** (`chassis/chassis_zlac_node.py` + `chassis/params.yaml`): `/cmd_vel`→`/odom_raw`, state machine INIT/READY/DRIVER_FAULT, порт `/dev/chassis`, `max_motor_rpm=50`, watchdog регистр `0x2000`/1000мс, `wheels.base=0.356`.

**(б) ESP32 micro-ROS firmware** (`firmware/esp32_chassis/esp32_chassis_modbus/`): L1-посредник между Jetson (micro-ROS USB CDC) и ZLAC (UART2 RS485 @115200). State machine INIT→WAIT_AGENT→READY→ACTIVE⇄SAFE_STOP→FAULT. `CMD_TIMEOUT_MS=500` (cmd_vel watchdog), `REG_COMM_WATCHDOG_MS=0x2007`/200мс (ZLAC comm-watchdog), `MAX_MOTOR_RPM=200`, порт `/dev/esp32_chassis`, `LEFT_SIGN=+1`/`RIGHT_SIGN=-1`, pub `/wheel_encoders` (50Hz, BEST_EFFORT), `/chassis/*`.

**Кинематика**: `WHEEL_BASE=0.3642`, `WHEEL_DIAMETER=0.196` (firmware + odometry_policy), `params.yaml=0.356` (третье значение). Регистры помечены `TODO VERIFY`.

**Нюанс**: три разных udev-имени в трёх местах (`/dev/chassis`, `/dev/esp32_chassis`, `/dev/zlac_chassis` в доке); разные регистры/периоды watchdog между Python и firmware.

Ссылка: `docs/v2/chassis_wiring.md` ⚠️ (лучший аппаратный референс, но udev-имя `/dev/zlac_chassis` не совпадает с кодом). **Не использовать** `docs/v2/hardware.md` (описывает старое шасси Cytron+AS5600+PID).

## Голосовой пайплайн

**FSM `recognition_node`** (`recognition/recognition_node.py`): `LISTENING_FOR_TRIGGER→CAPTURING_COMMAND→DIALOG_MODE→PAUSED`. Триггер-слова "вертер"/"робот" (15+ фонетических вариантов), стоп-слова. Публикует `verter_commands` (String). **Голосовое управление движением присутствует в коде** — формат `CHASSIS:LEFT:ASK:dist:pwm` (ASK=вперёд, DESK=назад): `default_distance=0.5`, `default_pwm=98`, `turn_distance=0.25`. Приоритет: stop→chassis→AI.

**STT×3** (`speech_to_text/`): CTC (`speech_to_text_node.py`, активен в `main.launch.py`), Sherpa (`speech_to_text_sherpa_node.py`), Transducer (`speech_to_text_transducer_node.py`). `speech_to_text_parakeet_node` зарегистрирован в `setup.py`, но **.py отсутствует**.

**AI** (`ai_assistant/ai_assistant_node.py`): YandexGPT через OpenAI SDK, `VECTOR_STORE_NAME="verter-medical-index"`, `tool_choice={"type":"required"}`, `DEFAULT_TEMPERATURE=0.3`, `DEFAULT_MAX_TOKENS=300` (он же `max_output_tokens`).

**TTS×3** (`text_to_speech/`): Silero (`silero_tts_node.py`, активен — speaker=`eugene`, `sample_rate=24000`, `model_version=v5_5_ru`), Piper (`text_to_speech_node.py`), Vosk (`vosk_tts_node.py`). Все публикуют `tts_control` (Bool) — глушит STT + recognition на время синтеза.

**Звук/DOA**: `sound_player_node` (`/text_to_speech`→аудио), `doa_node` (DOA_POLLING_INTERVAL=7.0с, Arduino Nano через serial, подписан на `head_commands`).

**Нюанс**: `YANDEX_CLOUD_API_KEY=''` в `main.launch.py` — AI требует ключ в runtime.

Ссылки: `docs/v2/voice/*` ⚠️ (осторожно: активен Silero не Piper; max_tokens=300 не 350; Vosk TTS не задокументирован). **Не использовать** `docs/v2/voice/refactor_notes.md` (критически устарел: утверждает, что голосовое управление движением удалено).

## Waypoints

Hexagonal-пакет `waypoints/`:
- **domain**: `WaypointStore`, `PatrolPolicy` (`PatrolState`: IDLE/PATROLLING/DEGRADED/FAULT).
- **application**: `SaveWaypoint`/`Navigate`/`PatrolUseCase`.
- **infrastructure**: `YamlStore` (атомарная запись).
- **adapters**: `WaypointManagerNode` → Nav2 `NavigateToPose` action (единственный actuator-interface, C1).

Сервисы: `/save_waypoint`, `/navigate_to_waypoint`, `/delete_waypoint`, `/list_waypoints` (кастомные `verter_admin_msgs`) + `/start_patrol`, `/stop_patrol` (`std_srvs/Trigger`).

`verter_admin_msgs` (ament_cmake, отдельный пакет): 4 srv — `SaveWaypoint`, `NavigateToWaypoint`, `DeleteWaypoint`, `ListWaypoints`. Должен собираться перед `verter_admin`.

**Нюанс**: `WaypointManagerNode` — plain `Node`, не LifecycleNode (намеренно конвертирован: "to avoid lifecycle-manager deadlock"). При этом integration-тесты могут вызывать `trigger_configure/activate` — проверить актуальность тестов.

## Web UI

`rosbridge_server` (порт 9090) + `web_server_node` (порт 8080) + React/Zustand фронтенд (`web/frontend/`). Hooks: `useROS`, `usePose`, `useCmdVel`, `useIdleTimer`; сервисы в `services/rosbridge.ts`. State: zustand.

**Нюанс**: фронтенд публикует в `/cmd_vel` **напрямую** (`rosbridge.ts: publish('/cmd_vel', ...)`, EmergencyStop, useCmdVel 10Hz, MappingPage) — нарушает QG-8, но gate проверяет только `*.py`, не `.ts/.tsx`. Прогресс навигации симулируется на фронтенде. i18n объявлен в `package.json`, но не настроен.

## Карта launch-файлов (8 файлов)

| Launch | Что запускает |
|---|---|
| `main.launch.py` | Продакшн: голос + сенсоры + EKF + `chassis_zlac_node` (Python). **Без** Nav2/SLAM/twist_mux. systemd `verter-admin` → сюда. |
| `nav2_navigation.launch.py` | Nav2-стек. velocity_smoother убран, `cmd_vel` не ремапится. |
| `autonomous_mapping_real.launch.py` | Полный автономный стек: twist_mux + proximity_safety + SLAM + Nav2 (TimerAction 18с) + explore_lite (TimerAction 60с). |
| `mapping.launch.py` | Большая часть **закомментирована** (twist_mux/odometry/slam_toolbox/micro_ros). Активны только EKF+robot_state_publisher+rplidar+laser_filter. |
| `waypoint_ui.launch.py` | Standalone (C9): rosbridge + waypoint_manager + web_server, порт 8080. |
| `chassis_bringup.launch.py` / `ssh_keyboard.launch.py` | Только micro_ros_agent. |
| `beta.launch.py` | Сломан (`config` не определён). |

**Нюансы**: `config/nav2/nav2_mapping_real_params.yaml` **пуст** (0 байт) → Nav2 в `autonomous_mapping_real` запускается с дефолтами; скрипт `verter` ссылается на `navigation.launch.py`, которого нет (есть `nav2_navigation.launch.py`).

Ссылка: `docs/v2/launch_files.md` ⚠️ (структура верна, детали устарели).

## Build / Tests / CI

```bash
# Сборка (msgs перед verter_admin)
cd ~/verter-robot/verter_admin
colcon build --packages-select verter_admin_msgs
colcon build --packages-select verter_admin
colcon build --symlink-install --packages-select verter_admin_msgs verter_admin
source install/setup.bash
```

**Frontend** (`src/verter_admin/web/frontend/`): `npm install` → `npm run build` (**обязателен до colcon**, `dist/` пакуется в share) → `npm run lint` (ESLint, `--max-warnings 0`). Dev: `npm run dev` (:3000, proxy /rosbridge → ws://localhost:9090).

**Unit-тесты** (чистый domain, без rclpy):
```bash
PYTHONPATH=src pytest tests/unit/ -v
PYTHONPATH=src pytest tests/unit/waypoints/test_waypoint_store.py -v
```

**CI** (`.github/workflows/verify-verter-admin-mvp.yml`, на каждый PR по `verter_admin/**`):

| Gate | Проверка |
|---|---|
| QG-1 | `python3 -m compileall -q src/verter_admin` |
| QG-2 | grep: zero ROS imports в `domain/`+`application/` |
| QG-3 | grep: `domain/`+`application/` не импортируют `adapters`/`infrastructure` |
| QG-4 | `PYTHONPATH=src pytest tests/unit/waypoints/` |
| QG-5 | `colcon build` — отложен на Jetson (CI без ROS toolchain) |
| QG-8 | `grep -rn --include="*.py" "cmd_vel"` в `waypoints/`+`web/` (**не ловит .ts/.tsx**) |
| C9 | `waypoint_ui.launch.py` standalone (без `IncludeLaunchDescription` навигации) |

Linter: `ruff`. `setup.py`: 21 entry point, `data_files` bundle моделей (Sherpa/Silero VAD/Piper/Silero TTS), `web/dist/`, звуки — должны существовать до сборки.

## Quick Start

### Полный голосовой стек (продакшн, на роботе)
```bash
systemctl --user start verter-admin          # или: ros2 launch verter_admin main.launch.py
systemctl --user status verter-admin
journalctl --user -u verter-admin -f
```

### Mapping
```bash
ros2 launch verter_admin mapping.launch.py                # ручной + teleop
ros2 launch verter_admin autonomous_mapping_real.launch.py
~/verter-robot/verter_admin/scripts/save_map.sh ~/maps/my_map
```

### Navigation
```bash
# launch файл — nav2_navigation.launch.py (не navigation.launch.py)
ros2 launch verter_admin nav2_navigation.launch.py map:=~/maps/my_map.yaml
```

### Waypoint Web UI
```bash
# waypoint_ui.launch.py — standalone. Навигацию запускать отдельно.
ros2 launch verter_admin nav2_navigation.launch.py map:=~/maps/my_map.yaml
ros2 launch verter_admin waypoint_ui.launch.py
# http://<jetson-ip>:8080
```

### Chassis-only bringup
```bash
ros2 launch verter_admin chassis_bringup.launch.py
```

## Environment Variables

Для AI-ассистента (runtime): `YANDEX_CLOUD_FOLDER`, `YANDEX_CLOUD_API_KEY`, `YANDEX_CLOUD_MODEL`. В `main.launch.py` ключ пуст — задаётся окружением.

## Documentation

Русская, MkDocs, в `docs/` (deploy в GitHub Pages через `.github/workflows/deploy-mkdocs.yml`). **Местами устарел** — сверять с кодом:
- `docs/v2/chassis_wiring.md` ⚠️, `docs/v2/launch_files.md` ⚠️ — частично актуальны.
- `docs/v2/navigation/architecture.md` ⚠️ — архитектурный обзор, параметры сверять с конфигами.
- `docs/v2/voice/*` ⚠️ — осторожно (Silero не Piper, max_tokens=300).
- `docs/v2/odometry.md`, `docs/v2/hardware.md`, `docs/v2/voice/refactor_notes.md` — **устарели, не использовать**.
- `.claude/rules/robotics_architecture_rulebook.md` — SDHR-принципы (нормативный документ).
