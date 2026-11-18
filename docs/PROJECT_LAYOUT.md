# Структура и устройство системы

Все пути ниже относительны `/home/jetson/verter-robot`. Код ROS, самостоятельные Python-сервисы, прошивки, конфигурация и данные разделены, поскольку имеют разные зависимости и способы запуска.

## Каталоги

| Каталог | Содержимое |
|---|---|
| `ros_ws/src/` | ROS-пакеты; сборка colcon |
| `services/nats_bridge/` | NATS API, метрики, ручное управление, отладочная страница и BLE BMS |
| `services/lidar/` | Пассивное чтение /scan и TF, поток NATS и окно 8090 |
| `services/camera/` | Захват Gemini 2, ROS, WebRTC и HTTP-видео |
| `services/wifi-connect/` | Утилита и страница настройки Wi-Fi |
| `firmware/` | Рабочие и диагностические PlatformIO-проекты ESP32 |
| `web/frontend/` | React/TypeScript интерфейс на 8081; это не NATS-мониторинг |
| `config/` | Настройки служб, приватные параметры и `assets_manifest.json` для проверки ресурсов |
| `deploy/` | Подготовка ОС, Python-зависимостей, systemd, udev; Ansible |
| `tools/` | Сборка, проверки, CLI и supervisor |
| `assets/` | Модели речи, звуки, dataset |
| `vendor/` | micro-ROS Agent, NATS server, Orbbec SDK |
| `data/` | Карты, сохранённые точки, JetStream/KV, SQLite, данные/логи BMS/NATS, результаты проверок в `diagnostics/` и приватные резервные файлы установщика камеры в `backups/camera-install/` |
| `run/` | Сокет supervisor, lock и журналы ROS-групп |
| `tests/` | Тесты кода, геометрии и управления процессами |
| `docs/` | Актуальные инструкции и контракты |
| `design/cad_preview/` | Отдельный CAD-просмотр; не активная модель аппаратного запуска |

`ros_ws/build`, `ros_ws/install`, `ros_ws/log` создаёт colcon. Редактируют `src`, а не install. `.python/site` содержит зависимости Python для ROS-нод; `.venv-nats`, `.venv-bms-ble`, `.venv-camera` — независимые окружения служб. Инструменты форматирования устанавливаются отдельно в `.venv-format`; правила находятся в корне проекта.

## ROS-пакеты

| Пакет | Ответственность |
|---|---|
| `verter_bringup` | Launch-файлы hardware/voice/web/SLAM/Nav2 и их параметры |
| `verter_slam` | Асинхронная нода SLAM с отбором сканов по сдвигу или повороту |
| `verter_description` | Активный URDF, meshes, TF и геометрия |
| `verter_perception` | Проекция depth в PointCloud2, валидация УЗ и семь Range для costmap |
| `verter_drivers` | ZLAC по Modbus/RS485, запуск micro-ROS агентов; дополнительные DOA и калибровка |
| `verter_voice` | STT, обработка текста, AI, TTS и воспроизведение |
| `verter_web` | HTTP-сервер статики React-интерфейса |
| `third_party/lslidar_driver` | Закреплённый драйвер Leishen M10P |
| `third_party/lslidar_msgs` | Сообщения внешнего драйвера Leishen |

Nav2, robot_localization, robot_state_publisher, laser_filters и rosbridge установлены как внешние ROS-пакеты. Системный SLAM Toolbox 2.6.10 предоставляет Karto, Ceres и ROS-интерфейсы; режим mapping запускает собственный `verter_slam/async_slam_toolbox_node`. В `verter_slam/third_party/slam_toolbox` хранится необходимая часть upstream-исходников (tag 2.6.10, ревизия и SHA-256 в `ORIGIN.json`), а в `verter_slam/patches` — применённое исправление. Сборка не обращается к GitHub и не зависит от файлов в `/tmp`. NATS API локализации, точек и определений маршрутов описан в LOCALIZATION_API_V1.md. Исполнение маршрутов и миссий пока не реализовано.

## Потоки данных

```text
ESP32 sensors → micro-ROS Agent → /imu/data → EKF
ZLAC ↔ Python Modbus driver → /odom_raw → EKF → /odometry/filtered
M10P → lslidar_driver_node → /scan_raw → laser_filter → /scan → SLAM либо AMCL/Nav2
URDF → robot_state_publisher → неподвижные TF
EKF → odom → base_footprint; SLAM либо AMCL → map → odom

Веб-мониторинг → HTTP → NATS request/reply → Python backend
ROS/BMS → Python backend → NATS update subjects + KV → приложения
Gemini 2 → ROS Image/CameraInfo + отдельное видео WebRTC/HTTP
Gemini depth → verter_perception → /perception/depth/points → Nav2 VoxelLayer
ESP32 /ultrasonics → verter_perception → семь Range → Nav2 RangeSensorLayer
```

Nav2 и ручной NATS-контур могут публиковать `/cmd_vel`. ESP32 ctrl — ещё один источник команд. Общего арбитража между ними нет: одновременно используется один источник. Ручное управление через NATS не включает автоматический объезд препятствий.

## Владельцы процессов

`verter-supervisor.service` запускает `tools/runtime.py` через `tools/verter serve`. Supervisor получает локальные JSON-команды через `run/control.sock` и владеет ROS launch-группами. `verter-autostart.service` вызывает `tools/autostart.sh`, который независимо пытается поднять hardware, voice и web.

Launch-файлы находятся в `ros_ws/src/verter_bringup/launch/`:

| Группа/режим | Файл | Состав |
|---|---|---|
| perception (отдельная systemd-служба) | `perception.launch.py` | depth_obstacles и ultrasonic_ranges; без команд движения |
| hardware | `hardware.launch.py` | ZLAC, агенты IMU/ctrl, M10P, фильтр сканов, URDF/TF, EKF |
| voice | `voice.launch.py` | STT GigaAM v3 RNNT, recognition, AI, Silero TTS, sound player |
| web | `web.launch.py` | rosbridge 9090 и статика 8081 |
| mapping | `mapping.launch.py` | SLAM Toolbox и map saver |
| navigation | `navigation.launch.py` | AMCL, map server, Nav2 и lifecycle managers |
| head | `head.launch.py` | DOA; запуск через supervisor сейчас заблокирован |

NATS, NATS backend/мониторинг, BMS, камера, обработка препятствий, просмотр лидара и kiosk — отдельные пользовательские systemd-службы. Wifi-connect — системная служба. Команды обслуживания — в [OPERATIONS.md](OPERATIONS.md).

**Единый владелец режимов — supervisor.** NATS bridge передаёт запросы в тот же `run/control.sock`, что и CLI `tools/verter`. `service.mode` отражает supervisor. Поддерживаются idle/mapping/localization/navigation; принятие запроса и завершение различаются по operation.status. Перезапуск bridge не останавливает режим. Контракт — [NATS_MODES_V1.md](NATS_MODES_V1.md).

## Где менять код и параметры

| Задача | Путь |
|---|---|
| ROS-параметры | `ros_ws/src/verter_bringup/config/` |
| Порты оборудования | `ros_ws/src/verter_bringup/launch/hardware.launch.py`, `config/chassis.yaml` внутри пакета |
| M10P и маска корпуса | `config/lidar/m10p.yaml`, `config/laser_filters/m10p_filter.yaml` внутри verter_bringup |
| Размеры/TF | `ros_ws/src/verter_description/urdf/verter_robot.urdf` |
| CLI и процессы | `tools/runtime.py`, `tools/verter`, `tools/autostart.sh` |
| Проверка входов навигации | `tools/ros_probe.py`, `tools/sensor_health.py` |
| NATS-контракты и метрики | `services/nats_bridge/robot_nats/contracts.py`, `protocol.py`, `service.py`, `telemetry.py` |
| Ручное управление | `services/nats_bridge/robot_nats/motion.py` |
| Веб 8088 | `services/nats_bridge/robot_nats/web.py`, `services/nats_bridge/static/`; [инструкция](../services/nats_bridge/README.md) |
| BMS | `services/nats_bridge/robot_nats/jk_ble.py`, `bms_service.py`, `battery_state.py` |
| Камера | `services/camera/capture.py`, `server.py`, `static/` |
| Препятствия depth/УЗ | `ros_ws/src/verter_perception/`, `verter_bringup/config/perception.yaml`, Nav2 config; [COSTMAP.md](COSTMAP.md) |
| Настройки служб | `config/nats/server.conf`, `bridge.json`, `bms.json`, `camera.json` |
| Облачные параметры голоса | `config/secrets.env`; шаблон `config/secrets.env.example` |

Конфигурации NATS содержат пароли. Их и резервные приватные файлы не включают в комплект API.

## Окружение и рабочие данные

`tools/env.sh` очищает унаследованные ROS overlay и подключает Humble, `vendor/microros_ws/install/local_setup.bash`, `ros_ws/install/local_setup.bash`. Он задаёт `VERTER_DATA_DIR`, `VERTER_ASSETS_DIR`, `VERTER_WEB_DIR`, `VERTER_RUN_DIR` и читает `config/secrets.env` как bash-файл.

Карты — `data/maps`, точки — `data/waypoints.yaml`. Наличие файла точек не означает наличия API маршрутов. NATS allowlist карт использует `/home/jetson/verter-robot/data/maps/dtest_map.yaml`; файлы карты находятся внутри проекта. NATS-режимы остаются отключены до отдельной интеграции с supervisor.

Git хранит исходники и документацию. Приватная конфигурация, модели, рабочие данные и `archive/` исключены из Git. Архив не участвует в сборке или запуске.

Зависимости от пользовательского и системного окружения — [HOME_DIRECTORY.md](HOME_DIRECTORY.md).


История аккумулятора: `services/nats_bridge/robot_nats/history.py` (SQLite, 7 дней),
`services/nats_bridge/static/battery-history.*` (графики),
`data/nats-bridge/battery-history.sqlite` (рабочая база).
Контракт: [BMS_HISTORY_V1.md](BMS_HISTORY_V1.md).


Живая карта: `services/nats_bridge/robot_nats/map_data.py` — ограниченный кеш снимков,
`map_observer.py` — read-only ROS /map и TF; `static/map.*` — окно OpenLayers.
Браузерные библиотеки и лицензии — `static/lib/`, их SHA-256 и версии — manifest.json.
Карта не зависит от подключения браузера. API: [MAP_API_V1.md](MAP_API_V1.md).

Живая карта: services/nats_bridge/robot_nats/map_data.py и map_observer.py;
сохранение сетки и графа: map_save.py. Интерфейс: static/map.html, map.js, map.css.
Сохранённые YAML/PGM, posegraph/data и manifest находятся в data/maps.
[Картографирование](MAPPING_GUIDE.md), [API](MAP_API_V1.md).

## Локализация и именованные точки

Отдельный режим `localization` запускает AMCL и map_server без контроллера движения. Начальная поза, подтверждение по скану, точки и определения маршрутов доступны в `/map`. Контракт и операторский порядок: [LOCALIZATION_API_V1.md](LOCALIZATION_API_V1.md). Данные точек — `data/nats-bridge/places.sqlite`; исполнение маршрутов не запускается.

## Отображение costmap

`services/nats_bridge/robot_nats/costmap_observer.py` пассивно читает две raw-сетки Nav2 и TF; `costmap_data.py` хранит ограниченные версионированные снимки и чанки. `static/costmap-ui.js` и `.css` добавляют слои в существующее окно `/map` на OpenLayers. Владелец наблюдателя и HTTP/NATS API — `verter-nats-bridge`; владельцем самих costmap остаётся Nav2 в режиме navigation. [COSTMAP_API_V1.md](COSTMAP_API_V1.md) описывает тот же интерфейс для серверного приложения.
