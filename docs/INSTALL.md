# Установка всей системы

Целевая платформа — Jetson с Ubuntu 22.04 / ROS 2 Humble, рабочий каталог `/home/jetson/verter-robot`. Для воспроизведения текущего окружения ориентир — JetPack/L4T R36.4.4; совместимость образа нужно сверять с моделью Jetson. Установку на полностью чистую систему пока не проверяли: ниже описаны фактические установщики, их побочные действия и оставшиеся ручные шаги. Единого установщика всей системы нет. Apt-репозитории и все транзитивные зависимости не закреплены полным снимком; это не гарантированно воспроизводимый офлайн-дистрибутив.

Команды выполняются на Jetson пользователем `jetson`, если явно не указан `sudo`. Этот документ предназначен для подготовки нового робота, а не для повторного запуска установщиков на работающем. Запуск оборудования, смена режима и прошивка согласуются отдельно. Это основной Git-репозиторий. Резервные деревья в `archive/` не используются при запуске и не включаются в Git.

## 1. ОС и копия проекта

Установить совместимый с моделью Jetson образ NVIDIA, настроить сеть, SSH, время и учётную запись. Путь проекта должен быть без пробелов, `%` и переводов строк. Получить исходники из Git в `/home/jetson/verter-robot` на выбранном теге или commit. Сохранить права исполнения скриптов и символические ссылки. Не переносить окружения и результаты сборки Windows как готовую Linux-установку.

Для полной системы нужны исходники, карты `data/maps`, модели `assets/`, прошивки и индивидуальная конфигурация. `build/`, `install/`, виртуальные окружения и бинарные зависимости с другого робота лучше пересобирать. Не копировать всем роботам одни NATS-пароли, `robot_id`, BLE-адрес, USB-пути и серийный номер камеры. Приватные конфигурации и полные резервные архивы не передавать как публичный дистрибутив.

```bash
cd /home/jetson/verter-robot
bash deploy/install-system.sh
sudo apt-get install -y python3-venv python3-dev python3-opencv \
  libusb-1.0-0-dev libudev-dev libssl-dev pkg-config bluez \
  network-manager wireless-tools
```

`install-system.sh` проверяет Ubuntu 22.04, подключает ROS apt-репозиторий, устанавливает ROS Humble, Nav2, SLAM Toolbox, robot_localization, laser_filters, RViz, rosbridge, средства сборки и зависимости Leishen. Инициализирует rosdep, добавляет пользователя в `dialout`, `audio`, `video`. После изменения групп заново войти в сеанс. Драйверы JetPack этот скрипт не устанавливает.

## 2. Python, micro-ROS и ROS-пакеты

```bash
bash deploy/install-python.sh core
bash deploy/build-agent.sh
bash tools/build.sh
bash tools/check.sh
```

Python-зависимости core устанавливаются в `.python/site`, не в системный Python. `build-agent.sh` собирает micro-ROS Agent в `vendor/microros_ws`; исходные основные ревизии закреплены в скрипте, все транзитивные зависимости не зафиксированы. `build.sh` последовательно собирает девять ROS-пакетов в `ros_ws`, включая Leishen M10P и `verter_slam`. Для `verter_slam` требуется системный `slam_toolbox` ровно 2.6.10: он предоставляет Karto, решатель Ceres и ROS-интерфейсы. Нода mapping собирается из закреплённых исходников проекта; подробности — [PROJECT_LAYOUT.md](PROJECT_LAYOUT.md). Для работы необходим доступ к apt, PyPI и GitHub.

`check.sh` проверяет синтаксис, unit-тесты и shell-скрипты; не запускает оборудование. Интеграционные сценарии из `tests/` не входят в процедуру установки и не должны запускаться в ROS-домене действующего робота.

## 3. Порты и конфигурация робота

До запуска проверить соответствие физических устройств конфигурации:

- ZLAC: `/dev/chassis`, прямой RS485/Modbus из Python.
- ESP32 ctrl и sensors: разные последовательные порты, отдельные micro-ROS Agent. Открытие порта может перезагрузить ESP32 через DTR.
- M10P: `ros_ws/src/verter_bringup/config/lidar/m10p.yaml`, 500000 бод. Используется `/dev/serial/by-id/...CP2102...`.
- URDF, EKF, Nav2 и маска лидара: [структура конфигурации](PROJECT_LAYOUT.md), размеры — [GEOMETRY.md](GEOMETRY.md).

Сначала сравнить `ls -l /dev/serial/by-id /dev/serial/by-path` и `udevadm info --query=property --name=/dev/ttyUSB0` с реальным подключением. Номер ttyUSB не является постоянным идентификатором. Некоторые USB-UART имеют одинаковые serial: тогда нужен физический by-path.

`deploy/99-robot-devices.jetson.rules` настроен под нынешний Jetson. Адаптировать его перед установкой на другой робот; `99-verter.rules.example` не является готовой конфигурацией M10P. После проверки правил:

```bash
sudo install -m 644 deploy/99-robot-devices.jetson.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

Новые правила применяются при следующем подключении соответствующего устройства. Не делать общий USB reset на работающей системе. Прошивку ESP32 установщики не выполняют: порядок сборки — [firmware/README.md](../firmware/README.md).

## 4. Голос и локальный интерфейс

```bash
bash deploy/install-python.sh voice
```

Отдельно установить в `.python/site` совместимые с архитектурой и Python версии Torch/torchaudio (pip `--target .python/site`); для GPU также нужна совместимость с JetPack. Готового универсального установщика этих wheel-файлов нет. Перенести необходимые голосовые модели в `assets/`, проверить командой `python3 tools/verify_assets.py` (или явно задать `VERTER_ASSETS_DIR`) по манифесту `config/assets_manifest.json`. Настроить звуковые устройства и приватный `config/secrets.env` для используемых облачных провайдеров. Список переменных смотреть в `tools/env.sh` и коде выбранного провайдера; не хранить реальные ключи в документации.

Для интерфейса 8081 нужен Node.js/npm. На текущей системе сборка использовала Node 20.20.2 / npm 10.8.2; это описание имеющегося окружения, а не обещание поддержки любой версии. Установщик ROS Node.js не устанавливает.

```bash
cd web/frontend
npm ci
npm run build
cd ../..
```

Результат `web/frontend/dist` обслуживает `verter_web`; копировать его в ROS install не требуется. Это отдельный интерфейс от отладочного NATS-мониторинга 8088. Подробности — [frontend README](../web/frontend/README.md).

## 5. Supervisor и ROS-автозапуск

```bash
bash deploy/install-service.sh
python3 - <<'PY'
from pathlib import Path
root = Path.cwd().resolve()
if any(c in str(root) for c in ' %\n'):
    raise SystemExit('Unsupported project path')
target = Path.home() / '.config/systemd/user'
target.mkdir(parents=True, exist_ok=True)
for name in ('verter-autostart', 'verter-kiosk'):
    src = root / 'deploy' / (name + '.service.in')
    (target / (name + '.service')).write_text(
        src.read_text().replace('@ROOT@', str(root)))
PY
systemctl --user daemon-reload
sudo loginctl enable-linger jetson
```

Это только установка unit-файлов. Supervisor сам стартует в `idle`; отдельный autostart запускает hardware, voice и web. Kiosk необязателен: нужен установленный Firefox и графический сеанс `DISPLAY=:0`. Для head/DOA запуск заблокирован, приоритет у ESP32 ctrl.

После проверки портов и согласования запуска оборудования:

```bash
systemctl --user enable --now verter-supervisor.service
systemctl --user enable --now verter-autostart.service
# Только при наличии локального дисплея и Firefox:
systemctl --user enable --now verter-kiosk.service
```

## 6. NATS и отладочный мониторинг

```bash
python3 services/nats_bridge/install.py
```

Установщик скачивает NATS Server 2.15.0 с проверкой checksum, создаёт `.venv-nats`, приватные файлы `config/nats/` и user units `verter-nats`, `verter-nats-bridge`. Выполняет daemon-reload, но не запускает службы.

До первого запуска проверить:

- Уникальные `robot_id`, JetStream domain и bucket: одинаковые согласованные значения в server/bridge/BMS/camera конфигурации и ACL. Текущий робот: `verter_01`, `verter_01`, `VERTER_01_STATE`; не полагаться на значения по умолчанию установщика для нового робота.
- Пароли NATS и права клиентов. NATS слушает loopback:4222, диагностический HTTP NATS — loopback:8222.
- `bridge.json`: ROS-домен, пути карт, `web_host`, `web_port`, `web_allowed_hosts`. Карты должны указывать на новый проект. Текущий allowlist использует `data/maps/dtest_map.yaml` внутри основного проекта.
- `allow_mode_changes=false` и `allow_manual_motion=false` на этапе подготовки. На действующем тестовом роботе ручное управление отдельно включено; копировать это разрешение автоматически не следует.
- В доверенной Wi-Fi сети web_host может быть `0.0.0.0`, но IP/имена нужно явно включить в allowed_hosts. В production `web_enabled=false`. Отсутствие HTTP-токена не отменяет NATS-аутентификацию.

Затем:

```bash
systemctl --user enable --now verter-nats.service
systemctl --user enable --now verter-nats-bridge.service
```

Leaf-подключение к центральному серверу ещё не настроено: `leafnodes.remotes` пуст. Для него нужны адрес hub, TLS/credentials и согласованные ограничения publish/subscribe/account на обеих сторонах. Приватный пример в `config/nats` не является работающим подключением. [Контракт и границы прав](NATS_CONTRACTS_V1.md).

## 7. BMS через BLE

```bash
python3 services/nats_bridge/install_bms.py
```

Создаёт `.venv-bms-ble`, конфигурацию, ограниченного NATS-пользователя и `verter-bms.service`; сам службу не запускает. Проверить адрес нужного BMS, Bluetooth-адаптер и ожидаемую модель в созданном JSON. Для нынешнего робота используется JK-B2A8S20P с BLE; штатного RS485-сервиса BMS нет. Сборщик запрашивает идентификацию/телеметрию, не меняет настройки заряда.

После проверки индивидуальной конфигурации и доступности Bluetooth:

```bash
systemctl --user kill --kill-who=main -s HUP verter-nats.service
systemctl --user enable --now verter-bms.service
```

Если bridge уже работал при изменении его конфигурации установщиком, перезапустить его по [OPERATIONS.md](OPERATIONS.md), предварительно убедившись, что он не управляет движением или режимом. Приложение телефона может занимать BLE-подключение; на время чтения отсоединить его от BMS.

## 8. Камера Gemini 2

```bash
bash deploy/install-camera-deps.sh
sudo install -m 644 deploy/99-verter-camera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

Скрипт создаёт `.venv-camera`, собирает pyorbbecsdk v1.3.4 в `vendor`. Правило даёт группе video доступ к USB 2bc5:0670; применить его при следующем подключении камеры. Проверить серийный номер, профили RGB/depth, положение камеры, web_allowed_hosts и ICE-конфигурацию в `config/nats/camera.json`. Образец находится в `services/camera`; его параметры относятся к текущему роботу и требуют адаптации.

Следующий установщик требует уже работающий NATS и **сразу включает и запускает захват камеры**:

```bash
python3 deploy/install-camera.py
```

Он создаёт конфигурацию при отсутствии, добавляет ACL, проверяет NATS-конфигурацию, посылает брокеру HUP и выполняет enable --now `verter-camera.service`. Существующую службу принудительно не перезапускает. API и ROS-топики — [CAMERA_API_V1.md](CAMERA_API_V1.md). Для SSH-просмотра использовать HTTP/MJPEG; для WebRTC между разными сетями отдельно настроить VPN или STUN/TURN.

### Обработка препятствий для Nav2

`verter_perception` собирается общей командой `tools/build.sh`; нужен системный `python3-opencv`, включённый в `install-system.sh`. Отдельная служба преобразует depth Gemini 2 и семь значений УЗ в наблюдения costmap. Она не запускает Nav2 и не отправляет команды движения.

```bash
python3 deploy/install-perception.py
# После проверки установки включить только адаптеры наблюдений:
python3 deploy/install-perception.py --activate
```

Параметры: `ros_ws/src/verter_bringup/config/perception.yaml`; слои обоих costmap — в `config/nav2/nav2_navigation_params.yaml` того же пакета. Подробности и ограничения — [COSTMAP.md](COSTMAP.md). Настройку высоты 1,5 м, индексов/FOV УЗ и физическое обнаружение мебели проверяют на конкретном роботе.

## 9. Необязательная настройка Wi-Fi

`services/wifi-connect` содержит ARM64 binary и скрипт точки настройки сети. Автоматического универсального установщика этого компонента нет. Скрипт содержит привязку к пути проекта, имени Wi-Fi-интерфейса и параметрам точки доступа — адаптировать для нового робота и назначить собственный пароль до включения.

Системный unit `wifi-connect.service` должен запускать `services/wifi-connect/wifi-connect-start.sh` после NetworkManager (`After` и `Requires`), с нужным рабочим каталогом и `Restart=no`. Проверить наличие `NetworkManager`, `iwgetid` и подходящего Wi-Fi-адаптера. Скрипт проверяет существующее соединение и может останавливать dnsmasq перед запуском своей точки. Это отдельная операция настройки сети, не часть перезапуска ROS. Не включать её удалённо без резервного доступа к Jetson.

## 10. Приёмка установки

```bash
tools/verter status
tools/verter health
systemctl --user --no-pager status verter-supervisor verter-autostart \
  verter-nats verter-nats-bridge verter-bms verter-camera
```

Проверить журналы, свежесть scan/IMU/одометрии/TF, BMS и изображений, отсутствие дублирующих издателей и ожидаемые сетевые адреса. `active` у systemd не доказывает исправность датчика. Эти проверки не требуют команд движения. Испытания управления, SLAM, локализации, торможения и объезда людей — отдельная согласованная приёмка.

## 11. Состояние Ansible

`deploy/ansible/site.yml` — частичный playbook, не установщик всей системы.
Он проверяет Ubuntu 22.04, копирует `release_dir`, опционально вызывает
`install-system.sh`, устанавливает Python core, собирает ROS и устанавливает
неактивный unit supervisor.

Ограничения текущего файла:

- `verter_root` по умолчанию всё ещё указывает на `~/verter-robot-refactored`;
  целевой путь системы — `/home/jetson/verter-robot`.
- Синхронизация не исключает все приватные и генерируемые каталоги:
  `archive/`, `config/nats/`, `.venv-*` и `vendor/` могут попасть в переносимый набор.
- Сборка micro-ROS Agent, голосовые модели/Torch, NATS, BMS, камера, frontend,
  индивидуальные udev-правила, полная настройка systemd и приёмка не объединены в playbook.
- Используются вызовы shell-установщиков; полноценное повторяемое приведение
  системы к заданной конфигурации и работа sudo на чистой машине не проверены.

Не запускать этот playbook как готовую установку всего робота. До его доработки
использовать этапы данного документа; чистая установка требует отдельной приёмки.

Работающие Python-процессы используют `PYTHONNOUSERSITE=1`; библиотеки в `~/.local` не считаются зависимостями установки. Текущий комплект зависимостей сохранён в `.python/site`. PlatformIO для сборки прошивок находится в `vendor/platformio`; `tools/env.sh` задаёт `PLATFORMIO_CORE_DIR`. Прошивки автоматически не загружаются.

## Просмотр лидара

После установки NATS bridge (окружение `.venv-nats`), ROS и модели робота:

```bash
python3 services/lidar/install.py
systemctl --user kill --kill-who=main -s HUP verter-nats.service
systemctl --user enable --now verter-lidar.service
```

Установщик не запускает службы: добавляет пользователя NATS только для своего
`lidar.update.*`, сохраняет `config/nats/lidar.json` и устанавливает unit. Существующие
параметры lidar.json сохраняются. Проверить robot_id, URI NATS, URDF, bind и allowed_hosts.
Служба только наблюдает /scan и TF; не запускает hardware и не публикует команды.
Без лидара/TF окно показывает отсутствие свежих данных. Порт 8090, контракт — [LIDAR_API_V1.md](LIDAR_API_V1.md).
Частичный Ansible-playbook компонент не устанавливает.
