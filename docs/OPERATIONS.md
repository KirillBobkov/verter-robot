# Сборка, запуск и обслуживание

Все команды ниже выполняются на Jetson под `jetson` из `/home/jetson/verter-robot`. Из SSH пользоваться `systemctl --user` без sudo для пользовательских служб. Установка с нуля — [INSTALL.md](INSTALL.md), расположение компонентов — [PROJECT_LAYOUT.md](PROJECT_LAYOUT.md).

## Проверка без изменения состояния

```bash
cd /home/jetson/verter-robot
tools/verter status
tools/verter health
systemctl --user --no-pager status verter-supervisor verter-autostart
systemctl --user --no-pager status verter-nats verter-nats-bridge verter-bms verter-camera verter-perception
```

`status` показывает процессы, принадлежащие supervisor; отдельные NATS/BMS/camera/perception-службы в него не входят. `health` пассивно проверяет свежесть scan, IMU, filtered odometry и odom TF; это не тест моторов, препятствий или точности локализации. `active (exited)` у oneshot `verter-autostart` нормально. Наличие PID или ROS-ноды не доказывает поступление данных.

Для ROS CLI сначала загрузить окружение:

```bash
source tools/env.sh
ros2 node list
ros2 topic list
ros2 topic info /scan --verbose
```

`env.sh` очищает унаследованные пути ROS-overlay и подключает Humble, micro-ROS Agent, рабочий `ros_ws/install`, `.python/site` и приватные настройки. Не подключать в этот shell setup-файлы других ROS workspace.

## Сборка после изменений

```bash
bash tools/build.sh
bash tools/check.sh
```

`build.sh` вызывает `colcon build --symlink-install --executor sequential` для всего workspace. Переданные ему дополнительные аргументы не перенаправляются в colcon. Для выборочной сборки:

```bash
source tools/env.sh
cd ros_ws
colcon build --symlink-install --executor sequential --packages-select verter_drivers
```

При изменении зависимостей использовать `--packages-up-to ИМЯ_ПАКЕТА`. Сборка ничего не перезапускает. Даже при symlink-install изменённый Python-код уже работающего процесса сам не перезагрузится; launch/config/ресурсы следует пересобрать и перезапустить владельца.

| Изменение | Сборка / применение |
|---|---|
| ROS Python/C++, launch, URDF, конфигурация в ROS-пакетах | colcon; затем соответствующая группа supervisor |
| `tools/runtime.py` | Не нужен colcon; перезапуск supervisor затрагивает все его группы |
| NATS bridge/BMS Python | Не нужен colcon; перезапуск соответствующего user unit |
| Камера Python/config | Не нужен colcon; перезапуск `verter-camera` |
| Проекция depth / адаптер УЗ | colcon для `verter_perception`; перезапуск `verter-perception` |
| `config/perception.yaml` в verter_bringup | colcon для `verter_bringup`; перезапуск `verter-perception` |
| Конфигурация costmap | colcon для `verter_bringup`; применяется при следующем запуске navigation |
| Python requirements | Обновить соответствующее `.python/site` или `.venv-*`; затем службу |
| React frontend | `cd web/frontend && npm ci && npm run build`; обновить страницу, dist читается напрямую |
| Unit-файл systemd | `systemctl --user daemon-reload`, затем соответствующий unit |
| NATS server.conf / ACL | Проверить конфигурацию сервером, затем HUP NATS |

Для проверки NATS-конфига использовать установленный бинарник `vendor/nats/nats-server` с `-t -c config/nats/server.conf`. Перед применением прав проверить, что не удаляются доступы камеры/BMS. `services/nats_bridge/migrate_contract.py` обновляет ACL контрактов; это не обычная команда сборки и не должна запускаться после каждого изменения документации.

## Как устроен автозапуск и восстановление

1. systemd запускает `verter-supervisor.service` → `tools/verter serve` → Unix socket `run/control.sock`.
2. `verter-autostart.service` → `tools/autostart.sh` независимо запрашивает hardware, voice и web. Ошибка одной группы не должна препятствовать попытке старта других.
3. Supervisor запускает launch-файлы из `verter_bringup`: hardware, voice, web и отдельно выбранный mode. После обычного старта режим `idle`; SLAM/Nav2 автоматически не включаются.
4. В hardware/voice/web ноды имеют `respawn=true`, задержка 5 с. При выходе процесса launch пытается поднять его снова. Это не обнаружение зависшего процесса и не гарантия восстановления устройства.
5. Для mapping/navigation выход процесса завершает группу режима; supervisor фиксирует ошибку. Режим автоматически не возобновляется.
6. NATS, bridge, BMS, камера, обработка препятствий и kiosk — отдельные user units с `Restart=on-failure`. Systemd восстанавливает процесс службы; свежесть её данных проверяется отдельно.

Синтаксическая ошибка launch, занятый эксклюзивный порт или конфликт издателей могут помешать запуску всей hardware-группы. Независимость процессов не отменяет этих общих проверок. `health` и preflight режима не являются непрерывным watchdog потери датчиков во время движения.

## Перезапуск оператором

Команды этого раздела меняют состояние оборудования. Выполнять на неподвижном роботе после согласования; завершение активного режима или ручной сессии может публиковать нулевую скорость. Не использовать их как пассивный тест. Здесь нет автоматического согласования с внешними источниками управления.

Перезапуск отдельной группы (CLI-команды `restart` нет):

```bash
tools/verter app web stop
tools/verter app web start
```

Для voice заменить `web` на `voice`. Head/DOA заблокирован. Отдельную ноду внутри hardware supervisor адресно не перезапускает: при аварийном выходе действует respawn, для планового обновления перезапускается вся hardware-группа. Перед её остановкой завершить активное картографирование/навигацию, предварительно сохранив карту:

```bash
tools/verter mode idle
tools/verter hardware stop
tools/verter hardware start
```

Полный перезапуск ROS-групп supervisor:

```bash
systemctl --user restart verter-supervisor.service
systemctl --user start verter-autostart.service
tools/verter status
tools/verter health
```

Он завершает принадлежащие supervisor группы и режим, затем поднимает hardware/voice/web в idle. NATS, BMS и камера этим не перезапускаются. Не перезапускать autostart как способ обновления уже работающих нод: повторный start группы не заменяет её stop/start.

Отдельные службы — выбирать только нужную строку:

```bash
systemctl --user restart verter-bms.service
systemctl --user restart verter-camera.service
systemctl --user restart verter-perception.service
systemctl --user restart verter-nats-bridge.service
systemctl --user restart verter-kiosk.service
```

Перед restart bridge убедиться в отсутствии активной ручной сессии (ROS-режимы принадлежат supervisor и этим restart не завершаются). Перезапуск камеры разрывает видеосеансы; её ROS-топики временно пропадают. Перезапуск брокера разрывает все NATS-соединения; для применимого изменения ACL достаточно HUP:

```bash
systemctl --user kill --kill-who=main -s HUP verter-nats.service
```

## Картографирование и навигация

Режимами текущего ROS запуска управляет `tools/verter`, не редактирование launch. Перед режимом должны быть свежие `/scan`, IMU, одометрия и TF. Preflight ждёт до 60 с; при ошибке проверить `health` и журналы, не обходить проверку запуском второго набора нод.

Операторская последовательность картографирования:

```bash
tools/verter mode mapping
# Движение выполняет оператор после отдельного согласования.
tools/verter save-map /home/jetson/verter-robot/data/maps/site_map
tools/verter mode idle
```

`save-map` сохраняет карту по базовому пути без расширения. Не выходить из mapping до успешного сохранения. `--discard` у команды mode явно разрешает отбросить несохранённую карту; это не стандартный способ выхода.

Запуск Nav2 с сохранённой картой:

```bash
tools/verter mode navigation --map /home/jetson/verter-robot/data/maps/site_map.yaml
```

Указать существующий YAML и доступное изображение карты. Проверить начальную позу/совпадение облака лидара с картой, при необходимости задать позу через RViz. Старт на базе предполагается организационно: нельзя считать малую ковариацию доказательством правильной локализации или ожидать гарантированной локализации в произвольной точке. Не отправлять цель, пока положение не подтверждено.

**Единый владелец режимов — supervisor.** NATS bridge передаёт запросы в тот же `run/control.sock`, что и CLI `tools/verter`. `service.mode` отражает supervisor. Поддерживаются idle/mapping/localization/navigation; принятие запроса и завершение различаются по operation.status. Перезапуск bridge не останавливает режим. Контракт — [NATS_MODES_V1.md](NATS_MODES_V1.md).

## Интерфейсы и журналы

| Компонент | Где смотреть |
|---|---|
| hardware, voice, web, mode | `run/hardware.log`, `run/voice.log`, `run/web.log`, `run/mode.log` |
| Supervisor / autostart | `journalctl _SYSTEMD_USER_UNIT=verter-supervisor.service` / `verter-autostart.service` |
| NATS / bridge | `data/nats-bridge/nats-server.log`, `data/nats-bridge/bridge.log` |
| BMS | `data/bms/service.log` |
| Камера | `journalctl _SYSTEMD_USER_UNIT=verter-camera.service` |
| Kiosk | `journalctl _SYSTEMD_USER_UNIT=verter-kiosk.service` |
| Системный Wi-Fi | `sudo journalctl -u wifi-connect.service` |

В journalctl добавить `-n 100 --no-pager` для последних записей или `-f` для наблюдения. На этой системе `journalctl --user` может не показывать журнал; фильтр `_SYSTEMD_USER_UNIT` читает доступные записи общего журнала.

Код, HTTP endpoints и применение изменений тестового веба описаны в [services/nats_bridge/README.md](../services/nats_bridge/README.md).

HTTP: 8081 — локальный React-интерфейс; 8088 — NATS-мониторинг/кнопки; 8089 — камера. 9090 — rosbridge для локального интерфейса. NATS — 4222; его служебный мониторинг — 8222. Для удалённого доступа:

```bash
ssh -p 3333 -N -L 8088:127.0.0.1:8088 -L 8089:127.0.0.1:8089 jetson@109.195.134.20
```

Открыть `http://127.0.0.1:8088`; камеру — `http://127.0.0.1:8089`, транспорт «Через SSH / HTTP». Обычный SSH TCP-туннель не переносит WebRTC UDP. В доверенной Wi-Fi сети использовать IP Jetson, включённый в web_allowed_hosts обоих сервисов. Если IP изменился — изменить соответствующую конфигурацию и перезапустить только её владельца.

## Типовые неисправности и пределы готовности

- Нет `/scan`: проверить питание/порт M10P, 500000 бод и журнал hardware. `scan_raw` есть, а `scan` нет — проверить filter и frame. Маска задней зоны находится в `laser_filters/m10p_filter.yaml`; физическое направление нулевого луча ещё требует проверки на роботе.
- Нет IMU/ESP32: проверить соответствующий micro-ROS Agent, эксклюзивность serial и прошивку. Не останавливать соседний ESP32 по предположению, что это тот же порт.
- BMS stale: проверить BLE, занятость устройством телефона, адрес и ошибки модели/кадров. Последнее значение KV не является живым измерением.
- Камера открывается без видео через SSH: выбрать HTTP; проверить свежесть RGB/depth и лимит зрителей. Камера публикует ROS-данные, но пока не включена в costmap или SLAM.
- Web 403: Host/Origin должны соответствовать разрешённому адресу; смена bind на 0.0.0.0 сама по себе allowlist не расширяет.
- UI-команда немедленно обнуляется: проверить lease, TTL/часы, потерю связи и ответ NATS. Не увеличивать таймауты вместо диагностики и не тестировать реальным `/cmd_vel` от агента.
- УЗ-данные ещё не приняты физическим тестом и не включены в гарантированную защиту/объезд. Навигация среди людей, дистанция 20–30 см и предел скорости требуют отдельной приёмки. Inflation/footprint не гарантируют дистанцию торможения.

Ручное управление NATS, ESP32 и Nav2 не имеют общего арбитра: разрешён один источник за раз. Программный stop не заменяет аппаратный аварийный выключатель. Проверки кнопок разработчиком выполняются только с подменённым backend или изолированным NATS при отключённом ROS; реальные движения, stop и навигационные цели без согласования не отправлять.

В текущем `nav2_navigation_params.yaml` у RPP задано `desired_linear_vel: 0.04` м/с. Предел ручного NATS-управления 0,5 м/с не означает, что Nav2 настроен на ту же скорость. Изменение скорости навигации требует отдельной проверки торможения и препятствий.

Для `run/*.log` автоматическая ротация не настроена: контролировать свободное место. HTTP 8081 и rosbridge не являются защищённым внешним шлюзом; не публиковать их напрямую в интернет.

## Хранение кода и данных

Код и документация синхронизируются через Git. `assets/`, `data/`, приватные
`config/nats/` и `config/secrets.env`, окружения и `archive/` исключены из Git:
обычный push не сохраняет карты, пароли, модели и базы данных. Их хранение организуют
отдельно. Автоматической синхронизации на другой компьютер в проекте нет.

Службы отключают пользовательские Python-пакеты из `~/.local`. ROS-логи находятся
в `run/ros-logs`, кеши моделей — в `data/cache`, профиль kiosk — в `data/kiosk-profile`.
Установленные units находятся в `~/.config/systemd/user`: удаление всего домашнего
каталога, кроме проекта, нарушит автозапуск. Подробнее — [HOME_DIRECTORY.md](HOME_DIRECTORY.md).

## Форматирование исходников

Используем 4 пробела для кода, 2 для JSON/YAML/XML/URDF и ориентир 120 символов в строке.
Короткие выражения остаются компактными; длинные вызовы, словари и блоки управления
переносятся по смыслу. Самостоятельные инструкции не объединяем через `;`.
Правила закреплены в `ruff.toml`, `.prettierrc.json`, `.clang-format` и `.editorconfig`.

Инструменты разработки устанавливаются отдельно от окружений работающих служб:

```bash
cd /home/jetson/verter-robot
python3 -m venv .venv-format
.venv-format/bin/pip install ruff==0.11.13 clang-format==18.1.8
npm install --prefix .venv-format/npm --no-audit --no-fund prettier@3.6.2 @prettier/plugin-xml@3.4.1 prettier-plugin-sh@0.18.0
export PATH="$PWD/.venv-format/bin:$PWD/.venv-format/npm/node_modules/.bin:$PATH"

python3 tools/format_code.py --check  # проверить, не меняя файлы
python3 tools/format_code.py          # применить форматирование
```

Нужны Python 3.10+ и Node.js 20+. Скрипт обрабатывает собственные Python, C++, shell,
JS/TS/TSX, HTML/CSS, JSON/YAML, XML/URDF и RViz-конфигурации. Архив, сторонние
библиотеки, `vendor`, `lib`, `build_artifacts`, результаты сборки, минифицированный
интерфейс Wi-Fi, lock-файлы и приватная конфигурация исключены. Перехода по
символическим ссылкам нет. Текст документации, INI и unit-файлы сохраняют ручное
форматирование. Встроенный Python в shell heredoc проверяется отдельно при правках.

Форматирование не запускает службы, не собирает и не прошивает ESP32. Манифесты
в `firmware/*/build_artifacts` относятся к конкретным сохранённым бинарникам,
а не обязательно к текущему тексту исходников.
Не обновлять эти хеши как будто бинарники были пересобраны.

## Отдельное окно лидара

В мониторинге 8088 выбрать «Открыть лидар в отдельном окне» либо открыть
`http://<IP-Jetson>:8090/`. Для SSH добавить `-L 8090:127.0.0.1:8090`.
Страница и WebSocket проходят через один TCP-порт. Подробности — [LIDAR_API_V1.md](LIDAR_API_V1.md).

```bash
systemctl --user status verter-lidar.service
journalctl _SYSTEMD_USER_UNIT=verter-lidar.service -n 100 --no-pager
```

Код: `services/lidar`, настройки: `config/nats/lidar.json`. Статические HTML/JS/CSS
применяются обновлением окна без сборки. После изменения Python, настроек или URDF:

```bash
systemctl --user restart verter-lidar.service
```

Этот сервис не владеет hardware, режимами или ручным управлением. Перезапуск закрывает
только окна потока лидара, браузер переподключается. При ошибке TF проверить
`base_footprint → frame_id` из /scan; без свежего скана точки скрываются.


## История BMS

На странице мониторинга 8088 ссылка «История BMS за неделю» открывает `/battery-history`.
Сбор выполняет `verter-nats-bridge` независимо от открытых браузеров.
База `data/nats-bridge/battery-history.sqlite` и файлы `-wal`/`-shm` — рабочие данные, не Git.
История сохраняется после перезагрузки; старше семи дней автоматически удаляется
при записи, не реже раза в час. При остановленном bridge сбор отсутствует.
Подробности: [BMS_HISTORY_V1.md](BMS_HISTORY_V1.md).


## Окно живой карты

В мониторинге 8088 ссылка «Карта» открывает `/map` в отдельном окне.
Оператор запускает mapping из основного окна. В idle карта не придумывается:
показывается ожидание. Кнопки масштаба/центра и клик по карте не отправляют цели.
При отсутствии свежего TF модель робота скрывается. После изменения режима
новая карта и модель появляются только после готовности supervisor.
Наблюдатель и NATS API работают в bridge; дополнительная служба или порт не нужны.
Перезапуск bridge обновляет идентификатор живой сессии, сохраняя ROS-режим supervisor.
Контракт — [MAP_API_V1.md](MAP_API_V1.md).

Основная область окна занята картой, справа находятся вкладки «Локализация»,
«Точки», «Маршруты», «Слои» и «Сохранение». На узком экране инструменты находятся
под картой. «Вся карта», «К роботу», «Следовать» и «Лидар» меняют только просмотр;
недоступные действия сопровождаются причиной. При указании позы или точки карта
не перемещается вместе с перетаскиванием; Esc или смена вкладки отменяет выбор.

Выбор сохранённого имени загружает каталог точек, но не открывает растровое
превью и не запускает режим. Для отображения этой карты требуется явный запуск
локализации. Вкладка «Маршруты» редактирует порядок точек; запуска маршрутов пока нет.

Наложение /scan включается на панели просмотра, costmap — во вкладке «Слои».
Во вкладке «Сохранение» задайте имя и нажмите
«Сохранить карту и граф SLAM», дождитесь завершения. Файлы в data/maps.
После успешного сохранения список карт в этом окне обновляется автоматически.
Подробная инструкция и проверка качества: [MAPPING_GUIDE.md](MAPPING_GUIDE.md).
Сохранение не завершает mapping и не отправляет команды движения.

На главной странице наверху показаны режим, SOC, свежесть основных датчиков и
возраст ответа. Переключение режимов и ручная панель расположены рядом;
подробности BMS, JSON и NATS находятся в раскрываемых разделах. Для смены одного
активного режима на другой сначала выбрать «Ожидание». Перед отправкой запроса
показывается подтверждение; завершение mapping требует заранее сохранить нужную
карту и явно подтвердить выход. Отпускание стрелки не останавливает ручное движение.

## Проверка драйвера M10P

Целевая сборка lslidar_driver, C++-тест и проверка через синтетический UART
описаны в [M10P_DRIVER.md](M10P_DRIVER.md). Тестовый UART использует отдельный
ROS-домен 87 и не открывает устройство робота. Новый формат сканов применяется
с новой сессией SLAM; не перезапускайте весь hardware ради обновления лидара.
Штатное завершение mapping отправляет нулевую скорость и требует согласования
при действующем запрете команд.

## Сборка SLAM

Mapping запускает `verter_slam/async_slam_toolbox_node`; исходники и исправление находятся
в `ros_ws/src/verter_slam`. Системный пакет `slam_toolbox` 2.6.10 используется как зависимость,
его файлы в `/opt/ros/humble` не редактируются. Другую версию нужно сначала согласовать
с закреплёнными исходниками и повторить изолированные проверки.

```bash
source tools/env.sh
cd ros_ws
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install --executor sequential \
  --packages-select verter_slam verter_bringup
ctest --test-dir build/verter_slam --output-on-failure
```

Полная сборка через `bash tools/build.sh` также включает `verter_slam`. Сборка и тест функции
отбора сканов не запускают ROS-ноды и не управляют роботом. Из `idle` следующая сессия mapping
подхватит новый бинарный файл и параметры. Уже запущенная сессия продолжит использовать
прежнюю версию до завершения; её остановка может отправить нулевую скорость и требует
отдельного согласования. Ради обновления SLAM перезапускать hardware, NATS или supervisor не нужно.

## Локализация и именованные точки

Отдельный режим `localization` запускает AMCL и map_server без контроллера движения. Начальная поза, подтверждение по скану, точки и определения маршрутов доступны в `/map`. Контракт и операторский порядок: [LOCALIZATION_API_V1.md](LOCALIZATION_API_V1.md). Данные точек — `data/nats-bridge/places.sqlite`; исполнение маршрутов не запускается.

## Обработка препятствий камеры и УЗ

`verter-perception.service` независимо запускает два адаптера наблюдений. Её старт/перезапуск не включает Nav2, не меняет режим и не публикует `/cmd_vel`. Для обновления пакета собрать `verter_perception` и `verter_bringup`; для установки unit — `python3 deploy/install-perception.py`, явный `--activate` включает службу в автозапуск и запускает её.

```bash
systemctl --user status verter-perception.service
journalctl --user -u verter-perception.service -n 100 --no-pager
source tools/env.sh
ros2 topic echo /perception/depth/status --once
ros2 topic echo /perception/ultrasonic/status --once
```

Проверять `ready/stale/error` камеры и `raw_stale/valid_mask/errors` УЗ. `tools/verter health` по-прежнему проверяет лидар/IMU/одометрию и не заменяет эту диагностику. УЗ не имеют временных меток отдельных замеров: свежий массив не доказывает свежесть каждого поля. Подробности слоёв, высот, очистки и изолированных тестов — [COSTMAP.md](COSTMAP.md). Ручное управление costmap не использует; испытание объезда требует отдельно согласованного автономного движения.

## Слои costmap в вебе

Открыть `/map` на порту 8088. Локальная costmap включена по умолчанию, глобальная включается переключателем. Доступны прозрачность и показ градиента стоимости. Переключатели только читают данные и не отправляют движение, начальную позу или цели.

Источники ROS: `/local_costmap/costmap_raw` и `/global_costmap/costmap_raw`, тип `nav2_msgs/Costmap`. Наблюдатель `verter_costmap_observer` принадлежит `verter-nats-bridge`; отдельного запуска costmap для просмотра нет. Вне navigation отображается «Nav2 не запущена». При проблеме проверить actual/stale режима, наличие raw-топиков и map→odom TF; обычный просмотр и `GET /api/costmap` не меняют состояние системы. Полный контракт — [COSTMAP_API_V1.md](COSTMAP_API_V1.md).
