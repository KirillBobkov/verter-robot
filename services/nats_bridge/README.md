# Тестовый веб-мониторинг и NATS backend

Рабочий корень — `/home/jetson/verter-robot`. Мониторинг работает на **8088**
в составе `verter-nats-bridge.service`. Все команды HTTP API проходят через
NATS request/reply. Веб-сервер не публикует ROS-команды напрямую.

## Код

| Файл относительно этого каталога | Назначение |
|---|---|
| `static/index.html` | Страница мониторинга и панель управления |
| `static/style.css` | Оформление |
| `static/app.js` | Состояние, метрики и действия интерфейса |
| `static/drive.js` | Кнопки, клавиатура, lease и показания одометрии |
| `static/camera-link.js` | Открытие отдельного окна камеры |
| `static/map.html`, `map.css`, `map-shell.js` | Карта и вкладки инструментов; локальные предпочтения просмотра |
| `static/map.js`, `localization-ui.js` | Карта/лидар, локализация, точки, описания маршрутов и сохранение |
| `static/costmap-ui.js`, `costmap-ui.css` | Наложение суммарных costmap Nav2 |
| `robot_nats/web.py` | HTTP-маршруты, NATS-клиент, Host/Origin guard |
| `robot_nats/service.py` | Обработка NATS-запросов и публикация метрик/KV |
| `robot_nats/contracts.py` | Subjects и машинный каталог метрик |
| `robot_nats/motion.py` | Ручная сессия, пределы скорости и тайм-аут |
| `robot_nats/telemetry.py` | ROS-наблюдения и отправка принятой команды скорости |
| `robot_nats/modes.py` | Адаптер единого supervisor, без собственных ROS-процессов |
| `robot_nats/bms_service.py`, `jk_ble.py` | Отдельная служба чтения BMS по BLE |

Режимы: [NATS_MODES_V1.md](../../docs/NATS_MODES_V1.md). Socket-клиент — `robot_nats/supervisor.py`.

## HTTP API

Главная страница показывает компактную сводку, режимы, ручное управление и
быстрые ссылки на окна наблюдения. Подробности BMS и диагностика раскрываются
по запросу. В `/map` карта находится рядом с вкладками «Локализация», «Точки»,
«Маршруты», «Слои» и «Сохранение». На мобильном экране инструменты идут под картой.
Выбор карты загружает каталог, но сам не запускает локализацию; запуск и
начальная поза требуют явного действия. Маршруты пока являются описаниями порядка точек.

Загрузка страниц, переключение вкладок, масштаб и слои только читают данные или
меняют локальное представление. Кнопки режимов проверяют текущий режим, карту,
свежесть и ручную сессию; причина блокировки видна в тексте и подсказке.
Результаты и ошибки действий показываются рядом с соответствующими элементами.

| Метод и путь | Действие |
|---|---|
| `GET /` | Страница |
| `GET /static/...` | JS/CSS |
| `GET /api/state` | NATS `service.command.state_get` |
| `GET /api/metrics` | NATS `service.command.metrics_get` |
| `GET /api/drive` | NATS `cmd_vel.command.status`, без управления |
| `POST /api/command` | Полный JSON-конверт с `action`; HTTP backend преобразует action в subject |

GET API возвращает NATS-конверт ответа `{v,id,ok,at,result}` либо ошибку.
POST принимает `application/json`, лимит HTTP-тела 16 KiB. Для скорости действует
дополнительный лимит NATS-конверта 4096 байт. Неизвестное action или невалидный JSON
дают HTTP 400, запрещённый Host/Origin — 403, неверный Content-Type — 415,
нет ответа NATS — 503. Ошибки исполнения команды передаются как `ok:false`.
Точные конверты, TTL, subjects и ошибки — [контракт NATS](../../docs/NATS_CONTRACTS_V1.md).

## Настройка и применение изменений

Настройки находятся в `config/nats/bridge.json` относительно корня проекта.
Текущие параметры: `web_host=0.0.0.0`, `web_port=8088`, `allow_manual_motion=true`,
`allow_mode_changes=true` на текущем роботе (по умолчанию установщика false). `web_allowed_hosts` ограничивает допустимые адреса;
при смене Wi-Fi IP список нужно обновить. Пароли NATS не помещать в HTML/JS.

HTTP-токена нет. Интерфейс предназначен для доверенной отладочной сети;
в production `web_enabled=false` отключает HTTP, сохраняя NATS API.

- Изменения `static/`: сборка не нужна, обновить страницу. При активном управлении
  сначала завершить сессию согласованным способом.
- Изменения Python или конфигурации: colcon не нужен, требуется согласованный
  перезапуск `verter-nats-bridge.service` при неактивном управлении.
- Форматирование: `tools/format_code.py`; установка инструментов описана
  в [OPERATIONS.md](../../docs/OPERATIONS.md).

Окно камеры на 8089 — отдельный `services/camera/static/` и `services/camera/server.py`.
[API камеры](../../docs/CAMERA_API_V1.md) описывает WebRTC и HTTP/MJPEG для SSH-туннеля.
React-интерфейс `web/frontend/` на 8081 обслуживается другой ROS-группой.

## Ограничения и проверки

NATS-режимы используют тот же supervisor, что и `tools/verter`: единственный
владелец SLAM/Nav2 — supervisor. Bridge только запрашивает переход и наблюдает
состояние; его перезапуск не завершает активный режим. Общего арбитра NATS/ESP32/Nav2 нет, используется один источник движения.

Unit-тесты находятся в `tests/test_*.py`; `test_motion.py` использует подменённый
получатель в памяти. Браузерные проверки кнопок выполняются только с подменённым
HTTP/backend либо изолированным NATS без ROS. Открытие страницы и GET-запросы
не отправляют движение или stop; нажатия кнопок на живом роботе — реальные команды.

Окно лидара на 8090 обслуживается отдельной службой `verter-lidar`; [API и подключение](../../docs/LIDAR_API_V1.md). Ссылка формируется в `static/camera-link.js`.


История BMS за неделю: `/battery-history` на том же порту 8088.
Все запросы истории проходят через NATS, сбор выполняется в backend без браузера.
Схема и хранение: [BMS_HISTORY_V1.md](../../docs/BMS_HISTORY_V1.md).


Окно карты — `GET /map`; данные — `GET /api/map?part=metadata|chunk|pose|model|scan|diagnostics|save_status|saved`.
Все запросы идут через NATS `<robot_id>.map.command.get`. Сбор ROS подписок
в map_observer.py, ограниченный кеш в map_data.py; издателей команд у них нет.
OpenLayers и зависимости находятся в static/lib и не требуют CDN в браузере.
Контракт: [MAP_API_V1.md](../../docs/MAP_API_V1.md).

Кнопка сохранения вызывает NATS `map.command.save` (HTTP action `map.save`).
`map_save.py` сохраняет сетку через supervisor и граф через slam_toolbox;
режим и движение не меняет. [Порядок картографирования](../../docs/MAPPING_GUIDE.md).
## Локализация, точки и определения маршрутов

Режим `localization` принадлежит supervisor и запускает только AMCL и map_server.
В окне `/map` доступны явная начальная поза, подтверждение по облаку лидара,
сохранение текущего положения, именованные точки и упорядоченные определения маршрутов.
Расчёт пути и выполнение маршрутов этим этапом не включаются.
Полный контракт: [LOCALIZATION_API_V1.md](../../docs/LOCALIZATION_API_V1.md).

Реализация: `robot_nats/localization.py`, `localization_ros.py`, `places.py`;
данные — `data/nats-bridge/places.sqlite`, последние снимки — KV
`localization.state`, `waypoint.catalog`, `route.catalog`.


## Costmap в окне карты

`/map` показывает локальный и глобальный costmap Nav2 с переключателями, прозрачностью и легендой. Используются `costmap_observer.py`, `costmap_data.py` и `static/costmap-ui.js/.css`. NATS: `<robot_id>.costmap.command.get`; HTTP: `GET /api/costmap`; обновления метаданных `.costmap.update.local/global`, KV `costmap.local/global`. Сетки передаются чанками, uint8+zlib, а не хранятся в KV.

Это только наблюдение за уже работающей navigation: режимы и costmap автоматически не запускаются. Локальная сетка привязывается к карте через TF на stamp сообщения; устаревшие слои скрываются. Цвет отражает стоимость общей сетки, без разделения по датчику. Контракт: [COSTMAP_API_V1.md](../../docs/COSTMAP_API_V1.md), каталог контрактов 1.8, JSON v1.
