# Живая карта и положение робота через NATS

Локальное окно: `/map` на том же порту 8088, ссылка в основном мониторинге.
OpenLayers 10.6.1 рисует карту, корпус и четыре колеса; масштаб, перемещение,
поворот вида, «Вся карта», «К роботу», следование за роботом работают в браузере.
Состояние карты и положение запрашиваются только через NATS. Клик по карте
показывает координаты и не задаёт цель. Открытие окна не включает SLAM/Nav2.
Режим mapping запускает оператор из основного окна через единый supervisor.

Общая панель ручного управления использует существующий drive.js и его API,
клавиатуру при фокусе, шаги и lease. Просмотр карты не создаёт команды скорости.
Тесты выполняются с полностью подменённым backend или отдельным NATS без ROS.

## Источники и координаты

Отдельная нода `verter_map_observer` только подписывается на ROS `/map`, `/scan` и TF.
Издателей скорости, навигационных целей и управляющих сервисов в ней нет.
Наблюдатель не запускает процессы. Карта доступна для активного режима supervisor
mapping/navigation с PID. При смене PID пересоздаётся transient-local подписка:
это получает сохранённое сообщение map_server и исключает старую сессию.
Сбой наблюдателя не останавливает BMS, управление, камеру или остальные службы.

OccupancyGrid — int8 в row-major порядке, i растёт по X, j по Y:
индекс j*width+i. -1 (байт 255) — неизвестно, 0 — свободно, 100 — занято,
значения 1..99 сохраняются. Для центра клетки:

`p_map = origin.xy + R(origin.yaw) * resolution * [i+0.5, j+0.5]`.

Origin может быть смещён и повёрнут, разрешение указано в метрах на клетку.
При рисовании строки изображения переворачиваются, потому что экранный Y растёт вниз.
Положение base_footprint вычисляет TF на Jetson непосредственно в map.
К x/y робота нельзя повторно прибавлять origin карты.

Модель — 2D-проекция размеров корпуса и колёс из geometry.json пакета
verter_description. Она представлена в base_footprint, затем поворачивается
и переносится по pose. Физическая достоверность позиции и качество SLAM
не подтверждаются самим наличием TF или картинки.

## NATS request/reply

Subject: `<robot_id>.map.command.get`. Это только чтение.
Стандартный конверт v1 с UUID, актуальным issued_at и TTL до 30 секунд:

```json
{
  "v": 1,
  "id": "f044fbcc-c447-48a2-a145-7a78f2253681",
  "issued_at": 1793001600,
  "ttl_s": 30,
  "args": {"part": "metadata"}
}
```

Таймстамп примера устаревший, его необходимо заменить. Поле action в NATS
не передаётся. Для HTTP POST /api/command action=`map.get`.
Ответ обычный `{v,id,ok,at,result}`; ошибка — `{v,id,ok:false,at,error}`.
Чтения не пишутся в журнал команд. Настройки allow_mode_changes и allow_manual_motion
не нужны для чтения карты. ACL client разрешает get, save и delete в домене map только своего робота;
подписка на `<robot_id>.*.update.>` уже включает сообщения карты.

### part=metadata (по умолчанию)

Нет карты: available=false, stale=true, reason (no_active_map/waiting_for_map/ошибка),
mode, map_id, frame_id, updated_at, save. Координаты и клетки не выдумываются.

Есть карта: available=true, robot_id, map_id, revision, frame_id=map,
width/height, resolution, origin={x,y,yaw}, stamp, updated_at, age_s, mode, stale, error.
Поле save содержит состояние последнего сохранения (см. ниже).
stamp — ROS header.stamp карты; age_s — возраст последнего получения /map.
Одинаковые клетки и геометрия не создают новую revision, но возраст обновляется.

map_id — идентификатор живой сессии (boot UUID наблюдателя + PID режима),
не имя сохранённого файла. После перезапуска bridge или смены режима он изменится.
revision — целое от 1, увеличивается при изменении клеток, размеров, origin или resolution.

Кодирование: encoding=`zlib-int8-row-major`, decoded_bytes=width*height,
compressed_bytes, chunk_bytes=49152, chunks=ceil(compressed_bytes/chunk_bytes),
sha256 — SHA-256 сжатых байтов, decoded_sha256 — SHA-256 исходных клеток.
Лимит — 4194304 клетки (4 МиБ без сжатия); более крупная карта явно отклоняется.
Неплоский origin и frame_id, отличный от map, также отклоняются.

### part=chunk

Args: `{part:"chunk",map_id:"...",revision:1,index:0}`.
Ответ: `{map_id,revision,index,chunks,data_b64}`; data_b64 содержит часть сжатого потока.
Клиент сначала читает metadata, получает все части строго одной версии, соединяет
байты, проверяет SHA-256, распаковывает zlib, проверяет размер и decoded_sha256.
Только после этого заменяет изображение. Не следует смешивать версии или считать
отсутствующую часть свободным пространством.

В памяти хранятся две последние версии. Чтение нулевой части дополнительно
закрепляет снимок на 60 секунд для медленного клиента; одновременно максимум
два закреплённых снимка. После истечения/вытеснения — ошибка
map_revision_unavailable_refetch_metadata: заново прочитать metadata и части.
Нет очереди всех кадров и хранения живой карты в JetStream.

### part=pose

Ответ: robot_id, map_id, revision, frame_id=map, child_frame_id=base_footprint,
updated_at, valid. При valid=true добавлены x/y в метрах, yaw в радианах,
stamp (ROS TF), sample_age_s и age_s. Положение имеет срок свежести 1 секунда.
Используется TF текущей сессии, полученный после первой карты.
При valid=false передаётся reason, координаты отсутствуют; нельзя рисовать робота
в (0,0) или продолжать показывать его как локализованного по старому TF.
Локальный веб рисует робота только при совпадении map_id/revision и свежих данных.

### part=model

Ответ available=true, frame_id=base_footprint, unit=m, source, footprint и
polygons=[{name,kind,points:[[x,y],...]}]. Kind — body/wheel.
При отсутствии геометрии available=false; интерфейс может показать только маркер
и стрелку направления по достоверной pose.

### part=scan

Последний скан, преобразованный через TF `map ← scan.header.frame_id` на точную
временную метку скана. JSON: v, robot_id, map_id, revision, frame_id=map,
updated_at, valid. При valid=true: stamp, sample_age_s, age_s, source_frame_id,
raw_count (число входных лучей), points=[[x,y],…] в метрах, не более 1800 точек.
Отбрасываются нечисловые и выходящие за range_min/range_max дальности;
более крупные сканы прореживаются по индексам. Это последняя 2D-проекция,
без накопления облака и без компенсации движения внутри одного оборота.
При отсутствии свежего TF облако не строится из приближённой позиции.
Срок свежести 1 с; при valid=false передаётся reason, поля points нет.
Проверяйте map_id/revision перед наложением, старые точки скрывайте.

### part=diagnostics

Статистика заголовков /scan за последние 10 с: samples_10s,
beam_count_min/max, angle_increment_min/max (рад), scan_time_min/max (с),
variable_scan_geometry. Кольцо ограничено 256 сообщениями. При отсутствии
данных samples_10s=0, variable_scan_geometry=null. Изменение количества лучей
или шага — повод проверить совместимость входа со SLAM; не оценка точности карты.

### part=save_status и part=saved

save_status возвращает последнее состояние задания сохранения и enabled/busy.
saved возвращает `{maps:[{name,grid,pose_graph,active,configured,has_definitions}]}`
из `data/maps`. `grid` и `pose_graph` обозначают наличие сетки и пары файлов графа.
`active` отмечает выбранную роботом карту, `configured` — карту из конфигурации
режимов, `has_definitions` — карту с точками или маршрутами. Список доступен
после запуска bridge, в том числе в режиме ожидания.

### part=saved_metadata и part=saved_chunk

Для просмотра сохранённой сетки вызовите `map.command.get` с
`{part:"saved_metadata",name:"floor_1"}`. Ответ содержит геометрию и поля кодирования
как у живой карты. Сетка читается из локальной пары YAML/PGM (P5 или P2).
Для PGM со стандартной палитрой Nav2 серая область неизвестных клеток
видна и в предпросмотре.
Для каждой части отправьте
`{part:"saved_chunk",name:"floor_1",map_id:"...",revision:1,index:0}`,
подставив идентификатор и ревизию из полученной metadata. При изменении карты
повторно запросите metadata и все части. Просмотр не выбирает карту для робота.
В режиме просмотра веб показывает сетку отдельно от живого скана, позы и costmap.

## Сохранение: map.command.save

Subject `<robot_id>.map.command.save`, обычный v1 request/reply-конверт без action,
args строго `{name:"floor_1"}`. Для HTTP `POST /api/command` добавляется action
`map.save`. Имя 1..64 ASCII-символа: первая буква/цифра, далее буквы/цифры/_/-.
Путь выбирает backend: `<project_root>/data/maps/<name>`. Произвольный путь,
перезапись существующих файлов и сохранение вне свежего mapping запрещены.

Настройка allow_map_save управляет доступностью; при её отсутствии используется
allow_mode_changes. Сохранение не требует allow_manual_motion. Backend передаёт
supervisor только save-map, затем вызывает `/slam_toolbox/serialize_map`.
Никаких mode/stop/cmd_vel/nav goals не отправляется. Смена режима через NATS
блокируется на время сохранения. Прямой CLI другого оператора остаётся отдельным
источником изменения режима: смена PID обнаруживается и помечает задание ошибкой.

Принятие запроса: ok=true, result.status=running. Завершение отслеживать через
map.get part=save_status или metadata.save. Поля: id, name, status
(idle/running/succeeded/failed/interrupted), stage, base_path, source_pid,
started_at/finished_at (Unix seconds), grid_saved, graph_saved, error,
files=[{name,bytes,sha256}], busy, enabled. Статус succeeded подтверждает четыре
непустых файла .yaml/.pgm/.posegraph/.data и .manifest.json с их хешами.
Сетка и граф сохраняются последовательно, не транзакционно; оператор должен
остановить робот перед сохранением. API сам не останавливает движение.

UUID пишется в журнал команд: повтор того же конверта возвращает исходное
подтверждение, не выполняя сохранение заново. TTL применяется при принятии;
он не является длительностью фонового задания. После таймаута нельзя считать
задание неисполненным: прочитайте save_status и список файлов. При обрыве bridge
задание не переигрывается автоматически. Частичные файлы сохраняются для
диагностики; повторная попытка с новым UUID должна использовать новое имя.

## Удаление сохранённой карты

Отправьте `<robot_id>.map.command.delete` с `args={"name":"floor_1"}`
в обычном v1-конверте. Через HTTP используйте `POST /api/command`
с `action: "map.delete"`. При успехе `result={"deleted":"floor_1"}`.
Для повторной отправки той же команды сохраните исходный UUID.

Настройка `allow_map_delete` по умолчанию наследует `allow_map_save`,
а затем `allow_mode_changes`. Активная карта, карта из конфигурации и карта
с сохранёнными точками либо маршрутами защищены. Если изображение используется
другой картой, удаление тоже отклоняется. При недоступном supervisor или
выполняющемся сохранении либо смене режима запрос нужно повторить после завершения операции.
Удаление убирает YAML, принадлежащий ему одноимённый PGM и имеющиеся файлы
графа SLAM и манифеста этой карты. Веб запрашивает подтверждение.

## Потоки и KV

- `<robot_id>.map.update.metadata`: та же metadata, до 1 Гц.
- `<robot_id>.map.update.pose`: та же pose, до 5 Гц.
- `<robot_id>.map.update.scan`: тот же scan, до 5 Гц, без хранения в KV/JetStream.
- KV текущего робота: `map.live` — только последняя metadata, без клеток.
- `service.snapshot.live_map`: та же компактная metadata для диагностики.

Сообщения потоков — прямой JSON, как lidar/camera API, без обёртки value/unit.
Core NATS не сохраняет кадры для отсутствовавшего зрителя: при подключении следует
выполнить map.command.get metadata/pose/model/scan и затем слушать update subjects.
Части передаются по запросу, это не бесконечный push всего изображения.
В mapping карта помечается stale через 15 секунд без /map; неизменяемая карта
navigation не требует повторного сообщения. Недоступность supervisor также ставит
stale. Пропажа TF скрывает модель робота, сохраняя само изображение карты.

## HTTP-адаптер и библиотеки

`GET /map` — окно. `GET /api/map?part=metadata|pose|model|scan|diagnostics|save_status|saved`;
`GET /api/map?part=chunk&map_id=...&revision=...&index=...`,
`GET /api/map?part=saved_metadata&name=...` и
`GET /api/map?part=saved_chunk&name=...&map_id=...&revision=...&index=...`.
Все запросы преобразуются в NATS request/reply, серверу доступен тот же контракт.
HTTP не читает отдельный обходной кеш карты. Действуют Host/Origin guards.
Ошибки числовых HTTP-параметров — 400; backend validation — 200 с ok=false;
нет NATS/ответа — 503. Дополнительный порт для SSH не нужен.

OpenLayers 10.6.1, pako 2.1.0 и js-sha256 0.11.1 находятся в static/lib вместе
с лицензиями и manifest.json (URL, версия в URL, SHA-256 файлов). В браузере нет
запросов к CDN и требований WebCrypto/HTTPS для проверки хешей. Библиотеки
включаются в Git как обычные ресурсы; они не находятся в игнорируемом vendor/.

Код: robot_nats/map_data.py, map_observer.py, static/map.html, map.css, map.js.
Сохранение реализует `map_save.py`, просмотр и удаление сохранённых карт — `saved_maps.py`.
Практический порядок работы: [MAPPING_GUIDE.md](MAPPING_GUIDE.md).

## Сохранённая карта и локализация

Наблюдатель карты работает в mapping/localization/navigation. Metadata содержит `saved_map`: имя выбранной сохранённой карты в localization/navigation, null в mapping. Это имя отличается от сессионного `map_id`. Щелчки для точек и initialpose применяются только к отображаемой выбранной saved_map. Контракт: [LOCALIZATION_API_V1.md](LOCALIZATION_API_V1.md).

### Согласование карты и costmap

Метаданные карты дополнительно содержат `mode_pid` — PID launch-группы текущего режима supervisor, либо null до появления владельца. Это наблюдение, а не разрешение движения. Наблюдатель costmap принимает карту только при совпадении этого PID с текущим режимом navigation; идентификатор `map_id` также связывает растры в вебе и API. Контракт наложения — [COSTMAP_API_V1.md](COSTMAP_API_V1.md).
