# API камеры v1: Gemini 2, ROS 2, NATS и WebRTC


## Устройство и профиль

- Orbbec Gemini 2, VID:PID 2bc5:0670, serial AY6966300ZT, firmware 1.4.92.
- USB path 1-2.2.3: root bus 1 → порт 2 → порт 2 вложенного хаба → порт 3.
- Текущее подключение — USB 2.0, 480 Мбит/с; тип соединения доступен в `camera.status.device`. Это логический путь, он не определяет цвет или положение разъёма на корпусе Jetson. При переносе на другой USB-порт скорость и путь нужно проверить заново.
- UVC создаёт /dev/video0…5. Номера могут меняться; сервис выбирает устройство по серийному номеру через SDK, а не по /dev/videoN.
- Используется pyorbbecsdk v1.3.4, включающий OrbbecSDK 1.10.16. Отдельное udev-правило разрешает группе video доступ только к USB 2bc5:0670.
- Gemini 2 использует активное стереозрение; это не ToF-камера в строгом смысле.

## Разделение транспорта

NATS Core request/reply передаёт SDP offer/answer и команды управления видеосеансом. WebRTC передаёт видео напрямую между Jetson и браузером/сервером. KV содержит только последние метрики. Кадры и SDP не сохраняются в JetStream/KV.

Передавать сжатые кадры через NATS технически можно, но тогда потребуются собственные очереди, отбрасывание устаревших кадров, регулирование скорости, кодирование/декодирование в браузере. Для интерактивного просмотра используем стандартный WebRTC. ROS Image сохраняет метрическую глубину; WebRTC depth — только цветная визуализация, не источник измерений.

## NATS subjects

| Subject | action | args |
|---|---|---|
| <robot_id>.camera.command.status | camera.status | {} |
| <robot_id>.camera.command.ice | camera.ice | {} |
| <robot_id>.camera.command.offer | camera.offer | {"type":"offer","sdp":"…","stream":"color"} |
| <robot_id>.camera.command.close | camera.close | {"session_id":"UUID из offer"} |

stream: color или depth. Сейчас robot_id = verter_01. В отличие от прямых service/cmd_vel NATS subjects, камера всегда требует action в конверте.

```json
{
  "v": 1,
  "id": "46b87471-5858-43c1-840f-eb7d28cfe620",
  "issued_at": 1790946000.0,
  "ttl_s": 30,
  "action": "camera.offer",
  "args": {"type":"offer","sdp":"v=0\r\n…","stream":"color"}
}
```

issued_at — текущее Unix-время отправителя; пример времени не переиспользовать. TTL (0,30], допускается не более 2 секунд опережения часов. Лимит camera-запроса 65536 байт, SDP до 60000 символов; также действует max_payload брокера. Один video m-line, без audio/datachannel. Клиент создаёт recvonly video transceiver.

Успех: {"v":1,"id":"…","ok":true,"result":{…}}. Ошибка: {"v":1,"id":"… или null","ok":false,"error":"…"}. id повторного побайтово одинакового запроса возвращает сохранённый ответ в пределах 60 секунд жизни процесса и TTL; изменённое тело с тем же id → id_conflict. После перезапуска сеансы/кэш не восстанавливаются.

offer возвращает session_id, type="answer", sdp (включая ICE candidates), stream, max_duration_s=1800. Это full-SDP, non-trickle ICE: клиент дожидается iceGatheringState=complete до отправки offer. После получения answer вызывает setRemoteDescription. NATS timeout клиента рекомендуется 20 секунд. Неоткрытый сеанс автоматически закрывается через 30 секунд; максимум два сеанса, каждый живёт до 30 минут. close идемпотентен. При timeout offer не считать успехом: повторять тот же конверт или дождаться очистки незавершённого сеанса.

Основные ошибки: invalid_envelope, unsupported_version, expired, invalid_action, invalid_offer, one_video_only, camera_unavailable, session_limit, id_conflict, busy. Неизвестные ошибки SDK/SDP могут возвращать диагностическую строку; клиент должен показывать её и корректно закрывать свой PeerConnection.

status содержит state, stale, serial, device, error, streams.{color,depth}.{width,height,sample_at,age_s,frames}, sessions, transport, ros_topics, ice_configured, max_sessions. Счётчики сбрасываются при перезапуске процесса. stale=true при отсутствии любого потока или возрасте кадра более 3 секунд. Проверять свежесть самостоятельно, а не только state="streaming".

ice возвращает {"iceServers":[]}, либо настроенные STUN/TURN. Доступ к TURN credentials ограничивать ролями просмотра. Не помещать их в публичный KV.

## HTTP отладочного окна

- GET / — окно просмотра, http://192.168.0.9:8089/ в текущей сети.
- GET /api/camera/status, GET /api/camera/ice — генерируют запрос через NATS.
- POST /api/camera/offer и POST /api/camera/close — полный JSON-конверт выше.
- Все ответы имеют тот же формат, что NATS. Ошибки транспорта дают HTTP 503, ошибки команды — ok=false.
- Host allowlist и same-origin проверка; токенов нет согласно требованиям отладочной сети. В production web_enabled=false выключает HTTP, сохраняя NATS, ROS и WebRTC.
- Браузер не запрашивает свою камеру/микрофон. Открытие страницы не создаёт видеосеанс. Закрытие окна отправляет best-effort close; при потере сети действует серверная очистка.

## Сервер и внешние сети

Будущий сервер использует те же четыре NATS subjects через leaf-соединение. Варианты: передать SDP между браузером и роботом через свой backend либо самому принять WebRTC как клиент и ретранслировать через SFU. HTTP-порт робота открывать в интернет для этого не требуется.

NATS не переносит UDP-медиа и сам не решает NAT traversal. Для разных сетей нужны корректная маршрутизация/VPN или STUN/TURN. Настройка ice_servers в config/nats/camera.json — массив стандартных объектов {"urls":["turn:…"],"username":"…","credential":"…"}; для изменений перезапустить только verter-camera. В текущей конфигурации `ice_servers` пуст, подключение к центральному NATS отсутствует. Работа через разные сети требует отдельной настройки маршрута и проверки STUN/TURN.

Доступы: camera публикует только camera.update.*, KV camera.* и ответы _INBOX, подписывается только на команды камеры своего robot_id. camera_viewer может отправлять только четыре camera-команды и получать ответы. Обычному локальному client добавлены эти четыре subjects. На сервере ограничивать robot_id и _INBOX обычными ACL/account exports; схема этой камеры не заменяет общую изоляцию роботов.

## Метрики и KV

Каждые 2 с: <robot_id>.camera.update.{state,stale,sessions,device,color_age,depth_age}. KV bucket VERTER_01_STATE, ключи camera.state, camera.stale, camera.sessions, camera.device, camera.color_age, camera.depth_age.

Формат observation совместим с общим контрактом: v,value,unit,sample_at,updated_at,sample_id,max_age_s,quality. max_age_s=6. color_age/depth_age — секунды с последнего полученного кадра. sample_at у этих метрик означает время вычисления возраста, не время съёмки; у самого кадра есть отдельный sample_at в status. quality=ok означает свежий ответ сервиса, поэтому обязательно смотреть camera.stale и возраст. После остановки сервиса значение в KV остаётся: получатель сам проверяет updated_at/max_age_s. Кадры в KV не пишутся.

Существующий service.schema_get описывает базовый bridge; отдельный сервис камеры описан этим документом и CAMERA_CATALOG_V1.json. Для камеры использовать camera.status, не ожидать её метрики в агрегате service.metrics_get.

## ROS 2

| Topic | Тип | Формат |
|---|---|---|
| /camera/color/image_raw | sensor_msgs/msg/Image | bgr8, 640×480 |
| /camera/color/camera_info | sensor_msgs/msg/CameraInfo | заводская калибровка выбранного RGB-профиля |
| /camera/depth/image_raw | sensor_msgs/msg/Image | 16UC1, 640×400, миллиметры, 0=нет измерения |
| /camera/depth/camera_info | sensor_msgs/msg/CameraInfo | заводская калибровка выбранного depth-профиля |

Оба профиля заданы на 10 fps; QoS sensor data (BEST_EFFORT). Измеряется глубина вдоль оптической оси Z. SDK depth_scale применяется один раз до публикации. CameraInfo описывает соответствующий native stream, глубина не зарегистрирована на RGB. Для ROS Image и соответствующего CameraInfo stamp одинаков; stamp — системное ROS-время публикации/приёма на Jetson, не аппаратно синхронизированные часы экспозиции. Точная RGB-depth синхронизация не реализована. Отдельная нода depth_obstacles строит метрическое облако из native depth; регистрация на RGB для costmap не требуется.

TF отдельного сервиса: base_footprint → camera_link (x=-0.033,y=0,z=0.650 м, направление вперёд) → camera_depth_optical_frame. Оптические оси: Z вперёд, X вправо, Y вниз. camera_color_optical_frame получает заводское смещение/поворот RGB относительно depth из SDK, миллиметры преобразованы в метры и D→C матрица инвертирована для TF. Механическая точка пользователя принята за depth-центр; точное положение оптического центра относительно корпуса ещё не измерено.

camera_tof/camera_tof_optical в URDF остаются неиспользуемыми для Gemini 2 визуальными frames. Frames сервиса камеры не конфликтуют с robot_state_publisher. Для настройки Nav2 использовать camera_* frames сервиса после физической проверки установки.

Камера подключена к локальному и глобальному costmap Nav2 через отдельную службу verter-perception: `/camera/depth/image_raw` + CameraInfo → `/perception/depth/points` (PointCloud2, исходный stamp и optical frame, BEST_EFFORT, до 5 Гц). Пол, корпус и невалидные возвраты обрабатываются согласно [COSTMAP.md](COSTMAP.md); диапазон маркировки высот пока 0,08–1,5 м. Камера не подключена к SLAM или EKF. Лидар остаётся источником SLAM/AMCL. Ручное управление не проходит через costmap.

## Обслуживание

Установка SDK, udev и службы — [INSTALL.md](INSTALL.md); перезапуск и журналы — [OPERATIONS.md](OPERATIONS.md). Конфигурация `config/nats/camera.json` и резервные server.conf содержат секреты, их нельзя включать в комплект API. Закрытие видео не останавливает ROS-захват. Физическая калибровка, TURN и приёмка поведения после переподключения требуют отдельных проверок.

Официальные источники: https://github.com/orbbec/pyorbbecsdk/tree/v1.3.4 ; https://www.orbbec.com/staging/wp-content/uploads/2023/04/ORBBEC_Datasheet_Gemini-2.pdf ; https://aiortc.readthedocs.io/en/latest/api.html ; https://docs.nats.io/learn/core-nats/ .


## Просмотр через SSH / HTTP

Окно камеры позволяет выбрать транспорт. При hostname localhost/127.0.0.1/::1 по умолчанию выбран HTTP, в локальной сети — WebRTC. Подключение начинается только после кнопки «Подключить». Все URL HTTP-потока относительные: работает и при другом локальном порте SSH-туннеля.

`GET /api/camera/mjpeg?stream=color` или `stream=depth` отдаёт `multipart/x-mixed-replace; boundary=frame`. Каждая часть — image/jpeg с Content-Length и X-Sample-At (Unix seconds, время получения кадра сервисом). Отсутствующий stream означает color. Неизвестные параметры/stream дают 400; нет свежего кадра — 503; превышен лимит зрителей — 429. Применяются те же Host/Origin/Sec-Fetch-Site проверки. HEAD не поддерживается.

До 5 кадров/с, JPEG quality 75, два одновременных HTTP-зрителя независимо от лимита двух WebRTC-сеансов. Кадры берутся из существующего захвата, новое соединение с USB не открывается. Медленному клиенту не накапливается очередь кадров: отправляется последний доступный, запись прерывается после 5 секунд блокировки. Сеанс ограничен 30 минутами. При потере камеры поток завершается, при отключении пользователя освобождается место. «Отключить видео» и закрытие окна отменяют HTTP-запрос; команды NATS/ROS для этого не нужны. Цветная визуализация depth в JPEG не заменяет метрические ROS-данные.

Это отдельный HTTP media endpoint, он не отправляет изображения через NATS. WebRTC использует отдельную NATS-сигнализацию, описанную выше. Поле sessions в camera.status считает только WebRTC. В production с web_enabled=false выключаются и страница, и MJPEG.

Пример SSH:

```bash
ssh -N -p 3333 -L 8089:127.0.0.1:8089 jetson@109.195.134.20
```

Открыть http://127.0.0.1:8089/, выбрать «Через SSH / HTTP» и нажать «Подключить». Дополнительные UDP-порты/TURN не нужны. При занятом локальном 8089 можно заменить только первое число после -L, затем открыть выбранный локальный порт. Основной мониторинг 8088 пробрасывается отдельно при необходимости.


Панель управления, HTTP-прокси команд и фокус клавиатуры описаны в
[NATS_MANUAL_CONTROL_V1.md](NATS_MANUAL_CONTROL_V1.md#панель-ручного-управления-в-дополнительных-окнах).
