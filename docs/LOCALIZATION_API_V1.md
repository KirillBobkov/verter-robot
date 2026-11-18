# Локализация, точки и маршруты через NATS

## Текущий объём

Режим `localization` запускает только `map_server`, `amcl` и их lifecycle manager.
Контроллер движения, behaviors, BT navigator и waypoint follower не запускаются.
Загрузка карты не задаёт начальную позу автоматически и не посылает скорости.
Начальную позу задаёт оператор. Малой ковариации недостаточно для подтверждения
физически правильного положения.

Именованные точки и определения маршрутов сохраняются. Выполнение маршрутов,
цели Nav2 и расчёт проходимого пути в этом API пока не реализованы. Линии между
точками в вебе — схема порядка, а не рассчитанная траектория обхода препятствий.
Управление скоростью существует отдельно, в общем виджете ручного управления.

## Порядок работы

1. Успешно сохранить карту в mapping. Проверить `map.get` с `part=save_status`.
2. Завершить mapping через `mode.set` с `mode=idle, discard=true`.
   Это разрешает штатный выход из сессии; существующий supervisor может отправить
   нулевую скорость. Переключение и любые реальные команды предварительно согласуются.
3. Запустить `mode.set` с `mode=localization, map=survey_20261030_173610`.
   Передаётся имя карты, а не произвольный путь. Readiness означает готовую карту
   и активный AMCL; отсутствие начальной позы не считается провалом запуска.
4. Открыть `/map`, выбрать ту же карту, указать положение и направление корпуса.
   Начальный угол в API — радианы, против часовой стрелки от оси +x карты.
5. Выполнить `localization.initialize`, дождаться оценки и отображения корпуса
   с облаком `/scan`. Проверить совпадение с неподвижными объектами.
6. Только после визуальной проверки выполнить `localization.confirm`.
7. Сохранить текущую позу как именованную точку или задать координаты вручную.
   Составить упорядоченный маршрут из сохранённых точек.

CLI, выполняемый только оператором после согласования:

```bash
cd /home/jetson/verter-robot
tools/verter mode localization --map /home/jetson/verter-robot/data/maps/survey_20261030_173610.yaml
tools/verter localization-status
```

Изменение активного режима сначала проходит через idle. Переход из localization
в полный navigation создаёт новую сессию AMCL: начальную позу и подтверждение
необходимо задать заново. Перезапуск bridge тоже отзывает его подтверждение.
Автоматический старт с позой базы пока не включён: случайное положение не угадывается.

## Транспорт

Subject: `<robot_id>.<domain>.command.<operation>`.
Домены: `localization`, `waypoint`, `route`.
NATS request/reply использует конверт `{v,id,issued_at,ttl_s,args}`.
`v=1`, `id` — новый UUID для нового намеренного действия, время — Unix seconds,
`0 < ttl_s <= 30`. Поле `action` передаётся только в HTTP `/api/command`.
Ответ: `{v,id,ok,at,result}` либо `{v,id,ok:false,at,error}`.
Повторы изменяющего запроса с тем же UUID и тем же payload возвращают
сохранённый ответ. Истёкший запрос не выполняется. Незавершённый результат после
сбоя требует чтения состояния; автоматического повтора initialpose нет.

| NATS operation | HTTP action | args |
|---|---|---|
| localization.command.get | localization.get | `{}` |
| localization.command.initialize | localization.initialize | `map, session_id, x, y, yaw, std_xy?, std_yaw?` |
| localization.command.confirm | localization.confirm | `map, session_id, initialization_id` |
| waypoint.command.list | waypoint.list | `map` |
| waypoint.command.save | waypoint.save | `map, id, label, x, y, yaw` либо `map, id, label, source:"current"` |
| waypoint.command.delete | waypoint.delete | `map, id` |
| route.command.list | route.list | `map` |
| route.command.save | route.save | `map, id, label, kind, points` |
| route.command.delete | route.delete | `map, id` |

В таблице опущен префикс `<robot_id>.`. Неизвестные поля отвергаются.
`std_xy` по умолчанию 0.25 м, допустимо `(0,10]`; `std_yaw` по умолчанию
pi/12 рад, допустимо `(0,pi]`. Поза должна лежать в прямоугольнике карты;
это не проверка коллизий. Только явный initialize публикует один
`geometry_msgs/PoseWithCovarianceStamped` в `/initialpose`, проверяя активность
AMCL, неизменность сессии и TTL непосредственно перед публикацией.
При неактивном AMCL запрос отвергается и не откладывается до активации.

## Состояние локализации

`localization.get` возвращает `map`, `session_id`, `initialization_id`, `status`,
`enabled`, `verified`, `operator_confirmed`, `ready_for_confirmation`, `reason`,
а при наличии пригодных измерений — `pose` и `amcl` с ковариацией и возрастом.
Статусы: `unavailable`, `publishing`, `waiting_estimate`, `unverified`, `verified`.
`verified` означает подтверждение оператором в этой сессии и доступность текущих
данных, а не независимую гарантию точности.

Сессия привязана к запуску bridge, PID режима и пути выбранной карты. Каждая
новая начальная поза меняет `initialization_id` и отзывает подтверждение.
Издатель `/initialpose` создаётся только по explicit command; при старте,
подключении NATS и опросе состояния ничего не публикуется.
Старое retained `/amcl_pose` не становится новым измерением при переподключении.
У стоящего робота AMCL может редко публиковать оценку: её возраст показывается
отдельно. Пригодность текущей позы дополнительно требует свежих TF и скана
текущей карты и оценки AMCL, относящейся к текущей сессии/инициализации.
Потеря данных блокирует сохранение текущей позы. Ненулевые скорости не отправляются.

## Точки

`id`: 1–64 ASCII буквы/цифры/`_`/`-`, первый символ буква или цифра.
`label`: 1–100 символов. Координаты: числа, метры, радианы, frame `map`.
`save` создаёт либо заменяет запись с тем же id внутри выбранной карты.
Максимум 200 точек и 200 маршрутов на карту, документ не более 200 KiB.

`source:"current"` не принимает явные координаты. Используется свежая текущая
поза после `localization.confirm`; карта точки должна совпасть с картой локализации.
Точки с явными координатами можно редактировать без запуска робота.
Веб принимает щелчки только на отображаемой карте выбранной сессии.
Удаление точки, на которую ссылается маршрут, отвергается.

Результат `waypoint.list`: `{map,map_revision,waypoints:[...]}`.
Запись содержит `id,label,x,y,yaw,frame_id,source,updated_at`.
`waypoint.save`: `{map,map_revision,waypoint:{...}}`; delete: `{map,map_revision,deleted:id}`.

## Определения маршрутов

`route.save`: `kind="visit"` или `"through"`, `points` — упорядоченный массив
из 1–100 существующих id точек той же карты. Повторы разрешены.
`visit` обозначает планируемые посещения с остановками, `through` — будущий
проход через промежуточные точки. Это семантика определения, исполнения ещё нет.
`route.list`: `{map,map_revision,routes:[{id,label,kind,points,updated_at}]}`.
`route.save`: `{map,map_revision,route:{...}}`; delete: `{map,map_revision,deleted:id}`.

## Хранение и обновления

Авторитетное долговременное хранилище — `data/nats-bridge/places.sqlite`.
Записи привязаны к имени карты и SHA256 семантических параметров YAML и изображения.
При замене геометрии карты старые определения не используются автоматически:
возвращается `saved_map_changed_existing_points_require_review`.

KV хранит последние снимки, а не очередь заданий:

| KV | Subject | Содержимое value |
|---|---|---|
| localization.state | `<robot_id>.localization.update.state` | статус локализации |
| waypoint.catalog | `<robot_id>.waypoint.update.catalog` | точки активной saved-карты |
| route.catalog | `<robot_id>.route.update.catalog` | определения маршрутов активной карты |

Используется общий observation-конверт `value,unit,sample_at,updated_at,sample_id,quality,max_age_s`.
В idle выбранной активной карты нет: каталоги пусты; сохранённые определения
доступны через list с явным map. Наличие определения в KV никогда не запускает движение.
Доступ клиента ограничивается своим robot_id, backend остаётся единственным
писателем KV. Публичных credentials в документации нет.

## Файлы реализации

`robot_nats/localization.py` — состояние и подтверждение;
`localization_ros.py` — один guarded initialpose;
`places.py` — привязка к карте и SQLite;
`static/localization-ui.js` — формы и OpenLayers overlays;
`verter_bringup/launch/localization.launch.py` — AMCL и map_server.
