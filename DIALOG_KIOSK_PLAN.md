# План: Диалоговый kiosk-фронтенд для робота Verter

> **Статус:** Этапы 0–5 реализованы. После реализации проведены два раунда аудита и упрощения (Раунд 2 — audit-правки, Раунд 3 — упрощение кода). Раздел «Python-правки» ниже описывает **первоначальный** план; актуальное состояние кода зафиксировано в разделе «История изменений (Раунды 2–3)» в конце и в `DIALOG_FLOW.md` (блок-схемы). При расхождениях верить коду + `DIALOG_FLOW.md`.
>
> **Доставка плана:** Этап 0 — экспорт этого плана в `DIALOG_KIOSK_PLAN.md` в корне репо (`/Users/kirbobkova/Documents/verter-robot/`), чтобы команда и агенты работали по единому документу.

## Context

Сейчас web-фронтенд `web/` — это набор страниц навигации/waypoints/маппинга с заглушкой диалога (`AskPage`), а диалог с роботом начинается **только** голосовым триггером «вертер». В коде нет ни единого топика статуса диалога, ни способа начать диалог кнопкой, ни таймаута на запрос к YandexGPT.

**Цель:** полноэкранный диалоговый kiosk-интерфейс на 10" экране робота. Пользователь нажимает «Начать диалог» → робот слушает вопрос (через микрофон/STT) → экран показывает вопрос, потом анимацию «думаю», потом ответ нейросети (typewriter-эффект, fade-переходы). Кнопка «Остановить диалог» повторяет существующий стоп-sequence. Ошибки AI/сети — типизированные дружелюбные сообщения + грустная анимация + голосовое дублирование. Запуск — systemd kiosk при старте робота.

**Принятые решения пользователя:**
1. Фронтенд — **только диалог**, всё навигационное удалить.
2. Python-ноды **менять разрешено**.
3. Дисплей — **kiosk на самом роботе** (Chromium fullscreen).
4. Веб-стек (rosbridge+web_server) **встроить в `main.launch.py`**.
5. Отображение — **одно текущее сообщение** (вопрос fade→ответ), не лента.
6. Ввод — **только голос** через микрофон робота (STT); на экране только кнопки.
7. «Остановить» = повторяет `_execute_stop_sequence` в recognition_node.
8. Ошибки: различать причину, дружелюбный текст + дублирование в `text_to_speech` + **грустная анимация**.
9. Триггер «вертер» **убрать**; диалог — только кнопкой.
10. Кнопка «Начать» → **сразу слушать вопрос** (без голосового приветствия).
11. **Жёсткий таймаут AI** (`AI_REQUEST_TIMEOUT=60`с; file_search YandexGPT может требовать >15с) → прервать, грустная анимация + сообщение + голос.
12. После ответа **продолжать диалог** (слушать следующий вопрос); возврат к приветствию по существующему `DIALOG_TIMEOUT=30с`.
13. **Показывать распознанный вопрос** на экране (через `ai_question`).
14. В idle (до кнопки) **STT выключен** (`speech_control False`).
15. rosbridge упал → **молча реконнектить**.
16. roslib → **локальный npm-бандл** (не CDN).

---

## Роли агентов

| Агент | Зона ответственности |
|---|---|
| **Бизнес-аналитик** | требования, user stories, acceptance criteria, traceability (требование → этап → проверка) |
| **Системный аналитик** | corner cases, контракты топиков, QoS, потоки данных, согласованность состояний |
| **Архитектор** | SDHR-соответствие, безопасность, запуск/systemd, зависимости, QoS-решения, review-гейты |
| **UX/UI дизайнер** | экраны, анимации робота, типографика (10", плохое зрение), палитра, расположение |
| **Разработчик** | реализация кода (Python + TS/CSS + launch + systemd) |

---

## Pipeline: этапы и review-гейты

Каждый этап выполняет Разработчик (или UX/UI для UI), после — **обязательный review-гейт** (Архитектор + Системный аналитик + Разработчик; для UI-этапа добавляется UX/UI). Гейт проверяет соответствие требованиям и контрактам; без прохождения — следующий этап не начинается.

### Этап 0 — Экспорт плана в репо
- **Исполнитель:** Разработчик.
- **Действие:** записать этот план в `DIALOG_KIOSK_PLAN.md` в корне репо.
- **Review-гейт 0:** Архитектор сверяет, что экспортированный план = утверждённому.

### Этап 1 — Python-ноды (backend-контракты)
- **Исполнитель:** Разработчик.
- **Действие:** правки `recognition_node.py` и `ai_assistant_node.py` (см. раздел «Python-правки»).
- **Review-гейт 1** (Архитектор + Системный аналитик + Разработчик):
  - Контракты `dialog_status`/`ui_dialog_control` соответствуют спецификации (значения, QoS, издатели).
  - SDHR: domain-классы чистые (QG-2/QG-3), safety-цепочка не затронута.
  - Corner cases: стоп во время thinking/speaking, таймаут AI, ре-активация STT после стопа (правка `_handle_tts_control`).
  - Триггер «вертер» убран; STT выключен в idle.
  - `colcon build` OK; ручная проверка `ros2 topic pub /ui_dialog_control`.
- **Acceptance (Бизнес-аналитик):** start → listening/STT вкл; stop → farewell + idle; нет сети → error:network + голос.

### Этап 2 — Launch + kiosk-инфра
- **Исполнитель:** Разработчик.
- **Действие:** правки `main.launch.py` + новые `services/verter-kiosk.service`, `services/start_kiosk.sh`.
- **Review-гейт 2** (Архитектор + Системный аналитик):
  - rosbridge/web_server в `main.launch.py`, порты 9090/8080.
  - Безопасность: rosbridge на localhost для kiosk; комментарий о неаутентифицированном доступе.
  - systemd: зависимости (After=graphical-session + verter-admin), рестарт, автологин.
  - `ros-humble-rosbridge-server` наличие проверено.
- **Acceptance:** `curl localhost:8080` → index.html; ws://localhost:9090 подключается.

### Этап 3 — Фронтенд-основа (roslib, stores, hooks)
- **Исполнитель:** Разработчик.
- **Действие:** удаление навигационных файлов; перепись `main.tsx`/`App.tsx`/`rosbridge.ts`; новые `store/dialogStore.ts`, `types/dialog.ts`, `hooks/useDialogROS.ts`; `package.json`.
- **Review-гейт 3** (Архитектор + Системный аналитик + Разработчик):
  - roslib из npm (не CDN); `npm run build` без TS-ошибок.
  - Подписки: `dialog_status`, `ai_question`, `text_to_speech`; публикация `ui_dialog_control`.
  - Тихий реконнект rosbridge (без UI).
  - QG-8: `grep -r "cmd_vel" web/frontend/src/` → пусто.
- **Acceptance:** `dialogStore` управляет состоянием; `useDialogROS` корректно маппит топики.

### Этап 4 — Фронтенд-UI (экраны, анимации, палитра)
- **Исполнитель:** UX/UI дизайнер (дизайн/анимации/CSS) + Разработчик (интеграция компонентов).
- **Действие:** `theme/*` (палитра ТЗ + анимации); `RobotAvatar`, `MessageZone`, `DialogControls`, `DialogPage` (+css).
- **Review-гейт 4** (UX/UI + Архитектор + Системный аналитик):
  - Палитра = ТЗ (`--accent:#c8ed00` и т.д.).
  - 5 анимаций аватара (idle/listening/thinking/speaking/sad); `errorType` приоритетнее `status` → sad сохраняется до listening/idle.
  - Typewriter появления + fade-down исчезновения; одно сообщение.
  - Типографика: ~24-32px, читаемость на 10" 1280x800 для плохого зрения; пропорциональное расположение.
  - Крупные touch-кнопки (min 64px); приветствие idle.
- **Acceptance:** fullscreen-интерфейс; кнопки публикуют `ui_dialog_control`.

### Этап 5 — Интеграция + E2E на роботе
- **Исполнитель:** Разработчик.
- **Действие:** `scripts/build_frontend.sh` → `colcon build --symlink-install`; установка kiosk service; E2E.
- **Review-гейт 5 — финальный** (Архитектор + Бизнес-аналитик + Системный аналитик + UX/UI):
  - Полный E2E-прогон по acceptance.
  - Traceability: все 16 требований покрыты (Бизнес-аналитик).
  - Rollback-готовность (независимые коммиты по этапам).
- **Acceptance:** Начать → вопрос → думаю → ответ (typewriter) → продолжение диалога; 30с тишины → idle; Остановить во время ответа → farewell + idle; нет сети → sad + сообщение + голос; rosbridge реконнект молча.

---

## Архитектура статусов диалога

### Топик `dialog_status` (std_msgs/String, QoS depth 10 RELIABLE, event-driven)
Единый источник правды для UI. Два издателя:
- **recognition_node** — агрегатор FSM: `idle`/`listening`/`speaking`.
- **ai_assistant_node** — transient: `thinking`, `error:timeout`/`error:network`/`error:unavailable`.

**Нормальный flow (гонки нет):** `listening` (UI start) → STT распознаёт → recognition шлёт `ai_question` + деактивирует STT → AI публикует `thinking` → AI публикует ответ в `text_to_speech` → TTS шлёт `tts_control=False` → recognition публикует `speaking` → TTS шлёт `tts_control=True` → recognition публикует `listening`.

**Стоп во время thinking** (единственная гонка): recognition публикует `idle`; AI-поток может дописать ответ → в `_process_request` проверка `if not self.dialog_active: return`.

### Топик `ui_dialog_control` (std_msgs/String, QoS depth 10 RELIABLE)
- **Издатель:** фронтенд (rosbridge, `publishDialogControl('start'|'stop')`).
- **Подписчик:** recognition_node (`_handle_ui_dialog_control`).
- `start` → `_ui_start_dialog()`: `speech_control True`, `CAPTURING_COMMAND`, `dialog_control "start_dialog"` в AI, `dialog_status "listening"`.
- `stop` → `_execute_stop_sequence()`.
- Corner: `start` в активном диалоге → игнор; `stop` в idle → no-op + `speech_control False` + `idle`.

---

## Python-правки

### `recognition/recognition_node.py`
1. **Новый publisher** (в `_setup_ros_communication`): `dialog_status` (String).
2. **Новая подписка:** `ui_dialog_control` → `_handle_ui_dialog_control`.
3. ~~**При запуске:** через стартовый таймер (2с, повтор 5с) опубликовать `speech_control False` + `dialog_status "idle"`~~ **→ заменено в Раунде 2:** TRANSIENT_LOCAL (latched) QoS на `/speech_control` (publisher + подписчик STT) — late-joining STT получает последнее значение автоматически, таймеры не нужны. В `__init__` один раз `_publish_speech_control(False)`. Дефолт STT инвертирован в `is_active=False` (defense-in-depth).
4. **Хелперы:** `_publish_dialog_status(status)`, `_publish_speech_control(active)`.
5. **Убрать триггер «вертер»:** `LISTENING_FOR_TRIGGER` остаётся как idle-состояние.
6. **`_handle_ui_dialog_control` + `_ui_start_dialog`:** программный старт. Гард: state ∈ {DIALOG_MODE, CAPTURING_COMMAND, PAUSED} → игнор.
7. ~~**`_execute_stop_sequence`:** 4 ветви~~ **→ свернуто в Раунде 3** в 2 ветви: `LISTENING_FOR_TRIGGER` → no-op; иначе → `_end_dialog(with_farewell=True)`.
8. **`_handle_tts_control` — КЛЮЧЕВАЯ ПРАВКА** (актуальный вид после Раунда 3, без watchdog):
   ```python
   def _handle_tts_control(self, msg: Bool) -> None:
       try:
           if msg.data:  # TTS закончил
               self._activate_recognition()
               self._resume_from_paused()  # DIALOG_MODE→listening; иначе→idle
           else:  # TTS начал говорить
               was_active = self.state_manager.state in (
                   RecognitionState.DIALOG_MODE,
                   RecognitionState.CAPTURING_COMMAND,
                   RecognitionState.PAUSED,
               )
               if was_active:
                   self._deactivate_recognition()  # только если диалог активен
                   if self.state_manager.state == RecognitionState.PAUSED:
                       self._publish_dialog_status('speaking')
       except Exception as e:
           self.get_logger().error(f"Ошибка TTS control: {e}")
   ```
   > `_resume_from_paused()`: если `DIALOG_MODE` → `speech_control True` + `listening` + `_reset_dialog_timer`; иначе (farewell в idle) → `speech_control False` + `idle`. Раунд 2 перенёс `_deactivate_recognition` внутрь `if was_active` (чинит «мёртвую кнопку Начать во время farewell»).
9. **`_activate_recognition`:** только `state_manager.resume()` (публикация `speech_control` — в `_resume_from_paused` по контексту).
10. **`_handle_command_timeout` / `_handle_dialog_timeout`:** в конец — `_publish_speech_control(False)` + `_publish_dialog_status('idle')`. Раунд 3: `_handle_command_timeout` упрощён до `_end_dialog()` + `idle`.
11. **`_end_dialog`:** в конец — `_publish_speech_control(False)`.
12. **`is_stop_command` (Раунд 3):** regex с `match` (привязка к началу фразы) + `\b` (граница слова). Стоп-слово только в начале: «спасибо за ответ» не останавливает, «конечно» не срабатывает на «конец».
13. **Раунд 2 — удалён мёртвый код** голосового управления шасси (`_process_chassis_command`, `CHASSIS:...`, `_transition_to_command_capture`, `SOUND_TRIGGER`) — нарушал SDHR (L3→actuator в обход safety-gate).

> SDHR: recognition_node — плоская структура, рефакторинг в гексагон не входит. Domain `CommandProcessor`/`StateManager` остаются чистыми (QG-2/QG-3).

### `ai_assistant/ai_assistant_node.py`
1. **Импорт** `httpx` наверх.
2. **Константа** `AI_REQUEST_TIMEOUT = 60`.
3. **Publisher** `dialog_status` (в `_setup_ros_interface`).
4. **Таймаут:** в `_initialize_ai` `OpenAI(..., timeout=httpx.Timeout(self.AI_REQUEST_TIMEOUT, connect=5.0))`.
5. **`_process_request`:** в начале `_publish_dialog_status('thinking')`; перед публикацией ответа/ошибки `if not self.dialog_active: return`. except-блок: `httpx.TimeoutException`→timeout, network (через модульную функцию `is_network_error`, Раунд 3 — дедупликация 3× проверки)→network, прочее→unavailable.
6. **`_handle_error(error_type)`:** публикует `dialog_status "error:{type}"` + дружелюбный текст в `text_to_speech`.
7. **`_publish_dialog_status(status)`:** хелпер.
8. ~~**`_dialog_lock`** вокруг check-and-publish~~ **→ удалён в Раунде 3.** Проверка `if not self.dialog_active: return` сохранена (3 точки: перед thinking/ответом/в error) как минимальный инвариант. Окно гонки ~1мс, последствие — лёгкий UX-дефект, lock избыточен.
9. **`_extract_text` (Раунд 3):** упрощен до 2 уровней (output_text + output scan).

### Тексты ошибок + анимация
| Тип | Текст (экран + голос) | Анимация |
|---|---|---|
| `error:timeout` | «Я бы попытался ответить, но мой мозг слишком долго думает. Может, попробуете ещё раз?» | грустный робот |
| `error:network` | «Я бы попытался ответить, но потерял связь с интернетом. Попробуйте через минуту.» | грустный робот |
| `error:unavailable` | «Я бы попытался ответить, но мой мозг не отвечает. Может, попробуете ещё раз?» | грустный робот |

> Все три — CSS-анимация `avatarSad`. Ошибка дублируется в голос (TTS) → `tts_control` → recognition ставит `speaking`. Фронтенд приоритезирует `errorType` для аватара → sad сохраняется до `listening`/`idle`.

---

## Launch + kiosk-инфра

### `launch/main.launch.py`
Добавить в `LaunchDescription`: `rosbridge_websocket` (port 9090), `web_server_node` (port 8080). Комментарий: rosbridge даёт неаутентифицированный доступ — на localhost для kiosk допустимо.

### `services/verter-kiosk.service` (новый, user unit)
```ini
[Unit]
Description=Verter Kiosk (Chromium fullscreen dialog UI)
After=graphical-session.target verter-admin.service
Wants=graphical-session.target
[Service]
Type=simple
Environment="DISPLAY=:0"
Environment="XDG_RUNTIME_DIR=/run/user/%U"
ExecStartPre=/bin/sleep 5
ExecStart=/home/jetson/verter-robot/verter_admin/services/start_kiosk.sh
Restart=always
RestartSec=10
[Install]
WantedBy=default.target
```
### `services/start_kiosk.sh` (новый)
Ждёт X-сессию + `http://localhost:8080` (до 30с), затем `exec chromium --kiosk --noerrdialogs --disable-infobars --no-first-run --start-fullscreen --window-size=1280,800 http://localhost:8080`. Имя браузера проверить на Jetson. Автологин: `AutomaticLoginEnable=True` в `/etc/gdm3/custom.conf`. Проверить `ros-humble-rosbridge-server`.

---

## Фронтенд-правки

### Удалить
- **pages/setup/** — всё; **pages/active/** — всё.
- **components/forms/** (LocationForm, LocationList); **components/layout/** (SetupLayout, ActiveLayout).
- **components/common/** — NavigationRail, EmergencyStop, MapPlaceholder, ActionCard, StatusLog, MaterialIcon, BigButton, Button.
- **hooks/** — usePose, useCmdVel, useIdleTimer.
- **store/** — navigationStore, waypointStore, uiStore.
- **types/** — domain.ts, roslib.d.ts; **config/** — pages.ts; **utils/** — quaternion.ts, keybindings.ts.

### Оставить (с правками)
- `components/common/ErrorBoundary.tsx` — без изменений.
- `hooks/useROS.ts` — убрать uiStore/recordActivity.
- `store/rosStore.ts` — как есть.
- `services/rosbridge.ts` — **полная перезапись**.
- `types/ros.ts` — вычистить навигационные типы.

### Создать
- `src/types/dialog.ts` — `DialogStatus`, `ErrorType`, `MessageRole`, `DialogMessage`.
- `src/store/dialogStore.ts` — zustand: `status`, `errorType`, `currentMessage {role,text}`, `isDialogActive` (= `status!=='idle'`); `setStatus` парсит `error:*`.
- `src/hooks/useDialogROS.ts` — подписки `dialog_status`/`ai_question`(→question)/`text_to_speech`(→роль по статусу: error→'error', иначе 'answer', idle→'farewell'); `startDialog()`/`stopDialog()`.
- `src/components/dialog/RobotAvatar.tsx` (+css) — CSS-only, 5 анимаций; `errorType` приоритетнее → sad.
- `src/components/dialog/MessageZone.tsx` (+css) — одно сообщение, typewriter + fadeDown; `clamp(1.5rem, 4vw, 2rem)`.
- `src/components/dialog/DialogControls.tsx` (+css) — «Начать диалог»(accent)/«Остановить»(danger), min 64px.
- `src/pages/dialog/DialogPage.tsx` (+css) — fullscreen flex-column: аватар/текст/кнопка; приветствие «Привет! Нажми на кнопку чтобы начать диалог!».

### Переписать
- **`main.tsx`** — убрать CDN roslib, `import 'roslib'` (npm).
- **`App.tsx`** — `useROS(true)` + `<ErrorBoundary><DialogPage/></ErrorBoundary>`, без роутинга.
- **`services/rosbridge.ts`** — npm `roslib`; тихий реконнект на `close` (3с, без UI); `publishDialogControl`/`subscribeDialogStatus`/`subscribeAIQuestion`/`subscribeTextToSpeech` + generic publish/subscribe; убрать cmd_vel/pose/waypoint/service.
- **`package.json`** — убрать `react-router-dom`, `i18next`, `react-i18next`; `roslib` остаётся.
- **`theme/variables.css`** — палитра ТЗ как первичные токены (`--accent:#c8ed00` и т.д.) + алиасы + `--dialog-font-size`, `--button-min-height:64px`.
- **`theme/animations.css`** — добавить `typewriter`, `fadeDown`, `avatarBreathe`, `avatarListen`, `avatarThink`, `avatarSpeak`, `avatarSad`; оставить `wave`/`blink`/`fadeIn`/`fadeOut`; убрать `emergencyPulse`/`shake`.
- **`theme/index.css`** — `html{font-size:20px}`, `body{overflow:hidden}`; убрать tablet media-queries.

> Палитра ТЗ: `--font-color:#000; --secondary-font-color:#313131; --bg-color:#f9f9f9; --accent:#c8ed00; --card-bg-color:#dbdbdbfc; --box-shadow:rgba(0,0%,0%,.155)` и т.д.

---

## Corner cases
| Кейс | Обработка |
|---|---|
| Старт во время speaking/thinking | state==PAUSED → `_ui_start_dialog` игнорит |
| Стоп во время thinking | PAUSED-ветка; AI-поток: `dialog_active` False → ответ не публикуется |
| Стоп во время speaking | PAUSED-ветка: farewell, idle; TTS договаривает |
| Таймаут AI (60с) | `error:timeout` + sad + голос; после TTS → `listening` |
| Нет сети при старте AI | уже в `_get_or_create_vector_store`; при запросе → `error:network` |
| Двойной start | гард по state → игнор |
| rosbridge упал | тихий реконнект 3с |
| Тишина 10с/30с | command/dialog timeout → `speech_control False` + `idle` |
| Farewell vs ответ в `text_to_speech` | роль по текущему статусу |
| Ошибка + голос | `errorType` приоритетнее `status` → sad до `listening`/`idle` |
| STT ре-активация после стопа | `_handle_tts_control` + `_resume_from_paused` (без watchdog) |
| TTS упал посередине речи | **не обрабатывается** (watchdog удалён в Раунде 3) — recognition зависает в PAUSED до рестарта сервиса; см. `DIALOG_FLOW.md` 4.7 |

## Риски / rollback
- `httpx.Timeout(AI_REQUEST_TIMEOUT=60)` прерывает ожидание ответа; для streaming-ответов таймаут применяется на ожидание данных. Резервный `threading.Timer` не требуется при 60с.
- ~~Стартовое `speech_control False` теряется~~ **→ решено в Раунде 2:** TRANSIENT_LOCAL QoS (latched) + дефолт STT `is_active=False`.
- **TTS упал посередине → зависание в PAUSED** (Раунд 3, watchdog удалён). Mitigation: systemd `Restart=always` на `verter-admin.service`. При необходимости — вернуть one-shot `threading.Timer`.
- Kiosk без X → `ExecStartPre=/bin/sleep 5`, `After=graphical-session.target`.
- `rosbridge_server` не установлен → `apt install`.
- **Rollback:** независимые коммиты по этапам → `git checkout` Python/launch/frontend; `systemctl --user disable verter-kiosk`.

## Verify
- **Unit/QG:** `PYTHONPATH=src pytest tests/unit/`; QG-2/QG-3; QG-8: `grep -r "cmd_vel" web/frontend/src/` → пусто.
- **Build:** `./scripts/build_frontend.sh && colcon build --packages-select verter_admin --symlink-install`.
- **E2E:** `systemctl --user start verter-admin`; `ros2 topic echo /dialog_status`; прогон по acceptance Этапа 5.

---

## История изменений (Раунды 2–3)

> Ниже — что изменилось в коде относительно первоначального плана выше. Блок-схемы актуального поведения — в `DIALOG_FLOW.md`.

### Раунд 2 — audit-правки (post-implementation)
1. **Init-таймеры → TRANSIENT_LOCAL.** Два отложенных таймера + 3 обёртки заменены latched QoS на `/speech_control` (обе стороны). Дефолт STT инвертирован в `is_active=False`.
2. **Стоп во время thinking** — `if not self.dialog_active: return` перед публикацией ответа/ошибки в AI.
3. **`_deactivate_recognition` в idle** — перенесён внутрь `if was_active` (чинит «мёртвую кнопку Начать во время farewell»).
4. **`command_timeout` в CAPTURING_COMMAND → `end_dialog`** в AI (раньше рассогласование FSM↔dialog_active).
5. **Гард `LISTENING_FOR_TRIGGER: return`** в `_handle_recognized_speech`.
6. **Удалён мёртвый код** голосового управления шасси (SDHR-нарушение).
7. Farewell-текст: `"Рад был помочь!"`; `dialog_status "idle"` публикуется **до** farewell.
8. ~~Monkey-patch OpenAI → `http_client=httpx.Client(transport=LoggingTransport(...))`.~~ **→ удалён:** логирующий HTTP-транспорт убран (избыточное логирование HTTP-трафика не несёт ценности на production). Клиент создаётся без `http_client`.

### Раунд 3 — упрощение кода (−121 строка)
1. **Watchdog на `tts_control` удалён полностью.** Решение пользователя: TTS надёжен, `try/finally` всегда публикует `tts_control=True`, падение процесса = рестарт сервиса через systemd. Угловой случай «TTS упал посередине» — известное ограничение (`DIALOG_FLOW.md` 4.7), не обрабатывается.
2. **`_dialog_lock` в AI удалён.** Проверка `if not self.dialog_active: return` сохранена (3 точки) как минимальный инвариант.
3. **`_execute_stop_sequence`** свёрнут в 2 ветви (`LISTENING_FOR_TRIGGER` → no-op; иначе → `_end_dialog(with_farewell=True)`).
4. **`_handle_command_timeout`** упрощён: `_end_dialog()` + `idle`.
5. **`_resume_from_paused`** — единый хелпер восстановления после `tts_control=True`.
6. **`is_stop_command`** — regex `match` (начало фразы) + `\b` (граница слова): «спасибо за ответ» не останавливает, «конечно» ≠ «конец».
7. **`_extract_text`** — 2 уровня. **`is_network_error`** — дедупликация 3× network-проверки (вынесена в модульную функцию уровня модуля).
8. **Удалён мёртвый код:** `is_testing`, `main_previous_id`, `_get_project_root`, `is_active_listening`, `_reset_to_listening_state`, `_start_command_timeout_timer` (inlined), `_is_valid_text`, `_compile_pattern`, `AUDIO_LATENCY`, `active_lock`, `shutdown()`, `_current_process`.

### Verify (Раунды 2–3)
- `python3 -m compileall -q src/verter_admin/recognition src/verter_admin/ai_assistant src/verter_admin/speech_to_text` — OK.
- `grep -n "TRANSIENT_LOCAL" recognition_node.py speech_to_text_node.py` — QoS есть.
- `grep -n "watchdog\|_dialog_lock\|_init_timer\|_initial_stt_off" recognition_node.py ai_assistant_node.py` — пусто.
- Сценарий «кнопка во время farewell»: press stop → farewell → press Start → стартует (раньше блокировался).
