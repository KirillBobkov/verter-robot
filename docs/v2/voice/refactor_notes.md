# Заметки о рефакторинге голосовой подсистемы

Документ фиксирует, что планировалось изменить в текущем раунде работ (масштаб: **тесты + документация**, точечные правки мёртвого кода), и известные риски на будущее. Часть пунктов (удаление голосового движения, unit-тесты) на текущий момент **не реализована** — см. пометки ниже.

## Что сделано

### Голосовое управление движением (запланировано, не реализовано)

**Проблема:** `recognition_node` формирует команды `CHASSIS:LEFT:ASK:distance:pwm;...` и публикует их в топик `verter_commands`. Подписчика на этот топик **нет нигде** в кодовой базе — никто не парсит протокол `CHASSIS:`. Голосовые команды движения («вперёд»/«назад»/«влево»/«вправо») фактически не двигают робота.

**Планировалось удалить из `recognition_node.py`, но на текущий момент код присутствует:**

- словарь `chassis_commands`, параметры `default_distance`/`default_pwm`/`turn_distance`;
- метод `get_chassis_command`, `chassis_pattern`;
- publisher `command_pub` (`verter_commands`);
- `_process_chassis_command` и его вызов;
- `_execute_chassis_command`.

Приоритет обработки в active-listening сейчас **stop → chassis → AI** (стадия chassis всё ещё в цепочке, `recognition_node.py`, метод `_handle_active_listening`).

**В `distance_sensors_node.py`** мёртвый publisher `verter_commands` (создаётся, но никогда не используется) также **остался** в коде (`_setup_publisher`).

Проверка: `ros2 topic info /verter_commands` → 2 издателя (`recognition_node`, `distance_sensors_node`), 0 подписчиков.

### Документация

Создан раздел `docs/v2/voice/` со схемой пайплайна, FSM, матрицей топиков, таймингами и бизнес-кейсами.

### Тесты (см. план)

Unit-тесты чистых классов (`SpeechConfig`, `CommandProcessor`, `StateManager`, `TimerManager`) и интеграционные сценарии FSM (запланировано, не реализовано — в `tests/` тестов голосовой подсистемы нет).

## Не тронуто (по решению)

- **STT-движки** (3 node-файла в `speech_to_text/`: `speech_to_text_node.py`, `speech_to_text_sherpa_node.py`, `speech_to_text_transducer_node.py`; в `setup.py` объявлена также точка входа `speech_to_text_parakeet_node`, но сам `.py`-файл отсутствует) — только задокументированы. Дублирование поиска устройства/VAD/буферизации сохранено.
- **AI-диалог**, звуки, напоминания, pause/resume, `dialog_control` — поведение сохранено.
- **Safety-цепочка движения** (twist_mux, proximity_safety) — осталась для teleop/Nav2; голос к ней не подключён (голосовое управление движением публикует в `verter_commands`, у которого нет подписчиков, и не входит в twist_mux).

## Известные риски и рекомендации на будущее

### TTS: перезапись pending text (bug)

`_pending_text` в `text_to_speech_node.py` / `silero_tts_node.py` хранит одну строку — новое сообщение перезаписывает ожидающее. Рекомендация: `collections.deque` вместо одного поля. Отнесено к опциональным правкам.

### TTS: `import time` внутри функции

`time` импортируется локально в `_synthesize_and_play` в `text_to_speech_node.py` (в `silero_tts_node.py` импорт уже на верхнем уровне) — вынести на верхний уровень файла.

### TTS: магическая задержка 0.1с после синтеза

`sleep(0.1)` для защиты от эха может быть недостаточен для длинных фраз. Вынести в параметр/константу.

### STT: массовое дублирование кода

3 реализации (`speech_to_text_node.py`, `speech_to_text_sherpa_node.py`, `speech_to_text_transducer_node.py`) содержат дублированные `_vad_predict`, `_handle_activation`, `_process_chunk_logic`; `_find_audio_device` есть в sherpa- и transducer-вариантах, но отсутствует в `speech_to_text_node.py`. Рекомендация: общий базовый класс/миксин, специфика только инференс. Не сделано в этом раунде.

### AI: нет retry и мониторинга vector store

`expires_after=10 дней` без автопересоздания; сетевые ошибки сразу дают fallback без повторной попытки.

### Конфигурируемость

Таймауты/пороги захардкожены в dataclass-ах, нет ROS-параметров для переопределения извне (`declare_parameter`).

## Источник решений

План работ: `.claude/plans/tidy-coalescing-fairy.md` (файл плана в репозитории отсутствует). Масштаб и решения согласованы с пользователем: «только тесты + документация», голосовое движение удаляется, STT не трогается. Фактически удаление голосового движения и unit-тесты **не выполнены** — соответствующий код (`chassis_commands`, `_process_chassis_command`, `_execute_chassis_command`, publisher `verter_commands`) остался в `recognition_node.py` и `distance_sensors_node.py`.
