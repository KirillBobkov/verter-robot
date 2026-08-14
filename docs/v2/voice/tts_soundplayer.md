# TTS и звук

## Text-to-Speech

Два движка TTS, переключаются в `main.launch.py`. Активен **Silero** (`silero_tts_node`); Piper (`text_to_speech_node`) закомментирован. Также в коде присутствует `vosk_tts_node.py` (закомментирован в `main.launch.py`).

### Piper (`text_to_speech_node.py`)

Закомментирован в `main.launch.py`.

| Параметр | Значение |
|----------|----------|
| Модель | `ru_RU-ruslan-medium.onnx` (~60 МиБ) |
| Путь | `share/verter_admin/text_to_speech/ru_RU-ruslan-medium.onnx` |
| Sample rate | из конфигурации модели |
| Вывод | `aplay -D pulse -q -f S16_LE -r <rate> -c 1 --buffer-size=4096 --period-size=512` |

### Silero (`silero_tts_node.py`)

Активен в `main.launch.py`.

| Параметр | Значение |
|----------|----------|
| Модель | `{model_version}.pt` (по умолчанию `v5_5_ru.pt`; допустимо `v5_ru`) |
| Speaker | `eugene` (допустимо: `aidar`, `baya`, `kseniya`, `xenia`, `eugene`) |
| Sample rate | 24000 Гц |
| model_version | `v5_5_ru` |
| Вывод | `sounddevice.OutputStream` (device=None → default при `audio_device='pulse'`) |

### Очередь синтеза

Оба движка используют общую логику:

- `_busy_lock` (`threading.Lock`) — защита от race condition.
- `_busy` — флаг занятости.
- `_pending_text` — ожидающий текст; синтез идёт в отдельном потоке.

!!! bug "Перезапись pending text"
    `_pending_text` хранит **одну** строку: новое сообщение перезаписывает предыдущее, а не ставится в очередь. Потеря фраз при быстрой подаче. Рекомендация — `collections.deque`. См. [refactor_notes.md](refactor_notes.md).

### Управление микрофоном (защита от эха)

Перед синтезом TTS публикует `tts_control=False` → `RecognitionNode` переходит в `PAUSED` и публикует `speech_control=False` (микрофон off в `speech_to_text_node`). После синтеза: `sleep(0.1)` → `tts_control=True` → `RecognitionNode` публикует `speech_control=True` → микрофон on. TTS публикует только `tts_control`; `speech_control` издаёт `recognition_node`.

## Sound player (`sound_player_node.py`)

Короткие звуковые эффекты, управляется топиком `play` (`std_msgs/String`).

### Набор звуков

| Файл | Команда | Назначение |
|------|---------|------------|
| `trigger.wav` | `trigger` | Обнаружен wake-word |
| `success.wav` | `success` | Команда принята |
| `fail_timeout.wav` | `fail_timeout` | Таймаут |

### Воспроизведение

- WAV → `aplay -D pulse -q -f S16_LE -r 44100 -c 2 --buffer-size=4096 --period-size=512`.
- MP3/OGG → `ffmpeg` → `aplay`.
- `stop_sound` → убийство текущих процессов; `_kill_all_audio_processes` → `pkill -f aplay && pkill -f ffmpeg`.

!!! warning "Нет блокировки с TTS"
    Sound player не проверяет занятость TTS — теоретически звук может проиграться поверх синтеза. Короткая длина эффектов снижает риск.
