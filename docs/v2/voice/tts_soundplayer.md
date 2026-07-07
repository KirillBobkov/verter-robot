# TTS и звук

## Text-to-Speech

Два движка TTS, переключаются в `main.launch.py`. Активен **Piper**, Silero закомментирован.

### Piper (`text_to_speech_node.py`)

| Параметр | Значение |
|----------|----------|
| Модель | `ru_RU-ruslan-medium.onnx` (63 МБ) |
| Путь | `share/verter_admin/text_to_speech/ru_RU-ruslan-medium.onnx` |
| Sample rate | из конфигурации модели |
| Вывод | `aplay -D pulse -q -f S16_LE -r <rate> -c 1 --buffer-size=4096 --period-size=512` |

### Silero (`silero_tts_node.py`)

| Параметр | Значение |
|----------|----------|
| Модель | `v5_ru.pt` |
| Speaker | `eugene` |
| Sample rate | 24000 Гц |
| Вывод | `sounddevice.OutputStream` (device=None → default) |
| Тишина в начале | 0.25с (фикс «глотания» первого слога) |

### Очередь синтеза

Оба движка используют общую логику:

- `_busy_lock` (`threading.Lock`) — защита от race condition.
- `_busy` — флаг занятости.
- `_pending_text` — ожидающий текст; синтез идёт в отдельном потоке.

!!! bug "Перезапись pending text"
    `_pending_text` хранит **одну** строку: новое сообщение перезаписывает предыдущее, а не ставится в очередь. Потеря фраз при быстрой подаче. Рекомендация — `collections.deque`. См. [refactor_notes.md](refactor_notes.md).

### Управление микрофоном (защита от эха)

Перед синтезом TTS публикует `tts_control=False` → `RecognitionNode` → `PAUSED`, `speech_control=False` (микрофон off). После синтеза: `sleep(0.1)` → `tts_control=True` → микрофон on.

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
