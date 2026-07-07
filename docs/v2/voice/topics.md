# Топики голосового пайплайна

Все топики используют тип-сообщение и QoS Profile depth=10 (Reliable). Имена без ведущего слэша — ноды в одном namespace.

## Издатели (publishers)

| Топик | Тип | Издатель | Назначение |
|-------|-----|----------|------------|
| `recognized_text` | `std_msgs/String` | `speech_to_text_node` | Распознанная фраза |
| `ai_question` | `std_msgs/String` | `recognition_node` | Вопрос для AI |
| `text_to_speech` | `std_msgs/String` | `recognition_node`, `ai_assistant_node` | Текст для синтеза |
| `play` | `std_msgs/String` | `recognition_node` | Команда звука (`trigger.wav`/`success.wav`/`fail_timeout.wav`) |
| `dialog_control` | `std_msgs/String` | `recognition_node` | `start_dialog`/`end_dialog` для AI |
| `speech_control` | `std_msgs/Bool` | `recognition_node` | Включить/выключить STT |
| `tts_control` | `std_msgs/Bool` | `text_to_speech_node` | Пауза/возобновление распознавания на время синтеза |

## Подписчики (subscribers)

| Топик | Подписчик | Обработчик |
|-------|-----------|------------|
| `recognized_text` | `recognition_node` | `_handle_recognized_text` |
| `tts_control` | `recognition_node` | `_handle_tts_control` |
| `tts_control` | `speech_to_text_node` | `_handle_activation` |
| `speech_control` | `speech_to_text_node` | `_handle_activation` |
| `text_to_speech` | `text_to_speech_node` | `text_callback` |
| `ai_question` | `ai_assistant_node` | обработчик вопроса |
| `dialog_control` | `ai_assistant_node` | управление контекстом |
| `play` | `sound_player_node` | `sound_command_callback` |

## Контракты сообщений

- `recognized_text.data` — строка на русском, нижний регистр не гарантируется (нормализация в `CommandProcessor`).
- `text_to_speech.data` — произвольный текст, длина не ограничена (разбиения на чанки нет).
- `tts_control.data=True` → микрофон включить; `False` → выключить.
- `dialog_control.data` принимает литералы `"start_dialog"` / `"end_dialog"`.

!!! warning "QoS"
    QoS depth=10, Reliability=Reliable. Командные пути (управление микрофоном, диалог) должны быть надёжными — потеря команды на путь аудио критична. Сенсорные потоки (аудио-чанки внутри STT) обрабатываются в процессе, не через ROS-топики.

## Удалённые топики

| Топик | Статус |
|-------|--------|
| `verter_commands` | **removed** — издатель удалён из `recognition_node` и `distance_sensors_node`. Голосовое управление движением не работало (не было подписчика). См. [refactor_notes.md](refactor_notes.md). |
