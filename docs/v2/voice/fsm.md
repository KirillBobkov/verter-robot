# FSM диалога

Конечный автомат голосового диалога реализован в `recognition_node.py`. Чистая логика состояний вынесена в класс `StateManager` (тестируется без ROS).

## Состояния

| Состояние | Описание |
|-----------|----------|
| `LISTENING_FOR_TRIGGER` | Пассивное прослушивание, ожидание wake-word («вертер»/«робот») |
| `CAPTURING_COMMAND` | Захват команды после триггера, активное прослушивание |
| `DIALOG_MODE` | Активный диалог с AI, ответы озвучиваются, контекст хранится |
| `PAUSED` | Микрофон выключен (TTS говорит), вся речь игнорируется |

## Диаграмма переходов

```
                 trigger                              команда
LISTENING_FOR_TRIGGER ───────▶ CAPTURING_COMMAND ────────────▶ DIALOG_MODE
        ▲                            │                              │
        │ stop / timeout             │ timeout                      │ stop / timeout
        └────────────────────────────┴──────────────────────────────┘
                         ▲ pause (tts_control=False) │ ▼ resume (tts_control=True)
                              PAUSED  ◀───────── любое активное состояние
```

## Таблица переходов

| Из | Событие | В | Действия |
|----|---------|----|----------|
| LISTENING_FOR_TRIGGER | обнаружен триггер (без команды) | CAPTURING_COMMAND | звук `trigger.wav`, старт `command_timeout` (10с) |
| LISTENING_FOR_TRIGGER | триггер + команда одним сообщением | DIALOG_MODE | сразу `start_dialog` + обработка команды |
| CAPTURING_COMMAND | получена команда | DIALOG_MODE | `start_dialog`, `dialog_control=start_dialog` |
| CAPTURING_COMMAND | стоп-команда | LISTENING_FOR_TRIGGER | farewell в TTS, сброс таймеров |
| CAPTURING_COMMAND | `command_timeout` (10с) | LISTENING_FOR_TRIGGER | звук `fail_timeout.wav` |
| DIALOG_MODE | стоп-команда | LISTENING_FOR_TRIGGER | farewell в TTS, `dialog_control=end_dialog` |
| DIALOG_MODE | `dialog_timeout` (30с) | LISTENING_FOR_TRIGGER | звук `fail_timeout.wav`, `end_dialog` |
| DIALOG_MODE | новая команда | DIALOG_MODE | сброс `dialog_timeout` (30с) |
| любое активное | `tts_control=False` | PAUSED | остановка таймеров, `speech_control=False` |
| PAUSED | `tts_control=True` | восстановление `_previous_state` | `speech_control=True`, рестарт таймеров |

## Приоритет обработки команд

В активном прослушивании (`CAPTURING_COMMAND`/`DIALOG_MODE`) распознанный текст проверяется в порядке приоритета:

1. **Стоп-команда** (`хватит`, `спасибо`, `пока`, `конец`, …) — высший приоритет.
2. **Команда AI** — всё остальное длиной ≥ `MIN_COMMAND_LENGTH` (5 символов).

!!! note "Голосовое движение удалено"
    Ранее между стопом и AI была ступень chassis-команд (`вперед`/`назад`/`влево`/`вправо`). Она удалена — см. [refactor_notes.md](refactor_notes.md).

## Управление микрофоном

Распознавание STT включается/выключается через `speech_control` (Bool). `RecognitionNode` деактивирует распознавание:

- при отправке вопроса в AI (чтобы не накапливать буфер во время обработки);
- при таймауте напоминания (TTS сам вернёт `tts_control=True` после озвучки);
- при получении `tts_control=False` от TTS.
