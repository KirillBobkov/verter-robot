# Голосовой пайплайн

Голосовая подсистема Verter Robot обеспечивает полный цикл речевого взаимодействия: распознавание речи (STT), маршрутизацию команд через конечный автомат, ответы AI-ассистента (YandexGPT) и синтез речи (TTS) со звуковыми эффектами. Запускается единым launch-файлом:

```bash
ros2 launch verter_admin main.launch.py
```

## Компоненты

| Компонент | Пакет / файл | Назначение |
|-----------|--------------|------------|
| STT | `speech_to_text/` | Распознавание речи с VAD (Silero), 4 движка на выбор |
| Recognition (FSM) | `recognition/recognition_node.py` | Конечный автомат диалога, парсинг команд, таймауты |
| AI-ассистент | `ai_assistant/ai_assistant_node.py` | YandexGPT, контекст диалога, векторный поиск |
| TTS | `text_to_speech/text_to_speech_node.py` | Синтез речи (Piper, активен; Silero закомментирован) |
| Sound player | `sound_player/sound_player_node.py` | Короткие звуковые эффекты (триггер, успех, таймаут) |

## Сквозная схема

```
ReSpeaker MIC ──▶ speech_to_text_node ──/recognized_text──▶ RecognitionNode (FSM)
 (VAD Silero)     (CTC по умолчанию)                           │
                                                              ├─/ai_question────▶ ai_assistant_node (YandexGPT)
                                                              │                       │
                                                              │                       └─/text_to_speech──▶ text_to_speech_node (Piper)
                                                              │                                                  │
                                                              ├─/text_to_speech──▶ text_to_speech_node ─────────┤
                                                              ├─/play────────────▶ sound_player_node            │
                                                              ├─/dialog_control──▶ ai_assistant_node            │
                                                              ├─/speech_control──▶ speech_to_text_node (on/off) │
                                                              └────────────────────────────────── /tts_control ◀┘ (Bool: pause/resume)
                                                                  (TTS глушит микрофон на время синтеза)
```

Поток данных:

1. Микрофон ReSpeaker → `speech_to_text_node` (VAD + инференс) → публикует `recognized_text`.
2. `RecognitionNode` принимает текст, маршрутизирует по состоянию FSM.
3. Вопросы уходят в `ai_assistant_node` → ответ возвращается в `text_to_speech`.
4. На время синтеза TTS шлёт `tts_control=False` → микрофон гасится (защита от эха), после — `tts_control=True`.

## Разделы документации

- [FSM диалога](fsm.md) — состояния, переходы, таблица.
- [Топики](topics.md) — матрица интерфейсов с QoS и направлением.
- [Тайминги и паузы](timings.md) — все временные константы.
- [Бизнес-кейсы диалога](dialog_cases.md) — сценарии взаимодействия.
- [STT-движки](stt_engines.md) — 4 реализации распознавания.
- [TTS и звук](tts_soundplayer.md) — синтез и звуковые эффекты.
- [AI-ассистент](ai_assistant.md) — YandexGPT, контекст, векторный поиск.
- [Заметки о рефакторинге](refactor_notes.md) — что изменено и известные риски.
