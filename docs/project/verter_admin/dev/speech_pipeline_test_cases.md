# Speech pipeline — тест-кейсы (актуально)

Это тест-кейсы для **текущего** пайплайна:

`speech_to_text_*` → `recognized_text` → `recognition_node` → (`verter_commands` | `ai_question`) → (`chassis_node` | `ai_assistant_node`) → `text_to_speech` → `text_to_speech_node` → `tts_control`.

## Контракты (что должно быть правдой)

- STT публикует **только** `recognized_text` (String).
- `recognition_node` **не распознаёт аудио**, он только роутит текст и управляет состояниями.
- Мьют на время озвучки делается через **`tts_control` (Bool)**:
  - `false` → STT должен перестать публиковать
  - `true` → STT снова может публиковать
- Дополнительно `recognition_node` может управлять STT через **`speech_control` (Bool)** (например, пока ждём AI).

## Кейс 1: Триггер → команда → AI → TTS

```
👤 "Робот как пройти в регистратуру"
STT → recognized_text="робот как пройти в регистратуру"
recognition_node:
  - триггер найден
  - команда не шасси, не стоп-слово → ai_question="как пройти в регистратуру"
ai_assistant_node → text_to_speech="..."
text_to_speech_node:
  - публикует tts_control=false
  - говорит
  - публикует tts_control=true
```

Проверки:
- `ai_question` ровно без триггера.
- Во время речи STT не публикует новый `recognized_text` от собственной озвучки.

## Кейс 2: Триггер без команды → CAPTURING_COMMAND → таймаут

```
👤 "Робот"
STT → recognized_text="робот"
recognition_node → CAPTURING_COMMAND + старт таймера
... тишина ...
recognition_node → звуковой сигнал timeout + возврат в LISTENING_FOR_TRIGGER
```

## Кейс 3: Команда шасси в активном режиме

```
👤 "Робот вперед"
STT → recognized_text="робот вперед"
recognition_node → публикует verter_commands="CHASSIS:LEFT:ASK:0.5:98;CHASSIS:RIGHT:ASK:0.5:98" (пример)
chassis_node → отправляет строку в Arduino по serial
```

Проверки:
- Команда в `verter_commands` соответствует текущим дефолтам (`default_distance/default_pwm/turn_distance`) из `recognition_node`.

## Кейс 4: Стоп-слова в диалоге

```
👤 "спасибо"
recognition_node → завершает диалог/сбрасывает состояние
```

## Кейс 5: Внешний мьют

### 5.1 Мьют от TTS (`tts_control`)

```
tts_control=false → STT выключается
tts_control=true  → STT включается
```

### 5.2 Мьют от recognition (`speech_control`)

```
recognition_node публикует speech_control=false (например, на время AI)
recognition_node публикует speech_control=true  (после завершения)
```


