# `recognition_node`

## Задача

`recognition_node` — это **роутер распознанного текста**: принимает `recognized_text` и решает, что делать дальше (шасси / AI / звуки / управление диалогом).

## Входы / выходы

- **Sub**: `recognized_text` (String) — вход от STT
- **Sub**: `tts_control` (Bool) — пауза распознавания на время озвучки

- **Pub**: `ai_question` (String) — вопрос/команда для AI
- **Pub**: `text_to_speech` (String) — прямой текст на озвучку (например “пока”)
- **Pub**: `play` (String) — короткие звуки (`trigger.wav`, `success.wav`, …)
- **Pub**: `dialog_control` (String) — управление диалогом для AI
- **Pub**: `verter_commands` (String) — команды движения/стопа в строковом протоколе
- **Pub**: `speech_control` (Bool) — включает/выключает STT, чтобы не копить буфер во время AI/TTS

## Простыми словами: state machine

Есть несколько состояний:

- **LISTENING_FOR_TRIGGER**: ждём триггерное слово (“робот/вертер/…”) в начале фразы
- **CAPTURING_COMMAND**: триггер был, ждём следующую фразу-команду
- **DIALOG_MODE**: активный диалог (последующие фразы идут в AI или как команды)
- **PAUSED**: пауза, пока TTS говорит (входы игнорируются)

## Приоритеты обработки команд

Когда нода “активно слушает” (CAPTURING_COMMAND / DIALOG_MODE):

1. **stop-слова** (“пока”, “спасибо”, …) → завершение диалога/сброс
2. **команды шасси** (“вперед/назад/влево/вправо/стоп”) → `verter_commands`
3. **всё остальное** → `ai_question`

## Формат команд шасси (главное)

Команды — это строки вида:

- `CHASSIS:STOP`
- `CHASSIS:LEFT:ASK:<distance>:<pwm>;CHASSIS:RIGHT:ASK:<distance>:<pwm>`

Их генерирует `CommandProcessor` внутри `recognition_node`, а исполняет `chassis_node` + Arduino.


