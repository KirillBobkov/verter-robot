# Verter Admin — код (простыми словами)

## TL;DR

Система устроена как конвейер из ROS2 нод:

1. **STT** (`speech_to_text_*_node`) слушает микрофон и публикует текст в `recognized_text`.
2. **Router/State machine** (`recognition_node`) принимает `recognized_text`, держит состояния/таймеры и решает:
   - это команда движения → публикует `verter_commands` (уедет шасси)
   - это вопрос → публикует `ai_question` (уйдет в AI)
   - это “стоп-слово” → завершает диалог
3. **AI** (`ai_assistant_node`) берет `ai_question` и (в проде) отвечает через YandexGPT + dataset, публикует `text_to_speech`.
4. **TTS** (`text_to_speech_node` или `silero_tts_node`) озвучивает `text_to_speech`, и пока говорит — шлёт `tts_control=false`, чтобы STT/recognition замолчали.
5. **Звуки** (`sound_player_node`) играет короткие звуки по топику `play`.

## Карта топиков

- `recognized_text` (String): STT → Recognition
- `speech_control` (Bool): Recognition → STT (вкл/выкл слушание)
- `tts_control` (Bool): TTS → STT + Recognition (вкл/выкл на время озвучки)
- `ai_question` (String): Recognition → AI
- `text_to_speech` (String): AI/Recognition → TTS
- `play` (String): Recognition/AI → SoundPlayer
- `verter_commands` (String): Recognition → Chassis (Arduino)
- `/cmd_vel` (Twist): Nav2 → Chassis
- `head_commands` (String): внешние команды → DOA node (Arduino головы)
- `doa_active` (Bool): включение/выключение DOA

## Дальше

- Документация по нодам и интерфейсам — в меню слева (разделы **Ноды** и **API**).


