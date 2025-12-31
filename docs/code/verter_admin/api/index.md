# API / интерфейсы

Это не “HTTP API”. Это интерфейсы уровня ROS2: **топики**, форматы сообщений и ожидаемая семантика.

## Топики (основные)

### `recognized_text` (std_msgs/String)
- **Кто публикует**: `speech_to_text_*_node`
- **Кто читает**: `recognition_node`
- **Что внутри**: распознанная фраза как строка (сырой текст)

### `speech_control` (std_msgs/Bool)
- **Кто публикует**: `recognition_node`
- **Кто читает**: `speech_to_text_*_node`
- **Смысл**: `true` = STT слушает/публикует, `false` = STT “глушится” (обычно на время TTS/паузы)

### `tts_control` (std_msgs/Bool)
- **Кто публикует**: `text_to_speech_node` (и потенциально другие TTS)
- **Кто читает**: `speech_to_text_*_node`, `recognition_node`
- **Смысл**: управление “мьютом” системы на время озвучки

### `ai_question` (std_msgs/String)
- **Кто публикует**: `recognition_node`
- **Кто читает**: `ai_assistant_node`
- **Смысл**: вопрос/команда для AI (уже очищенный текст без триггера)

### `text_to_speech` (std_msgs/String)
- **Кто публикует**: `ai_assistant_node`, `recognition_node`
- **Кто читает**: `text_to_speech_node`
- **Смысл**: финальная фраза, которую нужно озвучить

### `play` (std_msgs/String)
- **Кто публикует**: разные ноды (обычно `recognition_node`)
- **Кто читает**: `sound_player_node`
- **Смысл**: имя файла или команда `stop_sound`

### `verter_commands` (std_msgs/String)
- **Кто публикует**: `recognition_node` (и другие управляющие компоненты)
- **Кто читает**: `chassis_node` (и Arduino-часть через serial)
- **Смысл**: команды формата `CHASSIS:*` (см. `recognition_node` и Arduino sketch)


