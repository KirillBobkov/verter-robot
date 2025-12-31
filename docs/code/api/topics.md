# ROS2 интерфейсы (топики)

Это “контракт” между нодами. Полное описание по коду — в [кодовой документации](../verter_admin.md).

## Основные топики

- `recognized_text` (std_msgs/String): STT → `recognition_node`
- `speech_control` (std_msgs/Bool): `recognition_node` → STT (вкл/выкл распознавание)
- `tts_control` (std_msgs/Bool): TTS → STT + `recognition_node` (пауза на время озвучки)
- `ai_question` (std_msgs/String): `recognition_node` → `ai_assistant_node`
- `text_to_speech` (std_msgs/String): AI/Recognition → TTS
- `play` (std_msgs/String): команды звуков для `sound_player_node`
- `verter_commands` (std_msgs/String): `recognition_node` → `chassis_node` (и Arduino)
- `/cmd_vel` (geometry_msgs/Twist): Nav2/teleop → `chassis_node` (и `odometry_node`)


