# `text_to_speech_node` (Piper)

## Задача

Озвучивает строку из `text_to_speech`.

## Входы / выходы

- **Sub**: `text_to_speech` (String)
- **Pub**: `tts_control` (Bool) — чтобы “глушить” STT/recognition на время озвучки

## Простыми словами как работает

- Берёт текст → синтезирует в аудио через Piper модель `ru_RU-ruslan-medium.onnx`.
- Запускает `aplay` и стримит туда PCM чанки.
- Перед воспроизведением шлёт сигнал “выключить распознавание”, после окончания — включает обратно.

## Параметры

- `audio_device` (string, default `pulse`): устройство для `aplay -D ...`


