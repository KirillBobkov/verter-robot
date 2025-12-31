# `speech_to_text_*` (STT)

## Задача

Эти ноды **слушают микрофон** и публикуют итоговый текст в топик `recognized_text`.

## Варианты

В `verter_admin/setup.py` зарегистрированы несколько реализаций:

- `speech_to_text_node` — CTC (onnxruntime)
- `speech_to_text_transducer_node` — Transducer (onnxruntime)
- `speech_to_text_sherpa_node` — sherpa-onnx
- `speech_to_text_parakeet_node` — NeMo/Parakeet пайплайн

Все они в итоге делают одно и то же: `recognized_text` (String).

## Входы / выходы

- **Pub**: `recognized_text` (std_msgs/String)
- **Sub**: `speech_control` (std_msgs/Bool) — включить/выключить распознавание
- **Sub**: `tts_control` (std_msgs/Bool) — то же самое, но от TTS (пока TTS говорит)

## Простыми словами как оно работает

- Нода постоянно читает аудио чанки.
- VAD/логика сегментации решает, когда “фраза закончилась”.
- После этого выполняется инференс модели → получается строка.
- Если нода активна (`speech_control=true`) — публикует строку в `recognized_text`.

## Подводные камни

- Это аудио + ML: если “пошёл мусор”, `recognition_node` старается фильтровать по длине/контексту, но лучше решать на уровне STT (VAD + фильтры).
- `tts_control` должен реально “глушить” STT, иначе STT будет распознавать собственную озвучку.


