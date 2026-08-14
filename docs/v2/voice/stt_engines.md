# STT-движки

Распознавание речи реализовано в пакете `speech_to_text/`. Доступно **3 реализованных движка** на базе sherpa-onnx, общий VAD — Silero (`silero_vad.onnx`). Движки переключаются выбором ноды в `main.launch.py`. Parakeet TDT заявлен в `setup.py` (entry point + дата-файлы модели) и закомментирован в `main.launch.py`, но файл `speech_to_text_parakeet_node.py` в репозитории отсутствует — движок не реализован.

!!! note "Статус кода: без правок"
    Движки задокументированы как есть. Дублирование кода между реализациями (поиск устройства, VAD, буферизация) известно и не устраняется в текущем раунде работ — см. [refactor_notes.md](refactor_notes.md). Parakeet TDT присутствует только как entry point в `setup.py` и закомментированный блок в `main.launch.py`; исполняемого `.py`-файла в репозитории нет.

## Реализации

| Файл | Модель | Архитектура | Активен |
|------|--------|-------------|---------|
| `speech_to_text_node.py` | NeMo GigaAM v2 | CTC | ✅ по умолчанию |
| `speech_to_text_sherpa_node.py` | NeMo GigaAM v2 | Transducer (sherpa) | закомментирован |
| `speech_to_text_transducer_node.py` | NeMo GigaAM v2 | Transducer (ручной) | закомментирован |
| `speech_to_text_parakeet_node.py` | NeMo Parakeet TDT | Transducer | не реализован (файл отсутствует) |

## Логика распознавания

1. **Захват аудио** — блоками (512 или 1600 семплов, 16кГц) с ReSpeaker.
2. **VAD** — Silero определяет вероятность речи; порог `VAD_THRESHOLD`.
3. **Начало речи** — при `vad_prob > threshold` pre-buffer переносится в audio-buffer (предзахват ~0.5с).
4. **Конец фразы** — накопление тишины; при `silence_counter >= SILENCE_CHUNKS` буфер распознаётся.
5. **Фильтр** — текст < 2 символов отбрасывается (реализован в CTC- и sherpa-ноде; transducer-нода публикует любой непустой текст).
6. **Публикация** — `recognized_text`.

## Активация/деактивация

STT управляется топиком `speech_control` (Bool) и `tts_control` (Bool). При `False` распознавание останавливается (микрофон не отключается аппаратно, прекращается публикация результатов).

## Поиск аудиоустройства

Жёстко ищется устройство с именем `ReSpeaker` или `ArrayUAC10`; fallback — устройство 1, затем перебор. Логика дублирована во всех 3 реализованных файлах.

## Подробности параметров

Полная таблица порогов/размеров блоков — в [timings.md](timings.md). История решения проблем STT — в `v2/problems_solved/speech_to_text.md`.
