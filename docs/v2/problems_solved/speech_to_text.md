# Speech-to-Text

## `input overflow` после каждого распознавания

**Симптом:** `[WARN] Audio status: input overflow` после каждого успешного распознавания фразы.

**Причина:** Распознавание выполнялось внутри audio callback-потока, блокируя его на 1–2 секунды. За это время буфер заполнялся и данные терялись.

### Код «до»

```python
# Внутри audio callback — блокирует callback!
self._recognize_accumulated_audio()
```

### Код «после»

```python
# Не блокирует callback — запускает отдельный поток
threading.Thread(target=self._recognize_accumulated_audio, daemon=True).start()
```

### Что это даёт

- Нет `input overflow`
- Быстрее отклик — распознавание начинается мгновенно
- Меньше CPU — audio callback тратит минимум времени
- Стабильная работа — нет потери аудио данных

### Архитектура

```
Audio Callback  (каждые 100 мс, никогда не блокируется)
├── VAD проверка
├── Накопление в буфер
└── Запуск Thread при паузе

Recognition Thread  (отдельный поток)
├── Extract features (fbank)
├── ONNX inference
├── Transducer decode
└── Publish result
```

`recognition_lock` защищает ONNX модель от одновременного доступа.

---

## Дополнительные оптимизации

### Параметры

- `SILENCE_CHUNKS: 6 → 4` (триггер 400 мс вместо 600 мс)
- `NUM_THREADS: 4` (все ядра Cortex-A72)
- ONNX graph optimization (`ORT_ENABLE_ALL` + memory arena)
- Timing-логи для диагностики задержек

### Результаты

**До оптимизации:**
- `input overflow` после каждого распознавания
- Задержка 1.5–2 с от конца речи до результата
- Audio callback блокируется на время распознавания

**После оптимизации:**
- Нет `input overflow`
- Распознавание начинается сразу (в фоне)
- Audio callback работает непрерывно

### Ожидаемая общая задержка

| Этап | Время |
|------|-------|
| Ожидание паузы | ~400 мс |
| Feature extraction | ~150–300 мс |
| ONNX inference | ~200–400 мс |
| **Итого** | **~750–1100 мс** |

### Timing-логи

```
🎤 Распознано: 'привет'
⏱️  Timing: feature=250ms, inference=380ms, total=630ms
```

- **feature** — извлечение fbank признаков (зависит от длины аудио)
- **inference** — ONNX inference + CTC decode (зависит от модели и CPU)
- **total** — общее время распознавания

**Диагностика:**
- `feature > 400ms` → аудио слишком длинное или CPU перегружен
- `inference > 600ms` → попробовать `NUM_THREADS = 2` или уменьшить до 1
- `total > 1000ms` → проверить нагрузку на CPU от других процессов

### Дополнительные настройки

| Проблема | Настройка |
|----------|-----------|
| Пропускает начало слов | `PRE_BUFFER_CHUNKS: 7`, `VAD_THRESHOLD: 0.4` |
| Слишком долго ждёт конца фразы | `SILENCE_CHUNKS: 3` (300 мс, может резать фразы) |
| Режет фразы на середине | `SILENCE_CHUNKS: 5` (500 мс) |
| CPU высокий | `NUM_THREADS: 2` (может быть быстрее на некоторых ARM) |
| Inference медленный (>600ms) | `NUM_THREADS = 2`, закрыть другие процессы, проверить CPU governor |

### Проверка работы

```bash
ros2 launch verter_admin main.launch.py
```
Говорить и смотреть в логи:
1. Не должно быть `input overflow`
2. Должны быть timing-логи с разумными значениями
