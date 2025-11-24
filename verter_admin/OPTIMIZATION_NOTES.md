# Оптимизация speech_to_text_node для RPi4

## Проблема: `input overflow`

**Симптом:** После каждого распознавания появляется `[WARN] Аудио статус: input overflow`

**Причина:** Распознавание (`_recognize_accumulated_audio`) выполнялось **внутри audio callback потока**, блокируя его на ~1-2 секунды. За это время входящие аудио данные накапливались и переполняли буфер.

## Решение: Асинхронное распознавание ⭐

**Раньше:**
```python
if self.silence_counter >= self.config.SILENCE_CHUNKS:
    self._recognize_accumulated_audio()  # Блокирует callback!
```

**Сейчас:**
```python
if self.silence_counter >= self.config.SILENCE_CHUNKS:
    audio_to_recognize = list(self.audio_buffer)
    threading.Thread(
        target=self._recognize_accumulated_audio,
        args=(audio_to_recognize,),
        daemon=True
    ).start()  # НЕ блокирует callback!
```

### Что это даёт:
- ✅ **Нет `input overflow`** - audio callback никогда не блокируется
- ✅ **Быстрее отклик** - распознавание начинается мгновенно
- ✅ **Меньше CPU** - audio callback тратит минимум времени
- ✅ **Стабильная работа** - нет потери аудио данных

## Результаты

До оптимизации:
- `input overflow` после каждого распознавания
- Задержка 1.5-2s от конца речи до результата
- Audio callback блокируется на время распознавания

После оптимизации v1 (асинхронность):
- ✅ Нет `input overflow`
- ✅ Распознавание начинается сразу (в фоне)
- ✅ Audio callback работает непрерывно

После оптимизации v2 (скорость):
- ✅ **SILENCE_CHUNKS: 6 → 4** (400ms триггер вместо 600ms)
- ✅ **NUM_THREADS: 2 → 1** (лучше для ARM, меньше context switching)
- ✅ **ONNX graph optimization** (ORT_ENABLE_ALL + memory arena)
- ✅ **Timing logs** - видно где тормозит (feature extraction vs inference)

### Ожидаемая общая задержка:
- Ожидание паузы: **400ms** (вместо 600ms)
- Feature extraction: **~150-300ms** (зависит от длины фразы)
- ONNX inference: **~200-400ms** (с 1 потоком на ARM)
- **Итого: ~750-1100ms** от конца речи до результата

## Как проверить

```bash
# Запустить систему
ros2 launch verter_admin admin.launch.py

# Говорить и смотреть в логи
# 1. НЕ должно быть "input overflow"
# 2. Должны быть timing logs после каждого распознавания
```

### Интерпретация timing logs:

```
🎤 Распознано: 'привет'
⏱️  Timing: feature=250ms, inference=380ms, total=630ms
```

- **feature** - извлечение fbank признаков (зависит от длины аудио)
- **inference** - ONNX inference + CTC decode (зависит от модели и CPU)
- **total** - общее время распознавания

**Что оптимизировать если медленно:**
- `feature > 400ms` → аудио слишком длинное или CPU перегружен
- `inference > 600ms` → попробовать уменьшить NUM_THREADS до 1 или наоборот увеличить до 2
- `total > 1000ms` → проверить нагрузку на CPU от других процессов

## Дополнительные настройки (если нужно)

### Если пропускает начало слов:
```python
PRE_BUFFER_CHUNKS: int = 7  # Больше предыстории (700ms)
VAD_THRESHOLD: float = 0.4  # Ниже порог
```

### Если слишком долго ждёт конца фразы:
```python
SILENCE_CHUNKS: int = 3  # Ещё меньше паузы (300ms) - но может резать фразы!
```

### Если режет фразы на середине (из-за SILENCE_CHUNKS=4):
```python
SILENCE_CHUNKS: int = 5  # Чуть больше паузы (500ms)
```

### Если CPU всё равно высокий:
```python
NUM_THREADS: int = 2  # Попробуй 2 потока (может быть быстрее на некоторых ARM)
```

### Если inference медленный (>600ms в логах):
1. Попробуй `NUM_THREADS = 2`
2. Закрой другие процессы
3. Проверь CPU governor (должен быть ondemand или performance)

## Архитектура

```
┌─────────────────────┐
│  Audio Callback     │ <- Никогда не блокируется!
│  (каждые 100ms)     │
└──────────┬──────────┘
           │
           ├─> VAD проверка (быстро, <5ms)
           │
           ├─> Накопление в буфер (быстро)
           │
           └─> Если пауза → запуск Thread
                              │
                    ┌─────────▼──────────┐
                    │ Recognition Thread │
                    │  (работает в фоне) │
                    │  1. Extract features│
                    │  2. ONNX inference │
                    │  3. CTC decode     │
                    │  4. Publish result │
                    └────────────────────┘
```

## Важно

- Распознавание теперь **не блокирует** захват аудио
- Каждая фраза обрабатывается в **отдельном потоке**
- `recognition_lock` защищает ONNX модель от одновременного доступа
- Audio callback остается **легким и быстрым**

