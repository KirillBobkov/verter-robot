# План исправления проблемы с автозапуском аудио и распознавания речи

## Анализ проблемы

### Симптомы
При автозапуске через systemd:
1. **Нет звука** - aplay выдает ошибки `underrun!!!`
2. **Не распознается речь** - speech_to_text_node не захватывает аудио
3. При ручном запуске всё работает корректно

### Корневые причины

#### 1. Audio Underrun (буферные переполнения)
**Проблема:** Команда aplay использует слишком маленькие размеры буфера:
- `--buffer-size=256` (вместо рекомендуемых 4096+)
- `--period-size=64` (вместо рекомендуемых 512+)

**Почему это работает вручную:** При ручном запуске система уже загружена, CPU свободен, и маленький буфер справляется.

**Почему не работает при автозапуске:** При загрузке системы CPU занят другими процессами, маленький буфер не успевает наполняться данными.

#### 2. Низкая задержка PulseAudio
**Проблема:** `PULSE_LATENCY_MSEC=30` слишком агрессивная настройка для автозапуска.

**Почему это работает вручную:** Система уже инициализирована, аудио подсистема готова.

**Почему не работает при автозапуске:** PulseAudio может быть не полностью готов к работе с такой низкой задержкой.

#### 3. USB аудио устройства не готовы
**Проблема:** ReSpeaker USB устройство может быть не полностью инициализировано при старте узлов.

**Почему это работает вручную:** При ручном запуске устройство уже обнаружено и настроено.

**Почему не работает при автозапуске:** USB устройства инициализируются асинхронно, узлы могут запуститься раньше, чем устройство готово.

#### 4. Отсутствие GPU ускорения
**Проблема:** onnxruntime не может найти GPU устройство при автозапуске.

**Лог:** `GPU device discovery failed: device_discovery.cc:89 ReadFileContents Failed to open file: "/sys/class/drm/card1/device/vendor"`

**Почему это работает вручную:** При ручном запуске драйверы GPU уже загружены.

**Почему не работает при автозапуске:** GPU драйверы могут быть не полностью инициализированы.

## Решение

### Шаг 1: Увеличить размеры буфера aplay

#### Файл: `verter_admin/src/verter_admin/text_to_speech/text_to_speech_node.py`
**Изменение:**
```python
# Было:
cmd = ["aplay", "-D", self.audio_device, "-q", "-f", "S16_LE", "-r", sample_rate, "-c", "1", "--buffer-size=256", "--period-size=64"]

# Стало:
cmd = ["aplay", "-D", self.audio_device, "-q", "-f", "S16_LE", "-r", sample_rate, "-c", "1", "--buffer-size=4096", "--period-size=512"]
```

#### Файл: `verter_admin/src/verter_admin/sound_player/sound_player_node.py`
**Изменение:**
```python
# Для WAV файлов (строка 112-114):
# Было:
cmd = ['aplay', '-D', self.audio_device, '-q',
       '-f', 'S16_LE', '-r', '44100', '-c', '2',
       '--buffer-size=1024', '--period-size=256', sound_path]

# Стало:
cmd = ['aplay', '-D', self.audio_device, '-q',
       '-f', 'S16_LE', '-r', '44100', '-c', '2',
       '--buffer-size=4096', '--period-size=512', sound_path]

# Для MP3/OGG файлов (строка 136):
# Было:
cmd = ['aplay', '-D', self.audio_device, '-q', '-f', 'S16_LE', '-r', '22050', '-c', '1', '--buffer-size=2048', '--period-size=512']

# Стало:
cmd = ['aplay', '-D', self.audio_device, '-q', '-f', 'S16_LE', '-r', '22050', '-c', '1', '--buffer-size=4096', '--period-size=512']
```

### Шаг 2: Увеличить задержку PulseAudio

#### Файл: `verter_admin/src/verter_admin/text_to_speech/text_to_speech_node.py`
**Добавить в метод `_setup_environment`:**
```python
# Увеличиваем задержку для стабильности при автозапуске
self.env["PULSE_LATENCY_MSEC"] = "100"  # Было 30
```

#### Файл: `verter_admin/src/verter_admin/sound_player/sound_player_node.py`
**Изменить (строка 65):**
```python
# Было:
self.env["PULSE_LATENCY_MSEC"] = "30"

# Стало:
self.env["PULSE_LATENCY_MSEC"] = "100"
```

### Шаг 3: Добавить проверку готовности аудио устройств

#### Файл: `verter_admin/services/start_verter_admin.sh`
**Добавить после проверки PulseAudio:**
```bash
echo "Проверка USB аудио устройств..."
MAX_USB_WAIT=20
for ((i=1; i<=MAX_USB_WAIT; i++)); do
    if lsusb | grep -q "Generic.*USB.*Audio\|ReSpeaker\|ArrayUAC10"; then
        echo "USB аудио устройство обнаружено."
        break
    fi
    echo "Ждем USB аудио... $i/$MAX_USB_WAIT"
    sleep 1
done

# Дополнительная задержка для полной инициализации устройства
echo "Ожидание инициализации аудио устройств..."
sleep 2
```

### Шаг 4: Улучшить systemd сервис

#### Файл: `verter_admin/services/verter-admin.service`
**Изменить зависимости:**
```ini
[Unit]
Description=Verter Admin (User Session)
After=graphical-session.target sound.target network.target pulseaudio.service
Wants=graphical-session.target sound.target pulseaudio.service
```

**Добавить переменные окружения:**
```ini
[Service]
Type=simple
WorkingDirectory=%h/verter-robot
Environment="XDG_RUNTIME_DIR=/run/user/%U"
Environment="PULSE_LATENCY_MSEC=100"
Environment="PULSE_SERVER=unix:/run/user/%U/pulse/native"
ExecStart=/home/jetson/verter-robot/verter_admin/services/start_verter_admin.sh
```

### Шаг 5: Добавить диагностическое логирование

#### Файл: `verter_admin/services/start_verter_admin.sh`
**Добавить после ожидания аудио:**
```bash
echo "=== Диагностика аудио системы ==="
echo "PulseAudio sinks:"
pactl list sinks short 2>/dev/null || echo "Нет sinks"
echo ""
echo "PulseAudio sources:"
pactl list sources short 2>/dev/null || echo "Нет sources"
echo ""
echo "USB аудио устройства:"
lsusb | grep -i "audio\|repeaker\|arrayuac10" || echo "Не найдены"
echo ""
echo "Аудио устройства (sounddevice):"
python3 -c "import sounddevice as sd; print(sd.query_devices())" 2>/dev/null || echo "sounddevice недоступен"
echo "==================================="
```

### Шаг 6: Обработка GPU ускорения

#### Файл: `verter_admin/src/verter_admin/speech_to_text/speech_to_text_node.py`
**Улучшить логирование при ошибке GPU:**
```python
# В методе _load_models, добавить try-except для CUDA
try:
    if self.config.USE_CUDA:
        # Попытка использовать CUDA
        providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
    else:
        providers = ['CPUExecutionProvider']
except Exception as e:
    self.get_logger().warning(f"GPU недоступен, используем CPU: {e}")
    providers = ['CPUExecutionProvider']
```

## Диагностика

После применения исправлений:

1. **Пересобрать пакет:**
   ```bash
   cd /home/jetson/verter-robot/verter_admin
   colcon build --packages-select verter_admin
   source install/setup.bash
   ```

2. **Перезапустить сервис:**
   ```bash
   systemctl --user restart verter-admin.service
   ```

3. **Проверить логи:**
   ```bash
   journalctl --user -u verter-admin.service -f
   tail -f /home/jetson/verter-robot/verter_admin/journal.log
   ```

4. **Проверить аудио:**
   ```bash
   pactl list sinks short
   pactl list sources short
   lsusb | grep -i audio
   ```

## Ожидаемые результаты

- ✅ Нет ошибок `underrun!!!` в логах
- ✅ Звук воспроизводится корректно при автозапуске
- ✅ Речь распознается при автозапуске
- ✅ DOA узел корректно определяет направление звука
- ⚠️ GPU ускорение может быть недоступно при автозапуске (это нормально, CPU работает)

## Дополнительные рекомендации

1. **Мониторинг:** Добавить регулярную проверку аудио системы в crontab
2. **Автоматическое восстановление:** Скрипт для перезапуска аудио подсистемы при проблемах
3. **Логирование:** Сохранять диагностику аудио в отдельный файл
4. **Тестирование:** Создать тестовый скрипт для проверки аудио после автозапуска
