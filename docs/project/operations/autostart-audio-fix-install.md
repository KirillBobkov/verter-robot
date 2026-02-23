# Установка и тестирование исправлений автозапуска аудио

## Внесённые изменения

### 1. Увеличены размеры буфера aplay для предотвращения underrun

#### Файл: `verter_admin/src/verter_admin/text_to_speech/text_to_speech_node.py`
```python
# Было:
cmd = ["aplay", "-D", self.audio_device, "-q", "-f", "S16_LE", "-r", sample_rate, "-c", "1", "--buffer-size=256", "--period-size=64"]

# Стало:
cmd = ["aplay", "-D", self.audio_device, "-q", "-f", "S16_LE", "-r", sample_rate, "-c", "1", "--buffer-size=4096", "--period-size=512"]
```

#### Файл: `verter_admin/src/verter_admin/sound_player/sound_player_node.py`
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

### 2. Увеличена задержка PulseAudio для стабильности при автозапуске

#### Файл: `verter_admin/src/verter_admin/text_to_speech/text_to_speech_node.py`
```python
# Добавлено:
self.env["PULSE_LATENCY_MSEC"] = "100"  # Было 30
```

#### Файл: `verter_admin/src/verter_admin/sound_player/sound_player_node.py`
```python
# Было:
self.env["PULSE_LATENCY_MSEC"] = "30"

# Стало:
self.env["PULSE_LATENCY_MSEC"] = "100"
```

### 3. Добавлена проверка USB аудио устройств

#### Файл: `verter_admin/services/start_verter_admin.sh`
```bash
# Добавлено после ожидания PulseAudio:
echo "Проверка USB аудио устройств..."
MAX_USB_WAIT=20
for ((i=1; i<=MAX_USB_WAIT; i++)); do
    if lsusb | grep -qi "Generic.*USB.*Audio\|ReSpeaker\|ArrayUAC10"; then
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

### 4. Добавлено диагностическое логирование

#### Файл: `verter_admin/services/start_verter_admin.sh`
```bash
# Добавлено перед запуском ROS2:
echo "=== Диагностика аудио системы ==="
echo "PulseAudio sinks:"
pactl list sinks short 2>/dev/null || echo "Нет sinks"
echo ""
echo "PulseAudio sources:"
pactl list sources short 2>/dev/null || echo "Нет sources"
echo ""
echo "USB аудио устройства:"
lsusb | grep -i "audio\|repeaker\|arrayuac10" || echo "Не найдены"
echo "==================================="
```

### 5. Улучшен systemd сервис

#### Файл: `verter_admin/services/verter-admin.service`
```ini
# Было:
After=graphical-session.target sound.target network.target
Wants=graphical-session.target sound.target

# Стало:
After=graphical-session.target sound.target network.target pulseaudio.service
Wants=graphical-session.target sound.target pulseaudio.service
```

```ini
# Добавлено в секцию [Service]:
Environment="XDG_RUNTIME_DIR=/run/user/%U"
Environment="PULSE_LATENCY_MSEC=100"
Environment="PULSE_SERVER=unix:/run/user/%U/pulse/native"
```

## Установка исправлений

### Шаг 1: Пересобрать пакет ROS2

```bash
cd /home/jetson/verter-robot/verter_admin
colcon build --packages-select verter_admin
source install/setup.bash
```

### Шаг 2: Обновить systemd сервис

```bash
# Скопировать обновленный файл сервиса
cp /home/jetson/verter-robot/verter_admin/services/verter-admin.service ~/.config/systemd/user/

# Перезагрузить конфигурацию systemd
systemctl --user daemon-reload

# Включить сервис
systemctl --user enable verter-admin.service

# Перезапустить сервис
systemctl --user restart verter-admin.service
```

### Шаг 3: Проверить статус сервиса

```bash
# Проверить статус
systemctl --user status verter-admin.service

# Посмотреть логи в реальном времени
journalctl --user -u verter-admin.service -f

# Посмотреть логи файла
tail -f /home/jetson/verter-robot/verter_admin/journal.log
```

## Тестирование

### Тест 1: Проверка автозапуска

```bash
# Перезагрузить систему
sudo reboot

# После перезагрузки проверить статус сервиса
systemctl --user status verter-admin.service
```

**Ожидаемый результат:**
- ✅ Сервис запущен успешно
- ✅ Нет ошибок `underrun!!!` в логах
- ✅ Звук воспроизводится корректно
- ✅ Речь распознаётся

### Тест 2: Проверка аудио устройств

```bash
# Проверить PulseAudio sinks
pactl list sinks short

# Проверить PulseAudio sources
pactl list sources short

# Проверить USB аудио устройства
lsusb | grep -i "audio\|repeaker\|arrayuac10"
```

**Ожидаемый результат:**
- ✅ PulseAudio sinks доступны
- ✅ PulseAudio sources доступны
- ✅ USB аудио устройство обнаружено

### Тест 3: Проверка звукового вывода

```bash
# Тест воспроизведения звука
aplay -D pulse /usr/share/sounds/alsa/Front_Center.wav

# Или через ROS2
ros2 topic pub /play std_msgs/String "data: 'hello'"
```

**Ожидаемый результат:**
- ✅ Звук воспроизводится без прерываний
- ✅ Нет ошибок `underrun!!!`

### Тест 4: Проверка распознавания речи

```bash
# Проверить логи speech_to_text_node
grep -i "speech_to_text_node" /home/jetson/verter-robot/verter_admin/journal.log | tail -20

# Проверить распознанный текст
ros2 topic echo /recognized_text --once
```

**Ожидаемый результат:**
- ✅ SpeechToTextNode запущен
- ✅ Распознавание активно
- ✅ Текст распознаётся при голосовой команде

## Диагностика проблем

### Если звук не воспроизводится

```bash
# Проверить PulseAudio
pulseaudio --check -v
pulseaudio -k  # Перезапустить PulseAudio

# Проверить аудио устройства
pactl info
pactl list sinks
pactl list sources

# Проверить права доступа
ls -la /run/user/1000/pulse/
```

### Если распознавание не работает

```bash
# Проверить микрофон
arecord -D pulse -f S16_LE -r 16000 -c 1 -d 5 /tmp/test.wav
aplay /tmp/test.wav

# Проверить sounddevice
python3 -c "import sounddevice as sd; print(sd.query_devices())"

# Проверить ROS2 топики
ros2 topic list | grep -i speech
ros2 topic echo /recognized_text
```

### Если сервис не запускается

```bash
# Проверить логи systemd
journalctl --user -u verter-admin.service -n 50

# Проверить ошибки
journalctl --user -u verter-admin.service -p err

# Проверить журнал
cat /home/jetson/verter-robot/verter_admin/journal-error.log
```

## Возврат к предыдущим настройкам

Если исправления не помогли, можно вернуть предыдущие настройки:

### Возврат размеров буфера aplay

```python
# В text_to_speech_node.py:
cmd = ["aplay", "-D", self.audio_device, "-q", "-f", "S16_LE", "-r", sample_rate, "-c", "1", "--buffer-size=256", "--period-size=64"]

# В sound_player_node.py для WAV:
cmd = ['aplay', '-D', self.audio_device, '-q',
       '-f', 'S16_LE', '-r', '44100', '-c', '2',
       '--buffer-size=1024', '--period-size=256', sound_path]

# В sound_player_node.py для MP3/OGG:
cmd = ['aplay', '-D', self.audio_device, '-q', '-f', 'S16_LE', '-r', '22050', '-c', '1', '--buffer-size=2048', '--period-size=512']
```

### Возврат задержки PulseAudio

```python
# В text_to_speech_node.py и sound_player_node.py:
self.env["PULSE_LATENCY_MSEC"] = "30"
```

После внесения изменений нужно пересобрать пакет и перезапустить сервис.

## Дополнительная информация

Подробный план исправлений находится в `plans/autostart-audio-fix.md`.
