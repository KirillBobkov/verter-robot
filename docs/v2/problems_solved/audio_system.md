# Аудио система

## 1. Audio underrun при автозапуске через systemd

**Симптом:** Ошибки `underrun!!!` в логах `aplay` при автозапуске через systemd. При ручном запуске из терминала — всё работает.

**Почему работает вручную:** Система загружена, CPU свободен, драйверы инициализированы.

**Почему не работает при автозапуске:** CPU занят другими процессами, буфер успевает опустеть до поступления данных.

**Решение:** Увеличены размеры буфера в `text_to_speech_node.py` и `sound_player_node.py`:
- `--buffer-size=256` → `--buffer-size=4096`
- `--period-size=64` → `--period-size=512`

---

## 2. Низкая задержка PulseAudio

**Симптом:** Звук не воспроизводится при автозапуске, хотя команды `aplay` отправляются.

**Причина:** Переменная `PULSE_LATENCY_MSEC` установлена в 30 мс — слишком мало для стабильной работы при автозапуске.

**Решение:** `PULSE_LATENCY_MSEC` увеличена с 30 до 100 в обоих аудио-узлах.

---

## 3. USB-аудио устройства не готовы при старте сервиса

**Симптом:** Нет звука при автозапуске через systemd.

**Причина:** USB-аудиоустройство (ReSpeaker) ещё не инициализировано к моменту старта ROS2-узлов. USB-подключение асинхронно.

**Решение:** В `start_verter_admin.sh` добавлена проверка наличия USB-аудио устройств с ожиданием до 20 секунд перед запуском ROS2-узлов.

### Улучшение systemd-сервиса

Сервис `verter-admin.service` не имел зависимостей от `pulseaudio.service` и не имел нужных переменных окружения.

**Добавлено в `[Unit]`:**
```
After=... pulseaudio.service
Wants=... pulseaudio.service
```

**Добавлено в `[Service]`:**
```
Environment="XDG_RUNTIME_DIR=/run/user/%U"
Environment="PULSE_LATENCY_MSEC=100"
Environment="PULSE_SERVER=unix:/run/user/%U/pulse/native"
```

### Диагностическое логирование

В `start_verter_admin.sh` добавлен блок вывода состояния аудио:
```bash
pactl list sinks short
pactl list sources short
lsusb | grep -i "audio\|repeater\|arrayuac10"
```

---

## 4. Неправильный default sink PulseAudio

**Симптом:** Звук не воспроизводится ни при автозапуске, ни при ручном запуске. `aplay` завершается успешно без ошибок.

**Причина:** PulseAudio использует ReSpeaker (16 kHz) как default sink, но `aplay` пытается воспроизвести 44.1 kHz или 22.05 kHz. Это проблема ресемплинга PulseAudio — ReSpeaker sink поддерживает только 16 кГц. Также возможно, что Generic USB Audio (sink #1) или встроенная аудио (sink #2) физически не подключены к колонкам.

### Решения

**Вариант 1 — указать sink в коде:**
```python
self.audio_device = "pulse:1"  # или "pulse:2"
```

**Вариант 2 — ALSA напрямую (без PulseAudio):**
```python
self.audio_device = "hw:1,0"   # Generic USB Audio
# или
self.audio_device = "hw:2,0"   # встроенная аудио
```

**Вариант 3 — установить permanent default sink:**
Файл `~/.config/pulse/default.pa`:
```
set-default-sink alsa_output.usb-Generic_USB_Audio_20210726905926-00.analog-stereo
```

**Вариант 4 — `pactl set-default-sink`** для текущего сеанса.

### Диагностика sink'ов

1. `pactl info | grep "Default Sink"` — проверить default sink
2. `pactl list sinks short` — состояние всех sink'ов
3. `aplay -D pulse:0 test.wav` — протестировать sink #0, затем `pulse:1`, `pulse:2`
4. `pactl list sinks | grep -A 10 "Name:.*alsa_output"` — проверить громкость
5. `pactl list sinks | grep "Mute"` — проверить muted состояние

---

## 5. Отсутствие GPU-ускорения ONNX Runtime при автозапуске

**Симптом:** `GPU device discovery failed: device_discovery.cc:89 ReadFileContents Failed to open file: "/sys/class/drm/card1/device/vendor"`

**Причина:** При автозапуске драйверы GPU ещё не готовы.

**Решение:** Добавлен graceful fallback на `CPUExecutionProvider` с warning-логированием. Распознавание работает, но медленнее.
