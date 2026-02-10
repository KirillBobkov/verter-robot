# Диагностика проблемы отсутствия звука

## Проблема

Звук не воспроизводится ни при автозапуске, ни при ручном запуске из консоли.

## Наблюдения из логов

### aplay запускается успешно
```
[sound_player_node-5] [INFO]: Запуск aplay: aplay -D pulse -q -f S16_LE -r 44100 -c 2 --buffer-size=4096 --period-size=512 ...
[sound_player_node-5] [INFO]: aplay завершился успешно
```

### Но звука нет!

## Доступные аудио устройства (из диагностики)

### PulseAudio Sinks (выходные устройства):
```
0  alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo  s24le 2ch 16000Hz
1  alsa_output.usb-Generic_USB_Audio_20210726905926-00.analog-stereo      s16le 2ch 48000Hz
2  alsa_output.platform-sound.analog-stereo                                    s16le 2ch 44100Hz
```

### PulseAudio Sources (входные устройства):
```
0  alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo.monitor
1  alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input
2  alsa_output.usb-Generic_USB_Audio_20210726905926-00.analog-stereo.monitor
3  alsa_input.usb-Generic_USB_Audio_20210726905926-00.mono-fallback
4  alsa_output.platform-sound.analog-stereo.monitor
5  alsa_input.platform-sound.analog-stereo
```

## Возможные причины

### 1. Неправильный default sink
PulseAudio может использовать sink #0 (ReSpeaker 16kHz) как default, но aplay пытается воспроизвести 44.1kHz звук, что может вызывать проблемы с ресемплингом или просто не работать.

### 2. Проблема с ресемплингом
ReSpeaker sink поддерживает только 16kHz, но aplay пытается воспроизвести 44.1kHz или 22.05kHz звук. Это может вызывать проблемы.

### 3. Устройство вывода не подключено
Generic USB Audio (sink #1) или встроенная аудио (sink #2) могут быть не подключены к колонкам.

## План диагностики

### Шаг 1: Проверить default sink
```bash
pactl info | grep "Default Sink"
```

### Шаг 2: Проверить состояние sink'ов
```bash
pactl list sinks
```

### Шаг 3: Протестировать каждый sink напрямую
```bash
# Тест sink #0 (ReSpeaker 16kHz)
aplay -D pulse:0 -f S16_LE -r 16000 -c 2 /usr/share/sounds/alsa/Front_Center.wav

# Тест sink #1 (Generic USB Audio 48kHz)
aplay -D pulse:1 -f S16_LE -r 48000 -c 2 /usr/share/sounds/alsa/Front_Center.wav

# Тест sink #2 (встроенная аудио 44.1kHz)
aplay -D pulse:2 -f S16_LE -r 44100 -c 2 /usr/share/sounds/alsa/Front_Center.wav
```

### Шаг 4: Проверить громкость на sink'ах
```bash
# Проверить громкость default sink
pactl list sinks | grep -A 10 "Name:.*alsa_output"

# Установить громкость
pactl set-sink-volume <sink_name> 100%
```

### Шаг 5: Проверить muted состояние
```bash
# Проверить не muted ли sink
pactl list sinks | grep "Mute"

# Размутить sink
pactl set-sink-mute <sink_name> 0
```

## Решения

### Решение 1: Установить правильный default sink

Если sink #1 (Generic USB Audio) или sink #2 (встроенная аудио) работает:
```bash
pactl set-default-sink alsa_output.usb-Generic_USB_Audio_20210726905926-00.analog-stereo
# или
pactl set-default-sink alsa_output.platform-sound.analog-stereo
```

### Решение 2: Явно указать sink в коде

Изменить `audio_device` параметр с `pulse` на конкретный sink:
```python
# В text_to_speech_node.py и sound_player_node.py:
self.audio_device = "pulse:1"  # или "pulse:2"
```

### Решение 3: Использовать ALSA напрямую вместо PulseAudio

Если PulseAudio вызывает проблемы:
```python
# Использовать hw:1,0 или hw:2,0 вместо pulse
self.audio_device = "hw:1,0"  # Generic USB Audio
# или
self.audio_device = "hw:2,0"  # встроенная аудио
```

### Решение 4: Настроить permanent default sink

Создать файл `~/.config/pulse/default.pa`:
```
set-default-sink alsa_output.usb-Generic_USB_Audio_20210726905926-00.analog-stereo
```

Или использовать `pavucontrol` для настройки через GUI.

## Рекомендуемый подход

1. **Сначала протестировать каждый sink напрямую** - чтобы понять какой работает
2. **Установить рабочий sink как default** - это самое простое решение
3. **Если это не помогает** - явно указать sink в коде

## Следующие шаги

1. Выполнить диагностику (Шаги 1-5)
2. Определить какой sink работает
3. Применить соответствующее решение
4. Протестировать с ROS2
