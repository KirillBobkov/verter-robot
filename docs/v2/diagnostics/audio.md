# Диагностика аудио

## PulseAudio: устройства

```bash
pactl list short sinks              # вывод
pactl list short sources            # ввод
pactl info | grep "Default Sink"    # текущий default sink
```

## Громкость и mute

```bash
pactl list sinks | grep -A 10 "Name:.*alsa_output"   # состояние
pactl list sinks | grep "Mute"                        # замотушены ли
```

## Запись и воспроизведение

```bash
arecord -f cd -d 5 test.wav      # записать 5 секунд
aplay test.wav                   # воспроизвести
arecord -f cd -d 2 - | aplay     # запись + воспроизведение без файла
```

## Тестирование конкретного sink

```bash
aplay -D pulse:0 test.wav        # sink #0
aplay -D pulse:1 test.wav        # sink #1
```

## Сменить default sink

```bash
pactl set-default-sink alsa_output.usb-Generic_USB_Audio_20210726905926-00.analog-stereo
```

## USB-аудио в системе

```bash
lsusb | grep -i audio
python3 -c "import sounddevice as sd; print(sd.query_devices())"
```

## ALSA (обход PulseAudio)

```bash
aplay -l                              # список устройств
aplay -D hw:1,0 test.wav              # прямое воспроизведение
```

---

Конкретные проблемы: [Аудио система](../problems_solved/audio_system.md).
