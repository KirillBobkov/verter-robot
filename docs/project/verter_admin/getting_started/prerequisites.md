# Предпосылки

## ПО

- **ROS2 Humble** (рекомендуется Ubuntu 22.04; для Windows — WSL2, но аудио/USB там сложнее)
- **Python** (под ROS2 Humble)
- **PulseAudio / PipeWire**: чтобы `aplay`/`pactl` и звук нормально работали

## Железо (минимум)

- **Микрофон** (в идеале ReSpeaker USB Array)
- **Аудиовыход** (динамик/наушники)
- **Arduino**:
  - шасси (для `chassis_node`)
  - сенсоры (Arduino Mega для `distance_sensors_node`)
  - голова (для `doa_node`, если используется)


