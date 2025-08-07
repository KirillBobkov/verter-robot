# Verter Admin - Пакет распознавания речи

Простой ROS2 пакет для распознавания речи с помощью библиотеки Vosk.

## Требования

- ROS2 (humble/foxy)
- Python 3
- Vosk модель для русского языка

## Установка зависимостей

```bash
pip install vosk sounddevice numpy
```

## Установка модели Vosk

```bash
cd /home/verter/verter-robot/verter-admin
wget https://alphacephei.com/vosk/models/vosk-model-small-ru-0.22.zip
unzip vosk-model-small-ru-0.22.zip
```

## Сборка пакета

```bash
cd /home/verter/verter-robot/verter-admin
colcon build --packages-select verter_admin
source install/setup.bash
```

## Запуск

```bash
source install/setup.bash
ros2 run verter_admin speech_recognition_node
```

## Описание

Нода слушает микрофон и выводит распознанный текст в консоль. Автоматически находит доступное аудиоустройство и использует его для захвата звука.

Основные функции:
- Автоматическое обнаружение аудиоустройств
- Распознавание русской речи
- Вывод результатов в консоль и ROS2 логи
- Корректное завершение работы по Ctrl+C