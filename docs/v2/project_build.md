# Сборка проекта

После настройки [Ubuntu + ROS2](os_setup.md) и [udev правил](udev_rules.md).

---

## 1. Клонировать репозиторий

```bash
git clone <repo-url> ~/verter-robot
```

## 2. Настроить ROS2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -sf ~/verter-robot/verter_admin .
```

## 3. Собрать пакет

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select verter_admin --symlink-install
source install/setup.bash
```

Флаг `--symlink-install` — Python-файлы не копируются, а линкуются. Правки в коде применяются без пересборки (достаточно перезапустить ноду).

## 4. Python-зависимости

Зависимости указаны в `setup.py` (`install_requires`) и подтягиваются автоматически при colcon build:

| Пакет | Назначение |
|---|---|
| `sherpa-onnx` | Распознавание речи (STT) |
| `silero-tts` | Синтез речи (TTS) |
| `torch` | Для Silero TTS |
| `onnxruntime` | Исполнение ONNX-моделей |
| `yandex-cloud-ml-sdk` | AI-ассистент (YandexGPT) |
| `sounddevice` | Захват/воспроизведение аудио |
| `numpy<2.0`, `opencv-python<4.9` | Обработка данных |
| `pyserial` | Связь с Arduino по UART |
| `kaldi-native-fbank` | Извлечение аудио-признаков (MFCC) |
| `pyusb` | USB-устройства |

Если colcon не подтянул зависимости — установить вручную:

```bash
pip install sherpa-onnx silero-tts torch onnxruntime yandex-cloud-ml-sdk \
    sounddevice 'numpy<2.0' pyserial kaldi-native-fbank pyusb
```

## 5. Автозагрузка окружения

Добавить в `~/.bashrc`, чтобы ROS2 и workspace подхватывались при каждом входе:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 6. Проверить сборку

```bash
# Список нод пакета
ros2 pkg list | grep verter_admin

# Проверить launch-файлы
ros2 launch verter_admin test_lidar.launch.py
```
