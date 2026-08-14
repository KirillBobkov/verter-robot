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
colcon build --packages-select verter_admin_msgs   # отдельный ament_cmake пакет, собирается первым
colcon build --packages-select verter_admin --symlink-install
source install/setup.bash
```

`verter_admin_msgs` (кастомные сервисы `SaveWaypoint`, `ListWaypoints`, `DeleteWaypoint`, `NavigateToWaypoint`) — отдельный пакет `ament_cmake`, его надо собрать до `verter_admin`, иначе импорт `verter_admin_msgs.srv` в `waypoint_manager_node` завершится ошибкой.

Флаг `--symlink-install` — Python-файлы не копируются, а линкуются. Правки в коде применяются без пересборки (достаточно перезапустить ноду).

## 4. Python-зависимости

Зависимости указаны в `setup.py` (`install_requires`) и подтягиваются автоматически при colcon build:

| Пакет | Назначение |
|---|---|
| `sherpa-onnx` | Распознавание речи (STT) |
| `silero-tts` | Синтез речи (TTS, активный движок) |
| `piper-tts` | Синтез речи (TTS, движок Piper) |
| `vosk-tts` | Синтез речи (TTS, движок Vosk) |
| `torch` | Для Silero TTS |
| `onnxruntime` | Исполнение ONNX-моделей |
| `yandex-cloud-ml-sdk` | AI-ассистент (YandexGPT) |
| `openai` | AI-ассистент (OpenAI-совместимый клиент для YandexGPT) |
| `ru-normalizr` | Нормализация русского текста (TTS) |
| `sounddevice` | Захват/воспроизведение аудио |
| `numpy<2.0`, `opencv-python<4.9` | Обработка данных |
| `pyserial` | Связь с шасси ZLAC8015D (Modbus RTU) и датчиками по UART |
| `kaldi-native-fbank` | Извлечение аудио-признаков (MFCC) |
| `pyusb` | USB-устройства |

Если colcon не подтянул зависимости — установить вручную:

```bash
pip install sherpa-onnx silero-tts piper-tts vosk-tts torch onnxruntime \
    yandex-cloud-ml-sdk openai ru-normalizr sounddevice 'numpy<2.0' \
    pyserial kaldi-native-fbank pyusb
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

# Проверить launch-файлы (chassis-only bringup — простой smoke-тест)
ros2 launch verter_admin chassis_bringup.launch.py
```
