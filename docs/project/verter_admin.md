# Verter Admin - Система голосового управления роботом

ROS2 пакет для создания интеллектуальной системы голосового управления роботом с распознаванием речи, AI-ассистентом и синтезом речи.

## Описание системы

Verter Admin — это набор ROS2 нод, которые вместе дают голосовой интерфейс робота:

- **Speech-to-text (STT)**: несколько реализаций (`speech_to_text_*_node`), которые слушают микрофон и публикуют `recognized_text`
- **Роутинг/состояния**: `recognition_node` принимает `recognized_text` и решает “шасси / AI / звуки / диалог”
- **AI-ассистент**: `ai_assistant_node` отвечает на `ai_question` и публикует текст в `text_to_speech`
- **Text-to-speech (TTS)**: `text_to_speech_node` (Piper) / `silero_tts_node` озвучивают `text_to_speech` и управляют `tts_control`
- **Звуки**: `sound_player_node` играет короткие эффекты по `play`
- **Шасси**: `chassis_node` принимает `verter_commands` и `/cmd_vel`, отправляет команды в Arduino по serial
- **Сенсоры**: `distance_sensors_node` публикует 7 ультразвуков (`Range`) + IMU, плюс конвертеры в `LaserScan`
- **DOA**: `doa_node` поворачивает “голову” по направлению звука (ReSpeaker)

## Архитектура системы

### Ноды системы (суть)

#### 1. `recognition_node`
**Назначение**: маршрутизация распознанного текста + state machine (триггер → команда → диалог)

**Функциональность**:
- Ожидание триггерных слов ("робот", "вертер", "вектор" и др.)
- Переключение между режимами: ожидание триггера → захват команды → диалоговый режим
- Фильтрация команд движения шасси
- Публикация команд в `verter_commands` / `ai_question`, управление `speech_control`

**Состояния**:
- `LISTENING_FOR_TRIGGER` - ожидание триггерного слова
- `CAPTURING_COMMAND` - захват команды после триггера
- `DIALOG_MODE` - диалоговый режим с AI
- `PAUSED` - пауза во время синтеза речи

#### 2. `ai_assistant_node`
**Назначение**: Обработка голосовых команд с помощью YandexGPT

**Функциональность**:
- Получение вопросов из топика `ai_question`
- Поиск по локальному датасету медицинских знаний
- Генерация ответов через YandexGPT API
- Публикация ответов в топик `text_to_speech`
- Управление диалогом и контекстом разговора

**Датасет**: Содержит медицинские знания по темам здоровья, болезней, роботов

#### 3. `text_to_speech_node` (Piper TTS)
**Назначение**: Синтез речи из текста с помощью Piper TTS

**Функциональность**:
- Получение текста из топика `text_to_speech`
- Синтез речи с использованием модели `ru_RU-ruslan-medium.onnx`
- Воспроизведение через PulseAudio
- Управление состоянием распознавания речи во время синтеза

#### 4. `silero_tts_node` (Silero TTS)
**Назначение**: Альтернативный синтез речи с помощью Silero TTS

**Функциональность**:
- Синтез речи с использованием Silero TTS
- Поддержка различных голосов (aidar, baya, kseniya, xenia)
- GPU/CPU режимы работы
- Более быстрый синтез по сравнению с Piper

#### 5. `sound_player_node`
**Назначение**: Воспроизведение звуковых файлов для обратной связи

**Функциональность**:
- Воспроизведение системных звуков (trigger.wav, success.wav, fail_timeout.wav)
- Поддержка различных аудиоформатов (WAV, MP3, OGG)
- Управление процессами воспроизведения
- Интеграция с системой шуток и развлечений

#### 6. `chassis_node` / `distance_sensors_node` / `doa_node`
**Назначение**: мосты к Arduino (шасси / сенсоры / голова)

**Функциональность**:
- Подключение по USB-serial
- Протокол команд строками (`CHASSIS:*`, `HEAD:*`)
- Публикация сенсоров в ROS2 (Range/Imu)

## Кодовая документация

Если нужно понять “как это работает по коду” (топики, пайплайн, узкие места) — см.:

- [Verter Admin — код (простыми словами)](../code/verter_admin.md)

## Как начать

- [Предпосылки](getting_started/prerequisites.md)
- [Сборка и установка](getting_started/install.md)
- [Первый запуск](getting_started/run.md)
- [Голосовое управление](guides/voice.md)
- [Диагностика](operations/diagnostics.md)

## Системный сервис

Для настройки автозапуска и управления сервисом Verter Admin см. [systemd сервис](operations/services.md)

# Общий статус NM и устройств
nmcli general status
nmcli device status

# Подробно по конкретному девайсу (замени на свой, напр. wlan0/eth0)
nmcli device show wlan0


# Скан доступных Wi‑Fi
nmcli device wifi list

# Мониторинг событий NM в реальном времени
nmcli monitor

# Все интерфейсы и адреса
ip a
# Статус линка (UP/DOWN), MTU, ошибки
ip -s link

# Только IPv4 адреса
hostname -I

# Ваш реальный внешний IP
curl ifconfig.me

# Таблица маршрутов
ip r
# Какой маршрут пойдет до адреса
ip route get 8.8.8.8
```

## Структура проекта

```
verter_admin/
├── src/verter_admin/
│   ├── ai_assistant/          # AI-ассистент на YandexGPT
│   ├── arduino/              # Управление Arduino
│   ├── launch/               # Launch файлы
│   ├── sound_player/         # Воспроизведение звуков
│   ├── speech_recognition/   # Распознавание речи (Vosk)
│   └── text_to_speech_node/  # Синтез речи (Piper/Silero)
├── services/                 # Systemd сервисы и скрипты управления
│   ├── verter-admin.service  # Systemd unit файл
│   ├── start_verter_admin.sh # Скрипт запуска
│   ├── stop_verter_admin.sh  # Скрипт остановки
│   └── README.md            # Документация по сервисам
├── package.xml               # ROS2 пакет
├── setup.py                 # Конфигурация сборки
└── README.md                # Документация
```


python3 /tmp/test_tof_stable.py

cd /home/verter/verter-robot/verter_admin
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/local_setup.bash 2>/dev/null || true
python3 src/verter_admin/tof_camera/tof_camera_node.py

# воспроизведение
aplay -D default /usr/share/sounds/alsa/Front_Center.wav

192.168.0.7

109.195.134.20