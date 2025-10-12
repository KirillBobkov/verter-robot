# Verter Admin - Система голосового управления роботом

ROS2 пакет для создания интеллектуальной системы голосового управления роботом с распознаванием речи, AI-ассистентом и синтезом речи.

## Описание системы

Verter Admin - это комплексная система, состоящая из нескольких взаимосвязанных нод, которые обеспечивают:

- **Распознавание речи** с помощью библиотеки Vosk
- **AI-ассистент** на базе YandexGPT для обработки команд
- **Синтез речи** через Piper TTS или Silero TTS
- **Управление Arduino** для контроля движений робота
- **Воспроизведение звуков** для обратной связи

## Архитектура системы

### Ноды системы

#### 1. `speech_recognition_node`
**Назначение**: Распознавание голосовых команд с помощью библиотеки Vosk

**Функциональность**:
- Непрерывное прослушивание аудиопотока
- Распознавание триггерных слов ("робот", "вертер", "вектор" и др.)
- Переключение между режимами: ожидание триггера → захват команды → диалоговый режим
- Определение направления звука (DOA) для управления головой робота
- Фильтрация команд движения шасси
- Публикация распознанного текста в топик `ai_question`

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
- Публикация ответов в топик `ai_response`
- Управление диалогом и контекстом разговора

**Датасет**: Содержит медицинские знания по темам здоровья, болезней, роботов

#### 3. `text_to_speech_node` (Piper TTS)
**Назначение**: Синтез речи из текста с помощью Piper TTS

**Функциональность**:
- Получение текста из топика `ai_response`
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

#### 6. `arduino_node`
**Назначение**: Управление Arduino для контроля движений робота

**Функциональность**:
- Автоматическое подключение к Arduino через USB
- Отправка команд движения в топик `/verter_commands`
- Управление шасси робота (вперед, назад, влево, вправо, стоп)
- Управление головой робота (поворот влево/вправо/центр)

## Требования

### Системные требования
- Ubuntu 20.04+ или Windows с WSL2
- ROS2 Humble
- Python 3.8+
- PulseAudio (для аудио)

### Аппаратные требования
- Микрофон (рекомендуется ReSpeaker USB Array)
- Динамики или наушники
- Arduino Uno/Nano для управления роботом
- USB-кабель для подключения Arduino

## Установка

### 1. Установка зависимостей Python

```bash
pip install vosk sounddevice numpy pyusb yandex-cloud-ml-sdk silero-tts pyserial piper-tts torch torchaudio
```

### 2. Установка системных зависимостей

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install python3-pip python3-dev portaudio19-dev libasound2-dev
sudo apt install pulseaudio pulseaudio-utils
```

**Windows (WSL2):**
```bash
# Установка PulseAudio для WSL
sudo apt install pulseaudio pulseaudio-utils
```

### 3. Настройка API ключей

Создайте файл `src/verter_admin/ai_assistant/api_key.yaml`:
```yaml
yandex_api_key: "ваш_yandex_api_ключ"
```

### 4. Сборка пакета

```bash
cd /path/to/verter-robot/verter_admin
colcon build --packages-select verter_admin
source install/setup.bash
```

## Запуск системы

### Запуск всей системы (рекомендуется)

```bash
# Настройка окружения
source /opt/ros/humble/setup.bash
source install/setup.bash

# Запуск всей системы
ros2 launch verter_admin main.launch.py
```

### Запуск с Silero TTS

```bash
ros2 launch verter_admin silero.launch.py
```

### Запуск отдельных нод

```bash
# Настройка окружения
source /opt/ros/humble/setup.bash
source install/setup.bash

# Запуск нод по отдельности
ros2 run verter_admin speech_recognition_node
ros2 run verter_admin ai_assistant_node
ros2 run verter_admin text_to_speech_node
ros2 run verter_admin sound_player_node
ros2 run verter_admin arduino_node
```

## Управление системой

### Голосовые команды

**Триггерные слова:**
- "робот", "вертер", "вектор", "ветер" и др.

**Команды движения:**
- "вперед", "назад", "влево", "вправо", "стоп"

**Команды головы:**
- Автоматически по направлению звука

**Завершение диалога:**
- "хватит", "спасибо", "пока", "до свидания"

### ROS2 топики

**Входящие:**
- `ai_question` (String) - распознанный текст
- `dialog_control` (String) - управление диалогом
- `/play` (String) - команды воспроизведения звуков
- `/verter_commands` (String) - команды для Arduino

**Исходящие:**
- `ai_response` (String) - ответы AI-ассистента
- `set_recognition_active` (Bool) - управление распознаванием

## Системный сервис

### Установка как системный сервис

```bash
# Копирование сервисного файла
sudo cp verter-admin.service /etc/systemd/system/

# Перезагрузка systemd
sudo systemctl daemon-reload

# Включение автозапуска
sudo systemctl enable verter-admin

# Запуск сервиса
sudo systemctl start verter-admin
```

### Управление сервисом

```bash
Перезагрузить systemd: 
sudo systemctl daemon-reload

Запустить сервис: 
sudo systemctl start verter-admin.service

Проверить статус: 
systemctl status verter-admin.service

Подключиться к журналу: 
journalctl -u verter-admin.service -f
```

## Отладка и диагностика

### Проверка аудиоустройств

```bash
# Список аудиоустройств
pactl list short sources
pactl list short sinks

# Тест микрофона
arecord -f cd -d 5 test.wav
```

### Проверка Arduino

```bash
# Список USB устройств
lsusb

# Проверка последовательных портов
ls /dev/ttyUSB* /dev/ttyACM*
```


## Настройка WiFi (для Raspberry Pi)

```bash
# Подключение к WiFi
nmcli dev wifi connect "SSID" password "PASSWORD"

# Просмотр подключений
nmcli connection show
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
├── package.xml               # ROS2 пакет
├── setup.py                 # Конфигурация сборки
└── README.md                # Документация
```
