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



WINDOWS:


# Копируем исправленный проект и пересобираем
rm -rf ~/verter_admin && cp -r /mnt/c/Users/Пользователь/Documents/verter-robot/verter_admin ~/verter_admin && cd ~/verter_admin && rm -rf build install log && source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Проверяем что теперь файл создался в правильном месте
ls -la install/verter_admin/lib/verter_admin/

# Но нужно заново настроить окружение
source /opt/ros/humble/setup.bash
source install/setup.bash

# И запустить проект
ros2 launch verter_admin main.launch.py

# Собрать пакет
colcon build --packages-select verter_admin

# Запуск всей системы
ros2 launch verter_admin main.launch.py

# Или запуск нод по отдельности:
ros2 run verter_admin speech_recognition_node
ros2 run verter_admin ai_assistant_node

## Автозапуск через systemd-user

```bash
# 1. Скопировать unit в директорию пользователя
mkdir -p ~/.config/systemd/user
cp /home/verter/verter-robot/verter-admin.service ~/.config/systemd/user/

# 2. Перечитать конфигурацию
systemctl --user daemon-reload

# 3. Включить автозапуск и запустить сразу
systemctl --user enable --now verter-admin.service

# 4. Проверить статус
systemctl --user status verter-admin

# 5. Смотреть лог в реальном времени
journalctl --user -fu verter-admin

# 6. Перезапустить вручную (при обновлениях)
systemctl --user restart verter-admin

# 7. (Опционально) Разрешить запуск без GUI-логина
sudo loginctl enable-linger verter

# 8. Отключить автозапуск
systemctl --user disable --now verter-admin

systemctl --user stop verter-admin

systemctl --user status verter-admin.service

```

systemctl --user daemon-reload
systemctl --user restart verter-admin
journalctl --user -fu verter-admin


nmcli dev wifi connect "MySSID" password "MySuperSecret"
