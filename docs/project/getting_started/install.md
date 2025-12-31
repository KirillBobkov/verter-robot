# Сборка и установка

## 1) Собрать пакет

Из директории workspace (где лежит `verter_admin/`):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select verter_admin
source install/setup.bash
```

## 2) Python зависимости

`verter_admin/setup.py` содержит `install_requires`, поэтому при установке/сборке в ROS окружении зависимости должны подтягиваться согласно твоему флоу.

Если ты ставишь Python-зависимости вручную (venv/не-ROS окружение) — сверяйся с `verter_admin/setup.py` и ставь только нужное под твой набор нод (STT/TTS/AI).


