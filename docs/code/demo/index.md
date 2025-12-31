# Demo пакет `verter` (legacy)

Это “демо-ветка” кода в `demo/src/verter`. Судя по структуре, это более ранний стек: отдельные ноды под Arduino (шасси/руки) и Vosk-распознавание.

## Ноды (основные)

- `verter_system/arduino_moving_navigation_node.py`
  - слушает `verter_commands` (String)
  - по serial общается с Arduino шасси/навигации (devpath=1.4)
  - публикует `distance_h04` и `arduino_status`

- `verter_system/arduino_hands_node.py`
  - слушает `verter_commands` (String)
  - по serial общается с Arduino рук (devpath=1.1)

- `speech_recognition/vosk_node.py`
  - слушает микрофон, делает STT через Vosk
  - публикует команды в `verter_commands`

## Launch

- `launch/main.launch.py` включает:
  - `launch/verter_system.launch.py`
  - `launch/speech_system.launch.py`

## Связь с текущим стеком

Если ты развиваешь `verter_admin`, этот demo можно воспринимать как справочник/источник идей (протоколы serial, devpath), но документацию проекта лучше держать вокруг `verter_admin`.


