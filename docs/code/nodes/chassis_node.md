# `chassis_node`

## Задача

Это мост “ROS2 команды → Arduino шасси”.

Умеет управляться двумя источниками:

1. `verter_commands` — **строковый протокол** (обычно от `recognition_node`)
2. `/cmd_vel` — **стандарт Nav2** (`geometry_msgs/Twist`)

## Входы / выходы

- **Sub**: `verter_commands` (String)
- **Sub**: `/cmd_vel` (Twist)

## Простыми словами как работает

- При старте пытается найти и открыть serial-порт Arduino.
- Любую команду из `verter_commands` (строка) отправляет в Arduino по serial.
- `Twist` из `/cmd_vel` конвертирует в “понятные Arduino” команды и тоже отправляет по serial.

## Протокол команд (важное)

В строках используются префиксы `CHASSIS:*` (например `CHASSIS:STOP`, `CHASSIS:LEFT:ASK:...`).

Arduino-скетчи в папке `verter_admin/src/verter_admin/chassis/*.ino` ожидают этот формат.


