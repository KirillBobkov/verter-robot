# `waypoint_navigator`

## Задача

CLI-утилита/нода: отправляет робота к именованной точке из `waypoints.yaml` через action `navigate_to_pose` (Nav2).

## Входы / выходы

- **Action client**: `navigate_to_pose` (nav2_msgs/NavigateToPose)

## Простыми словами

- Читает `waypoints.yaml`
- Ищет waypoint по имени, которое передали аргументом
- Конвертит `yaw` в quaternion
- Отправляет goal в Nav2 и ждёт результат


