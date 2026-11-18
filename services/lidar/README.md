# Просмотр лидара

Пассивный сервис ROS `/scan` + TF → NATS Core → WebSocket → Canvas.
Контракт, подключение и ограничения — [LIDAR_API_V1.md](../../docs/LIDAR_API_V1.md).

- `geometry.py`: чистое преобразование скана и проекция collision из URDF.
- `server.py`: подписки ROS/TF, публикация NATS и read-only WebSocket; HTTP также обслуживает общий виджет управления.
- `static/`: отдельное окно с сеткой, масштабом, корпусом и четырьмя колёсами.
- `install.py`: создание приватной конфигурации/ACL и установка неактивного unit.
- `tests/test_lidar.py`: автономные проверки без оборудования, ROS-контекста и NATS.

Использует `.venv-nats` и ROS-окружение из `tools/env.sh`. Для тестов из корня:

```bash
PYTHONPATH=services/lidar .venv-nats/bin/python -m unittest discover -s services/lidar/tests -v
```

Публикация разрешена только в `<robot_id>.lidar.update.*`. Доступа к командным
subjects, `/cmd_vel`, KV и управлению другими службами у NATS-компонента нет. HTTP-виджет проксирует команды в существующий bridge.
