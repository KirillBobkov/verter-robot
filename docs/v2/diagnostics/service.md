# Systemd-сервис

Сервис `verter-admin.service` управляет полным запуском системы. Установка, удаление и переустановка описаны в [Система и сервисы](../problems_solved/system_services.md).

## Жизненный цикл

```bash
systemctl --user start verter-admin.service    # запустить
systemctl --user stop verter-admin.service     # остановить
systemctl --user restart verter-admin.service  # перезапустить
systemctl --user status verter-admin.service --no-pager
```

## Автозапуск

```bash
systemctl --user enable verter-admin.service   # включить
systemctl --user disable verter-admin.service  # отключить
```

После изменения файла сервиса:
```bash
systemctl --user daemon-reload && systemctl --user restart verter-admin.service
```

## Linger (запуск без входа в сессию)

Пользовательский systemd-сервис работает только после входа пользователя. Для headless-режима:

```bash
sudo loginctl enable-linger jetson
loginctl show-user jetson | grep Linger      # должно показать Linger=yes
```

## Ручной запуск (без systemd)

```bash
cd ~/verter-robot/verter_admin/services && ./start_verter_admin.sh
cd ~/verter-robot/verter_admin/services && ./stop_verter_admin.sh
```
