# WiFi Connect - Решенные проблемы

## Краткий список для быстрого reference

| # | Проблема | Решение |
|---|----------|---------|
| 1 | `dnsmasq: Address already in use` | Stop/start системного dnsmasq в wrapper-скрипте |
| 2 | iPhone не получает IP от AP | Добавить UFW rules для портов 67/68/53/80 на wlan0 |
| 3 | Сеть подключается и сразу отключается | `Restart=no` вместо `Restart=always` |
| 4 | После reboot не автоподключается | `sleep 15` + `iwgetid -r` check перед wifi-connect |
| 5 | Избыточная логика в wrapper | Использовать минимальную обёртку как у balena |
| 6 | NetworkManager vs wifi-connect конфликт | Timing: дать NM 15 сек на подключение |

## Ключевые команды диагностики

```bash
# Быстрая проверка
iwgetid -r                           # WiFi подключен?
nmcli -t g                          # Интернет есть?
systemctl status wifi-connect        # Статус сервиса

# Временная шкала после загрузки
journalctl -u NetworkManager --since "$(uptime -s)" | grep -E "Activation|connected"

# Проверка автоподключения
nmcli connection show "SSID" | grep autoconnect

# Проверка конфликтов
ss -tulpn | grep :53                # dnsmasq конфликт
ps aux | grep wifi-connect          # Процессы
```

## Best practices

✅ **DO:**
- Minimal wrapper (не дублировать логику wifi-connect)
- 15 sec delay для NetworkManager
- `Restart=no` в systemd
- `iwgetid -r` pre-check
- `autoconnect=yes` для сохраненных сетей
- Stop/start dnsmasq
- UFW rules для wlan0

❌ **DON'T:**
- `Restart=always` или `Restart=on-success`
- Полагаться на `network-online.target`
- Добавлять свою логику проверки интернета
- Забывать про системный dnsmasq
- Запускать wifi-connect слишком рано

## Timing

```
0s    → Boot
5s    → NetworkManager active
15s   → wifi-connect запускается
15-20s → iwgetid check
60s   → NetworkManager полностью готов (worst case)
```
