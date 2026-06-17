# WiFi Connect — Решенные проблемы

Реализация Reactive WiFi — робот работает как точка доступа, когда не подключён к известной сети.

## Timing-диаграмма запуска

```
0 сек    - Загрузка системы
~5 сек   - NetworkManager активен
~15 сек  - wifi-connect скрипт запускается
~15-20   - iwgetid проверка
~60 сек  - NetworkManager полностью подключен (worst case)
```

## 1. `dnsmasq: Address already in use`

**Причина:** Системный dnsmasq занимал порт 53, конфликтуя с dnsmasq из wifi-connect.

**Решение:** Stop/start системного dnsmasq в wrapper-скрипте `wifi-connect-start.sh`.

---

## 2. iPhone не получает IP от точки доступа

**Причина:** UFW (firewall) блокировал DHCP (порты 67/68), DNS (порт 53), HTTP (порт 80) на интерфейсе wlan0.

**Решение:**
```bash
sudo ufw allow in on wlan0 to any port 67 proto udp  # DHCP server
sudo ufw allow in on wlan0 to any port 68 proto udp  # DHCP client
sudo ufw allow in on wlan0 to any port 53 proto udp  # DNS
sudo ufw allow in on wlan0 to any port 80 proto tcp  # HTTP captive portal
sudo ufw reload
```

---

## 3. Сеть подключается и сразу отключается

**Причина:** `Restart=always` в systemd-сервисе. После успешного подключения wifi-connect перезапускался и поднимал AP заново на wlan0, конфликтуя с NetworkManager. Бесконечный цикл.

**Решение:** `Restart=no` вместо `Restart=always`. Сервис запускается 1 раз при загрузке.

---

## 4. После reboot не автоподключается к известной сети

**Причина:** wifi-connect запускался слишком рано — до того, как NetworkManager успевал подключиться к известной сети.

**Решение:** `sleep 15` + проверка `iwgetid -r` перед запуском wifi-connect. Если NetworkManager уже подключён — wifi-connect не нужен.

---

## 5. Избыточная логика в wrapper-скрипте

**Причина:** Дублировали проверки интернета, которые уже есть внутри wifi-connect.

**Решение:** Использована минимальная обёртка по официальному примеру balena — только проверка `iwgetid`.

---

## 6. Конфликт NetworkManager и wifi-connect

**Причина:** Оба процесса пытались управлять wlan0 одновременно: NetworkManager пытался подключиться к сети, а wifi-connect уже поднял AP.

**Решение:** Timing-подход: дать NetworkManager 15 секунд на подключение к известной сети перед запуском wifi-connect.

## Диагностика

```bash
# Проверка NetworkManager
systemctl status NetworkManager
nmcli connection show
nmcli device status

# Проверка WiFi интерфейса
ip link show wlan0
iwgetid -r

# Проверка автоподключения
nmcli connection show "ИМЯ_СЕТИ" | grep autoconnect

# Проверка конфликтов dnsmasq
systemctl status dnsmasq
ss -tulpn | grep :53

# Проверка firewall
sudo ufw status numbered

# Проверка wifi-connect процесса
ps aux | grep wifi-connect

# Временная шкала NM после загрузки
journalctl -u NetworkManager --since "$(uptime -s)" | grep -E "Activation|connected|disconnected"
```

## Команды управления

```bash
sudo systemctl status wifi-connect
sudo systemctl start wifi-connect
sudo systemctl stop wifi-connect
sudo systemctl restart wifi-connect
sudo journalctl -u wifi-connect -f
sudo journalctl -u wifi-connect -n 100
```

## Best Practices

- Minimal wrapper — не дублировать логику проверки интернета
- Single run — `Restart=no`, сервис запускается 1 раз
- dnsmasq management — остановить системный dnsmasq перед запуском
- Firewall rules — открыть порты 67/68/53/80 на wlan0
- НЕ запускать wifi-connect слишком рано — дать NM 15 секунд

См. также: [wifi-connect GitHub](https://github.com/balena-os/wifi-connect), [Официальный start.sh](https://github.com/balena-os/wifi-connect/blob/master/scripts/start.sh)
