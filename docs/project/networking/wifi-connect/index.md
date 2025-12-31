# WiFi Connect для Verter Robot

Автоматическая настройка WiFi через captive portal. Основано на [balena-os/wifi-connect](https://github.com/balena-os/wifi-connect).

## Как это работает (по официальной документации balena)

**WiFi Connect** - это утилита от Balena для динамической настройки WiFi через captive portal.

### Алгоритм работы:

```
┌─────────────────────────────────────────────────────────┐
│              ЗАГРУЗКА СИСТЕМЫ                           │
└────────────────────────┬────────────────────────────────┘
                         │
                         ▼
            ┌────────────────────────┐
            │  NetworkManager        │
            │  пытается подключиться │
            │  к сохраненным сетям   │
            └────────────┬───────────┘
                         │
                         │ (ждем 15 сек)
                         ▼
            ┌────────────────────────┐
            │  Проверка: iwgetid -r  │
            │  Есть ли WiFi?         │
            └────────┬───────────────┘
                     │
         ┌───────────┴───────────┐
         │ ДА                    │ НЕТ
         ▼                       ▼
┌─────────────────┐    ┌─────────────────────┐
│  Skip           │    │  Запуск wifi-connect│
│  wifi-connect   │    │  (бинарник)         │
│  EXIT 0         │    └──────────┬──────────┘
└─────────────────┘               │
                                  ▼
                       ┌─────────────────────┐
                       │  wifi-connect       │
                       │  проверяет интернет │
                       └──────────┬──────────┘
                                  │
                      ┌───────────┴──────────┐
                      │ ЕСТЬ                 │ НЕТ
                      ▼                      ▼
            ┌─────────────────┐   ┌─────────────────────┐
            │  EXIT 0         │   │  Поднять AP         │
            └─────────────────┘   │  "Verter-Setup"     │
                                  └──────────┬──────────┘
                                             │
                                             ▼
                                  ┌─────────────────────┐
                                  │  Captive Portal     │
                                  │  192.168.42.1       │
                                  └──────────┬──────────┘
                                             │
                                             ▼
                                  ┌─────────────────────┐
                                  │  Пользователь       │
                                  │  вводит WiFi        │
                                  └──────────┬──────────┘
                                             │
                                             ▼
                                  ┌─────────────────────┐
                                  │  Сохранить в        │
                                  │  NetworkManager +   │
                                  │  autoconnect=yes    │
                                  └──────────┬──────────┘
                                             │
                                             ▼
                                  ┌─────────────────────┐
                                  │  Подключиться       │
                                  │  EXIT 0             │
                                  └─────────────────────┘
```

### Ключевые моменты:

1. **Сервис запускается 1 раз** при загрузке системы
2. **Первая проверка** (в скрипте): `iwgetid -r` - есть ли уже WiFi соединение
3. **Вторая проверка** (внутри wifi-connect): есть ли интернет
4. **Если всё ОК** - просто выходит, не мешая работе
5. **Если нет WiFi** - поднимает AP и ждет пользователя
6. **После настройки** - сохраняет credentials в NetworkManager с `autoconnect=yes`

### Почему именно так:

- **Простота**: минимум кастомной логики, всё в wifi-connect
- **Надежность**: официальный подход от Balena, проверенный в production
- **Autoconnect**: после перезагрузки NetworkManager сам подключится к сохраненной сети

## Расположение файлов

```
/home/verter/verter-robot/wifi-connect/
├── wifi-connect              # Бинарник
├── wifi-connect-start.sh     # Обёртка с логикой проверки WiFi
├── wifi-connect.service      # Systemd unit file (скопировать в /etc/systemd/system/)
├── README.md                 # Документация
└── ui/                       # Веб-интерфейс captive portal
```

## Установка сервиса

```bash
cd /home/verter/verter-robot/wifi-connect
sudo cp wifi-connect.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable wifi-connect
sudo systemctl start wifi-connect
```

## Параметры

| Параметр | Значение по умолчанию | Описание |
|----------|----------------------|----------|
| `PORTAL_INTERFACE` | `wlan0` | WiFi интерфейс |
| `PORTAL_SSID` | `Verter-Setup` | Имя точки доступа |
| `PORTAL_PASSPHRASE` | `verter123` | Пароль точки доступа |
| `PORTAL_GATEWAY` | `192.168.42.1` | IP адрес captive portal |
| `PORTAL_DHCP_RANGE` | `192.168.42.2,192.168.42.254` | Диапазон DHCP |
| `ACTIVITY_TIMEOUT` | `300` | Таймаут неактивности (сек) |
| `CHECK_INTERVAL` | `10` | Интервал проверки WiFi (сек) |

Для изменения параметров редактируй `/etc/systemd/system/wifi-connect.service`:

```bash
sudo nano /etc/systemd/system/wifi-connect.service
sudo systemctl daemon-reload
sudo systemctl restart wifi-connect
```

## Управление сервисом

```bash
# Статус
sudo systemctl status wifi-connect

# Запустить
sudo systemctl start wifi-connect

# Остановить
sudo systemctl stop wifi-connect

# Перезапустить
sudo systemctl restart wifi-connect

# Включить автозапуск
sudo systemctl enable wifi-connect

# Отключить автозапуск
sudo systemctl disable wifi-connect

# Логи (в реальном времени)
sudo journalctl -u wifi-connect -f

# Логи (последние 100 строк)
sudo journalctl -u wifi-connect -n 100
```

## Тестирование

### Тест 1: Автоподключение после перезагрузки

```bash
# Должен автоматически подключиться к сохраненной сети
sudo reboot

# После загрузки проверяем
iwgetid -r
nmcli -t g
systemctl status wifi-connect
```

### Тест 2: Поднятие AP при отсутствии сохраненных сетей

```bash
# Забываем все сети
nmcli connection show | grep wifi | awk '{print $1}' | xargs -I {} nmcli connection delete {}

# Перезагружаемся
sudo reboot

# Должна появиться точка доступа "Verter-Setup"
# Подключаемся с телефона, настраиваем WiFi
```

## Файлы

| Файл | Путь | Описание |
|------|------|----------|
| Бинарник | `/home/verter/verter-robot/wifi-connect/wifi-connect` | Основной бинарник |
| Обёртка | `/home/verter/verter-robot/wifi-connect/wifi-connect-start.sh` | Скрипт с логикой |
| UI | `/home/verter/verter-robot/wifi-connect/ui/` | Веб-интерфейс |
| Сервис | `/etc/systemd/system/wifi-connect.service` | Systemd unit file |

## Известные проблемы и решения

При интеграции wifi-connect были выявлены и решены следующие проблемы. **Важно знать для будущих интеграций:**

| # | Проблема | Причина | Решение |
|---|----------|---------|---------|
| 1 | **dnsmasq не запускается** — `Address already in use` | Системный dnsmasq занимал порт 53 | Добавлена остановка/запуск системного dnsmasq в `wifi-connect-start.sh` |
| 2 | **Устройство не получает IP при подключении к AP** | UFW блокировал DHCP (67/68), DNS (53), HTTP (80) | Добавлены правила UFW для wlan0 (см. ниже) |
| 3 | **Сеть подключается, но через 5-10 сек отключается** | `wifi-connect` service с `Restart=always` перезапускался после успешного подключения и поднимал AP заново на wlan0, конфликтуя с NetworkManager | Установили `Restart=no` — сервис запускается 1 раз при загрузке |
| 4 | **После перезагрузки не автоподключается к сохраненной сети** | `wifi-connect` запускался слишком рано (до того как NetworkManager успел подключиться) и поднимал AP | Добавили `sleep 15` + проверку `iwgetid -r` в скрипт перед запуском wifi-connect |
| 5 | **Избыточная логика в wrapper-скрипте** | Дублировали проверки интернета, которые уже есть внутри wifi-connect | Переписали скрипт по официальному примеру balena — минимальная обёртка с проверкой `iwgetid` |
| 6 | **NetworkManager vs wifi-connect конфликт на wlan0** | Оба пытались управлять wlan0 одновременно: NetworkManager пытался подключиться к сети, а wifi-connect уже поднял AP | Решено через timing: даём NetworkManager 15 сек на автоподключение, потом проверяем результат |

### Необходимые правила UFW

Для корректной работы captive portal необходимо разрешить порты на интерфейсе wlan0:

```bash
sudo ufw allow in on wlan0 to any port 67 proto udp  # DHCP server
sudo ufw allow in on wlan0 to any port 68 proto udp  # DHCP client
sudo ufw allow in on wlan0 to any port 53 proto udp  # DNS
sudo ufw allow in on wlan0 to any port 80 proto tcp  # HTTP captive portal
sudo ufw reload
```

---

## Команды диагностики

### Быстрая проверка состояния

```bash
# WiFi подключен?
iwgetid -r

# Интернет есть?
nmcli -t g

# Статус сервиса
systemctl status wifi-connect

# Последние логи
journalctl -u wifi-connect -n 50 --no-pager
```

### Полная диагностика

```bash
# 1. Проверка NetworkManager
systemctl status NetworkManager
nmcli connection show
nmcli device status

# 2. Проверка WiFi интерфейса
ip link show wlan0
iw dev wlan0 info

# 3. Проверка автоподключения
nmcli connection show "ИМЯ_СЕТИ" | grep autoconnect

# 4. Проверка конфликтов dnsmasq
systemctl status dnsmasq
ss -tulpn | grep :53

# 5. Проверка firewall
sudo ufw status numbered

# 6. Проверка wifi-connect процесса
ps aux | grep wifi-connect

# 7. Временная шкала NetworkManager после загрузки
journalctl -u NetworkManager --since "$(uptime -s)" | grep -E "Activation|connected|disconnected"
```

### Тестирование вручную

```bash
# Запуск скрипта напрямую (с полными логами)
sudo /home/verter/verter-robot/wifi-connect/wifi-connect-start.sh

# Запуск только бинарника (минимум)
sudo /home/verter/verter-robot/wifi-connect/wifi-connect \
  --portal-ssid Verter-Test \
  --ui-directory /home/verter/verter-robot/wifi-connect/ui
```

---

## Решение проблем

### Точка доступа не появляется

1. **Проверь WiFi статус:**
   ```bash
   iwgetid -r  # Должно быть пусто, иначе уже подключен
   ```

2. **Проверь что NetworkManager активен:**
   ```bash
   systemctl status NetworkManager
   ```

3. **Проверь интерфейс wlan0:**
   ```bash
   ip link show wlan0
   nmcli device status | grep wlan0
   ```

4. **Проверь логи сервиса:**
   ```bash
   journalctl -u wifi-connect -f
   ```

### Captive portal не открывается

1. Подключись к точке "Verter-Setup"
2. Открой браузер и перейди на http://192.168.42.1
3. Если не работает, проверь IP:
   ```bash
   ip addr show wlan0
   ```

### Не подключается к выбранной сети

1. Проверь правильность пароля
2. Проверь что сеть в зоне доступа:
   ```bash
   nmcli device wifi list
   ```
3. Попробуй подключиться вручную:
   ```bash
   nmcli device wifi connect "SSID" password "PASSWORD"
   ```

---

## Best Practices для интеграции wifi-connect

### ✅ Правильный подход:

1. **Minimal wrapper** — не дублировать логику, которая уже есть в wifi-connect
2. **Timing is critical** — дать NetworkManager время (~15 сек) на автоподключение перед запуском wifi-connect
3. **Single run** — `Restart=no` в systemd, запускать 1 раз при загрузке
4. **Pre-check** — использовать `iwgetid -r` для быстрой проверки WiFi соединения
5. **Autoconnect** — всегда устанавливать `connection.autoconnect yes` для сохраненных сетей
6. **dnsmasq management** — остановить системный dnsmasq перед запуском AP, перезапустить после
7. **Firewall rules** — открыть порты 67/68/53/80 на wlan0 интерфейсе

### ❌ Частые ошибки:

1. **НЕ использовать** `Restart=always` или `Restart=on-success` — вызывает повторные запуски AP
2. **НЕ полагаться** на `network-online.target` — NetworkManager может быть "online" но еще не подключен
3. **НЕ добавлять** свою логику проверки интернета — она уже есть внутри wifi-connect
4. **НЕ забывать** про conflict с системным dnsmasq на порту 53
5. **НЕ запускать** wifi-connect слишком рано — приводит к конфликту с NetworkManager на wlan0

### Ключевые timing моменты:

```
0 сек    - Загрузка системы
~5 сек   - NetworkManager активен
~15 сек  - wifi-connect скрипт запускается
~15-20   - iwgetid проверка: есть WiFi?
          ├─ ДА → skip wifi-connect
          └─ НЕТ → запуск wifi-connect бинарника
~60 сек  - NetworkManager полностью подключен (worst case)
```

**Вывод:** 15 секунд задержки достаточно для большинства случаев автоподключения.

---

## Ссылки

- [wifi-connect GitHub](https://github.com/balena-os/wifi-connect)
- [Официальный start.sh пример](https://github.com/balena-os/wifi-connect/blob/master/scripts/start.sh)
- [Документация по аргументам](https://github.com/balena-os/wifi-connect/blob/master/docs/command-line-arguments.md)
- [NetworkManager](https://networkmanager.dev/)


