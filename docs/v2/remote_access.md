# Удалённое подключение к роботу

Два сценария:

1. **SSH-подключение к роботу** — управление, отладка, запуск нод
2. **RVIZ с ноутбука** — визуализация карты и навигации через X-forwarding

---

## 1. SSH-подключение к роботу

### Локальная сеть / прямой Ethernet

В лаборатории можно подключиться по кабелю Ethernet напрямую (без WiFi):

```bash
ssh jetson@192.168.0.9
```

Тот же адрес работает и в локальной WiFi-сети.

### Внешний доступ (через NAT)

```bash
ssh jetson@109.195.134.20 -p 3333
```

Пароль: пароль пользователя `---`.

### Определение адресов на роботе

Если вы уже зашли на робота (или у вас есть физический доступ к терминалу):

```bash
# Показать локальный IP в сети
hostname -I

# Показать внешний IP (через NAT)
curl ifconfig.me
```

`hostname -I` пригодится когда нужно узнать, какой IP получил робот от DHCP. `curl ifconfig.me` — когда заходите из дома и нужен внешний адрес для `-p 3333`.

### Работа с файлами

Можно копировать файлы через `scp`, но удобнее подключиться через **Visual Studio Code Remote SSH**:

1. Установите расширение **Remote — SSH** в VS Code
2. Нажмите `F1` → **Remote-SSH: Connect to Host...**
3. Введите: `ssh jetson@192.168.0.9` (локально) или `ssh jetson@109.195.134.20 -p 3333` (внешне)
4. Откроется полноценный редактор с файловой системой робота — редактируйте, копируйте, перетаскивайте файлы

Пароль запрашивается один раз при подключении.

### Найти IP робота в локальной сети

Если IP неизвестен:

```bash
# Через broadcast ping
sudo apt install arp-scan
sudo arp-scan --localnet | grep -i "jetson\|nvidia"

# Или через nmap
sudo apt install nmap
nmap -sn 192.168.0.0/24 | grep -B2 jetson
```

---

## 2. RVIZ с ноутбука

На Jetson нет монитора — управляем роботом и видим карту с ноутбука в одной локальной сети.

### Требования

- X11-совместимый SSH-клиент:
  - **Linux / macOS** — встроенный `ssh` (дополнительно на macOS может понадобиться [XQuartz](https://www.xquartz.org/))
  - **Windows** — [MobaXterm](https://mobaxterm.mobatek.net/) (X-server встроен) или PuTTY + [Xming](https://sourceforge.net/projects/xming/)

### Запуск RVIZ через SSH

```bash
ssh -X jetson@192.168.0.9 "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && rviz2 -d ~/verter-robot/verter_admin/src/verter_admin/config/rviz/waypoint_navigation.rviz"
```

Флаг `-X` включает X-forwarding — окно RVIZ откроется на вашем ноутбуке, а считать всё будет на Jetson.

### Пошагово (интерактивный режим)

```bash
# 1. Подключиться с X-forwarding
ssh -X jetson@192.168.0.9

# 2. На роботе загрузить ROS2 и запустить RVIZ
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/verter-robot/verter_admin/src/verter_admin/config/rviz/waypoint_navigation.rviz
```

### Диагностика X-forwarding

```bash
# Проверить, что DISPLAY установлен
echo $DISPLAY
# Ожидаемый вывод: localhost:10.0 (или :0)

# Если пусто — X-forwarding не включён
# Проверьте флаг -X при ssh
```

### Если RVIZ не открывается (`Cannot open display`)

1. **macOS**: установите [XQuartz](https://www.xquartz.org/), перезагрузите сессию терминала.
2. **Linux**: `echo $DISPLAY` — должно быть `:0` или `:1`. Если пусто: `export DISPLAY=:0`.
3. **Windows**: убедитесь что X-server запущен (MobaXterm стартует автоматически; Xming — вручную через XLaunch).
4. При подключении через внешний адрес X-forwarding может работать медленно — используйте локальную сеть.
