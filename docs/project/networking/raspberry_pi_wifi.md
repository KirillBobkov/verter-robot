# WiFi на Raspberry Pi (nmcli)

Примеры ниже — просто шпаргалка под NetworkManager.

```bash
# Подключение к WiFi
nmcli dev wifi connect "YOUR_SSID" password "YOUR_PASSWORD"

# Автоподключение
nmcli connection modify "YOUR_SSID" connection.autoconnect yes

# Просмотр подключений
nmcli connection show
```


