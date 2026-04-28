# Verter Robot Web UI

React + TypeScript фронтенд для управления сервисным роботом Verter.

## Разработка

### Установка зависимостей
```bash
cd src/verter_admin/web/frontend
npm install
```

### Запуск dev сервера
```bash
npm run dev
```

Dev сервер запускается на http://localhost:3000

### Переменные окружения
- `VITE_ROSBRIDGE_HOST` - хост rosbridge (по умолчанию: localhost)
- `VITE_ROSBRIDGE_PORT` - порт rosbridge (по умолчанию: 9090)

Также можно указать порт через URL: `?rosbridge_port=9095`

## Сборка

### Production build
```bash
npm run build
```

Собранные файлы попадают в `dist/`

### Интеграция с ROS2

После сборки файлы из `dist/` автоматически копируются при установке пакета:
```bash
cd ../../../..
colcon build --packages-select verter_admin --symlink-install
```

Файлы будут доступны в `install/verter_admin/share/verter_admin/web/`

## Структура проекта

```
src/
├── components/       # Переиспользуемые компоненты
│   ├── common/       # Button, BigButton, MapPlaceholder, etc.
│   ├── forms/        # LocationForm, LocationList
│   └── layout/       # SetupLayout, ActiveLayout, EmergencyStop
├── hooks/            # Custom React hooks (useROS, usePose, useCmdVel, etc.)
├── i18n/             # Переводы (ru, en)
├── pages/            # Страницы приложения
│   ├── setup/        # Режим настройки (Mapping, Home, Points)
│   └── active/       # Режим активной работы (Idle, Menu, Select, etc.)
├── services/         # ROS abstraction layer
├── store/            # Zustand stores
├── theme/            # CSS переменные и анимации
├── types/            # TypeScript типы
├── utils/            # Утилиты (quaternion, keybindings)
├── App.tsx           # Роутинг
└── main.tsx          # Точка входа
```

## ROS2 интеграция

Фронтенд communicates с ROS2 через rosbridge_websocket:
- **Topics**: `/cmd_vel`, `/amcl_pose`
- **Services**: `/save_waypoint`, `/navigate_to_waypoint`, `/delete_waypoint`, `/list_waypoints`, `/start_patrol`, `/stop_patrol`

## Архитектурные принципы

- **State Management**: Zustand для глобального состояния
- **Стилизация**: CSS Modules с глобальными CSS переменными
- **ROS2**: roslib.js через WebSocket
- **Безопасность**: EMERGENCY STOP всегда доступен на экранах активного режима
