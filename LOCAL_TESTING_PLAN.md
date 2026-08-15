# План локального тестирования DIALOG_FLOW

> **Коммит:** f564aa3 (mvp: new flow with web)
> **Дата:** 2026-08-15
> **Цель:** Проверить диалоговый kiosk-интерфейс и коммуникацию UI ↔ ROS

---

## Текущее состояние (известное)

- ✅ ROS2 Humble установлен
- ✅ Пакет собран: `colcon build --packages-select verter_admin --symlink-install`
- ✅ Фронтенд собран (dist/ существует)
- ✅ Silero TTS модель есть (v5_5_ru.pt)

---

## Шаг 1: Подготовка окружения

```bash
cd ~/repos/verter-robot

# Source ROS2 и workspace
source /opt/ros/humble/setup.bash
source verter_admin/install/setup.bash

# Проверить наличие rosbridge_server
ros2 pkg list | grep rosbridge
# Если нет: sudo apt install ros-humble-rosbridge-server
```

---

## Шаг 2: Запуск ROS2 стека

**Вариант A: Полный запуск**
```bash
# Установить Yandex Cloud API ключ (обязательно для AI)
export YANDEX_CLOUD_API_KEY="your-key-here"

# Запуск
ros2 launch verter_admin main.launch.py
```

**Проверка запуска:**
- rosbridge на порту 9090: `curl http://localhost:9090`
- web_server на порту 8080: `curl http://localhost:8080`

---

## Шаг 3: Проверка ROS топиков (другой терминал)

```bash
# Source окружения
source /opt/ros/humble/setup.bash
source ~/repos/verter-robot/verter_admin/install/setup.bash

# Список топиков
ros2 topic list | grep -E "dialog|speech|ai_"

# Проверка подписчиков
ros2 topic info /dialog_status
ros2 topic info /ui_dialog_control

# Мониторинг
ros2 topic echo /dialog_status
```

---

## Шаг 4: Запуск фронтенда

**Вариант A: Dev режим (hot reload)**
```bash
cd ~/repos/verter-robot/verter_admin/src/verter_admin/web/frontend
npm run dev
# http://localhost:3000 (proxy на ws://localhost:9090)
```

**Вариант B: Через web_server_node (уже в main.launch.py)**
```
Открыть: http://localhost:8080
```

---

## Шаг 5: Проверка коммуникации UI ↔ ROS

**В браузере (DevTools Console):**

1. Проверить подключение к rosbridge
2. Нажать "Начать диалог" → проверить `/ui_dialog_control`
3. Проверить состояния FSM: idle → listening → thinking → speaking

**В ROS терминале:**
```bash
ros2 topic echo /ui_dialog_control      # Должен показать "start"
ros2 topic echo /dialog_status          # Должен показать переходы
```

---

## Критические файлы для понимания

| Файл | Назначение |
|------|-----------|
| `src/verter_admin/launch/main.launch.py` | rosbridge + web_server |
| `src/verter_admin/recognition/recognition_node.py` | FSM диалога |
| `src/verter_admin/web/frontend/src/hooks/useDialogROS.ts` | Логика в UI |

---

## Карта топиков диалога

| Топик | Тип | Значения |
|-------|-----|----------|
| `/dialog_status` | String | idle, listening, speaking, thinking, error:* |
| `/ui_dialog_control` | String | start, stop |
| `/ai_question` | String | Текст вопроса |
| `/text_to_speech` | String | Текст ответа |
| `/speech_control` | Bool | True=STT вкл, False=выкл (latched) |
| `/tts_control` | Bool | False=TTS начал, True=TTS закончил |

---

## Диагностика проблем

| Проблема | Решение |
|----------|---------|
| `rosbridge_server` не найден | `sudo apt install ros-humble-rosbridge-server` |
| Порт 9090 занят | `sudo lsof -i :9090` |
| Порт 8080 занят | `sudo lsof -i :8080` |
| UI не подключается | Проверить ws://localhost:9090 в Console |
| `/dialog_status` пуст | Проверить запуск recognition_node |
| AI ошибки | Проверить `YANDEX_CLOUD_API_KEY` |

---

## Quality Gates (опционально)

```bash
# QG-1: Компиляция
python3 -m compileall -q src/verter_admin

# QG-2/3: Hexagonal boundaries
grep -rn "import.*adapters\|import.*infrastructure" \
  src/verter_admin/domain/ src/verter_admin/application/

# QG-4: Unit тесты
cd ~/repos/verter-robot/verter_admin
PYTHONPATH=src pytest tests/unit/ai_assistant/ -v

# QG-8: cmd_vel в web
grep -rn "cmd_vel" src/verter_admin/web/frontend/src/
```

---

**При проблемах или вопросах - обращаться!**
