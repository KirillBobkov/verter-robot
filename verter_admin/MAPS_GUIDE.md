# 🗺️ Руководство по работе с картами SLAM

## 📋 Содержание
1. [Сохранение карты](#сохранение-карты)
2. [Загрузка карты](#загрузка-карты)
3. [Переключение режимов](#переключение-режимов)
4. [Примеры использования](#примеры-использования)

---

## 💾 Сохранение карты

### Способ 1: Использовать готовый скрипт (рекомендуется)

```bash
# Во время работы SLAM
cd ~/verter/verter-robot/verter_admin
./save_map.sh hospital_map
```

**Что происходит:**
- Сохраняется полная карта SLAM (`.posegraph`)
- Создаётся изображение карты (`.pgm`, `.yaml`)
- Все файлы помещаются в `~/maps/`

### Способ 2: Вручную через ROS2 сервисы

```bash
# Создать директорию для карт
mkdir -p ~/maps

# Сохранить карту
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/maps/my_map'}}"

# Дополнительно сохранить как PNG/PGM
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### Способ 3: Через RViz (интерактивно)

1. Откройте RViz
2. Добавьте панель `Panels → Add New Panel → SlamToolboxPlugin`
3. Нажмите кнопку `Save Map`
4. Введите путь к файлу

---

## 📂 Структура сохранённой карты

После сохранения карты `hospital_map` создаются файлы:

```
~/maps/
├── hospital_map.posegraph  # Полный граф SLAM (для продолжения картирования)
├── hospital_map.data       # Вспомогательные данные SLAM
├── hospital_map.pgm        # Изображение карты (чёрно-белое)
└── hospital_map.yaml       # Метаданные (разрешение, origin)
```

**Форматы:**
- **`.posegraph`** - для SLAM Toolbox (можно продолжить картирование)
- **`.pgm/.yaml`** - для Nav2 (статическая карта для локализации)

---

## 🔄 Загрузка карты

### Способ 1: Использовать готовый скрипт

```bash
# Загрузить сохранённую карту
./load_map.sh hospital_map
```

### Способ 2: Вручную через ROS2 сервис

```bash
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: {data: '$HOME/maps/hospital_map'}}"
```

### Способ 3: Настроить автозагрузку в конфиге

**Редактировать:** `config/slam/slam_toolbox_params.yaml`

```yaml
slam_toolbox:
  ros__parameters:
    mode: localization  # Переключить на локализацию
    map_file_name: /home/user/maps/hospital_map  # Путь к карте
    map_start_at_dock: true  # Загружать карту при старте
```

---

## 🔀 Переключение режимов SLAM

### Режим 1: Mapping (Картирование)

**Когда использовать:** Создание новой карты или расширение существующей.

```yaml
# slam_toolbox_params.yaml
mode: mapping
```

**Что делает:**
- Строит карту с нуля
- Оптимизирует граф в реальном времени
- Можно сохранить карту в любой момент

### Режим 2: Localization (Локализация)

**Когда использовать:** Навигация по готовой карте.

```yaml
# slam_toolbox_params.yaml
mode: localization
map_file_name: /home/user/maps/hospital_map
map_start_at_dock: true
```

**Что делает:**
- Использует готовую карту
- Определяет позицию робота на карте
- НЕ изменяет карту
- Быстрее и стабильнее

---

## 🎯 Примеры использования

### Пример 1: Создать карту больницы

```bash
# 1. Запустить робота в режиме картирования
ros2 launch verter_admin real_robot_navigation.launch.py

# 2. Управлять роботом (телеоп или через RViz)
ros2 run verter_admin teleop_keyboard

# 3. Проехать по всей территории больницы

# 4. Сохранить карту
./save_map.sh hospital_floor_1

# 5. Проверить сохранённые карты
ls -lh ~/maps/
```

### Пример 2: Навигация по сохранённой карте

```bash
# 1. Изменить режим на localization
# Отредактировать config/slam/slam_toolbox_params.yaml:
#   mode: localization
#   map_file_name: /home/user/maps/hospital_floor_1

# 2. Запустить робота
ros2 launch verter_admin real_robot_navigation.launch.py

# 3. В RViz установить начальную позицию (2D Pose Estimate)

# 4. Отправить цель (2D Nav Goal)
```

### Пример 3: Продолжить картирование существующей карты

```bash
# 1. Запустить в режиме mapping
ros2 launch verter_admin real_robot_navigation.launch.py

# 2. Загрузить существующую карту
./load_map.sh hospital_floor_1

# 3. Продолжить исследование (новые области добавятся к карте)

# 4. Сохранить обновлённую карту
./save_map.sh hospital_floor_1_extended
```

---

## 🛠️ Полезные команды

### Просмотр доступных карт

```bash
ls -1 ~/maps/*.posegraph | xargs -n 1 basename | sed 's/.posegraph//'
```

### Просмотр изображения карты

```bash
# Открыть карту в просмотрщике
eog ~/maps/hospital_map.pgm
```

### Проверка параметров карты

```bash
# Посмотреть YAML метаданные
cat ~/maps/hospital_map.yaml
```

### Конвертация карты в PNG

```bash
# Если хотите PNG вместо PGM
convert ~/maps/hospital_map.pgm ~/maps/hospital_map.png
```

### Список SLAM Toolbox сервисов

```bash
# Показать все доступные сервисы
ros2 service list | grep slam_toolbox
```

**Основные сервисы:**
- `/slam_toolbox/save_map` - сохранить карту
- `/slam_toolbox/deserialize_map` - загрузить карту
- `/slam_toolbox/serialize_map` - сериализовать текущую карту
- `/slam_toolbox/toggle_interactive_mode` - интерактивный режим
- `/slam_toolbox/clear_queue` - очистить очередь

---

## ⚠️ Важные замечания

1. **Всегда сохраняйте карту перед выключением робота!**
   - SLAM Toolbox не сохраняет автоматически
   - Используйте `./save_map.sh` или сервис

2. **Разные карты для разных этажей/зданий**
   - Создавайте отдельную карту для каждой области
   - Используйте понятные имена: `building_A_floor_1`

3. **Backup карт**
   ```bash
   # Создать резервную копию
   cp -r ~/maps ~/maps_backup_$(date +%Y%m%d)
   ```

4. **Формат .posegraph vs .pgm**
   - `.posegraph` - для продолжения SLAM (рекомендуется)
   - `.pgm/.yaml` - для статической локализации

5. **Установка начальной позиции**
   - В режиме локализации ОБЯЗАТЕЛЬНО укажите начальную позицию в RViz
   - Инструмент: `2D Pose Estimate`

---

## 🔍 Troubleshooting

### Проблема: "Failed to save map"

**Решение:**
```bash
# Проверить, что директория существует
mkdir -p ~/maps

# Проверить права доступа
chmod 755 ~/maps

# Проверить, что SLAM Toolbox работает
ros2 node list | grep slam_toolbox
```

### Проблема: "Map not found"

**Решение:**
```bash
# Проверить путь к карте
ls -la ~/maps/

# Использовать полный путь
ros2 service call /slam_toolbox/deserialize_map \
  slam_toolbox/srv/DeserializePoseGraph \
  "{filename: {data: '/home/USERNAME/maps/hospital_map'}}"
```

### Проблема: Робот не локализуется на карте

**Решение:**
1. Убедитесь, что режим = `localization`
2. Установите начальную позицию в RViz (2D Pose Estimate)
3. Проверьте, что датчики публикуют данные:
   ```bash
   ros2 topic hz /scan
   ros2 topic hz /ultrasonic/ranges
   ```

---

## 📚 Дополнительные ресурсы

- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 Map Server](https://navigation.ros.org/configuration/packages/configuring-map-server.html)
- [ROS2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Services.html)
