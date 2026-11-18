# Отдельный CAD-просмотр

`verter_cad_preview` — статическая модель корпуса и четырёх колёс для просмотра геометрии. Она не входит в аппаратный запуск, не содержит collision/inertial для динамической симуляции; колёса закреплены fixed. Активная модель робота находится в `ros_ws/src/verter_description`, её размеры, TF и просмотр описаны в [GEOMETRY.md](../../docs/GEOMETRY.md).

В preview `base_link` находится на высоте ведущей оси 98 мм; у активной модели — 100 мм. Не переносить между ними TF без пересчёта. Преобразование CAD записано в `measurements.json`. `preview.png` — статический рендер, не снимок RViz. Датчики в этот отдельный просмотр не включены.

Для просмотра скопировать пакет в `src` отдельного Linux workspace с ROS 2 Humble и RViz:

```bash
source /opt/ros/humble/setup.bash
cd ~/verter_preview_ws
colcon build --packages-select verter_cad_preview
source install/setup.bash
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
ros2 launch verter_cad_preview preview.launch.py
```

Домен 77 должен быть свободен и отличаться от домена робота. Запускаются только robot_state_publisher и RViz; оборудование не требуется.
