#!/usr/bin/env python3
"""
===================================================================================
ТЕСТОВЫЙ ROS2 УЗЕЛ ДЛЯ ЗАПУСКА ROBOT_STATE_PUBLISHER
===================================================================================

ПРОСТОЙ ЗАПУСК robot_state_publisher ДЛЯ VERTER ROBOT

ЧТО ДЕЛАЕТ:
-----------
Запускает robot_state_publisher напрямую через команду ros2 run

КАК ЗАПУСТИТЬ:
--------------
cd /путь/к/verter_admin
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
python3 src/verter_admin/launch/test_urdf_node.py

===================================================================================
"""

import os
import sys
import subprocess


def main():
    print("\n" + "="*80)
    print("ЗАПУСК ROBOT_STATE_PUBLISHER")
    print("="*80 + "\n")

    # Получаем путь к URDF
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, '..', 'urdf', 'verter_robot.urdf')
    urdf_path = os.path.normpath(urdf_path)

    if not os.path.exists(urdf_path):
        print(f"❌ URDF файл не найден: {urdf_path}")
        return

    print(f"📁 URDF файл: {urdf_path}")

    # Читаем URDF и создаем временный файл без кириллицы в комментариях
    print("🔄 Обработка URDF...")

    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_lines = f.readlines()

    # Создаем временный URDF без длинных комментариев
    import tempfile
    temp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False, encoding='utf-8')

    in_comment = False
    for line in urdf_lines:
        # Пропускаем длинные блоки комментариев в начале
        if '===' in line or 'ЧТО ТАКОЕ' in line or 'ЗАЧЕМ' in line:
            continue
        # Оставляем только XML
        if not line.strip().startswith('<!--') or '-->' in line:
            temp_urdf.write(line)

    temp_urdf.close()
    print(f"✅ Создан временный URDF: {temp_urdf.name}")

    # Запускаем robot_state_publisher
    print("\n🚀 Запуск robot_state_publisher...")
    print("="*80)

    try:
        # Читаем содержимое временного URDF
        with open(temp_urdf.name, 'r', encoding='utf-8') as f:
            urdf_content = f.read()

        # Запускаем через ros2 run с передачей URDF через --ros-args
        cmd = [
            'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
            '--ros-args',
            '-p', f'robot_description:={urdf_content}',
            '-p', 'publish_frequency:=50.0'
        ]

        print("Команда: ros2 run robot_state_publisher robot_state_publisher")
        print("Параметры:")
        print("  - robot_description: <URDF content>")
        print("  - publish_frequency: 50.0 Hz")
        print("")

        # Запускаем процесс
        print("✅ Запуск...")
        process = subprocess.run(cmd, check=True)

        print("\n✅ robot_state_publisher завершен")

    except subprocess.CalledProcessError as e:
        print(f"❌ Ошибка запуска: {e}")
        print("\nПопробуйте запустить вручную:")
        print(f"  ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"$(cat {temp_urdf.name})\"")

    except KeyboardInterrupt:
        print("\n⚠️  Прервано пользователем (Ctrl+C)")

    finally:
        # Удаляем временный файл
        if os.path.exists(temp_urdf.name):
            os.remove(temp_urdf.name)
            print(f"🗑️  Временный файл удален")


if __name__ == '__main__':
    main()
