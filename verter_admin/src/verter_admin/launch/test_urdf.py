#!/usr/bin/env python3
"""
===================================================================================
ТЕСТОВЫЙ СКРИПТ ДЛЯ ЗАПУСКА ROBOT_STATE_PUBLISHER
===================================================================================

ЧТО ДЕЛАЕТ ЭТОТ СКРИПТ:
-----------------------
1. Читает URDF файл verter_robot.urdf
2. Запускает robot_state_publisher с URDF моделью
3. Публикует TF трансформации между всеми частями робота

ЗАЧЕМ НУЖЕН ЭТОТ СКРИПТ:
-------------------------
Обычный launch файл не работает в conda окружении из-за проблем с путями.
Этот скрипт обходит проблему, читая URDF напрямую и передавая его как параметр.

КАК ЗАПУСТИТЬ:
--------------
# Активировать conda окружение
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

# Запустить скрипт
cd /путь/к/verter_admin
python3 src/verter_admin/launch/test_urdf.py

ПРОВЕРКА ЧТО РАБОТАЕТ:
----------------------
# В другом терминале:
ros2 topic list       # Должен быть /robot_description
ros2 topic echo /robot_description -n 1  # Показать URDF
ros2 run tf2_ros tf2_echo base_link wheel_left  # Проверить TF

===================================================================================
"""

import os
import sys

# Добавляем путь к пакету
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess


def read_urdf_file():
    """
    Читает URDF файл и возвращает его содержимое как строку.

    Returns:
        str: Содержимое URDF файла
    """
    # Получаем путь к URDF файлу
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, '..', 'urdf', 'verter_robot.urdf')
    urdf_path = os.path.normpath(urdf_path)

    print(f"📁 Чтение URDF файла: {urdf_path}")

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"❌ URDF файл не найден: {urdf_path}")

    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    print(f"✅ URDF файл прочитан успешно ({len(urdf_content)} символов)")
    return urdf_content


def main():
    """
    Главная функция: запускает robot_state_publisher с URDF моделью.
    """
    print("\n" + "="*80)
    print("ЗАПУСК ROBOT_STATE_PUBLISHER ДЛЯ VERTER ROBOT")
    print("="*80 + "\n")

    # Инициализируем ROS2
    print("🔧 Инициализация ROS2...")
    rclpy.init()

    # Читаем URDF файл
    try:
        urdf_content = read_urdf_file()
    except Exception as e:
        print(f"❌ Ошибка чтения URDF: {e}")
        rclpy.shutdown()
        return

    # Получаем путь к URDF файлу (используем оригинальный файл)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_file_path = os.path.join(script_dir, '..', 'urdf', 'verter_robot.urdf')
    urdf_file_path = os.path.normpath(urdf_file_path)

    print(f"📝 Используем URDF файл: {urdf_file_path}")

    # Запускаем robot_state_publisher как subprocess
    print("\n🚀 Запуск robot_state_publisher...")
    print("="*80)

    temp_urdf_name = None  # Для finally блока

    try:
        # Используем xacro для обработки URDF (xacro может работать с кириллицей)
        # Команда для запуска robot_state_publisher
        # Используем shell=True чтобы правильно обработать xacro команду

        cmd = f'ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat {urdf_file_path})" -p publish_frequency:=50.0'

        print(f"Команда: ros2 run robot_state_publisher robot_state_publisher ...")
        print("")

        # Запускаем процесс через shell для корректной работы с $(cat ...)
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=False  # Используем bytes вместо text для избежания проблем с кодировкой
        )

        print("✅ robot_state_publisher запущен (PID: {})".format(process.pid))
        print("\n" + "="*80)
        print("ИНФОРМАЦИЯ")
        print("="*80)
        print("robot_state_publisher публикует:")
        print("  • TF трансформации: base_link → wheels, sensors")
        print("  • Topic: /robot_description")
        print("  • Частота: 50 Hz")
        print("\nДля проверки (в другом терминале):")
        print("  ros2 topic list")
        print("  ros2 run tf2_ros tf2_echo base_link wheel_left")
        print("  ros2 run tf2_tools view_frames")
        print("\nДля остановки: Ctrl+C")
        print("="*80 + "\n")

        # Ждем завершения процесса
        try:
            stdout, stderr = process.communicate()

            if stdout:
                try:
                    stdout_text = stdout.decode('utf-8', errors='ignore')
                    if stdout_text.strip():
                        print("STDOUT:")
                        print(stdout_text)
                except:
                    pass

            if stderr:
                try:
                    stderr_text = stderr.decode('utf-8', errors='ignore')
                    if stderr_text.strip():
                        print("STDERR:")
                        print(stderr_text)
                except:
                    pass

        except KeyboardInterrupt:
            print("\n⚠️  Получен сигнал остановки (Ctrl+C)")
            print("🛑 Останавливаем robot_state_publisher...")
            process.terminate()
            process.wait(timeout=5)
            print("✅ robot_state_publisher остановлен")

    except Exception as e:
        print(f"❌ Ошибка при запуске robot_state_publisher: {e}")

    finally:
        # Завершаем ROS2
        rclpy.shutdown()
        print("\n✅ Скрипт завершен")


if __name__ == '__main__':
    main()
