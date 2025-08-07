from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Основной launch-файл, который запускает все ключевые системы робота.
    """
    
    # Путь к директории с launch-файлами текущего пакета
    launch_dir = FindPackageShare('verter')
    
    # 1. Запуск основной системы (verter_system.launch.py)
    verter_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/launch/verter_system.launch.py'])
    )
    
    # 2. Запуск системы распознавания речи (speech_system.launch.py)
    speech_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir, '/launch/speech_system.launch.py'])
    )

    return LaunchDescription([
        verter_system_launch,
        speech_system_launch
    ]) 