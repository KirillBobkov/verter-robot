from setuptools import setup, find_packages
import os
from glob import glob

# Функция для рекурсивного сбора файлов из директории
def get_all_files(directory):
    files = []
    for root, dirs, filenames in os.walk(directory):
        for filename in filenames:
            filepath = os.path.join(root, filename)
            files.append(filepath)
    return files

package_name = 'verter'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Включаем звуковые файлы
        (os.path.join('share', package_name, 'sounds'), glob(os.path.join('sounds', '*.mp3'))),
        # Включаем модели в пакет
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f])
          for f in get_all_files('vosk-model-small-ru-0.22')],
    ],
    install_requires=['setuptools', 'pyserial', 'vosk', 'sounddevice', 'pygame', 'numpy'],
    zip_safe=True,
    maintainer='Имя Пользователя',
    maintainer_email='user@example.com',
    description='Пакет для демонстрации работы датчика YL-63 с Arduino и ROS 2, включая управление движением',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'distance_controller = verter_system.distance_controller_node:main',
            'arduino_moving_navigation = verter_system.arduino_moving_navigation_node:main',
            'arduino_hands = verter_system.arduino_hands_node:main',
            'vosk_node = speech_recognition.vosk_node:main',
            'sound_player = speech_recognition.sound_player_node:main', 
        ],
    },
)
