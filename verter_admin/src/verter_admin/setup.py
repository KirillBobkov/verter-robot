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

package_name = 'verter_admin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Включаем модели в пакет
        *[(os.path.join('share', package_name, os.path.dirname(f)), [f])
          for f in get_all_files('vosk-model-small-ru-0.22')],
    ],
    install_requires=['setuptools', 'vosk', 'sounddevice', 'numpy'],
    zip_safe=True,
    maintainer='Имя Пользователя',
    maintainer_email='user@example.com',
    description='Пакет для распознавания речи с помощью Vosk',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_recognition_node = speech_recognition.speech_recognition_node:main',
        ],
    },
)