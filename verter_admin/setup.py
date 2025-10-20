from setuptools import setup, find_packages
import os
from glob import glob

# Функция для рекурсивного сбора файлов из директории  
def get_model_files():
    """Специальная функция для копирования модели Vosk в правильное место"""
    model_files = []
    model_dir = 'src/verter_admin/speech_recognition/vosk-model-small-ru-0.22'
    
    for root, dirs, filenames in os.walk(model_dir):
        for filename in filenames:
            src_path = os.path.join(root, filename)
            # Убираем префикс 'src/verter_admin/speech_recognition/' из пути
            rel_path = os.path.relpath(src_path, 'src/verter_admin/speech_recognition/')
            dest_dir = os.path.join('share', 'verter_admin', os.path.dirname(rel_path))
            model_files.append((dest_dir, [src_path]))
    
    return model_files

def get_dataset_files():
    """Специальная функция для копирования dataset в правильное место"""
    dataset_files = []
    dataset_dir = 'src/verter_admin/ai_assistant/dataset'
    
    for root, dirs, filenames in os.walk(dataset_dir):
        for filename in filenames:
            src_path = os.path.join(root, filename)
            # Убираем префикс 'src/verter_admin/ai_assistant/' из пути  
            rel_path = os.path.relpath(src_path, 'src/verter_admin/ai_assistant/')
            dest_dir = os.path.join('share', 'verter_admin', os.path.dirname(rel_path))
            dataset_files.append((dest_dir, [src_path]))
    
    return dataset_files

def get_tts_model_files():
    """Специальная функция для копирования TTS моделей в правильное место"""
    tts_files = []
    tts_dir = 'src/verter_admin/text_to_speech_node'
    
    for root, dirs, filenames in os.walk(tts_dir):
        for filename in filenames:
            if filename.startswith('ru_') and (filename.endswith('.onnx') or filename.endswith('.json') or filename.endswith('.txt')):
                src_path = os.path.join(root, filename)
                # Убираем префикс 'src/verter_admin/' из пути
                rel_path = os.path.relpath(src_path, 'src/verter_admin/')
                dest_dir = os.path.join('share', 'verter_admin', os.path.dirname(rel_path))
                tts_files.append((dest_dir, [src_path]))
    
    return tts_files

def get_sound_files():
    """Специальная функция для копирования звуковых файлов в правильное место"""
    sound_files = []
    sound_dir = 'src/verter_admin/sound_player/sounds'
    
    for root, dirs, filenames in os.walk(sound_dir):
        for filename in filenames:
            if filename.endswith('.mp3') or filename.endswith('.wav') or filename.endswith('.ogg'):
                src_path = os.path.join(root, filename)
                # Убираем префикс 'src/verter_admin/sound_player/' из пути
                rel_path = os.path.relpath(src_path, 'src/verter_admin/sound_player/')
                dest_dir = os.path.join('share', 'verter_admin', os.path.dirname(rel_path))
                sound_files.append((dest_dir, [src_path]))
    
    return sound_files

package_name = 'verter_admin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('src/verter_admin/launch', '*.launch.py'))),
        # Включаем URDF модели робота
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('src/verter_admin/urdf', '*.urdf'))),
        # Включаем конфигурацию Nav2
        (os.path.join('share', package_name, 'config', 'nav2'), glob(os.path.join('src/verter_admin/config/nav2', '*.yaml'))),
        # Включаем конфигурацию SLAM
        (os.path.join('share', package_name, 'config', 'slam'), glob(os.path.join('src/verter_admin/config/slam', '*.yaml'))),
        # Включаем Arduino скетчи для distance_sensors
        (os.path.join('share', package_name, 'distance_sensors'), [
            'src/verter_admin/distance_sensors/sensors.ino',
        ]),
        # Включаем dataset в пакет
        *get_dataset_files(),
        # Включаем модели Vosk в пакет
        *get_model_files(),
        # Включаем TTS модели в пакет
        *get_tts_model_files(),
        # Включаем звуковые файлы в пакет
        *get_sound_files(),
    ],
    install_requires=['setuptools', 'vosk', 'sounddevice', 'numpy', 'pyusb', 'yandex-cloud-ml-sdk', 'silero-tts', 'pyserial'], 
    zip_safe=True,
    maintainer='Имя Пользователя',
    maintainer_email='user@example.com',
    description='Пакет для распознавания речи с помощью Vosk',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_recognition_node = verter_admin.speech_recognition.speech_recognition_node:main',
            'ai_assistant_node = verter_admin.ai_assistant.ai_assistant_node:main',
            'text_to_speech_node = verter_admin.text_to_speech_node.text_to_speech_node:main',
            'silero_tts_node = verter_admin.text_to_speech_node.silero_tts_node:main',
            'sound_player_node = verter_admin.sound_player.sound_player_node:main',
            'chassis_node = verter_admin.chassis.chassis_node:main',
            'distance_sensors_node = verter_admin.distance_sensors.distance_sensors_node:main',
            'ultrasonic_to_laserscan_node = verter_admin.distance_sensors.ultrasonic_to_laserscan_node:main',
            'odometry_node = verter_admin.odometry.odometry_node:main',
            'doa_node = verter_admin.doa.doa_node:main',
        ],
    },
)
