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
        # Включаем dataset в пакет
        *get_dataset_files(),
        # Включаем модели в пакет
        *get_model_files(),
    ],
    install_requires=['setuptools', 'vosk', 'sounddevice', 'numpy', 'pyusb', 'yandex-cloud-ml-sdk'], 
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
        ],
    },
)
