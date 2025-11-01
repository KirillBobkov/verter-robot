from setuptools import setup, find_packages
import os
from glob import glob

# Функция для рекурсивного сбора файлов из директории  
def get_sherpa_model_files():
    """Специальная функция для копирования моделей Sherpa-ONNX в правильное место"""
    model_files = []
    
    # CTC модель и токены
    ctc_model_dir = 'src/verter_admin/speech_to_text/sherpa-onnx-nemo-ctc-giga-am-v2-russian-2025-04-19'
    if os.path.isdir(ctc_model_dir):
        for root, dirs, filenames in os.walk(ctc_model_dir):
            for filename in filenames:
                if filename in ['model.int8.onnx', 'tokens.txt']:
                    src_path = os.path.join(root, filename)
                    rel_path = os.path.relpath(src_path, 'src/verter_admin/speech_to_text/')
                    dest_dir = os.path.join('share', 'verter_admin', os.path.dirname(rel_path))
                    model_files.append((dest_dir, [src_path]))
    
    # Transducer модель
    transducer_model_dir = 'src/verter_admin/speech_to_text/sherpa-onnx-nemo-transducer-giga-am-v2-russian-2025-04-19'
    if os.path.isdir(transducer_model_dir):
        for root, dirs, filenames in os.walk(transducer_model_dir):
            for filename in filenames:
                if filename in ['encoder.int8.onnx', 'decoder.onnx', 'joiner.onnx', 'tokens.txt']:
                    src_path = os.path.join(root, filename)
                    rel_path = os.path.relpath(src_path, 'src/verter_admin/speech_to_text/')
                    dest_dir = os.path.join('share', 'verter_admin', os.path.dirname(rel_path))
                    model_files.append((dest_dir, [src_path]))
    
    # Parakeet TDT модель
    parakeet_model_dir = 'src/verter_admin/speech_to_text/sherpa-onnx-nemo-parakeet-tdt-0.6b-v3-int8'
    if os.path.isdir(parakeet_model_dir):
        for root, dirs, filenames in os.walk(parakeet_model_dir):
            for filename in filenames:
                if filename in ['encoder.int8.onnx', 'decoder.int8.onnx', 'joiner.int8.onnx', 'tokens.txt']:
                    src_path = os.path.join(root, filename)
                    rel_path = os.path.relpath(src_path, 'src/verter_admin/speech_to_text/')
                    dest_dir = os.path.join('share', 'verter_admin', os.path.dirname(rel_path))
                    model_files.append((dest_dir, [src_path]))
    
    # VAD модель
    vad_model = 'src/verter_admin/speech_to_text/silero_vad.onnx'
    if os.path.exists(vad_model):
        model_files.append(('share/verter_admin', [vad_model]))
    
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
    tts_dir = 'src/verter_admin/text_to_speech'
    
    for root, dirs, filenames in os.walk(tts_dir):
        for filename in filenames:
            if filename.startswith('ru_') and (filename.endswith('.onnx') or filename.endswith('.json') or filename.endswith('.txt')):
                src_path = os.path.join(root, filename)
                # Убираем префикс 'src/verter_admin/text_to_speech/' из пути
                rel_path = os.path.relpath(src_path, 'src/verter_admin/text_to_speech/')
                dest_dir = os.path.join('share', 'verter_admin', 'text_to_speech', os.path.dirname(rel_path))
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
        # Включаем конфигурацию Explore Lite
        (os.path.join('share', package_name, 'config', 'explore'), glob(os.path.join('src/verter_admin/config/explore', '*.yaml'))),
        # Включаем Arduino скетчи для distance_sensors
        (os.path.join('share', package_name, 'distance_sensors'), [
            'src/verter_admin/distance_sensors/sensors.ino',
        ]),
        # Включаем dataset в пакет
        *get_dataset_files(),
        # Включаем модели Sherpa-ONNX в пакет
        *get_sherpa_model_files(),
        # Включаем TTS модели в пакет
        *get_tts_model_files(),
        # Включаем звуковые файлы в пакет
        *get_sound_files(),
    ],
    install_requires=[
        'setuptools',
        'sounddevice',
        'numpy<2.0',
        'pyusb',
        'yandex-cloud-ml-sdk',
        'silero-tts',
        'pyserial',
        'opencv-python',
        'kaldi-native-fbank',
        'onnxruntime',
        'torch',
        'sherpa-onnx',
    ], 
    zip_safe=True,
    maintainer='Имя Пользователя',
    maintainer_email='user@example.com',
    description='Пакет для распознавания речи с помощью Sherpa-ONNX CTC',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_to_text_node = verter_admin.speech_to_text.speech_to_text_node:main',
            'speech_to_text_transducer_node = verter_admin.speech_to_text.speech_to_text_transducer_node:main',
            'speech_to_text_sherpa_node = verter_admin.speech_to_text.speech_to_text_sherpa_node:main',
            'speech_to_text_parakeet_node = verter_admin.speech_to_text.speech_to_text_parakeet_node:main',
            'recognition_node = verter_admin.recognition.recognition_node:main',
            'ai_assistant_node = verter_admin.ai_assistant.ai_assistant_node:main',
            'text_to_speech_node = verter_admin.text_to_speech.text_to_speech_node:main',
            'sound_player_node = verter_admin.sound_player.sound_player_node:main',
            'chassis_node = verter_admin.chassis.chassis_node:main',
            'distance_sensors_node = verter_admin.distance_sensors.distance_sensors_node:main',
            'ultrasonic_to_laserscan_node = verter_admin.distance_sensors.ultrasonic_to_laserscan_node:main',
            'odometry_node = verter_admin.odometry.odometry_node:main',
            'doa_node = verter_admin.doa.doa_node:main',
            'tof_camera_node = verter_admin.tof_camera.tof_camera_node:main',
            'autonomous_explorer_node = verter_admin.autonomous_explorer.autonomous_explorer_node:main',
        ],
    },
)
