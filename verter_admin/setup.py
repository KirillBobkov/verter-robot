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

def get_gigaam_v3_model_files():
    """Функция для копирования моделей GigaAM v3 в правильное место"""
    model_files = []
    gigaam_v3_dir = 'src/verter_admin/speech_to_text/gigaam-v3-sherpa-onnx'

    if os.path.isdir(gigaam_v3_dir):
        # CTC модель
        ctc_files = [
            'gigaam_v3_ctc_int8.onnx',
            'gigaam_v3_ctc_tokens.txt',
        ]
        for filename in ctc_files:
            src_path = os.path.join(gigaam_v3_dir, filename)
            if os.path.exists(src_path):
                model_files.append(('share/verter_admin/gigaam-v3-sherpa-onnx', [src_path]))

    # GigaAM v3 RNNT (transducer) с пунктуацией
    gigaam_v3_rnnt_dir = 'src/verter_admin/speech_to_text/sherpa-onnx-nemo-transducer-punct-giga-am-v3-russian-2025-12-16'
    if os.path.isdir(gigaam_v3_rnnt_dir):
        rnnt_files = ['encoder.int8.onnx', 'decoder.onnx', 'joiner.onnx', 'tokens.txt']
        for filename in rnnt_files:
            src_path = os.path.join(gigaam_v3_rnnt_dir, filename)
            if os.path.exists(src_path):
                model_files.append(('share/verter_admin/sherpa-onnx-nemo-transducer-punct-giga-am-v3-russian-2025-12-16', [src_path]))

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
            # Включаем файлы Piper (.onnx, .json), Silero (.pt) и Vosk TTS
            if (filename.startswith('ru_') and (filename.endswith('.onnx') or filename.endswith('.json') or filename.endswith('.txt'))) or \
               filename.endswith('.pt') or \
               filename in ['model.onnx', 'dictionary', 'config.json', 'vocab.txt']:
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

def get_web_frontend_files():
    """Копирует собранный React frontend из frontend/dist/"""
    files = []
    dist_dir = 'src/verter_admin/web/frontend/dist'
    if os.path.isdir(dist_dir):
        for root, dirs, filenames in os.walk(dist_dir):
            for filename in filenames:
                src_path = os.path.join(root, filename)
                # rel_path будет 'index.html', 'assets/index.xxx.js', etc.
                rel_path = os.path.relpath(src_path, dist_dir)
                dest_dir = os.path.join('share', package_name, 'web', os.path.dirname(rel_path))
                files.append((dest_dir, [src_path]))
    return files

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
        # Включаем конфигурацию шасси
        (os.path.join('share', package_name, 'chassis'), glob(os.path.join('src/verter_admin/chassis', '*.yaml'))),
        # Включаем конфигурацию Nav2
        (os.path.join('share', package_name, 'config', 'nav2'), glob(os.path.join('src/verter_admin/config/nav2', '*.yaml'))),
        # Включаем конфигурацию SLAM
        (os.path.join('share', package_name, 'config', 'slam'), glob(os.path.join('src/verter_admin/config/slam', '*.yaml'))),
        # Включаем конфигурацию Robot Localization (EKF)
        (os.path.join('share', package_name, 'config', 'robot_localization'), glob(os.path.join('src/verter_admin/config/robot_localization', '*.yaml'))),
        # Включаем конфигурацию Explore Lite
        (os.path.join('share', package_name, 'config', 'explore'), glob(os.path.join('src/verter_admin/config/explore', '*.yaml'))),
        # Включаем конфигурацию Twist Mux
        (os.path.join('share', package_name, 'config', 'twist_mux'), glob(os.path.join('src/verter_admin/config/twist_mux', '*.yaml'))),
        # Включаем конфигурацию Laser Filters
        (os.path.join('share', package_name, 'config', 'laser_filters'), glob(os.path.join('src/verter_admin/config/laser_filters', '*.yaml'))),
        # Включаем веб-интерфейс (React build)
        # Сборка React приложения должна быть выполнена заранее: cd src/verter_admin/web/frontend && npm run build
        # Файлы из dist/ копируются в share/verter_admin/web/
        *get_web_frontend_files(),
        # Включаем скрипты (save_map.sh и др.)
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*.sh'))),
        # Включаем конфигурацию RViz (waypoint_navigation.rviz)
        (os.path.join('share', package_name, 'config', 'rviz'), glob(os.path.join('src/verter_admin/config/rviz', '*.rviz'))),
        # Arduino скетчи не включаем в сборку
        # Включаем dataset в пакет
        *get_dataset_files(),
        # Включаем модели Sherpa-ONNX в пакет
        *get_sherpa_model_files(),
        # Включаем модель GigaAM v3 RNNT INT8 в пакет
        *get_gigaam_v3_model_files(),
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
        'piper-tts',
        'vosk-tts',
        'ru-normalizr',
        'pyserial',
        'opencv-python<4.9',  # Совместимо с numpy<2.0
        'kaldi-native-fbank',
        'onnxruntime',
        'torch',
        'sherpa-onnx',
        'openai',
    ], 
    zip_safe=True,
    maintainer='Имя Пользователя',
    maintainer_email='user@example.com',
    description='Пакет для распознавания речи с помощью Sherpa-ONNX CTC',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'speech_to_text_node = verter_admin.speech_to_text.speech_to_text_node:main',
            'speech_to_text_transducer_node = verter_admin.speech_to_text.speech_to_text_transducer_node:main',
            'speech_to_text_sherpa_node = verter_admin.speech_to_text.speech_to_text_sherpa_node:main',
            'speech_to_text_parakeet_node = verter_admin.speech_to_text.speech_to_text_parakeet_node:main',
            'speech_to_text_gigaam_v3_ctc_node = verter_admin.speech_to_text.speech_to_text_gigaam_v3_ctc_node:main',
            'speech_to_text_gigaam_v3_rnnt_node = verter_admin.speech_to_text.speech_to_text_gigaam_v3_rnnt_node:main',
            'recognition_node = verter_admin.recognition.recognition_node:main',
            'ai_assistant_node = verter_admin.ai_assistant.ai_assistant_node:main',
            'text_to_speech_node = verter_admin.text_to_speech.text_to_speech_node:main',
            'silero_tts_node = verter_admin.text_to_speech.silero_tts_node:main',
            'vosk_tts_node = verter_admin.text_to_speech.vosk_tts_node:main',
            'sound_player_node = verter_admin.sound_player.sound_player_node:main',

            'chassis_zlac_node = verter_admin.chassis.chassis_zlac_node:main',
            'distance_sensors_node = verter_admin.control.adapters.ros.distance_sensors_node:main',
            'range_converter_node = verter_admin.control.adapters.ros.range_converter_node:main',
            'odometry_node = verter_admin.control.adapters.ros.odometry_node:main',
            'doa_node = verter_admin.doa.doa_node:main',
            'teleop_keyboard = verter_admin.control.adapters.ros.teleop_keyboard:main',
            'calibration_node = verter_admin.calibration.calibration_node:main',
            'rplidar_node = verter_admin.control.adapters.ros.rplidar_node:main',
            'proximity_safety_node = verter_admin.control.adapters.ros.proximity_safety_node:main',
            'waypoint_manager_node = verter_admin.waypoints.adapters.ros.waypoint_manager_node:main',
            'web_server_node = verter_admin.web.adapters.ros.web_server_node:main'
        ],
    },
)
