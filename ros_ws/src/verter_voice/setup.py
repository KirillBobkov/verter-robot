from pathlib import Path
from setuptools import setup, find_packages

name = 'verter_voice'
data = [('share/ament_index/resource_index/packages', ['resource/' + name]), ('share/' + name, ['package.xml'])]

for folder in ['launch', 'config', 'urdf', 'meshes', 'rviz', 'web']:
    for p in Path(folder).rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc':
            data.append(('share/' + name + '/' + str(p.parent), [str(p)]))

setup(
    name=name,
    version='0.1.0',
    packages=find_packages(),
    data_files=data,
    install_requires=['setuptools'],
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'speech_to_text_node = verter_voice.speech_to_text.speech_to_text_node:main',
            'speech_to_text_transducer_node = verter_voice.speech_to_text.speech_to_text_transducer_node:main',
            'speech_to_text_sherpa_node = verter_voice.speech_to_text.speech_to_text_sherpa_node:main',
            'speech_to_text_gigaam_v3_ctc_node = verter_voice.speech_to_text.speech_to_text_gigaam_v3_ctc_node:main',
            'speech_to_text_gigaam_v3_rnnt_node = verter_voice.speech_to_text.speech_to_text_gigaam_v3_rnnt_node:main',
            'recognition_node = verter_voice.recognition.recognition_node:main',
            'ai_assistant_node = verter_voice.ai_assistant.ai_assistant_node:main',
            'text_to_speech_node = verter_voice.text_to_speech.text_to_speech_node:main',
            'silero_tts_node = verter_voice.text_to_speech.silero_tts_node:main',
            'vosk_tts_node = verter_voice.text_to_speech.vosk_tts_node:main',
            'sound_player_node = verter_voice.sound_player.sound_player_node:main',
        ]
    },
)
