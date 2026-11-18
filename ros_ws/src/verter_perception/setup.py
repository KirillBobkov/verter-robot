from pathlib import Path

from setuptools import find_packages, setup


name = 'verter_perception'
data = [
    ('share/ament_index/resource_index/packages', ['resource/' + name]),
    ('share/' + name, ['package.xml']),
]

for folder in ('launch', 'config'):
    for path in Path(folder).rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.suffix != '.pyc':
            data.append(('share/' + name + '/' + str(path.parent), [str(path)]))

setup(
    name=name,
    version='0.1.0',
    packages=find_packages(),
    data_files=data,
    install_requires=['setuptools'],
    zip_safe=False,
    entry_points={
        'console_scripts': [
            'depth_obstacles = verter_perception.depth_obstacles:main',
            'ultrasonic_ranges = verter_perception.ultrasonic_ranges:main',
        ]
    },
)
