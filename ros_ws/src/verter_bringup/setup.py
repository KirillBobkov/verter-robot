from pathlib import Path
from setuptools import setup, find_packages

name = 'verter_bringup'
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
    entry_points={'console_scripts': []},
)
