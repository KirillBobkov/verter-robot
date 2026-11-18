from pathlib import Path
from setuptools import setup, find_packages

name = 'verter_drivers'
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
            'micro_ros_serial_agent = verter_drivers.agent:main',
            'chassis_zlac_node = verter_drivers.chassis.chassis_zlac_node:main',
            'doa_node = verter_drivers.doa.doa_node:main',
            'calibration_node = verter_drivers.calibration.calibration_node:main',
        ]
    },
)
