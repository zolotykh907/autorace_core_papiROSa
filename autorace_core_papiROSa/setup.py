import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'autorace_core_papiROSa'

data_files = []

start_point = os.path.join('model')
for root, dirs, files in os.walk(start_point):
    root_files = [os.path.join(root, i) for i in files]
    data_files.append((os.path.join('share', package_name, root), root_files))

data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
data_files.append((os.path.join('share', package_name, 'model'), glob(os.path.join('model', '*.pt'))))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='papiROSa',
    maintainer_email='e.pyzhyanov@g.nsu.ru',
    description='Autorace 2024',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "line_detect = autorace_core_papiROSa.line_detect:main",
            "line_follow = autorace_core_papiROSa.line_follow:main",
            "sign_detection = autorace_core_papiROSa.sign_detection:main",
            "robot_rotator = autorace_core_papiROSa.robot_rotator:main",
            "traffic_light = autorace_core_papiROSa.traffic_light:main",
            "intersection = autorace_core_papiROSa.intersection:main",
        ],
    },
)
