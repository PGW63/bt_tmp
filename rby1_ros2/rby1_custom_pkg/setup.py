from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rby1_custom_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'configs'), glob(os.path.join('configs', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_navigate = rby1_custom_pkg.autonomous:main',
            'yolov8_node = rby1_custom_pkg.yolo:main',
            'semantic_navigator2 = rby1_custom_pkg.tmp:main',
            'semantic_navigator3 = rby1_custom_pkg.action:main',
        ],
    },
)
