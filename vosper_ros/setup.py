from setuptools import setup

package_name = 'vosper_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vosper.launch.py']),
        ('share/' + package_name + '/launch', ['launch/receptionist.launch.py']),
        ('share/' + package_name + '/config', ['config/data.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 wrapper for vosper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vosper_node = vosper_ros.vosper_node:main',
            'vosper_node_receptionist = vosper_ros.vosper_node_receptionist:main',
            'speech_to_tts = vosper_ros.speech_to_tts:main',
            'vosper_node2 = vosper_ros.bt_vosper:main',
        ],
    },
)

