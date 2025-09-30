from setuptools import find_packages, setup

package_name = 'task'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gw',
    maintainer_email='adgjl06@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = task.detector:main',
            'auto_captioner_node = task.vlm:main',
            'yolo2_node = task.detect2:main',
        ],
    },
)
