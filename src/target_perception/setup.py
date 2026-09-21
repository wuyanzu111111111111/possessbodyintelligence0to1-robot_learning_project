from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'target_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maybe',
    maintainer_email='1614928784@qq.com',
    description='Deterministic ROS 2 perception and target-following pipeline',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_publisher_node = target_perception.image_publisher_node:main',
            'image_detector_node = target_perception.image_detector_node:main',
            'target_follower_node = target_perception.target_follower_node:main',
        ],
    },
)
