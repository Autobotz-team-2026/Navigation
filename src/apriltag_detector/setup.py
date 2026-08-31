from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'apriltag_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedro',
    maintainer_email='pedro@email.com',
    description='Deteccao de AprilTag com estimativa de pose',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_pose_node = apriltag_detector.apriltag_pose_node:main',
        ],
    },
)
