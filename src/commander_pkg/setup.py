from setuptools import find_packages, setup

package_name = 'commander_pkg'

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
    maintainer='vinicius',
    maintainer_email='araujodeoliveiravinicius@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav_states = commander_pkg.nav_states:main',
            'central_controller = commander_pkg.central_controller:main',
            'apriltag_pose_node = commander_pkg.apriltag_pose_node:main',
        ],
    },
)
