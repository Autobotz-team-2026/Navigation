from setuptools import find_packages, setup

package_name = 'scara_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jpedro',
    maintainer_email='joaopedroevangelista63@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scara_controller = scara_controller.scara:main',
            'sensor_tests = scara_controller.sensors_test:main',
            'goal_pub = scara_controller.goal_pub:main',
            'height_sensor = scara_controller.laser_to_distance_sensor_convertor:main'
        ],
    },
)
