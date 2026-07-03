from setuptools import find_packages, setup

package_name = 'simulation_tests'

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
            'goal_pub = simulation_tests.goal_pub:main',
            'block_pub = simulation_tests.block_pub:main'

        ],
    },
)
