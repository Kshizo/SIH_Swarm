from setuptools import find_packages, setup

package_name = 'drone_control'

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
    maintainer='atharv',
    maintainer_email='atharv@todo.todo',
    description='Keyboard teleop and autonomous control for ArduPilot using native DDS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = drone_control.teleop_node:main',
            'autonomous_script = drone_control.autonomous_script:main',
            'frontier_exploration = drone_control.frontier_exploration:main',
        ],
    },
)
