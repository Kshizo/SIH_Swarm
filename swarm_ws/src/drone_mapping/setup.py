from setuptools import find_packages, setup

package_name = 'drone_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam_mapping.launch.py', 'launch/beta_stack.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/drone.urdf']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async.yaml', 'config/ekf.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atharv',
    maintainer_email='atharvprasad6@gmail.com',
    description='Lidar odometry plumbing and the vision-position bridge for GPS-denied flight',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pose_to_tf = drone_mapping.pose_to_tf:main',
            'vio_to_ardupilot = drone_mapping.vio_to_ardupilot:main',
            'optical_flow = drone_mapping.optical_flow:main'
        ],
    },
)
