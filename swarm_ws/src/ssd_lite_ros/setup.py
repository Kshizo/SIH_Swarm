from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ssd_lite_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kanot',
    maintainer_email='ojaskanotra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'person_detector = ssd_lite_ros.person_detector:main',
        'camera_publisher = ssd_lite_ros.camera_publisher:main',
    ],
},
)
