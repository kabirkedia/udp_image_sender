from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image_sender'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['image_sender/bandwidth_manager.py']),
        
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dtc',
    maintainer_email='kabirkedia0111@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = image_sender.client_ros2_final:main',
            'server = image_sender.server_ros2:main',
            'bandwidth_manager = image_sender.bandwidth_manager.main',
        ],
    },
)
