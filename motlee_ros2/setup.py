from setuptools import setup
from glob import glob
import os

package_name = 'motlee_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'cfg'), glob('cfg/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='masonbp',
    maintainer_email='mbpeterson70@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mot_node = motlee_ros2.mot_node:main',
            'track_viewer_node = motlee_ros2.track_viewer_node:main',
            'mapper_node = motlee_ros2.mapper_node:main'
        ],
    },
)
