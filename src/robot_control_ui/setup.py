from setuptools import setup
from glob import glob
import os

package_name = 'robot_control_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'templates'), glob('robot_control_ui/templates/*.html')),
        (os.path.join('share', package_name, 'static'), glob('robot_control_ui/static/*.jpg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for robot control UI with Flask',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flask_node = robot_control_ui.flask_node:main',
        ],
    },
)