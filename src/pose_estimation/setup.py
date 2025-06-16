from setuptools import setup

package_name = 'pose_estimation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/json', [
            'json/rmis_left_arm.json',
            'json/rmis_right_arm.json',
            'json/rmis_left_leg.json',
            'json/rmis_right_leg.json',
            'json/rmisurdf.urdf',
        ]),
        ('share/' + package_name + '/launch', ['launch/pose_estimation_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='enesisaoglu',
    maintainer_email='enesisaoglu@todo.todo',
    description='Pose estimation package for RMIS robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_processing_node = pose_estimation.image_processing_node:main',
            'rmis_mimic_node = pose_estimation.rmis_mimic_node:main'
        ],
    },
)