from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5_curobo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mani',
    maintainer_email='usman.asad@ceme.nust.edu.pk',
    description='UR5 control using Curobo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'curobo_control_node = ur5_curobo_control.curobo_control_node:main',
            'face_safety_monitor = ur5_curobo_control.face_safety_monitor:main',
        ],
    },
)
