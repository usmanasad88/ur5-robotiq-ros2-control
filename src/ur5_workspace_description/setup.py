from setuptools import setup
import os
from glob import glob

package_name = 'ur5_workspace_description'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UR5 Workspace Maintainer',
    maintainer_email='user@example.com',
    description='Workspace environment visualization and collision objects for UR5 robot',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'workspace_markers = ur5_workspace_description.workspace_markers:main',
            'workspace_collision_objects = ur5_workspace_description.workspace_collision_objects:main',
        ],
    },
)
