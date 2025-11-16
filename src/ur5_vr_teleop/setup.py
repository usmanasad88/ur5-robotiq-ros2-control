from setuptools import find_packages, setup

package_name = 'ur5_vr_teleop'

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
    maintainer='mani',
    maintainer_email='usman.asad@ceme.nust.edu.pk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_teleop_node = ur5_vr_teleop.vr_teleop_node:main',
        ],
    },
)
