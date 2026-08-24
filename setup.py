import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'waterlinked_sonar_3d15'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'wlsonar>=0.6.0',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Julian Valdez',
    maintainer_email='julian.valdez@example.com',
    description='ROS 2 driver for the Water Linked Sonar 3D-15.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_node = waterlinked_sonar_3d15.sonar_node:main',
            'sonar_diag = waterlinked_sonar_3d15.sonar_diag:main',
        ],
    },
)
