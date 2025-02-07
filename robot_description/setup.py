from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.py')),  # Install launch files
        ('share/' + package_name + '/urdf',
         glob('urdf/*.xacro')),  # Install URDF files
        ('share/' + package_name + '/worlds',
         glob('worlds/*.world')),  # Install world files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='esube',
    maintainer_email='esubalewchekol6@gmail.com',
    description='A package for defining the robot world',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
