from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'xsens_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
        (os.path.join('share', package_name), glob('params/*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrelorent',
    maintainer_email='andrelorenzotorres@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_driver = xsens_ros2.ros2_driver:main'
        ],
    },
)
