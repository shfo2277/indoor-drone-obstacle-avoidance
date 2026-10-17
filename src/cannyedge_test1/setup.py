from setuptools import setup
from glob import glob
import os

package_name = 'cannyedge_test1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', glob('resource/*')),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/msg', glob(os.path.join(package_name, 'msg', '*.msg'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 package for Canny edge detection with custom hole detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'canny_edge_detector = cannyedge_test1.canny_edge_detector:main'
        ],
    },
)

