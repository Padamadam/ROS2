from setuptools import setup
import os
from glob import glob

package_name = 'lab6'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "config"), glob('config/*')),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/urdf', glob("urdf/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='petro',
    maintainer_email='01168893@pw.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['listener = lab6.some_node:main',
        'marker_publisher = lab6.marker_publisher:main',
        'joint_state_publisher = lab6.joint_state_publisher:main',
        'camera_link_publisher = lab6.camera_link_publisher:main',
        'pick_node = lab6.PickNode:main',
        'lab6node = lab6.lab6node:main'

        ],
    },
)
