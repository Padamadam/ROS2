from setuptools import setup
import os
from glob import glob


package_name = 'lab3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/display.launch.py')),
        (os.path.join('share', package_name), glob('urdf/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
        (os.path.join('share', package_name), glob('config/params.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adam',
    maintainer_email='01168912@pw.edu.pl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
            ["forward_kin_node = lab3.forward_kin_node:main",
             "state_publisher = lab3.state_publisher:main",
             "marker_publisher = lab3.marker_publisher:main",
             "marker_broker = lab3.marker_broker:main",
             "pick_node = lab3.pick_node:main", 
             "inverse_kin_node = lab3.inverse_kin:main",
             "pick_sim = lab3.pick_sim:main"],
    },
)
