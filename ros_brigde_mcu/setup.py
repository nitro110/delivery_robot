import os
from glob import glob

from setuptools import setup
package_name = 'ros_brigde_mcu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khanh',
    maintainer_email='kdamquoc@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [    
                "sensor_data_publisher = ros_brigde_mcu.ros_read_serial:main",
                "cmd_vel_serial = ros_brigde_mcu.ros_send_serial:main",   
        ],
    },
)
