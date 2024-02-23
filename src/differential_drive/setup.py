from glob import glob
import os
from setuptools import setup

package_name = "differential_drive"
SHARE_DIR = os.path.join("share", package_name)

setup(
    name=package_name,
    version='0.0.1',
    packages=["differential_drive"],
    package_dir={'': 'src', },
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml")))],
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyside2',
                      'numpy'],
    author='Matt Williamson',
    maintainer='Matt Williamson',
    keywords=['ROS2'],
    description='The differential_drive packageProvides some basic tools for interfacing a differential-drive robot with the ROS navigation stack.',
    license='BSD',
    entry_points={
        'console_scripts': ['diff_tf = differential_drive.diff_tf:main',
                            'pid_velocity = differential_drive.pid_velocity:main',
                            'twist_to_motors = differential_drive.twist_to_motors:main',
                            'virtual_joystick = differential_drive.virtual_joystick:main',
                            'wheel_loopback = differential_drive.wheel_loopback:main',
                            'wheel_scaler = differential_drive.wheel_scaler:main'
                            ],
    }
)
