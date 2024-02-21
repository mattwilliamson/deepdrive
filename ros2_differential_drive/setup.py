from glob import glob
import os
from setuptools import setup

PACKAGE_NAME = "differential_drive"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=["differential_drive"],
    package_dir={'': 'src', },
    data_files=[
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml")))],
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyside2'],
    author='Jon Stephan',
    maintainer='Jon Stephan, Mark Rose',
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
