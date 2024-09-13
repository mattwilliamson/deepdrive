from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ble_odom'

SHARE_DIR = os.path.join("share", package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'bleak',
    ],
    zip_safe=True,
    maintainer='Matt Williamson',
    maintainer_email='matt@aimatt.com',
    description='Basic position estimation based on RSSI of Eddystone BLE beacons',
    license='MIT',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "calibrate = ble_odom.scan:main",
        ],
    },
)
