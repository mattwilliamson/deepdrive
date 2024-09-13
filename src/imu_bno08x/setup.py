from glob import glob
import os
from setuptools import setup

package_name = "imu_bno08x"
SHARE_DIR = os.path.join("share", package_name)

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join(SHARE_DIR, "rviz"), glob(os.path.join("rviz", "*.rviz"))),
        (os.path.join(SHARE_DIR, "urdf"), glob(os.path.join("urdf", "*.urdf"))),
    ],
    install_requires=[
        "setuptools",
        "Adafruit_CircuitPython_BNO08x",
        "Adafruit-extended-bus",
        "Jetson.GPIO",
    ],
    zip_safe=True,
    maintainer="Matt Williamson",
    maintainer_email="matt@aimatt.com",
    description="ROS2 Node for interface via I2C with a IMU_BNO08X Sensor and publish the IMU msg data and the Sensor orientation (roll, pitch, yaw/heading) ",
    license="MIT",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "imu_bno08x_publisher = imu_bno08x.publisher:main",
            "dummy_odom = imu_bno08x.dummy_odom:main",
        ],
    },
)
