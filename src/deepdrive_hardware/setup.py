from setuptools import find_packages
from setuptools import setup

package_name = 'deepdrive_hardware'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'Jetson.GPIO',
    ],
    zip_safe=True,
    author='Matt Williamson',
    author_email='matt@aimatt.com',
    maintainer='Matt Williamson',
    maintainer_email='matt@aimatt.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Deepdrive motor controls'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = deepdrive_hardware.script.motor_controller:main'
        ],
    },
)
