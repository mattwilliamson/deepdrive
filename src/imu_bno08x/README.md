# imu_bno08x

## Hardware Connection

Jetson Orin Nano

| Function | Pin Number |
| - | - |
| 3.3v | 1 |
| GND | 6 |
| I2C0_SDA | 27 |
| I2C0_SCL | 28 |

Enabled I2C
```sh
sudo /opt/nvidia/jetson-io/jetson-io.py
```

Chek it (4b)

```sh
$ sudo apt-get install -y python3-smbus i2c-tools

$ sudo  i2cdetect -r -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- UU -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: UU -- -- -- -- -- -- -- -- -- -- 4b -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --         
```

## Running

```sh
colcon build --symlink-install --packages-select imu_bno08x
source /ros_ws/install/setup.sh
ros2 run imu_bno08x imu_bno08x_publisher

ros2 launch imu_bno08x imu.launch.py
ros2 launch imu_bno08x description.launch.py
ros2 launch imu_bno08x display.launch.py
```

TODO: shake detector only lasts for a moment. Set a trigger. Param?
TODO: tf publisher to show imu working in rviz
TODO: covariance?
TODO: shake detection
TODO: change IMU axes ENU etc