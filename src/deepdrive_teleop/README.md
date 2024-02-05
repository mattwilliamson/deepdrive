# deepdrive_teleop

# PS4/DS4 Controller

## Pair Bluetooth

Don't do this in docker.

Hold PS+Share button until it starts flashing to enter pairing mode AFTER starting to scan

Pair
```sh
sudo apt install -y joystick jstest-gtk evtest
bluetoothctl
[bluetooth]# scan on

[NEW] Device F4:93:9F:9B:D8:D1 Wireless Controller

[bluetooth]# pair F4:93:9F:9B:D8:D1
[bluetooth]# trust F4:93:9F:9B:D8:D1
[bluetooth]# connect F4:93:9F:9B:D8:D1
Attempting to connect to F4:93:9F:9B:D8:D1
[CHG] Device F4:93:9F:9B:D8:D1 ServicesResolved: yes
Connection successful

[bluetooth]# info F4:93:9F:9B:D8:D1
Device F4:93:9F:9B:D8:D1 (public)
	Name: Wireless Controller
	Alias: Wireless Controller
	Class: 0x00002508
	Icon: input-gaming
	Paired: yes
	Trusted: yes
	Blocked: no
	Connected: no
	LegacyPairing: no
	UUID: Human Interface Device... (00001124-0000-1000-8000-00805f9b34fb)
	UUID: PnP Information           (00001200-0000-1000-8000-00805f9b34fb)
	Modalias: usb:v054Cp09CCd0100
	RSSI: -65
```

```sh
# sudo apt-get install -y jstest-gtk ros-humble-joy-linux ros-humble-teleop-twist-joy

make dockershell
root@deepdrive:/ros_ws# ros2 run joy joy_enumerate_devices
ID : GUID                             : GamePad : Mapped : Joystick Device Name
-------------------------------------------------------------------------------
 0 : 050000004c050000cc09000000810000 :    true :   true : PS4 Controller
```


```sh
# Start robot
ros2 launch deepdrive_bringup launch_control.launch.py

# Start teleop
# ros2 run teleop_twist_joy teleop_node --ros-args -p axis_linear.x:=1 -p axis_angular.yaw:=0
ros2 run joy joy_node --ros-args --params-file src/deepdrive_bringup/config/joystick.yaml
ros2 run teleop_twist_joy teleop_node --ros-args --params-file src/deepdrive_bringup/config/joystick.yaml -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```





## Keyboard WASDX

```sh
ros2 run deepdrive_teleop teleop_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```