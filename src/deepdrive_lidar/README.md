# LD19 LIDAR Scanner

https://www.waveshare.com/wiki/DTOF_LIDAR_LD19
https://www.aliexpress.us/item/3256804109024401.html


## Installation

Setup device alias to `/dev/ldlidar`

```sh
sudo apt install libudev-dev

cat << EOF | sudo tee /etc/udev/rules.d/99-ldlidar.rules
# set the udev rule , make the device_port be fixed by ldlidar
# CP210x USB Device
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="ldlidar"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Check it:
```sh
screen -L /dev/ldlidar 230400
```

## Running

```sh
ros2 launch deepdrive_lidar ld19_lidar.launch.py
```
