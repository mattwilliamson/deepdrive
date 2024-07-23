# Matek 3901-l0x Optical Flow Sensor

## About the device

Optical flow sensor
Used to calculate odometry by pixels moving across a camera.

https://www.mateksys.com/?portfolio=3901-l0x
https://www.aliexpress.us/item/3256805958447669.html

- Optical Flow: PMW3901
- Lidar: ST VL53L0X (max. range 2 m)
- Interface UART
- Protocol: MSP v2
- Working Range: 8cm~200cm
- Field of view: 42 degree(PMW3901), 27 degree(VL53L0X)
- Minimum Illumination >60Lux
- Input voltage: 4.5~5.5V
- Power Consumption: 40mA


## Installation

Setup device alias to `/dev/matek`

```sh
$ sudo dmesg -w
[12484.664665] usb 1-2.4.1.3: new full-speed USB device number 17 using tegra-xusb
[12484.771878] usb 1-2.4.1.3: New USB device found, idVendor=067b, idProduct=2303, bcdDevice= 4.00
[12484.771886] usb 1-2.4.1.3: New USB device strings: Mfr=1, Product=2, SerialNumber=0
[12484.771889] usb 1-2.4.1.3: Product: USB-Serial Controller D
[12484.771892] usb 1-2.4.1.3: Manufacturer: Prolific Technology Inc. 
[12484.773857] pl2303 1-2.4.1.3:1.0: pl2303 converter detected
[12484.789378] usb 1-2.4.1.3: pl2303 converter now attached to ttyUSB1
```

```sh
sudo apt install libudev-dev

cat << EOF | sudo tee /etc/udev/rules.d/99-matek.rules
# Alias the serial converter to /dev/matek
KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0777", SYMLINK+="matek"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

Check it:
```sh
screen -L /dev/matek 115200
```

## Running

```sh
ros2 launch deepdrive_optical_flow matek.launch.py
```
