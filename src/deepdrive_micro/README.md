# deepdrive_micro

ros micro interface to raspberry pi pico

Interfaces:
- Motors
- Motor rotary encoder
- LED?
- GPS?
- LIDAR?


## Mount RPI Pico

*Hold boot button when plugging in*

```sh
sudo mkdir /mnt/sda1
sudo vi /etc/fstab

# fstab
/dev/sda1 /mnt/sda1 vfat defaults 0 0

sudo mount /mnt/sda1
```

## Build & Flash

```sh
cbuild --cmake-args -DPICO_BOARD=pico_w --packages-select deepdrive_micro

# Copy it to src so the host machine has access
cp build/deepdrive_micro/deepdrive_micro.uf2 src/deepdrive_micro

# Copy it to rpi pico
sudo cp src/deepdrive_micro/deepdrive_micro.uf2 /mnt/sda1
```

## Run micro ros Agent
```sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 baudrate=115200

ros2 topic echo deepdrive_micro_publisher
```

## Publish speed
```sh
ros2 topic pub --once deepdrive_micro/cmd std_msgs/msg/Int32 "{data: 65000}"
```

